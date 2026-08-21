"""
Does the headline finding hold under a mecanum drivetrain, not just
tank -- the same question ftc/layout_benchmark.py asks about field
layout, asked here about drivetrain instead. A large fraction of real
FTC teams run mecanum, not tank, and until now nothing in this project
showed whether the best-value sensor choice changes for them.

ftc/drivetrain_benchmark.py already crosses SUITE_ORDER against
{tank, mecanum} -- but at REDUCED rigor (15 trials, 4 levels), crossed
with fidelity tier on top, and answering a narrower, different
question (does holding a fixed heading toward a tag wall pay off for
AprilTag specifically). Its own numbers are already cited in README.md
and WRITEUPS.md and are explicitly frozen (see that module's docstring)
-- this is not the place to add a full-rigor, all-suites version of a
different comparison.

This module instead reruns the *same* full-rigor sweep ftc/
suite_benchmark.py itself runs (all 11 variance_level steps, all 3
deviation types, TRIALS_PER_COMBO trials/point, nothing reduced -- the
same reasoning ftc/layout_benchmark.py's own docstring gives for why
this question needs the same statistical footing as the headline
number itself) once under TANK and once under MECANUM, for all 7
headline suites, and reports success rate for every (suite, drivetrain)
pair.

Reuses ftc/suite_benchmark.py's run_sweep/aggregate/overall_success_rate
wholesale rather than reimplementing the sweep -- the "tank" pass this
module runs uses the exact same trial_seed formula as ftc/
suite_benchmark.py's own __main__, so its rows are trial-for-trial
identical to ftc_suite_results.csv (bar the unseeded wall-clock
planning_time_ms field) as a built-in consistency check, the same one
ftc/layout_benchmark.py runs for its "cluttered" pass.

Leaves ftc/suite_benchmark.py and ftc/drivetrain_benchmark.py, and
everything they write, completely untouched -- this module only *adds*
files.

Writes benchmark_results/ftc_drivetrain_suite_results.csv (every trial,
raw, with an added `drivetrain` column), benchmark_results/
ftc_drivetrain_suite_comparison.png (one reliability-per-dollar panel
per drivetrain, same chart type as ftc_reliability_per_dollar.png), and
benchmark_results/ftc_drivetrain_suite_writeup.md (per-drivetrain
best-value suite, and whether the "best value" conclusion is
drivetrain-dependent -- if it is, that's a finding, not a failure, and
is reported as such).
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt

from nav.stats import bootstrap_ci

from ftc.config import usd
from ftc.drivetrain import DRIVETRAIN_LABELS, DRIVETRAIN_ORDER, DRIVETRAINS
from ftc.field import build_grid, tag_sites_for
from ftc.sensors import SUITES, SUITE_ORDER, SUITE_LABELS
from ftc.suite_benchmark import (
    DEVIATION_TYPE_ORDER, SUITE_COLORS, SUMMARY_MIN_LEVEL, TRIALS_PER_COMBO, VARIANCE_LEVELS,
    aggregate, overall_success_rate, run_sweep,
)

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

DRIVETRAIN_DISPLAY_LABELS = {
    "tank": "Tank (headline default)",
    "mecanum": "Mecanum",
}


def run_drivetrain(drivetrain_name):
    """Full-rigor sweep under one drivetrain, using ftc/suite_benchmark.py's
    own seed formula unmodified (run_sweep's base_seed()) -- so the
    "tank" pass through this function reproduces ftc_suite_results.csv's
    rows trial-for-trial (see module docstring)."""
    grid = build_grid("cluttered")
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for("cluttered")

    rows = run_sweep(DEVIATION_TYPE_ORDER, VARIANCE_LEVELS, TRIALS_PER_COMBO, grid, free_cells, tag_sites,
                      drivetrain=DRIVETRAINS[drivetrain_name])
    for row in rows:
        row["drivetrain"] = drivetrain_name
    return rows


def _bootstrap_seed(drivetrain, suite):
    return 16_000_000 + DRIVETRAIN_ORDER.index(drivetrain) * 100_000 + SUITE_ORDER.index(suite)


def value_ranking(rows, drivetrain_name):
    """{suite: {rate, per_100, ci_lo, ci_hi}} for every non-baseline
    suite under this drivetrain, plus the best-value suite name -- same
    shape as ftc/layout_benchmark.py's own value_ranking."""
    matching_all = [r for r in rows if r["variance_level"] >= SUMMARY_MIN_LEVEL]
    baseline_matching = [r for r in matching_all if r["suite"] == "dead_reckoning"]
    baseline_rate = sum(r["success"] for r in baseline_matching) / len(baseline_matching)

    results = {}
    for suite in SUITE_ORDER:
        if suite == "dead_reckoning":
            continue
        matching = [r for r in matching_all if r["suite"] == suite]
        n = len(matching)
        successes = sum(r["success"] for r in matching)
        rate = successes / n
        cost = SUITES[suite].cost_usd + DRIVETRAINS[drivetrain_name].cost_usd
        per_100 = (rate - baseline_rate) / (cost / 100) * 100 if cost > 0 else float("inf")
        ci_lo, ci_hi = bootstrap_ci(successes, n, seed=_bootstrap_seed(drivetrain_name, suite))
        results[suite] = {"rate": rate, "per_100": per_100, "ci_lo": ci_lo, "ci_hi": ci_hi}
    best = max(results, key=lambda s: results[s]["per_100"])
    return results, baseline_rate, best


def write_csv(all_rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
        writer.writeheader()
        writer.writerows(all_rows)


def plot_drivetrain_comparison(per_drivetrain, path):
    fig, axes = plt.subplots(1, len(DRIVETRAIN_ORDER), figsize=(13, 5), sharey=True)
    for ax, drivetrain_name in zip(axes, DRIVETRAIN_ORDER):
        results, baseline_rate, best = per_drivetrain[drivetrain_name]
        suites = [s for s in SUITE_ORDER if s != "dead_reckoning"]
        colors = [SUITE_COLORS[s] for s in suites]
        heights = [results[s]["per_100"] for s in suites]
        bars = ax.bar([SUITE_LABELS[s] for s in suites], heights, color=colors)
        for bar, suite in zip(bars, suites):
            marker = " *best*" if suite == best else ""
            ax.annotate(f"${usd(SUITES[suite].cost_usd)}{marker}", (bar.get_x() + bar.get_width() / 2,
                        bar.get_height()), ha="center", va="bottom" if bar.get_height() >= 0 else "top",
                        fontsize=8, rotation=0)
        ax.axhline(0, color="black", linewidth=0.8)
        ax.set_title(f"{DRIVETRAIN_DISPLAY_LABELS[drivetrain_name]}\nbaseline={baseline_rate:.0%}", fontsize=10)
        ax.tick_params(axis="x", labelrotation=30, labelsize=8)
    axes[0].set_ylabel("Success-rate gain over DeadReckoningSuite\nper $100 spent (pp/$100)")
    fig.suptitle(f"Reliability per dollar by drivetrain (variance_level >= {SUMMARY_MIN_LEVEL}, "
                  f"{TRIALS_PER_COMBO} trials/point, all deviation types, 'cluttered' layout)", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(path, dpi=150)


def write_writeup(per_drivetrain, stats_by_drivetrain, path):
    lines = [
        "# Does the headline finding hold under mecanum, not just tank?",
        "",
        "`ftc_suite_writeup.md`'s headline sweep runs under `ftc/match.py`'s legacy "
        "no-drivetrain default, which is byte-for-byte the same model as an explicit TANK drivetrain "
        "(`ftc/drivetrain.py`). This reruns the identical full-rigor sweep -- same "
        f"{len(VARIANCE_LEVELS)} variance_level steps, same 3 deviation types, same "
        f"{TRIALS_PER_COMBO} trials/point, nothing reduced -- once per drivetrain in `ftc/drivetrain.py`'s "
        "`DRIVETRAIN_ORDER` (tank, mecanum), for all 7 headline suites, and checks whether the best-value "
        "suite (and the overall success-rate ranking) changes on mecanum. Cost per suite includes that "
        "drivetrain's own premium (`MECANUM_WHEEL_COST_USD`/`TANK_WHEEL_COST_USD`, `ftc/config.py`) on top "
        "of the sensor cost, since a team buying a suite also has to buy wheels. Raw data (with an added "
        "`drivetrain` column) in `ftc_drivetrain_suite_results.csv`, chart in "
        "`ftc_drivetrain_suite_comparison.png`.",
        "",
        "This is a DIFFERENT, separately-scoped comparison from `ftc_drivetrain_writeup.md` (`ftc/"
        "drivetrain_benchmark.py`), which crosses the headline suites against {tank, mecanum} x "
        "{optimistic, realistic} fidelity at reduced rigor to isolate one specific mechanism (does holding "
        "a fixed heading toward a tag wall keep AprilTag's camera aimed at tags long enough to pay for "
        "mecanum's premium). That study's own numbers are frozen and untouched by this one -- this module "
        "asks the broader question at full statistical rigor: for EVERY headline suite, not just AprilTag, "
        "does success rate -- and the best-value recommendation -- change under mecanum?",
        "",
        "## Success rate by suite x drivetrain",
        "",
        "| Suite | Tank | Mecanum | Difference |",
        "|---|---:|---:|---:|",
    ]
    tank_stats = stats_by_drivetrain["tank"]
    mecanum_stats = stats_by_drivetrain["mecanum"]
    for suite in SUITE_ORDER:
        tank_rate = overall_success_rate(tank_stats, suite)
        mecanum_rate = overall_success_rate(mecanum_stats, suite)
        lines.append(f"| {SUITE_LABELS[suite]} | {tank_rate:.0%} | {mecanum_rate:.0%} | "
                      f"{(mecanum_rate - tank_rate):+.0%} |")

    lines += ["", "## Per-drivetrain value ranking", ""]
    for drivetrain_name in DRIVETRAIN_ORDER:
        results, baseline_rate, best = per_drivetrain[drivetrain_name]
        stats = stats_by_drivetrain[drivetrain_name]
        lines += [
            f"### {DRIVETRAIN_DISPLAY_LABELS[drivetrain_name]}",
            "",
            f"DeadReckoningSuite baseline (variance_level >= {SUMMARY_MIN_LEVEL}): {baseline_rate:.0%}",
            "",
            "| Suite | Cost (incl. drivetrain) | Overall success rate | Value (pp/$100) |",
            "|---|---:|---:|---:|",
        ]
        overall = {s: overall_success_rate(stats, s) for s in SUITE_ORDER}
        for suite in sorted(SUITE_ORDER, key=lambda s: -overall[s]):
            cost = SUITES[suite].cost_usd + DRIVETRAINS[drivetrain_name].cost_usd
            per_100_str = "n/a (free)" if suite == "dead_reckoning" else f"{results[suite]['per_100']:+.1f}"
            lines.append(f"| {SUITE_LABELS[suite]} | ${usd(cost)} | {overall[suite]:.0%} | {per_100_str} |")
        lines += ["", f"Best value under {DRIVETRAIN_DISPLAY_LABELS[drivetrain_name]}: {SUITE_LABELS[best]}.", ""]

    winners = {d: per_drivetrain[d][2] for d in DRIVETRAIN_ORDER}
    unanimous = len(set(winners.values())) == 1
    lines += ["## Does the conclusion hold?", ""]
    if unanimous:
        winner = winners[DRIVETRAIN_ORDER[0]]
        lines.append(
            f"{SUITE_LABELS[winner]} is the best-value suite under both tank and mecanum. The headline "
            "recommendation is not tank-specific -- teams running mecanum, a large fraction of the FTC "
            "population, should reach the same sensing decision."
        )
    else:
        lines.append(
            "The best-value suite is drivetrain-dependent. " + "; ".join(
                f"{DRIVETRAIN_DISPLAY_LABELS[d]} -> {SUITE_LABELS[w]}" for d, w in winners.items()
            ) + ". This is a real finding, not a failure of the sweep: mecanum's strafe speed/drift "
            "penalty (`MECANUM_STRAFE_SPEED_FACTOR`/`MECANUM_STRAFE_DRIFT_MULTIPLIER`, `ftc/config.py`) "
            "changes which sensing investment pays off, not just how well any one of them does. Treat "
            "`ftc_suite_writeup.md`'s single-drivetrain recommendation as conditional on tank (or an "
            "unstated drivetrain, which is the same thing), not universal."
        )
        tank_winner = winners["tank"]
        for drivetrain_name in DRIVETRAIN_ORDER:
            if drivetrain_name == "tank" or winners[drivetrain_name] == tank_winner:
                continue
            results, _, best = per_drivetrain[drivetrain_name]
            tank_result = per_drivetrain["tank"][0].get(tank_winner)
            this_result = results[best]
            if tank_result is None:
                continue
            overlap = not (this_result["ci_lo"] > tank_result["ci_hi"]
                            or tank_result["ci_lo"] > this_result["ci_hi"])
            if overlap:
                lines.append(
                    f"\nNote: under {DRIVETRAIN_DISPLAY_LABELS[drivetrain_name]}, {SUITE_LABELS[best]}'s "
                    f"success-rate CI still overlaps {SUITE_LABELS[tank_winner]}'s at this trial count -- "
                    "that particular flip could plausibly be sampling noise rather than a genuine "
                    "drivetrain effect."
                )

    lines += ["", "## Consistency check", "",
              "The 'tank' pass in this module uses the exact same trial_seed formula as "
              "`ftc/suite_benchmark.py`'s own `__main__` (tank and 'no drivetrain' are byte-for-byte the "
              "same model, `ftc/drivetrain.py`'s own module docstring), so it reruns the identical "
              "scenarios. Overall success rate here: " + ", ".join(
                  f"{SUITE_LABELS[s]} {overall_success_rate(stats_by_drivetrain['tank'], s):.0%}"
                  for s in SUITE_ORDER
              ) + " -- compare against `ftc_suite_writeup.md`'s table; any mismatch would mean this "
              "module accidentally changed what the tank pass measures rather than just adding a mecanum "
              "one."]

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    all_rows = []
    per_drivetrain = {}
    stats_by_drivetrain = {}
    for drivetrain_name in DRIVETRAIN_ORDER:
        print(f"drivetrain={drivetrain_name} ...")
        rows = run_drivetrain(drivetrain_name)
        all_rows.extend(rows)
        stats_by_drivetrain[drivetrain_name] = aggregate(rows)
        per_drivetrain[drivetrain_name] = value_ranking(rows, drivetrain_name)

    write_csv(all_rows, OUTPUT_DIR / "ftc_drivetrain_suite_results.csv")
    plot_drivetrain_comparison(per_drivetrain, OUTPUT_DIR / "ftc_drivetrain_suite_comparison.png")
    write_writeup(per_drivetrain, stats_by_drivetrain, OUTPUT_DIR / "ftc_drivetrain_suite_writeup.md")

    print(f"\nWrote {len(all_rows)} trials to {OUTPUT_DIR / 'ftc_drivetrain_suite_results.csv'}")
    print(f"Chart saved to {OUTPUT_DIR / 'ftc_drivetrain_suite_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'ftc_drivetrain_suite_writeup.md'}\n")
    for drivetrain_name in DRIVETRAIN_ORDER:
        _, baseline_rate, best = per_drivetrain[drivetrain_name]
        print(f"{drivetrain_name}: best value = {best} (baseline={baseline_rate:.0%})")
