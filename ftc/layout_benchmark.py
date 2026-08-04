"""
Does the headline finding (AprilTag = best value by success-rate-gained-
per-dollar) hold on a field layout other than the one it was measured
on? ftc/suite_benchmark.py hardcodes LAYOUT = "cluttered" -- the richest
test of both obstacle-sensing suites and the hard-footprint-inflation
routing this project's grids add over nav/'s point-robot ones (see
ftc/field.py's module docstring), but "richest test" is exactly the
kind of layout choice that could be doing unacknowledged work in the
result. This reruns the *same* full-rigor sweep (all 11 variance_level
steps, all 3 deviation types, TRIALS_PER_COMBO trials/point -- nothing
reduced, unlike ftc/robustness.py's tipping-point search, because this
question needs the same statistical footing as the headline number
itself, not just a "did it flip") on each of ftc/field.py's three
layouts and compares the best-value suite across them.

Reuses ftc/suite_benchmark.py's run_combo/aggregate/overall_success_rate
wholesale rather than reimplementing the sweep -- the "cluttered" pass
this module runs uses the exact same trial_seed formula as
ftc/suite_benchmark.py's own __main__, so its rows are trial-for-trial
identical to ftc_suite_results.csv (bar the unseeded wall-clock
planning_time_ms field) as a built-in consistency check that this
module didn't quietly change what "cluttered" means.

Leaves ftc/suite_benchmark.py itself, and everything it writes
(ftc_suite_results.csv, ftc_suite_comparison.png,
ftc_reliability_per_dollar.png, ftc_suite_writeup.md), completely
untouched -- this module only *adds* files, so the headline single-
layout numbers this README and the conference poster cite cannot be
disturbed by running it.

Writes benchmark_results/ftc_layout_results.csv (every trial, raw, with
an added `layout` column), benchmark_results/ftc_layout_comparison.png
(one reliability-per-dollar panel per layout, same chart type as
ftc_reliability_per_dollar.png), and benchmark_results/
ftc_layout_writeup.md (per-layout best-value suite, and whether the
"AprilTag wins" conclusion is layout-dependent -- if it is, that's a
finding, not a failure, and is reported as such).
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt

from nav.stats import bootstrap_ci

from ftc.field import LAYOUTS, build_grid, tag_sites_for
from ftc.sensors import SUITES, SUITE_ORDER, SUITE_LABELS
from ftc.suite_benchmark import (
    DEVIATION_TYPE_ORDER, SUITE_COLORS, SUMMARY_MIN_LEVEL, TRIALS_PER_COMBO, VARIANCE_LEVELS,
    aggregate, overall_success_rate, run_combo,
)

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

# Field.py's own definition order (sparse -> cluttered -> corridor,
# least to most constrained routing) -- "cluttered" is listed second
# here deliberately, matching ftc/suite_benchmark.py's choice of it as
# the headline layout, so every "layout 2 of 3" reference in the
# writeup lines up with the one number readers already have.
LAYOUT_ORDER = list(LAYOUTS)
LAYOUT_LABELS = {
    "sparse": "Sparse (near-open field)",
    "cluttered": "Cluttered (headline layout)",
    "corridor": "Corridor (single narrow gap)",
}


def run_layout(layout_name):
    """Full-rigor sweep on one layout, using ftc/suite_benchmark.py's own
    seed formula unmodified -- so a "cluttered" pass through this
    function reproduces ftc_suite_results.csv's rows trial-for-trial
    (see module docstring)."""
    grid = build_grid(layout_name)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(layout_name)

    rows = []
    for deviation_type in DEVIATION_TYPE_ORDER:
        for level in VARIANCE_LEVELS:
            base_seed = 6_000_000 + DEVIATION_TYPE_ORDER.index(deviation_type) * 1_000_000 + round(level * 100)
            rows.extend(run_combo(deviation_type, level, TRIALS_PER_COMBO, base_seed, grid, free_cells, tag_sites))
    for row in rows:
        row["layout"] = layout_name
    return rows


def _bootstrap_seed(layout, suite):
    return 7_000_000 + LAYOUT_ORDER.index(layout) * 100_000 + SUITE_ORDER.index(suite)


def value_ranking(rows, layout):
    """{suite: {rate, per_100, ci_lo, ci_hi}} for every non-baseline
    suite on this layout, plus the best-value suite name. CI is computed
    directly on the pooled successes/n over the SUMMARY_MIN_LEVEL window
    -- equivalent to overall_success_rate's mean-of-per-cell-rates since
    every cell has the same trial count, but bootstrap_ci needs raw
    counts rather than an already-averaged rate."""
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
        cost = SUITES[suite].cost_usd
        per_100 = (rate - baseline_rate) / (cost / 100) * 100 if cost > 0 else float("inf")
        ci_lo, ci_hi = bootstrap_ci(successes, n, seed=_bootstrap_seed(layout, suite))
        results[suite] = {"rate": rate, "per_100": per_100, "ci_lo": ci_lo, "ci_hi": ci_hi}
    best = max(results, key=lambda s: results[s]["per_100"])
    return results, baseline_rate, best


def write_csv(all_rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
        writer.writeheader()
        writer.writerows(all_rows)


def plot_layout_comparison(per_layout, path):
    fig, axes = plt.subplots(1, len(LAYOUT_ORDER), figsize=(15, 5), sharey=True)
    for ax, layout in zip(axes, LAYOUT_ORDER):
        results, baseline_rate, best = per_layout[layout]
        suites = [s for s in SUITE_ORDER if s != "dead_reckoning"]
        colors = [SUITE_COLORS[s] for s in suites]
        heights = [results[s]["per_100"] for s in suites]
        bars = ax.bar([SUITE_LABELS[s] for s in suites], heights, color=colors)
        for bar, suite in zip(bars, suites):
            marker = " *best*" if suite == best else ""
            ax.annotate(f"${SUITES[suite].cost_usd:.0f}{marker}", (bar.get_x() + bar.get_width() / 2,
                        bar.get_height()), ha="center", va="bottom" if bar.get_height() >= 0 else "top",
                        fontsize=8, rotation=0)
        ax.axhline(0, color="black", linewidth=0.8)
        ax.set_title(f"{LAYOUT_LABELS[layout]}\nbaseline={baseline_rate:.0%}", fontsize=10)
        ax.tick_params(axis="x", labelrotation=30, labelsize=8)
    axes[0].set_ylabel("Success-rate gain over DeadReckoningSuite\nper $100 spent (pp/$100)")
    fig.suptitle(f"Reliability-per-dollar by field layout (variance_level >= {SUMMARY_MIN_LEVEL}, "
                  f"{TRIALS_PER_COMBO} trials/point, all deviation types)", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(path, dpi=150)


def write_writeup(per_layout, stats_by_layout, path):
    lines = [
        "# Does the headline finding hold across field layouts?",
        "",
        "`ftc_suite_writeup.md`'s headline sweep runs on `ftc/field.py`'s `'cluttered'` layout "
        "only. This reruns the identical full-rigor sweep -- same "
        f"{len(VARIANCE_LEVELS)} variance_level steps, same 3 deviation types, same "
        f"{TRIALS_PER_COMBO} trials/point, nothing reduced -- on all three layouts `ftc/field.py` "
        "ships, and checks whether the best-value suite (and the overall success-rate ranking) "
        "changes. Raw data (with an added `layout` column) in `ftc_layout_results.csv`, chart in "
        "`ftc_layout_comparison.png`.",
        "",
        "## Per-layout results",
        "",
    ]
    for layout in LAYOUT_ORDER:
        results, baseline_rate, best = per_layout[layout]
        stats = stats_by_layout[layout]
        lines += [
            f"### {LAYOUT_LABELS[layout]}",
            "",
            f"DeadReckoningSuite baseline (variance_level >= {SUMMARY_MIN_LEVEL}): {baseline_rate:.0%}",
            "",
            "| Suite | Cost | Overall success rate | Value (pp/$100) |",
            "|---|---:|---:|---:|",
        ]
        overall = {s: overall_success_rate(stats, s) for s in SUITE_ORDER}
        for suite in sorted(SUITE_ORDER, key=lambda s: -overall[s]):
            cost = SUITES[suite].cost_usd
            per_100_str = "n/a (free)" if suite == "dead_reckoning" else f"{results[suite]['per_100']:+.1f}"
            lines.append(f"| {SUITE_LABELS[suite]} | ${cost:.0f} | {overall[suite]:.0%} | {per_100_str} |")
        lines += ["", f"Best value on this layout: {SUITE_LABELS[best]}.", ""]

    winners = {layout: per_layout[layout][2] for layout in LAYOUT_ORDER}
    unanimous = len(set(winners.values())) == 1
    lines += ["## Does the conclusion hold?", ""]
    if unanimous:
        winner = winners[LAYOUT_ORDER[0]]
        lines.append(
            f"{SUITE_LABELS[winner]} is the best-value suite on all three layouts -- sparse, "
            "cluttered, and corridor. The headline recommendation is not an artifact of testing on "
            "the one layout with the most obstacles to sense; it holds on a near-open field and a "
            "single-forced-corridor field too."
        )
    else:
        lines.append(
            "The best-value suite is layout-dependent. " + "; ".join(
                f"{LAYOUT_LABELS[layout]} -> {SUITE_LABELS[w]}" for layout, w in winners.items()
            ) + ". This is a real finding, not a failure of the sweep: which sensing investment "
            "pays off depends on the field, not just the deviation type. Treat "
            "`ftc_suite_writeup.md`'s single-layout recommendation as conditional on a "
            "cluttered-style field, not universal."
        )
        # CI-overlap check on the cluttered-layout winner vs. its
        # per-layout runner-up wherever the winner changes, so a flip
        # driven by sampling noise (unlikely at 25 trials/point over the
        # full VARIANCE_LEVELS range, but checked rather than assumed)
        # is called out plainly instead of stated as fact.
        cluttered_winner = winners["cluttered"]
        for layout in LAYOUT_ORDER:
            if layout == "cluttered" or winners[layout] == cluttered_winner:
                continue
            results, _, best = per_layout[layout]
            cluttered_result = per_layout["cluttered"][0].get(cluttered_winner)
            this_result = results[best]
            if cluttered_result is None:
                continue
            overlap = not (this_result["ci_lo"] > cluttered_result["ci_hi"]
                            or cluttered_result["ci_lo"] > this_result["ci_hi"])
            if overlap:
                lines.append(
                    f"\nNote: on {LAYOUT_LABELS[layout]}, {SUITE_LABELS[best]}'s success-rate CI still "
                    f"overlaps {SUITE_LABELS[cluttered_winner]}'s at this trial count -- that particular "
                    "flip could plausibly be sampling noise rather than a genuine layout effect."
                )

    lines += ["", "## Consistency check", "",
              "The 'cluttered' pass in this module uses the exact same trial_seed formula as "
              "`ftc/suite_benchmark.py`'s own `__main__`, so it reruns the identical scenarios. "
              f"Overall success rate here: " + ", ".join(
                  f"{SUITE_LABELS[s]} {overall_success_rate(stats_by_layout['cluttered'], s):.0%}"
                  for s in SUITE_ORDER
              ) + " -- compare against `ftc_suite_writeup.md`'s table; any mismatch would mean this "
              "module accidentally changed what 'cluttered' means rather than just adding two more "
              "layouts."]

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    all_rows = []
    per_layout = {}
    stats_by_layout = {}
    for layout in LAYOUT_ORDER:
        print(f"layout={layout} ...")
        rows = run_layout(layout)
        all_rows.extend(rows)
        stats_by_layout[layout] = aggregate(rows)
        per_layout[layout] = value_ranking(rows, layout)

    write_csv(all_rows, OUTPUT_DIR / "ftc_layout_results.csv")
    plot_layout_comparison(per_layout, OUTPUT_DIR / "ftc_layout_comparison.png")
    write_writeup(per_layout, stats_by_layout, OUTPUT_DIR / "ftc_layout_writeup.md")

    print(f"\nWrote {len(all_rows)} trials to {OUTPUT_DIR / 'ftc_layout_results.csv'}")
    print(f"Chart saved to {OUTPUT_DIR / 'ftc_layout_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'ftc_layout_writeup.md'}\n")
    for layout in LAYOUT_ORDER:
        _, baseline_rate, best = per_layout[layout]
        print(f"{layout}: best value = {best} (baseline={baseline_rate:.0%})")
