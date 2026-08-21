"""
The headline sweep (ftc/suite_benchmark.py) at all three MODEL_FIDELITY
tiers (ftc/config.py) side by side -- the deliverable this whole
fidelity-tier addition exists to produce: a single place showing the
range from "optimistic" (the pre-Priority-1 assumptions, and the tier
every previously-published number in this repo corresponds to) through
"realistic" (real camera FOV gating + heading drift) to "pessimistic"
(narrower FOV, more heading drift, detection dropout).

Reruns the IDENTICAL full-rigor sweep (all 11 variance_level steps, all
3 deviation types, TRIALS_PER_COMBO trials/point, same 'cluttered'
layout, same seed formula) ftc/suite_benchmark.py's own __main__ uses,
once per tier, via ftc.suite_benchmark.run_combo's `fidelity` parameter
-- nothing reduced, and nothing about ftc/suite_benchmark.py or its own
outputs is touched (the "optimistic" pass reproduces ftc_suite_
results.csv trial-for-trial, verified in ftc/scratch/fidelity_test.py,
and this module's own optimistic-tier numbers below are that exact
reproduction, not a separate estimate).

Writes benchmark_results/ftc_fidelity_results.csv (every trial, raw,
with an added `fidelity` column), benchmark_results/
ftc_fidelity_comparison.png (success rate vs. deviation, one row per
deviation type, one line per tier, averaged across suites so the
figure stays readable -- see plot docstring), and benchmark_results/
ftc_fidelity_writeup.md (the headline table at all three tiers, and
which suite is the best value at each one).
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt

from ftc.config import usd
from ftc.field import build_grid, tag_sites_for
from ftc.sensors import SUITES, SUITE_ORDER, SUITE_LABELS
from ftc.suite_benchmark import (
    DEVIATION_TYPE_LABELS, DEVIATION_TYPE_ORDER, LAYOUT, SUITE_COLORS, SUMMARY_MIN_LEVEL,
    TRIALS_PER_COMBO, VARIANCE_LEVELS, aggregate, overall_success_rate, run_sweep,
)

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

FIDELITY_ORDER = ["optimistic", "realistic", "pessimistic"]
FIDELITY_LABELS = {
    "optimistic": "Optimistic (pre-Priority-1 assumptions -- the published headline numbers)",
    "realistic": "Realistic (real camera FOV + heading drift)",
    "pessimistic": "Pessimistic (narrower FOV, more drift, detection dropout)",
}
FIDELITY_LINESTYLES = {"optimistic": "-", "realistic": "--", "pessimistic": ":"}


def run_tier(fidelity):
    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)

    rows = run_sweep(DEVIATION_TYPE_ORDER, VARIANCE_LEVELS, TRIALS_PER_COMBO, grid, free_cells, tag_sites,
                      fidelity=fidelity)
    for row in rows:
        row["fidelity"] = fidelity
    return rows


def write_csv(all_rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
        writer.writeheader()
        writer.writerows(all_rows)


def plot_fidelity_comparison(stats_by_tier, path):
    """One panel per deviation type, one line per tier -- AVERAGED across
    every suite (unlike ftc_suite_comparison.png's per-suite lines) so
    three tiers x seven suites x three deviation types doesn't collapse
    into an unreadable 15-line-per-panel chart. The point of this
    figure is "how much does the tier itself move the curve," not
    re-litigating which suite wins (ftc_suite_writeup.md's per-tier
    table below is where that's answered precisely)."""
    fig, axes = plt.subplots(len(DEVIATION_TYPE_ORDER), 1, figsize=(8, 12), sharex=True)
    for ax, deviation_type in zip(axes, DEVIATION_TYPE_ORDER):
        for fidelity in FIDELITY_ORDER:
            stats = stats_by_tier[fidelity]
            avg_success = [
                sum(stats[(s, deviation_type, level)]["success_rate"] for s in SUITE_ORDER) / len(SUITE_ORDER)
                for level in VARIANCE_LEVELS
            ]
            ax.plot(VARIANCE_LEVELS, avg_success, FIDELITY_LINESTYLES[fidelity], marker="o", markersize=3,
                     label=fidelity, color="tab:blue" if fidelity == "optimistic" else
                     ("tab:orange" if fidelity == "realistic" else "tab:red"))
        ax.set_ylabel("Mean success rate\n(averaged across all 7 suites)")
        ax.set_ylim(-0.05, 1.05)
        ax.set_title(DEVIATION_TYPE_LABELS[deviation_type])
    axes[0].legend(loc="lower left", fontsize=8)
    axes[-1].set_xlabel("variance_level")
    fig.suptitle(f"Model fidelity tier comparison ({TRIALS_PER_COMBO} trials/point, '{LAYOUT}' layout)",
                  fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(path, dpi=150)


def _value_ranking(stats):
    """per_100 is None (NOT infinity) at cost_usd == 0.0 -- IMU is a
    second $0 headline suite besides the dead-reckoning baseline itself
    now; `best` is picked only among suites with a defined per_100, same
    convention as ftc/suite_benchmark.py's own write_writeup."""
    baseline = overall_success_rate(stats, "dead_reckoning")
    results = {}
    for suite in SUITE_ORDER:
        if suite == "dead_reckoning":
            continue
        rate = overall_success_rate(stats, suite)
        cost = SUITES[suite].cost_usd
        per_100 = None if cost == 0 else (rate - baseline) / (cost / 100) * 100
        results[suite] = {"rate": rate, "per_100": per_100}
    priced = {s: v for s, v in results.items() if v["per_100"] is not None}
    best = max(priced, key=lambda s: priced[s]["per_100"])
    return results, baseline, best


def write_writeup(stats_by_tier, path):
    lines = [
        "# Headline results at all three model fidelity tiers",
        "",
        "ftc/config.py's MODEL_FIDELITY picks which tier's camera-FOV-gating, heading-drift, and "
        "AprilTag-heading-correction/dropout assumptions ftc/sensors.py and ftc/match.py use. This "
        "reruns ftc/suite_benchmark.py's exact full-rigor headline sweep (same layout, same seed "
        f"formula, same {TRIALS_PER_COMBO} trials/point, all {len(VARIANCE_LEVELS)} variance_level "
        "steps, all 3 deviation types) once per tier. The 'optimistic' row below is byte-for-byte the "
        "same sweep `ftc_suite_writeup.md` already reports -- see ftc/scratch/fidelity_test.py's "
        "regression check -- included here for direct comparison, not as a separate estimate. Raw data "
        "(with an added `fidelity` column) in `ftc_fidelity_results.csv`, chart in "
        "`ftc_fidelity_comparison.png`.",
        "",
        "## Headline table, all three tiers",
        "",
        f"Overall success rate, variance_level >= {SUMMARY_MIN_LEVEL} across all three deviation types:",
        "",
        "| Suite | Cost | Optimistic | Realistic | Pessimistic |",
        "|---|---:|---:|---:|---:|",
    ]
    overall_by_tier = {
        fidelity: {s: overall_success_rate(stats_by_tier[fidelity], s) for s in SUITE_ORDER}
        for fidelity in FIDELITY_ORDER
    }
    for suite in sorted(SUITE_ORDER, key=lambda s: -overall_by_tier["optimistic"][s]):
        lines.append(
            f"| {SUITE_LABELS[suite]} | ${usd(SUITES[suite].cost_usd)} | "
            f"{overall_by_tier['optimistic'][suite]:.0%} | {overall_by_tier['realistic'][suite]:.0%} | "
            f"{overall_by_tier['pessimistic'][suite]:.0%} |"
        )

    lines += ["", "## Best-value suite at each tier (success-rate gain over dead reckoning, per $100)", "",
              "| Tier | Best value | pp/$100 | Full suite's pp/$100 |", "|---|---|---:|---:|"]
    best_by_tier = {}
    for fidelity in FIDELITY_ORDER:
        results, baseline, best = _value_ranking(stats_by_tier[fidelity])
        best_by_tier[fidelity] = best
        lines.append(f"| {fidelity} | {SUITE_LABELS[best]} | {results[best]['per_100']:+.1f} | "
                      f"{results['full_suite']['per_100']:+.1f} |")

    unanimous = len(set(best_by_tier.values())) == 1
    lines += ["", "## Does the best-value recommendation survive tightening the model?", ""]
    if unanimous:
        lines.append(
            f"{SUITE_LABELS[best_by_tier['optimistic']]} is the best-value suite at every fidelity tier -- "
            "camera-FOV gating and heading drift change the raw numbers (see the table above) but not "
            "which suite is the best buy. The published `ftc_suite_writeup.md` recommendation is stated "
            "at the optimistic tier; this confirms it isn't an artifact of that tier's specific "
            "assumptions."
        )
    else:
        lines.append(
            "The best-value suite changes across tiers: " + "; ".join(
                f"{fidelity} -> {SUITE_LABELS[best_by_tier[fidelity]]}" for fidelity in FIDELITY_ORDER
            ) + ". This is a real finding, not a failure of the sweep -- it means the published "
            "optimistic-tier recommendation is conditional on the optimistic tier's assumptions "
            "(omnidirectional camera, perfect heading knowledge), not universal. Fidelity tiers BOUND "
            "the camera-FOV/heading-error gap in the model (README.md's \"Threats to validity\"); they do "
            "not CALIBRATE it -- only ftc/calibration.py run against real measured data does that."
        )

    lines += ["", "## What this does and does not prove", "",
              "This shows the RANGE the published headline numbers sit in as the camera-FOV and "
              "heading-error assumptions tighten -- it does not tell you which tier is closer to any "
              "particular real robot/field. All three tiers' non-optimistic constants (ftc/config.py's "
              "CAMERA_FOV_DEG_BY_TIER, HEADING_DRIFT_DEG_PER_CELL_BY_TIER, etc.) are documented ballpark "
              "engineering estimates, the same status as every other estimated constant in this project "
              "-- see README.md's \"Threats to validity\" and ftc/robustness.py for how sensitive the "
              "optimistic-tier recommendation is to ITS OWN estimated constants being wrong."]

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    all_rows = []
    stats_by_tier = {}
    for fidelity in FIDELITY_ORDER:
        print(f"fidelity={fidelity} ...")
        rows = run_tier(fidelity)
        all_rows.extend(rows)
        stats_by_tier[fidelity] = aggregate(rows)

    write_csv(all_rows, OUTPUT_DIR / "ftc_fidelity_results.csv")
    plot_fidelity_comparison(stats_by_tier, OUTPUT_DIR / "ftc_fidelity_comparison.png")
    write_writeup(stats_by_tier, OUTPUT_DIR / "ftc_fidelity_writeup.md")

    print(f"\nWrote {len(all_rows)} trials to {OUTPUT_DIR / 'ftc_fidelity_results.csv'}")
    print(f"Chart saved to {OUTPUT_DIR / 'ftc_fidelity_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'ftc_fidelity_writeup.md'}\n")
    for fidelity in FIDELITY_ORDER:
        _, baseline, best = _value_ranking(stats_by_tier[fidelity])
        print(f"{fidelity}: best value = {best} (dead_reckoning baseline={baseline:.0%})")
