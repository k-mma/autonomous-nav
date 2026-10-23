"""Regenerates severity_curves.png: the one thing none of the other poster figures show --
DEGRADATION. Every other chart is a single-point summary; this one
shows success rate collapsing as the field deviates further from the
map, and that the three deviation types collapse at very different
rates.

A deliberate simplification of benchmark_results/ftc_suite_comparison.png
(this project's own version), which plots all 7 suites with 95% CI bands
in 3 stacked panels -- correct, but 7 overlapping translucent bands is
unreadable at poster size. Here: 3 suites, one per row of the sensor
story (free baseline / cheapest useful buy / best single buy), no CI
bands, panels side by side.

Recomputed from benchmark_results/ftc_suite_results.csv -- no new
simulation run.

Regenerate after re-running ftc/suite_benchmark.py (i.e. whenever
benchmark_results/ftc_suite_results.csv changes):

    python3 "screenshots/poster figures/generate_severity_curves.py"
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter

OUT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(OUT_DIR))
from poster_common import REPO_ROOT  # noqa: E402

RESULTS_CSV = REPO_ROOT / "benchmark_results" / "ftc_suite_results.csv"
OUT = OUT_DIR / "severity_curves.png"

DEVIATION_ORDER = ["start_drift", "obstacle_drift", "unplanned_blocker"]
DEVIATION_LABELS = {
    "start_drift": "Start drift\n(pose error)",
    "obstacle_drift": "Obstacle drift\n(map error)",
    "unplanned_blocker": "Unplanned blocker\n(opponent robot)",
}
# Three suites, not seven: the free baseline, the best value-per-dollar
# buy, and the best raw performer. Same colors ftc/suite_benchmark.py's
# SUITE_COLORS uses for these three.
SUITES = [
    ("dead_reckoning", "Dead reckoning ($0)", "#d62728"),
    ("apriltag", "AprilTag ($25)", "#9467bd"),
    ("odometry_pods", "Odometry pods ($195)", "#ff7f0e"),
]


def load():
    rows = list(csv.DictReader(open(RESULTS_CSV)))
    agg = defaultdict(lambda: [0, 0])
    levels = set()
    for r in rows:
        lvl = float(r["variance_level"])
        levels.add(lvl)
        key = (r["suite"], r["deviation_type"], lvl)
        agg[key][0] += int(r["success"] == "True")
        agg[key][1] += 1
    return agg, sorted(levels)


def main():
    agg, levels = load()

    fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.9), sharey=True)
    for ax, dev in zip(axes, DEVIATION_ORDER):
        for suite, label, color in SUITES:
            ys = []
            for lvl in levels:
                hits, n = agg[(suite, dev, lvl)]
                ys.append(hits / n if n else 0.0)
            ax.plot(levels, ys, "o-", color=color, linewidth=2.4, markersize=5,
                    label=label, zorder=3)
        ax.set_title(DEVIATION_LABELS[dev], fontsize=13, fontweight="bold", pad=10)
        ax.set_xlabel("Deviation severity", fontsize=11)
        ax.set_ylim(-0.03, 1.0)
        ax.set_xlim(-0.03, 1.03)
        ax.grid(alpha=0.25, zorder=0)
        ax.spines[["top", "right"]].set_visible(False)

    axes[0].set_ylabel("Success rate", fontsize=11.5)
    axes[0].yaxis.set_major_formatter(PercentFormatter(xmax=1.0))
    axes[0].legend(loc="upper right", fontsize=9.5, framealpha=0.95)

    fig.suptitle("Every sensor degrades — but not at the same rate, and not on the same failure",
                  fontsize=15.5, fontweight="bold", x=0.008, ha="left", y=0.985)

    fig.tight_layout(rect=[0, 0.02, 1, 0.93])
    fig.savefig(OUT, dpi=170, facecolor="white")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
