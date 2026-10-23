"""Regenerates figure3_reliability_per_dollar.png: "of the sensors we tested, which one is the best buy?"

Same underlying metric as ftc/suite_benchmark.py's own
plot_reliability_per_dollar (success-rate gain over the free
DeadReckoningSuite baseline, per $100 spent, at variance_level >= 0.3
across all 3 deviation types), recomputed here straight from
benchmark_results/ftc_suite_results.csv rather than re-run -- this
script only changes the PRESENTATION, to fix several readability
problems the original chart (and an earlier FTC-Sensor-Study-Poster
draft's own "Figure 1") had:

  - horizontal bars, sorted best-value-first (top to bottom), instead
    of vertical bars in an arbitrary suite order -- "which is the best
    buy" is readable at a glance instead of requiring a scan.
  - every bar directly labeled with its exact value, cost, AND raw
    success rate -- no separate side table needed to answer "ok, but
    what's the actual success rate".
  - the free-to-run IMU suite (cost_usd == 0, so "per dollar" is
    undefined) gets an explicit footnote instead of silently vanishing
    from the chart with no explanation.
  - a plain-language subtitle alongside the precise variance_level >=
    0.3 threshold, so a reader unfamiliar with this project's own
    terminology still gets the gist.

Regenerate after re-running ftc/suite_benchmark.py (i.e. whenever
benchmark_results/ftc_suite_results.csv changes):

    python3 "screenshots/poster figures/generate_figure3_reliability_per_dollar.py"
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

OUT_DIR = Path(__file__).resolve().parent


def usd(cost):
    """Whole-dollar display rounding for a cost. Plain round()/f"{:.0f}"
    both use banker's rounding, which turns a real $94.50 sensor cost
    into a displayed "$94" -- not wrong, but reads as a typo next to
    the exact number. This always rounds a .50 case up, matching how
    a price tag would actually be written."""
    return int(cost + 0.5)
RESULTS_CSV = REPO_ROOT / "benchmark_results" / "ftc_suite_results.csv"
MIN_LEVEL = 0.3

SUITE_LABELS = {
    "odometry_pods": "Odometry pods",
    "distance_sensors": "Distance sensors",
    "apriltag": "AprilTag (front camera)",
    "dual_camera_apriltag": "Front + rear cameras",
    "full_suite": "Full suite",
    "imu": "IMU",
}
# Same hues as every other chart in this project (ftc/suite_benchmark.py's
# SUITE_COLORS) -- kept identical here so a reader who's seen any other
# figure in this project doesn't have to relearn a color mapping.
SUITE_COLORS = {
    "odometry_pods": "tab:orange",
    "distance_sensors": "tab:blue",
    "apriltag": "tab:purple",
    "dual_camera_apriltag": "tab:brown",
    "full_suite": "tab:green",
}


def load_stats():
    rows = list(csv.DictReader(open(RESULTS_CSV)))
    cost = {}
    for r in rows:
        cost[r["suite"]] = float(r["cost_usd"])

    def rate(suite):
        vals = [r["success"] == "True" for r in rows
                 if r["suite"] == suite and float(r["variance_level"]) >= MIN_LEVEL]
        return sum(vals) / len(vals)

    baseline = rate("dead_reckoning")
    stats = {}
    for suite in SUITE_LABELS:
        r = rate(suite)
        c = cost[suite]
        stats[suite] = {
            "cost": c,
            "success_rate": r,
            "gain_pts": (r - baseline) * 100,
            "per_100": (r - baseline) / (c / 100) * 100 if c > 0 else None,
        }
    return baseline, stats


def main():
    baseline, stats = load_stats()

    # IMU is free (cost_usd == 0) -- "value per dollar" is undefined
    # there, so it's reported in a footnote instead of on the chart,
    # same exclusion ftc/suite_benchmark.py's own plot already makes,
    # just made explicit here instead of silent.
    chartable = [s for s in stats if stats[s]["per_100"] is not None]
    chartable.sort(key=lambda s: stats[s]["per_100"], reverse=True)

    fig, ax = plt.subplots(figsize=(10, 5.5))
    ys = list(range(len(chartable)))[::-1]
    colors = [SUITE_COLORS[s] for s in chartable]
    values = [stats[s]["per_100"] for s in chartable]
    bars = ax.barh(ys, values, color=colors, height=0.62, zorder=3)

    xmax = max(values) * 1.32
    xmin = min(0, min(values) * 1.55)
    ax.set_xlim(xmin, xmax)

    for y, s, bar in zip(ys, chartable, bars):
        st = stats[s]
        v = st["per_100"]
        # Positive bars: label just past the bar's own tip. Negative
        # bars: label just to the RIGHT of the zero line instead of
        # further left past the bar's tip -- that space is otherwise
        # empty for that row, whereas further left runs straight into
        # the y-axis category labels.
        if v >= 0:
            label_x, ha = v + (xmax - xmin) * 0.012, "left"
        else:
            label_x, ha = (xmax - xmin) * 0.012, "left"
        ax.text(label_x, y, f"{v:+.1f} pts / $100", va="center", ha=ha,
                 fontsize=12.5, fontweight="bold", zorder=4)

    ax.set_yticks(ys)
    ax.set_yticklabels(
        [f"{SUITE_LABELS[s]}\n${usd(stats[s]['cost'])} · {stats[s]['success_rate']:.0%} success"
          for s in chartable],
        fontsize=11.5,
    )
    ax.axvline(0, color="black", linewidth=1.1, zorder=2)
    ax.set_xlabel("Success-rate gain per $100 spent (pts)", fontsize=11.5)
    ax.set_title(
        "Reliability per dollar: which sensor is the best buy?\n"
        f"Variance ≥ {MIN_LEVEL} · dead-reckoning baseline: {baseline:.0%}",
        fontsize=13.5, fontweight="bold", pad=14, loc="left",
    )
    ax.grid(axis="x", color="#e3e3e3", zorder=0)
    ax.spines[["top", "right", "left"]].set_visible(False)
    ax.tick_params(left=False)

    fig.text(0.02, -0.02,
              f"IMU excluded: free add-on, {stats['imu']['gain_pts']:+.1f} pts gain — $/100 is undefined at $0.",
              fontsize=9.5, color="#444444", ha="left")

    fig.tight_layout()
    out_path = OUT_DIR / "figure3_reliability_per_dollar.png"
    fig.savefig(out_path, dpi=170, facecolor="white", bbox_inches="tight")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
