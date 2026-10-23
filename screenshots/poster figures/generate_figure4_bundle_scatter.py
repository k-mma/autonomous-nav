"""Regenerates figure4_bundle_scatter.png: a single, decluttered scatter of cost vs. weighted success rate for
every buildable robot (individual sensors AND bundles), recomputed
from benchmark_results/ftc_optimizer_results.csv (ftc/optimizer.py's
bundle search -- 23 distinct robots, 5 scenario profiles x 25 trials
each). No new simulation is run; this only re-presents that CSV.

Readability fixes vs. ftc_optimizer_frontier.png's left panel (this
project's other version of this chart) and the FTC-Sensor-Study-Poster
draft's own "Figure 3":

  - DEDUPLICATED points. Every bundle's "+IMU" twin (IMU is a free,
    $0 no-op in this catalog -- adding it changes neither cost nor
    measured success) sits exactly on top of its non-IMU twin. Plotting
    both is silent overplotting; this draws ONE marker per unique
    (cost, success) combination and says so in a footnote.
  - That dedup drops 23 candidates to 12 visually distinct points --
    few enough to label ALL of them directly, not just the 4-point
    Pareto frontier, so individual sensors are identified by name
    alongside the bundles.
  - Individual-reachable points (square) vs. bundle-only points
    (circle) are shaped differently, not just colored differently, so
    the distinction survives grayscale printing too.

Regenerate after re-running ftc/optimizer_benchmark.py (i.e. whenever
benchmark_results/ftc_optimizer_results.csv changes):

    python3 "screenshots/poster figures/generate_figure4_bundle_scatter.py"
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

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

OUT_DIR = Path(__file__).resolve().parent
RESULTS_CSV = REPO_ROOT / "benchmark_results" / "ftc_optimizer_results.csv"


def usd(cost):
    """Whole-dollar display rounding for a cost -- see generate_figure3_
    reliability_per_dollar.py's usd() for why plain round()/f"{:.0f}"
    (banker's rounding) isn't used here."""
    return int(cost + 0.5)

PROFILE_ORDER = ["pose_drift", "map_error", "opponent", "combined_realistic", "corridor"]


def load_candidates():
    rows = list(csv.DictReader(open(RESULTS_CSV)))
    by_bundle = defaultdict(list)
    for r in rows:
        by_bundle[r["bundle"]].append(r)

    candidates = {}
    for key, rs in by_bundle.items():
        per_profile = {}
        for p in PROFILE_ORDER:
            prows = [r for r in rs if r["profile"] == p]
            per_profile[p] = sum(int(r["success"]) for r in prows) / len(prows)
        candidates[key] = dict(
            key=key, parts_label=rs[0]["parts_label"], cost=float(rs[0]["cost_usd"]),
            n_components=int(rs[0]["n_components"]),
            weighted=sum(per_profile.values()) / len(per_profile),
        )
    return candidates


def dedupe(candidates):
    groups = defaultdict(list)
    for v in candidates.values():
        groups[(round(v["cost"], 2), round(v["weighted"], 4))].append(v)

    points = []
    for (cost, rate), members in groups.items():
        members_sorted = sorted(members, key=lambda v: (v["n_components"], len(v["parts_label"])))
        rep = members_sorted[0]
        points.append(dict(
            cost=cost, rate=rate, label=rep["parts_label"], n=len(members),
            has_single=any(m["n_components"] == 1 for m in members),
        ))
    points.sort(key=lambda d: d["cost"])
    return points


def pareto_frontier(points):
    ordered = sorted(points, key=lambda d: (d["cost"], -d["rate"]))
    frontier, best = [], None
    for p in ordered:
        if best is None or p["rate"] > best:
            frontier.append(p)
            best = p["rate"]
    return frontier


# Hand-tuned label offsets (dx, dy points, ha) so none of the 12 labels
# collide -- the tradeoff for labeling every point instead of just the
# frontier is that automatic placement isn't reliable at this density,
# and this dataset is fixed/small enough that hand placement is a
# one-time cost, not a maintenance burden. Keys are the (cost, rate)
# points computed by dedupe() -- if the underlying CSV changes enough
# to shift these, points falling back to the default offset will still
# render, just not optimally spaced.
LABEL_OFFSETS = {
    (0.0, 0.136): (6, -18, "left"),
    (25.0, 0.168): (6, 14, "left"),
    (50.0, 0.168): (10, -15, "left"),
    (94.5, 0.096): (0, -18, "center"),
    (119.5, 0.128): (10, 10, "left"),
    (144.5, 0.128): (10, -14, "left"),
    (194.85, 0.256): (-10, 14, "right"),
    (219.85, 0.344): (-10, 16, "right"),
    (244.85, 0.344): (10, -6, "left"),
    (289.35, 0.128): (10, -36, "left"),
    (314.35, 0.2): (-10, 18, "right"),
    (339.35, 0.2): (10, -18, "left"),
}


def main():
    candidates = load_candidates()
    points = dedupe(candidates)
    frontier = pareto_frontier(points)
    frontier_keys = {(p["cost"], p["rate"]) for p in frontier}

    fig, ax = plt.subplots(figsize=(11.5, 7.2))

    singles = [p for p in points if p["has_single"] and (p["cost"], p["rate"]) not in frontier_keys]
    bundle_only = [p for p in points if not p["has_single"] and (p["cost"], p["rate"]) not in frontier_keys]

    ax.scatter([p["cost"] for p in bundle_only], [p["rate"] for p in bundle_only],
               s=90, facecolor="#7fa8e0", edgecolor="#2f5fb0", linewidth=1.2, zorder=3,
               label="Bundle-only")
    ax.scatter([p["cost"] for p in singles], [p["rate"] for p in singles],
               s=100, marker="s", facecolor="#f4b860", edgecolor="#c97a1a", linewidth=1.2, zorder=3,
               label="Single sensor")
    ax.plot([p["cost"] for p in frontier], [p["rate"] for p in frontier],
            "-", color="#2e8b3d", linewidth=2.2, zorder=2)
    ax.scatter([p["cost"] for p in frontier], [p["rate"] for p in frontier],
               s=170, marker="*", facecolor="#2e8b3d", edgecolor="#1a5c26", linewidth=1.2, zorder=4,
               label="Pareto frontier")

    for p in points:
        key = (p["cost"], p["rate"])
        dx, dy, ha = LABEL_OFFSETS.get(key, (10, 8, "left"))
        note = "" if p["n"] == 1 else "  †"
        on_frontier = key in frontier_keys
        # Frontier points get the full callout (name + cost + rate,
        # bold) since they're the headline recommendation at each price
        # tier. Everything else gets just its name -- the cost and rate
        # are already readable off the axes at that point's position,
        # and the shorter label is most of what keeps 12 labels from
        # colliding with only 4 of them being genuinely load-bearing.
        if on_frontier:
            text = f"{p['label']}{note}\n${usd(p['cost'])} · {p['rate']:.0%}"
        else:
            text = f"{p['label']}{note}"
        fontsize = 10 if on_frontier else 8.7
        weight = "bold" if on_frontier else "normal"
        ax.annotate(text, (p["cost"], p["rate"]),
                    textcoords="offset points", xytext=(dx, dy), ha=ha, fontsize=fontsize,
                    fontweight=weight, linespacing=1.25, zorder=5)

    ax.set_xlim(-20, 460)
    ax.set_ylim(0.0, 0.40)
    ax.yaxis.set_major_formatter(PercentFormatter(xmax=1.0))
    ax.set_xlabel("Bundle cost (USD)", fontsize=11.5)
    ax.set_ylabel("Weighted success rate (5 scenarios)", fontsize=11.5)
    ax.set_title(
        "Every buildable robot: what you pay vs. what you get\n"
        "23 candidates → 12 unique points",
        fontsize=14, fontweight="bold", pad=14, loc="left",
    )
    ax.grid(alpha=0.25, zorder=0)
    ax.legend(loc="lower right", fontsize=9.5, framealpha=0.95)

    fig.text(0.02, -0.01,
              "† IMU (free) never changes cost or success -- \"+IMU\" bundles overlap their twin exactly.",
              fontsize=8.5, color="#444444")

    fig.tight_layout()
    out_path = OUT_DIR / "figure4_bundle_scatter.png"
    fig.savefig(out_path, dpi=170, facecolor="white", bbox_inches="tight")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
