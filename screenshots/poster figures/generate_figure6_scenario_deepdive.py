"""Regenerates figure6_scenario_deepdive.png for the SEED symposium
poster: a per-scenario deep-dive asking "which specific robot wins THIS
scenario, at what cost, and how does it compare to the one generalist
bundle a team would buy without knowing its scenario?" Same source data
as figure4_bundle_scatter.png (benchmark_results/ftc_optimizer_results.csv),
no new simulation run -- this only re-presents that CSV.

Two bars per scenario:
  - "Best for this scenario" -- whichever candidate (individual sensor
    OR bundle, whichever wins) scores highest on THAT scenario alone,
    ties broken toward the cheaper option. Colored orange if the
    winner is a single sensor, blue if it's a bundle -- same color
    coding figure4 uses, so a reader who's seen that chart doesn't
    relearn a mapping.
  - "Generalist pick" -- odometry pods + front camera ($304.99), the
    single bundle figure4's Pareto frontier says is the best buy ON
    AVERAGE across all 5 scenarios. Same green as figure4's frontier
    star, for the same cross-chart-consistency reason.

Where the two bars tie, that IS the finding (the generalist already is
scenario-optimal); where they don't (Heavy pose drift), that's the one
scenario in this catalog where chasing a cheaper, scenario-specific
bundle beats sticking with the generalist.

Regenerate after re-running ftc/optimizer_benchmark.py (i.e. whenever
benchmark_results/ftc_optimizer_results.csv changes):

    python3 "screenshots/poster figures/generate_figure6_scenario_deepdive.py"
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.ticker import PercentFormatter

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

OUT_DIR = Path(__file__).resolve().parent
RESULTS_CSV = REPO_ROOT / "benchmark_results" / "ftc_optimizer_results.csv"

PROFILE_ORDER = ["pose_drift", "map_error", "opponent", "combined_realistic", "corridor"]
PROFILE_LABELS = {
    "pose_drift": "Heavy pose\ndrift",
    "map_error": "Field doesn't\nmatch the map",
    "opponent": "Opponent parks\nin the route",
    "combined_realistic": "Everything at once\n(realistic fidelity)",
    "corridor": "Tight corridor,\nmixed deviation",
}
GENERALIST_LABEL = "odometry pods + front camera"


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
            n_components=int(rs[0]["n_components"]), per_profile=per_profile,
        )
    return candidates


def main():
    candidates = load_candidates()
    generalist = next(v for v in candidates.values() if v["parts_label"] == GENERALIST_LABEL)

    rows = []
    for p in PROFILE_ORDER:
        best_rate = max(v["per_profile"][p] for v in candidates.values())
        tied = [v for v in candidates.values() if abs(v["per_profile"][p] - best_rate) < 1e-9]
        winner = min(tied, key=lambda v: v["cost"])
        is_bundle = winner["n_components"] > 1
        n_tied_individuals = sum(1 for v in tied if v["n_components"] == 1)
        rows.append(dict(
            profile=p, winner_label=winner["parts_label"], winner_cost=winner["cost"],
            winner_rate=best_rate, is_bundle=is_bundle, n_tied=len(tied),
            n_tied_individuals=n_tied_individuals,
            generalist_rate=generalist["per_profile"][p],
        ))

    fig, ax = plt.subplots(figsize=(11.5, 6.8))
    x = list(range(len(rows)))
    bar_w = 0.34

    SINGLE_FACE, SINGLE_EDGE = "#f4b860", "#c97a1a"
    BUNDLE_FACE, BUNDLE_EDGE = "#7fa8e0", "#2f5fb0"
    GENERALIST_FACE, GENERALIST_EDGE = "#8fc98f", "#2e8b3d"

    winner_colors = [BUNDLE_FACE if r["is_bundle"] else SINGLE_FACE for r in rows]
    winner_edges = [BUNDLE_EDGE if r["is_bundle"] else SINGLE_EDGE for r in rows]
    ax.bar([xi - bar_w / 2 - 0.01 for xi in x], [r["winner_rate"] for r in rows], width=bar_w,
           color=winner_colors, edgecolor=winner_edges, linewidth=1.3, zorder=3)
    ax.bar([xi + bar_w / 2 + 0.01 for xi in x], [r["generalist_rate"] for r in rows], width=bar_w,
           color=GENERALIST_FACE, edgecolor=GENERALIST_EDGE, linewidth=1.3, zorder=3)

    # Explicit proxy handles rather than relying on the bar calls' own
    # labels -- the "best for this scenario" bars are two colors within
    # ONE bar() call (color varies per scenario), which only ever
    # produces one merged legend entry from the call itself; separate
    # Patch handles are what let the orange and blue readings each get
    # their own labeled swatch instead of being described in a single
    # entry's text.
    legend_handles = [
        Patch(facecolor=SINGLE_FACE, edgecolor=SINGLE_EDGE, linewidth=1.3,
              label="Best pick: single sensor"),
        Patch(facecolor=BUNDLE_FACE, edgecolor=BUNDLE_EDGE, linewidth=1.3,
              label="Best pick: bundle"),
        Patch(facecolor=GENERALIST_FACE, edgecolor=GENERALIST_EDGE, linewidth=1.3,
              label=f"Generalist bundle (${generalist['cost']:.0f}, from Fig. 4)"),
    ]

    for xi, r in zip(x, rows):
        star = "*" if r["n_tied_individuals"] > 1 and not r["is_bundle"] else ""
        label = r["winner_label"] if r["n_tied_individuals"] <= 1 else f"any of {r['n_tied_individuals']} individuals"
        ax.annotate(f"{label}{star}\n${r['winner_cost']:.0f} · {r['winner_rate']:.0%}",
                    (xi - bar_w / 2 - 0.01, r["winner_rate"]), textcoords="offset points",
                    xytext=(0, 6), ha="center", fontsize=9, linespacing=1.25, zorder=4)
        ax.annotate(f"{r['generalist_rate']:.0%}",
                    (xi + bar_w / 2 + 0.01, r["generalist_rate"]), textcoords="offset points",
                    xytext=(0, 6), ha="center", fontsize=9, fontweight="bold", color="#1f6b28", zorder=4)

    ax.set_xticks(x)
    ax.set_xticklabels([PROFILE_LABELS[r["profile"]] for r in rows], fontsize=10.5)
    ax.set_ylim(0, 0.80)
    ax.yaxis.set_major_formatter(PercentFormatter(xmax=1.0))
    ax.set_ylabel("Success rate (this scenario)", fontsize=11.5)
    ax.set_title(
        "Which robot wins YOUR scenario?\n"
        "Scenario-best vs. the generalist pick",
        fontsize=13.5, fontweight="bold", pad=14, loc="left",
    )
    ax.grid(axis="y", alpha=0.25, zorder=0)
    ax.legend(handles=legend_handles, loc="upper right", fontsize=9, framealpha=0.95)

    fig.text(0.02, -0.02,
              "* Ties shown at the cheapest option. Pose drift is the one case the generalist loses.",
              fontsize=8.7, color="#444444")

    fig.tight_layout()
    out_path = OUT_DIR / "figure6_scenario_deepdive.png"
    fig.savefig(out_path, dpi=170, facecolor="white", bbox_inches="tight")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
