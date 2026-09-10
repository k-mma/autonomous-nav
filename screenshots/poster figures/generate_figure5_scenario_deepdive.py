"""Regenerates figure5_scenario_deepdive.png: a per-scenario deep-dive asking "does buying a
BUNDLE actually beat buying the single best sensor, in THIS specific scenario?" Reads
benchmark_results/ftc_optimizer_match_results.csv -- ftc/optimizer.py's MATCH_PROFILES catalog,
not the DEFAULT_PROFILES one figure4_bundle_scatter.py reads -- no new simulation run, this only
re-presents that CSV.

Why a different CSV than figure4. DEFAULT_PROFILES exists to ATTRIBUTE a bundle's success to a
capability, which requires each profile to isolate ONE deviation axis (see ftc/optimizer.py's
module comment above DEFAULT_PROFILES) -- but that isolation is exactly what broke this figure's
original version. With only one failure mode live, only one sensor category can act on it, a
bundle's second part is inert by construction, and the best bundle can score at best a TIE with
the best single sensor. Three of DEFAULT_PROFILES' five scenarios did precisely that: odometry
pods and odometry pods + front camera succeeded on the identical trial set under map_error,
opponent, and combined_realistic, because front camera only ever fixes POSE and none of those
three profiles' dominant axis was pose drift. That is a property of the scenario design, not a
finding about bundles -- this figure was measuring DEFAULT_PROFILES, not the robots. MATCH_PROFILES
keeps one clearly-named dominant axis per scenario (so "different scenarios, different answers"
still reads) but always has a second axis live too, so a bundle has something for its second part
to actually do.

Two bars per scenario, both always populated (the previous version plotted only whichever of
"scenario winner" / "generalist" happened to win, so one bar was routinely empty):
  - "Best single sensor" -- the highest-success-rate candidate built from a single purchased
    sensor. A free IMU riding along on top of one paid sensor does NOT promote that candidate to
    a bundle here: "odometry pods + IMU" is the odometry pods single wearing a $0 add-on, not a
    second thing bought (ImuSuite is priced at $0 -- ftc/config.py's IMU_COST_USD). This is a
    classification rule, not an empirical one: four of MATCH_PROFILES' five scenarios actually run
    at "realistic" fidelity (ftc/optimizer.py's DEFAULT_PROFILES/MATCH_PROFILES module comment --
    only `corridor` stays at "optimistic," the same shared profile figure4's catalog uses), so
    IMU's heading correction is live there and DOES change some trial outcomes (e.g. dead_reckoning
    vs. imu differ under pose_drift_match/opponent_match/combined_match in the CSV) -- it just
    never changes which PAID parts a candidate is built from, which is what this figure classifies
    by. Classification is by PAID parts (ftc/config.py's PART_COSTS_USD), not by suite count, suite
    name, or whether a trial's outcome happened to tie, which is also why a two-camera single
    suite like dual_camera_apriltag counts as a bundle here -- it's two purchased webcams,
    whatever ftc/sensors.py calls the suite that bolts them together.
  - "Best bundle" -- the highest-success-rate candidate with 2+ distinct PAID parts.
Colored orange/blue, matching figure4's single-vs-bundle coding, so a reader who's seen that
chart doesn't relearn a mapping. Ties (bundle bar no taller than single) ARE a finding, not a
rendering bug -- see ftc_optimizer_match_writeup.md's paired significance test for whether a given
gap is distinguishable from noise at this trial count.

Regenerate after re-running `python3 -m ftc.optimizer_benchmark --profiles match` (i.e. whenever
benchmark_results/ftc_optimizer_match_results.csv changes):

    python3 "screenshots/poster figures/generate_figure5_scenario_deepdive.py"
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path
from textwrap import fill

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.ticker import PercentFormatter

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

from ftc.config import PART_COSTS_USD  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent
RESULTS_CSV = REPO_ROOT / "benchmark_results" / "ftc_optimizer_match_results.csv"


def usd(cost):
    """Whole-dollar display rounding for a cost -- see generate_figure3_
    reliability_per_dollar.py's usd() for why plain round()/f"{:.0f}"
    (banker's rounding) isn't used here."""
    return int(cost + 0.5)


# Mirrors ftc/optimizer.py's MATCH_PROFILES order and labels exactly --
# duplicated rather than imported so this module (like figure4's own
# generator) stays a pure CSV reader with no matplotlib-free-but-still-
# a-dependency import into ftc/.
PROFILE_ORDER = ["pose_drift_match", "map_error_match", "opponent_match", "combined_match", "corridor"]
PROFILE_LABELS = {
    "pose_drift_match": "Heavy pose drift\n(map also off)",
    "map_error_match": "Field doesn't match\nthe map (also drifting)",
    "opponent_match": "Opponent parks in\nthe route (also drifting)",
    "combined_match": "Everything at once\n(match-realistic mix)",
    "corridor": "Tight corridor,\nmixed deviation",
}


def load_candidates():
    rows = list(csv.DictReader(open(RESULTS_CSV)))
    by_bundle = defaultdict(list)
    for r in rows:
        by_bundle[r["bundle"]].append(r)

    candidates = {}
    for key, rs in by_bundle.items():
        parts = [p for p in rs[0]["parts"].split("|") if p]
        # PAID parts only -- a free part (today, only "imu": ftc/config.py's
        # IMU_COST_USD is 0.0) doesn't make a one-sensor purchase into a
        # bundle. This is a PART count, not a suite-name count: a suite
        # like dual_camera_apriltag is one name in ftc/sensors.py but two
        # purchased webcams, and is classified as a bundle here on that
        # basis.
        paid_parts = [p for p in parts if PART_COSTS_USD.get(p, 0) > 0]
        per_profile = {}
        for p in PROFILE_ORDER:
            prows = [r for r in rs if r["profile"] == p]
            per_profile[p] = sum(int(r["success"]) for r in prows) / len(prows)
        candidates[key] = dict(
            key=key, parts_label=rs[0]["parts_label"], cost=float(rs[0]["cost_usd"]),
            n_paid_parts=len(paid_parts), per_profile=per_profile,
        )
    return candidates


def best_of(candidates, predicate, profile):
    """Highest success rate on `profile` among candidates matching
    `predicate`, ties broken toward the cheaper option -- same
    tiebreak ftc/optimizer.py's best_per_profile and best_under_budget
    use throughout, so a team never reads "cheaper for the identical
    result" as some other candidate's win."""
    pool = [v for v in candidates.values() if predicate(v)]
    return max(pool, key=lambda v: (v["per_profile"][profile], -v["cost"]))


def main():
    candidates = load_candidates()

    rows = []
    for p in PROFILE_ORDER:
        single = best_of(candidates, lambda v: v["n_paid_parts"] <= 1, p)
        bundle = best_of(candidates, lambda v: v["n_paid_parts"] >= 2, p)
        rows.append(dict(
            profile=p,
            single_label=single["parts_label"], single_cost=single["cost"],
            single_rate=single["per_profile"][p],
            bundle_label=bundle["parts_label"], bundle_cost=bundle["cost"],
            bundle_rate=bundle["per_profile"][p],
        ))

    fig, ax = plt.subplots(figsize=(13, 7.2))
    x = list(range(len(rows)))
    bar_w = 0.34

    SINGLE_FACE, SINGLE_EDGE = "#f4b860", "#c97a1a"
    BUNDLE_FACE, BUNDLE_EDGE = "#7fa8e0", "#2f5fb0"

    ax.bar([xi - bar_w / 2 - 0.01 for xi in x], [r["single_rate"] for r in rows], width=bar_w,
           color=SINGLE_FACE, edgecolor=SINGLE_EDGE, linewidth=1.3, zorder=3)
    ax.bar([xi + bar_w / 2 + 0.01 for xi in x], [r["bundle_rate"] for r in rows], width=bar_w,
           color=BUNDLE_FACE, edgecolor=BUNDLE_EDGE, linewidth=1.3, zorder=3)

    legend_handles = [
        Patch(facecolor=SINGLE_FACE, edgecolor=SINGLE_EDGE, linewidth=1.3, label="Best single sensor"),
        Patch(facecolor=BUNDLE_FACE, edgecolor=BUNDLE_EDGE, linewidth=1.3, label="Best bundle (2+ paid parts)"),
    ]

    # Two collisions to guard against, both from labels being wide
    # relative to how close together the bars are: (1) a single/bundle
    # tie or near-tie puts both labels at the same height right next to
    # each other, and (2) two ADJACENT scenarios' bars landing at
    # similar heights puts their labels within reach of each other
    # across the group boundary. Wrapping long labels onto two lines
    # shrinks the horizontal footprint that causes (2); staggering the
    # bundle label higher whenever it's within a few points of the
    # single bar's height fixes (1).
    for xi, r in zip(x, rows):
        close = abs(r["bundle_rate"] - r["single_rate"]) < 0.035
        single_label = fill(r["single_label"], 14)
        bundle_label = fill(r["bundle_label"], 14)
        ax.annotate(f"{single_label}\n${usd(r['single_cost'])} · {r['single_rate']:.0%}",
                    (xi - bar_w / 2 - 0.01, r["single_rate"]), textcoords="offset points",
                    xytext=(0, 6), ha="center", fontsize=7.6, linespacing=1.2, zorder=4)
        ax.annotate(f"{bundle_label}\n${usd(r['bundle_cost'])} · {r['bundle_rate']:.0%}",
                    (xi + bar_w / 2 + 0.01, r["bundle_rate"]), textcoords="offset points",
                    xytext=(0, 30 if close else 6), ha="center", fontsize=7.6, fontweight="bold",
                    color="#1f4a8a", linespacing=1.2, zorder=4)

    ax.set_xticks(x)
    ax.set_xticklabels([PROFILE_LABELS[r["profile"]] for r in rows], fontsize=10.5)
    ax.set_ylim(0, 0.80)
    ax.yaxis.set_major_formatter(PercentFormatter(xmax=1.0))
    ax.set_ylabel("Success rate (this scenario)", fontsize=11.5)
    ax.set_title(
        "Which robot wins YOUR scenario?\n"
        "Best single sensor vs. best bundle",
        fontsize=13.5, fontweight="bold", pad=14, loc="left",
    )
    ax.grid(axis="y", alpha=0.25, zorder=0)
    ax.legend(handles=legend_handles, loc="upper right", fontsize=9, framealpha=0.95)

    fig.text(0.02, -0.02,
              "Ties (bundle bar no taller than single) are a real result at this trial count, not a "
              "missing bar -- see ftc_optimizer_match_writeup.md's paired significance test per bundle.",
              fontsize=8.7, color="#444444")

    fig.tight_layout()
    out_path = OUT_DIR / "figure5_scenario_deepdive.png"
    fig.savefig(out_path, dpi=170, facecolor="white", bbox_inches="tight")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
