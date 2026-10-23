"""Regenerates figure2_ftc_field.png: "here's how we picked A* to plan across the simulated field."

One panel, built from real benchmark data (not a mockup): aggregate
stats pulled straight from benchmark_results/results.csv, nav/
benchmark.py's own 20-trial Dijkstra vs A* vs RRT vs RRT* output -- the
comparison Pillar 1's own poster text cites for why A* was the one
selected to plan every match in this project.

This figure used to carry a second, left-hand panel rendering the
simulated field with the robot mid-route on its A*-planned path. It's
gone now: Figure 1's two panels (pose error / obstacle error) already
render that same field, route, and robot -- by the time a reader
reaches this figure they've seen that visual twice, and the panel
wasn't earning its half of the canvas. Cutting it hands the whole
figure to the one thing this panel is actually for -- comparing five
algorithms -- so labels, bars, and annotations can all run bigger.
draw_field's old code (grid rendering, footprint math, etc.) still
lives in poster_common.py / generate_figure1's imports if a field panel
is ever needed here again.

Regenerate after re-running nav/benchmark.py or nav/replan_benchmark.py:

    python3 "screenshots/poster figures/generate_figure2_ftc_field.py"
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

from nav.replan_benchmark import verdict

OUT_DIR = Path(__file__).resolve().parent
RESULTS_CSV = REPO_ROOT / "benchmark_results" / "results.csv"
REPLAN_CSV = REPO_ROOT / "benchmark_results" / "replan_results.csv"
# The benchmark grid size closest to this project's 24x24 FTC field --
# what "at field scale" means in the D* Lite row below.
REPLAN_FIELD_SIZE = 25


def load_benchmark_stats():
    rows = list(csv.DictReader(open(RESULTS_CSV)))
    n = len(rows)

    def avg(key):
        return sum(float(r[key]) for r in rows) / n

    dij_t, ast_t = avg("dijkstra_runtime_ms"), avg("astar_runtime_ms")
    rrt_t, rrts_t = avg("rrt_runtime_ms"), avg("rrt_star_runtime_ms")
    dij_c, ast_c = avg("dijkstra_cells_explored"), avg("astar_cells_explored")

    fewer_pct = (1 - ast_c / dij_c) * 100
    rrt_over = sum(float(r["rrt_path_length"]) / float(r["path_length"]) - 1
                    for r in rows if r["rrt_found_path"] == "True") / n * 100
    rrts_over = sum(float(r["rrt_star_path_length"]) / float(r["path_length"]) - 1
                     for r in rows if r["rrt_star_found_path"] == "True") / n * 100
    rrts_slower = rrts_t / ast_t

    return {
        "n": n,
        "dijkstra": {"time": dij_t, "cells": dij_c},
        "astar": {"time": ast_t, "cells": ast_c, "fewer_pct": fewer_pct},
        "rrt": {"time": rrt_t, "over_pct": rrt_over},
        "rrt_star": {"time": rrts_t, "over_pct": rrts_over, "slower_x": rrts_slower},
        "dstar_lite": load_replan_stats(),
    }


def load_replan_stats():
    """How D* Lite fares against from-scratch A* at FTC field scale --
    as a VERDICT ("tie"/"loss"/"win"), not as printed constants.

    The constants were the first draft and were wrong to show: these are
    wall-clock microbenchmarks, and the 25x25 moving-obstacle ratio
    lands either side of 1.0 from run to run (0.98x and 1.08x on two
    consecutive runs here). A poster that prints "1.08x" invites a
    reader to treat a coin-flip margin as a measurement. The
    win/tie/loss characterization is what's actually stable across runs,
    so that's what this returns and what the figure draws.
    The threshold comes from nav.replan_benchmark.verdict, so this
    figure and replan_writeup.md can never disagree about what counts
    as a tie.

    Kept OFF the bar chart on purpose (see draw_benchmark): every other
    algorithm there is timed on "produce a route from scratch," while
    D* Lite's entire point is repairing an existing route after the
    world changes. nav/algorithms.py's own dstar_lite branch says the
    same thing -- a single find_path call can't demonstrate what the
    algorithm is for. Plotting a repair cost against four planning
    costs on one axis would invite a comparison neither number
    supports.

    Aggregated exactly the way nav/replan_benchmark.py's `average` +
    plot/writeup path does: mean total-ms per trial at one grid size,
    then astar_mean / dstar_mean. Recomputed here rather than read from
    replan_writeup.md, because that writeup's numbers are stale -- the
    CSV was regenerated (commit c73fe1d) without regenerating the
    writeup alongside it, so its table still reports an older run.
    """
    totals = {}
    with open(REPLAN_CSV) as f:
        for row in csv.DictReader(f):
            if int(row["grid_size"]) != REPLAN_FIELD_SIZE:
                continue
            a, d = totals.setdefault(row["scenario"], [[], []])
            a.append(float(row["astar_total_ms"]))
            d.append(float(row["dstar_total_ms"]))

    def speedup(scenario):
        a, d = totals[scenario]
        return (sum(a) / len(a)) / (sum(d) / len(d))

    mo, sd = verdict(speedup("moving_obstacle")), verdict(speedup("sensor_discovery"))
    if mo == "win" and sd == "win":
        summary = "already beats A* at FTC field scale"
    elif mo == "win" or sd == "win":
        summary = ("mixed at FTC field scale — wins where a\n"
                   "replan stays local, loses where it doesn't")
    else:
        summary = ("no faster than A* at FTC field scale;\n"
                   "only pays off on much larger grids")
    return {"size": REPLAN_FIELD_SIZE, "summary": summary}


def draw_benchmark(ax, stats):
    n = stats["n"]
    ds = stats["dstar_lite"]
    # The 5th entry carries time=None: D* Lite was tested, but on a
    # different quantity (repair-after-change, not plan-from-scratch --
    # see load_replan_stats), so it gets a labeled row and a note
    # instead of a bar. Dropping it entirely would leave the panel
    # titled "5 pathfinding algorithms tested" over four bars, which is
    # the inconsistency this row exists to close.
    algos = [
        ("Dijkstra", stats["dijkstra"]["time"], "#3c6fce", False,
         f"{stats['dijkstra']['cells']:.0f} cells explored"),
        ("A*  (selected)", stats["astar"]["time"], "#2e8b3d", True,
         f"{stats['astar']['cells']:.0f} cells explored ({stats['astar']['fewer_pct']:.0f}% fewer than Dijkstra)"),
        ("RRT", stats["rrt"]["time"], "#e0863c", False,
         f"{stats['rrt']['over_pct']:.1f}% longer route than shortest possible"),
        ("RRT*", stats["rrt_star"]["time"], "#7a4a35", False,
         f"{abs(stats['rrt_star']['over_pct']):.0f}% shorter route than A*, {stats['rrt_star']['slower_x']:.0f}× longer to compute"),
        ("D* Lite", None, "#9aa0a8", False,
         f"repairs a route, doesn't plan one\n{ds['summary']}"),
    ]
    ys = list(range(len(algos)))[::-1]
    ax.set_xscale("log")
    ax.set_xlim(0.05, 500)
    # Extra headroom below the last row so the bar-less D* Lite entry
    # doesn't sit on top of the x-axis spine and its tick labels.
    ax.set_ylim(-1.15, len(algos) - 0.5)
    for y, (label, t, color, selected, note) in zip(ys, algos):
        weight = "bold" if selected else "normal"
        if t is None:
            # No bar, and an em dash where the timing would sit, so the
            # row reads as "measured, but not on this axis" rather than
            # as a missing or zero value. The rule above it marks where
            # the shared planning-time axis stops applying.
            ax.axhline(y + 0.55, color="#c9ced6", linewidth=1.4, linestyle=(0, (4, 3)), zorder=1)
            ax.text(0.062, y, "—", va="center", ha="left", fontsize=26, color=color)
            ax.text(1.06, y, note, va="center", ha="left", fontsize=15.5,
                    color="#5b6470", linespacing=1.45, style="italic",
                    transform=ax.get_yaxis_transform())
            continue
        edge = "black" if selected else "none"
        lw = 3.5 if selected else 0
        ax.barh(y, t, color=color, edgecolor=edge, linewidth=lw, height=0.6, zorder=3)
        ax.text(t * 1.15, y, f"{t:.2f} ms", va="center", fontsize=22, fontweight=weight)
        ax.text(1.06, y, note, va="center", ha="left", fontsize=18,
                fontweight=weight, linespacing=1.35, transform=ax.get_yaxis_transform())
    ax.set_yticks(ys)
    ax.set_yticklabels([a[0] for a in algos], fontsize=23, fontweight="bold")
    for tick, (_, t, color, selected, _) in zip(ax.get_yticklabels(), algos):
        if selected:
            tick.set_color("#2e8b3d")
        elif t is None:
            tick.set_color(color)
    ax.set_xlabel(f"Planning time (ms, log scale, n={n})", fontsize=18)
    ax.tick_params(axis="x", labelsize=15)
    ax.set_title("5 pathfinding algorithms tested — which one plans the route",
                  fontsize=26, fontweight="bold", pad=16, loc="left")
    ax.spines[["top", "right", "left"]].set_visible(False)
    ax.tick_params(left=False)
    ax.grid(axis="x", which="major", color="#dddddd", zorder=0)


def main():
    stats = load_benchmark_stats()

    fig = plt.figure(figsize=(20, 9.6), dpi=200)
    # Left margin wide enough for the bold, 23pt y-tick labels (the
    # longest is "A*  (selected)") -- these used to sit in a narrower
    # gridspec cell next to a field panel and don't fit flush against
    # the figure edge at this larger poster-legible size.
    ax = fig.add_axes([0.145, 0.1, 0.855, 0.8])
    # Shrink the bar-chart axes within the figure so the per-bar
    # annotation text (placed at axes-fraction x=1.06 in draw_benchmark)
    # has room to the right of the bars without running off the figure.
    pos = ax.get_position()
    ax.set_position([pos.x0, pos.y0, pos.width * 0.5, pos.height])

    draw_benchmark(ax, stats)

    out_path = OUT_DIR / "figure2_ftc_field.png"
    fig.savefig(out_path, facecolor="white")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
