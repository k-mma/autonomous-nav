"""Regenerates figure1_ftc_field.png for the SEED symposium poster's
Pillar 1: "we simulated the field, and here's how we picked A* to plan
across it."

Two panels, built from two different (and independent) pieces of this
project's real code -- neither panel is a mockup:

  Left  -- ftc.field.build_grid("cluttered"), an actual FTC-scale
           (144in x 144in, 6in cells = 24x24) grid, planned with
           nav.algorithms.astar, with the robot's footprint drawn via
           the same rotated-footprint corner math ftc/field.py's
           footprint_overlaps_cells collision check uses
           (_footprint_corners), over the *eroded* (real, uninflated)
           obstacle cells eroded_obstacle_cells() returns.

  Right -- aggregate stats pulled straight from benchmark_results/
           results.csv, nav/benchmark.py's own 20-trial Dijkstra vs A*
           vs RRT vs RRT* output -- the comparison Pillar 1's own
           poster text cites for why A* was the one selected to plan
           every match in this project.

Regenerate after any change to ftc/field.py, ftc/match.py, or the
footprint/collision geometry those two share, or after re-running
nav/benchmark.py:

    python3 "screenshots/poster figures/generate_figure1_ftc_field.py"
"""
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

from ftc.field import _footprint_corners, build_grid, eroded_obstacle_cells
from nav.algorithms import astar

OUT_DIR = Path(__file__).resolve().parent
RESULTS_CSV = REPO_ROOT / "benchmark_results" / "results.csv"

LAYOUT = "cluttered"
START = (21, 2)
GOAL = (2, 21)
# How far along the A*-planned path the robot has driven, as a fraction
# of the route -- purely illustrative ("mid-route" framing), not tied
# to any timing model.
ROBOT_PROGRESS = 0.35

FIELD_BG = "#eef0f2"
CELL_BG = "#b7b9bc"
GRID_LINE = "#8f9194"
OBSTACLE = "#3c4452"
WALL_BLUE = "#2f5fd0"
WALL_RED = "#d0332f"
WALL_BLACK = "#1a1a1a"
ROBOT_BLUE = "#1f6fd6"
GOLD = "#e8a33d"
ROUTE_BLUE = "#2f5fd0"
GREEN = "#2e8b3d"


def heading_deg(prev_cell, next_cell):
    """atan2(d_row, d_col), in degrees -- the same heading convention
    every consumer in ftc/ uses (see ftc/field.py's _footprint_corners
    docstring)."""
    dr = next_cell[0] - prev_cell[0]
    dc = next_cell[1] - prev_cell[1]
    return math.degrees(math.atan2(dr, dc))


def draw_field(ax, grid, path, robot_idx):
    size = grid.size
    ax.set_facecolor(FIELD_BG)
    ax.set_xlim(-0.6, size + 0.6)
    ax.set_ylim(size + 0.6, -0.6)
    ax.set_aspect("equal")
    ax.axis("off")

    ax.add_patch(Rectangle((0, 0), size, size, facecolor=CELL_BG, edgecolor="none", zorder=0))
    for i in range(4, size, 3):
        ax.axvline(i, color=GRID_LINE, linewidth=1, zorder=1)
        ax.axhline(i, color=GRID_LINE, linewidth=1, zorder=1)

    # Perimeter walls -- blue (left) / red (right) alliance walls, black
    # top/bottom, matching FTC field convention.
    wall_w = 6
    ax.plot([0, 0], [0, size], color=WALL_BLUE, linewidth=wall_w, solid_capstyle="butt", zorder=5)
    ax.plot([size, size], [0, size], color=WALL_RED, linewidth=wall_w, solid_capstyle="butt", zorder=5)
    ax.plot([0, size], [0, 0], color=WALL_BLACK, linewidth=wall_w, solid_capstyle="butt", zorder=5)
    ax.plot([0, size], [size, size], color=WALL_BLACK, linewidth=wall_w, solid_capstyle="butt", zorder=5)

    # Obstacles -- the REAL (eroded) game-element cells, not the
    # planner's hard-inflated safety margin.
    for (r, c) in eroded_obstacle_cells(grid):
        pad = 0.12
        ax.add_patch(FancyBboxPatch(
            (c + pad, r + pad), 1 - 2 * pad, 1 - 2 * pad,
            boxstyle="round,pad=0,rounding_size=0.08",
            facecolor=OBSTACLE, edgecolor="none", zorder=2,
        ))

    # Path: gold = driven so far, bold blue = route remaining.
    driven = path[:robot_idx + 1]
    remaining = path[robot_idx:]
    for segment, color, lw in ((driven, GOLD, 4), (remaining, ROUTE_BLUE, 5)):
        xs = [c + 0.5 for (r, c) in segment]
        ys = [r + 0.5 for (r, c) in segment]
        ax.plot(xs, ys, color=color, linewidth=lw, solid_joinstyle="round",
                 solid_capstyle="round", zorder=3)

    # Start marker (green square outline).
    sr, sc = START
    ax.add_patch(Rectangle((sc + 0.15, sr + 0.15), 0.7, 0.7, facecolor="none",
                            edgecolor=GREEN, linewidth=3, zorder=4))

    # Goal marker (red square outline + downward flag/arrow glyph).
    gr, gc = GOAL
    ax.add_patch(Rectangle((gc + 0.15, gr + 0.15), 0.7, 0.7, facecolor="none",
                            edgecolor=WALL_RED, linewidth=3, zorder=4))
    ax.annotate("", xy=(gc + 0.5, gr + 0.75), xytext=(gc + 0.5, gr + 0.25),
                arrowprops=dict(arrowstyle="-|>", color=WALL_RED, linewidth=2.5), zorder=5)

    # Robot: actual rotated footprint (the same corner math ftc/field.py's
    # collision check uses), corner dots, and a heading triangle.
    r_row, r_col = path[robot_idx]
    prev_idx = max(robot_idx - 1, 0)
    next_idx = min(robot_idx + 1, len(path) - 1)
    hdg = heading_deg(path[prev_idx], path[next_idx])
    corners, fwd_axis, right_axis = _footprint_corners((r_row + 0.5, r_col + 0.5), hdg)
    poly_xy = [(c, r) for (r, c) in corners]
    ax.add_patch(plt.Polygon(poly_xy, closed=True, facecolor=ROBOT_BLUE,
                              edgecolor="white", linewidth=2, zorder=6))
    for (r, c) in corners:
        ax.add_patch(plt.Circle((c, r), 0.07, facecolor="black", edgecolor="none", zorder=7))
    tip = (r_col + 0.5 + 0.55 * fwd_axis[1], r_row + 0.5 + 0.55 * fwd_axis[0])
    left = (r_col + 0.5 - 0.28 * right_axis[1] - 0.15 * fwd_axis[1],
             r_row + 0.5 - 0.28 * right_axis[0] - 0.15 * fwd_axis[0])
    right = (r_col + 0.5 + 0.28 * right_axis[1] - 0.15 * fwd_axis[1],
              r_row + 0.5 + 0.28 * right_axis[0] - 0.15 * fwd_axis[0])
    ax.add_patch(plt.Polygon([tip, left, right], closed=True, facecolor="white",
                              edgecolor="none", zorder=8))

    ax.set_title(
        f"Simulated FTC field ({int(grid.size * 6)}in × {int(grid.size * 6)}in)\n"
        "Robot mid-route on its A*-planned path",
        fontsize=15.5, fontweight="bold", pad=12, loc="left",
    )


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
    }


def draw_benchmark(ax, stats):
    n = stats["n"]
    algos = [
        ("Dijkstra", stats["dijkstra"]["time"], "#3c6fce", False,
         f"{stats['dijkstra']['cells']:.0f} cells explored"),
        ("A*  (selected)", stats["astar"]["time"], "#2e8b3d", True,
         f"{stats['astar']['cells']:.0f} cells (−{stats['astar']['fewer_pct']:.0f}% vs. Dijkstra)"),
        ("RRT", stats["rrt"]["time"], "#e0863c", False,
         f"+{stats['rrt']['over_pct']:.1f}% longer path"),
        ("RRT*", stats["rrt_star"]["time"], "#7a4a35", False,
         f"−{abs(stats['rrt_star']['over_pct']):.0f}% path, {stats['rrt_star']['slower_x']:.0f}× slower"),
    ]
    ys = list(range(len(algos)))[::-1]
    ax.set_xscale("log")
    ax.set_xlim(0.05, 500)
    for y, (label, t, color, selected, note) in zip(ys, algos):
        edge = "black" if selected else "none"
        lw = 2.5 if selected else 0
        ax.barh(y, t, color=color, edgecolor=edge, linewidth=lw, height=0.55, zorder=3)
        weight = "bold" if selected else "normal"
        ax.text(t * 1.15, y, f"{t:.2f} ms", va="center", fontsize=15, fontweight=weight)
        ax.text(1.04, y, note, va="center", ha="left", fontsize=12.5,
                fontweight=weight, linespacing=1.3, transform=ax.get_yaxis_transform())
    ax.set_yticks(ys)
    ax.set_yticklabels([a[0] for a in algos], fontsize=16, fontweight="bold")
    for tick, (_, _, _, selected, _) in zip(ax.get_yticklabels(), algos):
        if selected:
            tick.set_color("#2e8b3d")
    ax.set_xlabel(f"Planning time (ms, log scale, n={n})", fontsize=12.5)
    ax.set_title("5 pathfinding algorithms tested", fontsize=17, fontweight="bold", pad=12, loc="left")
    ax.spines[["top", "right", "left"]].set_visible(False)
    ax.tick_params(left=False)
    ax.grid(axis="x", which="major", color="#dddddd", zorder=0)


def main():
    grid = build_grid(LAYOUT)
    path, settled, came_from = astar(grid, START, GOAL)
    if path is None:
        raise RuntimeError(f"no path from {START} to {GOAL} on layout {LAYOUT!r}")
    robot_idx = max(1, min(len(path) - 2, round(len(path) * ROBOT_PROGRESS)))

    stats = load_benchmark_stats()

    fig = plt.figure(figsize=(20, 9.6), dpi=150)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 0.85], wspace=0.05,
                            left=0.05, right=0.99, top=0.88, bottom=0.1)
    ax_field = fig.add_subplot(gs[0, 0])
    ax_bench = fig.add_subplot(gs[0, 1])
    # Shrink the bar-chart axes within its gridspec cell so the per-bar
    # annotation text (placed at axes-fraction x=1.04 in draw_benchmark)
    # has room to the right of the bars without running off the figure.
    pos = ax_bench.get_position()
    ax_bench.set_position([pos.x0, pos.y0, pos.width * 0.6, pos.height])

    draw_field(ax_field, grid, path, robot_idx)
    draw_benchmark(ax_bench, stats)

    fig.text(0.045, 0.03,
              "start · goal · driven · remaining",
              fontsize=13, color="#333333")

    out_path = OUT_DIR / "figure1_ftc_field.png"
    fig.savefig(out_path, facecolor="white")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
