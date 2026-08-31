"""Regenerates deviation_types.png: the three deviation types swept in Pillar 3, as three field
thumbnails instead of three table rows. Each panel turns ON exactly one
deviation axis (the other two held at 0), which is the same isolation
ftc/suite_benchmark.py's sweep uses -- so the panels show the literal
experimental conditions, not illustrations of them.

Produced by nav.field_variance.generate_ground_truth, the same function
every benchmark in this project calls.

Note the overlap with figure1: this figure's middle panel and
figure1's right panel show the same obstacle-drift mechanism. The two
are alternatives more than companions -- figure1 contrasts the two
FAILURE MODES (pose vs. obstacle), while this one enumerates the three
DEVIATION AXES the sweep actually varies. Putting both on one poster
mostly buys the start-drift and blocker panels.

Regenerate after any change to ftc/field.py or nav/field_variance.py:

    python3 "screenshots/poster figures/generate_deviation_types.py"
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(OUT_DIR))

from poster_common import (  # noqa: E402
    GREEN, ROUTE_BLUE, WALL_RED,
    draw_field_base, draw_marker, draw_obstacle_cell, draw_path,
)

from ftc.field import build_grid, eroded_obstacle_cells  # noqa: E402
from nav.algorithms import astar  # noqa: E402
from nav.field_variance import generate_ground_truth  # noqa: E402

OUT = OUT_DIR / "deviation_types.png"

LAYOUT = "cluttered"
START = (21, 2)
GOAL = (2, 21)
LEVEL = 0.6
ALERT = "#c0392b"

# (key, title, subtitle, scales, seed)
PANELS = [
    ("start_drift", "Start drift", "robot doesn't begin where the plan assumed",
     dict(start_drift_scale=1.0, obstacle_drift_scale=0.0, blocker_scale=0.0), 3),
    ("obstacle_drift", "Obstacle drift", "game elements aren't where the map said",
     dict(start_drift_scale=0.0, obstacle_drift_scale=1.0, blocker_scale=0.0), 3),
    # Seed 14 rather than an arbitrary one: the blocker is PROBABILISTIC
    # (it fires with probability = severity, so ~40% of seeds produce no
    # blocker at all at 0.6), and a panel captioned "something parks on
    # the planned route" showing an empty route would be actively
    # misleading. 14 puts it mid-route where it reads clearly.
    ("unplanned_blocker", "Unplanned blocker", "something parks on the planned route",
     dict(start_drift_scale=0.0, obstacle_drift_scale=0.0, blocker_scale=1.0), 14),
]


def raw_obstacles(grid):
    return {(r, c) for r in range(grid.size) for c in range(grid.size) if grid.is_obstacle(r, c)}


def main():
    grid = build_grid(LAYOUT)
    path, _, _ = astar(grid, START, GOAL)
    assumed_raw = raw_obstacles(grid)
    baseline = eroded_obstacle_cells(grid)

    fig, axes = plt.subplots(1, 3, figsize=(15.6, 6.2))

    for ax, (key, title, subtitle, scales, seed) in zip(axes, PANELS):
        truth, actual_start = generate_ground_truth(
            grid, START, GOAL, variance_level=LEVEL, seed=seed, **scales)
        truth_raw = raw_obstacles(truth)
        added = truth_raw - assumed_raw
        removed = assumed_raw - truth_raw

        draw_field_base(ax, grid, obstacle_cells=[])
        for (r, c) in baseline - removed:
            draw_obstacle_cell(ax, r, c)
        for (r, c) in removed & baseline:
            draw_obstacle_cell(ax, r, c, facecolor="none", edgecolor="#6c7480",
                                linewidth=1.5, linestyle=(0, (3, 2)), alpha=0.95)
        for (r, c) in added:
            draw_obstacle_cell(ax, r, c, facecolor=ALERT, edgecolor="#7d1f13", linewidth=1.3)

        draw_path(ax, path, color=ROUTE_BLUE, linewidth=3.0, alpha=0.85)
        draw_marker(ax, GOAL, WALL_RED, linewidth=2.2)

        if key == "start_drift" and actual_start != START:
            # Planned start (green) vs. where the robot really is (red),
            # with the displacement drawn between them.
            draw_marker(ax, START, GREEN, linewidth=2.6)
            draw_marker(ax, actual_start, ALERT, linewidth=3.0)
            ax.annotate("", xy=(actual_start[1] + 0.5, actual_start[0] + 0.5),
                        xytext=(START[1] + 0.5, START[0] + 0.5),
                        arrowprops=dict(arrowstyle="-|>", color=ALERT, linewidth=2.4,
                                         shrinkA=8, shrinkB=8), zorder=9)
            # Label pushed right and up from the drifted start: both
            # markers sit in the bottom-left corner, so a label centered
            # on them runs off the panel edge.
            ax.annotate("really starts here",
                        xy=(actual_start[1] + 0.5, actual_start[0] + 0.5),
                        xytext=(actual_start[1] + 6.4, actual_start[0] - 2.4),
                        arrowprops=dict(arrowstyle="-|>", color=ALERT, linewidth=1.8,
                                         shrinkA=2, shrinkB=7),
                        color=ALERT, fontsize=11, fontweight="bold", ha="center", va="center",
                        zorder=9, bbox=dict(boxstyle="round,pad=0.24", facecolor="white",
                                             edgecolor=ALERT, linewidth=1.1, alpha=0.95))
        else:
            draw_marker(ax, START, GREEN, linewidth=2.6)

        ax.set_title(f"{title}\n{subtitle}", fontsize=13, fontweight="bold",
                      pad=10, linespacing=1.45)

    fig.suptitle("Three independent ways the real field can differ from the assumed map — "
                  "each swept on its own",
                  fontsize=15.5, fontweight="bold", x=0.008, ha="left", y=0.985)
    fig.text(0.008, 0.028,
              "Solid gray = unchanged game element   ·   dashed = map said obstacle, field is clear   ·   "
              f"red = obstacle the map never had   ·   severity {LEVEL} of 1.0",
              fontsize=10.5, color="#333333")

    fig.tight_layout(rect=[0, 0.055, 1, 0.94])
    fig.savefig(OUT, dpi=170, facecolor="white")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
