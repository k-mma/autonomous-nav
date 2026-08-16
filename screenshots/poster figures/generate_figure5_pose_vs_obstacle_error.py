"""Regenerates figure5_pose_vs_obstacle_error.png for the SEED
symposium poster's Background section: the two failure modes the whole
study is organized around, side by side ("2 things you're solving for:
pose error vs. obstacle error", which the poster otherwise states only
in prose).

Left  -- POSE ERROR. The robot's believed position (gray dashed) has
         drifted from its true position (solid blue). It is driving the
         route it planned, correctly, from a belief that is wrong -- so
         the true track (gold) diverges from the planned one (blue).

Right -- OBSTACLE ERROR. The robot's position is perfect, but the field
         doesn't match the map it planned against: obstacles the map
         says are there (dashed outline) aren't, and real ones (solid
         red-edged) are somewhere the map never mentioned. The planned
         route drives straight into one.

Both panels use the real ftc.field grid and real ground-truth
generation (nav.field_variance.generate_ground_truth, the same function
every benchmark in this project uses to build its deviations) -- the
divergences shown are produced, not drawn by hand.

Regenerate after any change to ftc/field.py or nav/field_variance.py:

    python3 "screenshots/poster figures/generate_figure5_pose_vs_obstacle_error.py"
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(OUT_DIR))

from poster_common import (  # noqa: E402
    BELIEVED_GRAY, GOLD, GREEN, ROUTE_BLUE, WALL_RED,
    draw_field_base, draw_marker, draw_obstacle_cell, draw_path, draw_robot, heading_deg,
)

from ftc.field import build_grid, eroded_obstacle_cells  # noqa: E402
from nav.algorithms import astar  # noqa: E402
from nav.field_variance import generate_ground_truth  # noqa: E402

OUT = OUT_DIR / "figure5_pose_vs_obstacle_error.png"

LAYOUT = "cluttered"
START = (21, 2)
GOAL = (2, 21)
# Where along the route each panel freezes the robot.
POSE_PROGRESS = 0.45
OBST_PROGRESS = 0.42
# Believed-vs-true offset for the left panel, in cells. Illustrative of
# the magnitude dead reckoning accumulates over a long traverse (see
# ftc/config.py's DEAD_RECKONING drift note: "several cells by the end
# of a long path"), drawn at a size that reads at poster scale.
POSE_OFFSET = (2.1, -1.7)


def panel_pose_error(ax, grid, path):
    draw_field_base(ax, grid)

    idx = max(1, min(len(path) - 2, round(len(path) * POSE_PROGRESS)))
    hdg = heading_deg(path[max(idx - 1, 0)], path[min(idx + 1, len(path) - 1)])

    # The robot executes its plan in its OWN BELIEF frame: its believed
    # pose tracks the planned route exactly (it thinks it is driving the
    # route correctly, and by its own reckoning it is), while its TRUE
    # pose is the one that has drifted off. Drawing it the other way
    # round -- true robot on the route, belief off it -- would invert
    # the whole mechanism.
    believed_cell = path[idx]
    believed_pos = (believed_cell[0] + 0.5, believed_cell[1] + 0.5)
    true_pos = (believed_pos[0] + POSE_OFFSET[0], believed_pos[1] + POSE_OFFSET[1])

    draw_path(ax, path, color=ROUTE_BLUE, linewidth=3.4, alpha=0.55)
    # True track: the same route, progressively displaced by the drift
    # that accumulates with distance driven (zero at the start, full
    # POSE_OFFSET by now) -- the random-walk-with-distance model
    # ftc/config.py's drift constants describe.
    true_track = [(r + POSE_OFFSET[0] * (i / idx), c + POSE_OFFSET[1] * (i / idx))
                   for i, (r, c) in enumerate(path[:idx + 1])]
    xs = [c + 0.5 for (r, c) in true_track]
    ys = [r + 0.5 for (r, c) in true_track]
    ax.plot(xs, ys, color=GOLD, linewidth=4, solid_capstyle="round", zorder=3)

    draw_marker(ax, START, GREEN)
    draw_marker(ax, GOAL, WALL_RED)

    draw_robot(ax, believed_pos, hdg, facecolor=BELIEVED_GRAY, outline_only=True,
                linestyle=(0, (4, 2)), zorder=6)
    draw_robot(ax, true_pos, hdg, zorder=8)

    # The error vector itself -- the quantity every suite in this study
    # is measured on. Label placed perpendicular to the vector so it
    # clears both robot footprints.
    ax.annotate("", xy=(true_pos[1], true_pos[0]), xytext=(believed_pos[1], believed_pos[0]),
                arrowprops=dict(arrowstyle="-|>", color="#c0392b", linewidth=2.6,
                                 shrinkA=10, shrinkB=10), zorder=9)
    mid = ((true_pos[0] + believed_pos[0]) / 2, (true_pos[1] + believed_pos[1]) / 2)
    ax.annotate("pose error", xy=(mid[1], mid[0]), xytext=(mid[1] - 5.2, mid[0] + 3.0),
                arrowprops=dict(arrowstyle="-", color="#c0392b", linewidth=1.4,
                                 shrinkA=2, shrinkB=4),
                color="#c0392b", fontsize=12, fontweight="bold", ha="center", va="center",
                zorder=9,
                bbox=dict(boxstyle="round,pad=0.28", facecolor="white",
                           edgecolor="#c0392b", linewidth=1.2, alpha=0.95))
    ax.text(believed_pos[1] + 1.9, believed_pos[0] - 1.6, "believes\nit's here",
            color="#5c5c66", fontsize=10.5, fontweight="bold", ha="center", va="center",
            linespacing=1.2, zorder=9,
            bbox=dict(boxstyle="round,pad=0.22", facecolor="white", edgecolor="none", alpha=0.8))

    ax.set_title("Pose error — the robot is where it shouldn't be",
                  fontsize=13.5, fontweight="bold", pad=10, loc="left")


def panel_obstacle_error(ax, assumed_grid, truth_grid, path):
    # Diffed on the RAW grid cells, not the eroded ones: obstacle drift
    # flips individual cells, and a lone flipped cell is exactly what
    # morphological erosion is designed to delete -- diffing the eroded
    # sets would silently drop every obstacle this panel exists to show.
    # The eroded set is still used for the unchanged game elements, so
    # the baseline field still reads as real elements rather than as the
    # planner's inflation halo.
    def raw_obstacles(grid):
        return {(r, c) for r in range(grid.size) for c in range(grid.size)
                if grid.is_obstacle(r, c)}

    assumed_raw, truth_raw = raw_obstacles(assumed_grid), raw_obstacles(truth_grid)
    added = truth_raw - assumed_raw          # real obstacle the map never had
    removed = assumed_raw - truth_raw        # map said obstacle, field is clear

    baseline = eroded_obstacle_cells(assumed_grid)

    draw_field_base(ax, assumed_grid, obstacle_cells=[])

    for (r, c) in baseline - removed:
        draw_obstacle_cell(ax, r, c)
    for (r, c) in removed & baseline:
        draw_obstacle_cell(ax, r, c, facecolor="none", edgecolor="#6c7480",
                            linewidth=1.6, linestyle=(0, (3, 2)), alpha=0.95)
    on_route = set(path)
    for (r, c) in added:
        draw_obstacle_cell(ax, r, c, facecolor="#c0392b", edgecolor="#7d1f13", linewidth=1.4)
    # Call out the one that actually blocks the planned route -- the
    # others are map error the robot happens to get away with.
    for (r, c) in sorted(added & on_route):
        ax.annotate("route is blocked here",
                    xy=(c + 0.5, r + 0.5), xytext=(c + 3.4, r - 2.6),
                    arrowprops=dict(arrowstyle="-|>", color="#c0392b", linewidth=2.2,
                                     shrinkA=2, shrinkB=6),
                    color="#c0392b", fontsize=11.5, fontweight="bold", ha="left", va="center",
                    zorder=9,
                    bbox=dict(boxstyle="round,pad=0.28", facecolor="white",
                               edgecolor="#c0392b", linewidth=1.2, alpha=0.95))
        break

    draw_path(ax, path, color=ROUTE_BLUE, linewidth=3.6)
    draw_marker(ax, START, GREEN)
    draw_marker(ax, GOAL, WALL_RED)

    # Park the robot a few steps SHORT of the blocking cell rather than
    # at a fixed fraction of the route: at a fixed fraction it lands on
    # top of the blocker and hides the one obstacle the panel is about.
    blockers = [i for i, cell in enumerate(path) if cell in added]
    if blockers:
        idx = max(1, blockers[0] - 4)
    else:
        idx = max(1, min(len(path) - 2, round(len(path) * OBST_PROGRESS)))
    cell = path[idx]
    hdg = heading_deg(path[max(idx - 1, 0)], path[min(idx + 1, len(path) - 1)])
    draw_robot(ax, (cell[0] + 0.5, cell[1] + 0.5), hdg, zorder=8)

    ax.set_title("Obstacle error — the field isn't what the map said",
                  fontsize=13.5, fontweight="bold", pad=10, loc="left")


def main():
    grid = build_grid(LAYOUT)
    path, _, _ = astar(grid, START, GOAL)
    if path is None:
        raise RuntimeError("no path on the assumed grid")

    # Both MAP-side deviations, start drift held at 0 so the panel shows
    # only the failure mode it claims to: obstacle drift (cells flipped
    # either way) plus the unplanned blocker, which by construction
    # lands on an interior cell of the assumed route -- that on-route
    # obstacle is the whole point of the panel, and drift alone rarely
    # happens to put one there.
    # Seed 6 chosen because it puts exactly ONE added obstacle on the
    # planned route (checked across seeds, not assumed) -- the blocker
    # is probabilistic, so most seeds put none there and a few put two,
    # neither of which reads as cleanly at a glance.
    truth_grid, _actual_start = generate_ground_truth(
        grid, START, GOAL, variance_level=0.55, seed=6,
        start_drift_scale=0.0, obstacle_drift_scale=1.0, blocker_scale=1.0,
    )

    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(15, 8.2))
    panel_pose_error(ax_left, grid, path)
    panel_obstacle_error(ax_right, grid, truth_grid, path)

    fig.suptitle("Two independent ways an autonomous run fails — and no single sensor fixes both",
                  fontsize=16, fontweight="bold", x=0.012, ha="left", y=0.98)

    legend_y = 0.055
    fig.text(0.012, legend_y,
              "Left:   solid robot = true position   ·   gray dashed = where the robot believes it is   ·   "
              "blue = planned route   ·   gold = true track",
              fontsize=10.5, color="#333333")
    fig.text(0.012, legend_y - 0.032,
              "Right:  solid gray = obstacle on both map and field   ·   dashed outline = map said obstacle, field is clear   ·   "
              "red = real obstacle the map never had",
              fontsize=10.5, color="#333333")

    fig.tight_layout(rect=[0, 0.085, 1, 0.955])
    fig.savefig(OUT, dpi=170, facecolor="white")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
