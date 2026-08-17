"""Shared field-drawing helper for the poster figures that render the
FTC field (figure1, figure9). Same visual language as
generate_figure2_ftc_field.py's own draw_field (colors, wall
convention, eroded obstacle cells) so every field panel across the
poster reads as one set rather than as separate drawings.

figure2 keeps its own copy of that drawing code rather than importing
this: it predates this module and is the only two-panel figure, so
leaving it alone keeps a working figure working. If figure2 is ever
reworked, folding it onto these helpers is the obvious cleanup.
"""
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

from ftc.field import _footprint_corners, eroded_obstacle_cells  # noqa: E402

# Shared with generate_figure2_ftc_field.py -- keep in sync.
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
# Believed-pose gray, matching pygame_app/ftc_viz/field_view.py's
# BELIEVED_ROBOT_COLOR = (150, 150, 158).
BELIEVED_GRAY = "#96969e"


def heading_deg(prev_cell, next_cell):
    """atan2(d_row, d_col), in degrees -- the heading convention every
    consumer in ftc/ uses."""
    dr = next_cell[0] - prev_cell[0]
    dc = next_cell[1] - prev_cell[1]
    return math.degrees(math.atan2(dr, dc))


def draw_field_base(ax, grid, obstacle_cells=None, tile_lines=True):
    """Field floor, tile seams, perimeter walls, and obstacles. Returns
    nothing; callers layer paths/robots on top."""
    size = grid.size
    ax.set_facecolor(FIELD_BG)
    ax.set_xlim(-0.6, size + 0.6)
    ax.set_ylim(size + 0.6, -0.6)
    ax.set_aspect("equal")
    ax.axis("off")

    ax.add_patch(Rectangle((0, 0), size, size, facecolor=CELL_BG, edgecolor="none", zorder=0))
    if tile_lines:
        for i in range(4, size, 4):
            ax.axvline(i, color=GRID_LINE, linewidth=0.9, zorder=1)
            ax.axhline(i, color=GRID_LINE, linewidth=0.9, zorder=1)

    wall_w = 5
    ax.plot([0, 0], [0, size], color=WALL_BLUE, linewidth=wall_w, solid_capstyle="butt", zorder=5)
    ax.plot([size, size], [0, size], color=WALL_RED, linewidth=wall_w, solid_capstyle="butt", zorder=5)
    ax.plot([0, size], [0, 0], color=WALL_BLACK, linewidth=wall_w, solid_capstyle="butt", zorder=5)
    ax.plot([0, size], [size, size], color=WALL_BLACK, linewidth=wall_w, solid_capstyle="butt", zorder=5)

    cells = eroded_obstacle_cells(grid) if obstacle_cells is None else obstacle_cells
    for (r, c) in cells:
        draw_obstacle_cell(ax, r, c)


def draw_obstacle_cell(ax, r, c, facecolor=OBSTACLE, edgecolor="none",
                        linewidth=0, linestyle="solid", zorder=2, alpha=1.0):
    pad = 0.12
    ax.add_patch(FancyBboxPatch(
        (c + pad, r + pad), 1 - 2 * pad, 1 - 2 * pad,
        boxstyle="round,pad=0,rounding_size=0.08",
        facecolor=facecolor, edgecolor=edgecolor, linewidth=linewidth,
        linestyle=linestyle, zorder=zorder, alpha=alpha,
    ))


def draw_robot(ax, position, heading, facecolor=ROBOT_BLUE, edgecolor="white",
                outline_only=False, zorder=6, linewidth=2, linestyle="solid"):
    """Robot at its real 18in x 18in footprint, rotated to `heading` --
    the same corner math ftc/field.py's collision check uses.
    `position` is a (row, col) cell CENTER in continuous coords."""
    corners, fwd_axis, right_axis = _footprint_corners(position, heading)
    poly_xy = [(c, r) for (r, c) in corners]
    if outline_only:
        # White underlay first: a gray dashed outline over a gray field
        # floor is nearly invisible at poster scale, and this footprint
        # is half the point of the figure it appears in.
        ax.add_patch(plt.Polygon(poly_xy, closed=True, facecolor="white", alpha=0.45,
                                  edgecolor="none", zorder=zorder - 0.1))
        ax.add_patch(plt.Polygon(poly_xy, closed=True, facecolor="none",
                                  edgecolor=facecolor, linewidth=3.0,
                                  linestyle=linestyle, zorder=zorder))
    else:
        ax.add_patch(plt.Polygon(poly_xy, closed=True, facecolor=facecolor,
                                  edgecolor=edgecolor, linewidth=linewidth, zorder=zorder))
    # Heading triangle.
    row, col = position
    tip = (col + 0.55 * fwd_axis[1], row + 0.55 * fwd_axis[0])
    left = (col - 0.28 * right_axis[1] - 0.15 * fwd_axis[1],
             row - 0.28 * right_axis[0] - 0.15 * fwd_axis[0])
    right = (col + 0.28 * right_axis[1] - 0.15 * fwd_axis[1],
              row + 0.28 * right_axis[0] - 0.15 * fwd_axis[0])
    tri_color = facecolor if outline_only else "white"
    ax.add_patch(plt.Polygon([tip, left, right], closed=True, facecolor=tri_color,
                              edgecolor="none", zorder=zorder + 1))


def draw_path(ax, path, color=ROUTE_BLUE, linewidth=4, linestyle="solid", zorder=3, alpha=1.0):
    xs = [c + 0.5 for (r, c) in path]
    ys = [r + 0.5 for (r, c) in path]
    ax.plot(xs, ys, color=color, linewidth=linewidth, linestyle=linestyle,
            solid_joinstyle="round", solid_capstyle="round", zorder=zorder, alpha=alpha)


def draw_marker(ax, cell, color, zorder=4, linewidth=2.6):
    r, c = cell
    ax.add_patch(Rectangle((c + 0.15, r + 0.15), 0.7, 0.7, facecolor="none",
                            edgecolor=color, linewidth=linewidth, zorder=zorder))
