"""
Builds a nav.grid.Grid at FTC scale from a parameterized field layout --
deliberately not tied to any one season's game, since the actual game
elements change every year and hardcoding one would date this project
the moment the season rotates. A layout is just a list of rectangular
obstacles in inches (FieldElement) plus optional AprilTag mount sites;
a team can drop in their own season's surveyed layout as data without
touching any code here.

The obstacles get hard-inflated by the robot's radius (nav.config's
Minkowski-sum trick, applied here rather than in nav/ since it's this
module that first has a robot with a real footprint) before the point-
robot planner in nav/algorithms.py ever sees the grid -- an 18in robot
in a 6in cell is 3 cells wide, and planning it as a point (nav/'s
default assumption everywhere else) is optimistic: a route squeezed
directly between two obstacles one cell apart would actually clip the
robot's corners in reality. Hard-inflating first and then planning a
point robot on the inflated grid is exactly equivalent to planning the
real footprint on the original grid, and lets every existing nav/
algorithm run completely unmodified.
"""
import math
from dataclasses import dataclass, field as dataclass_field

from nav.config import COST_INFLUENCE_RADIUS, COST_MAX_EXTRA
from nav.grid import Grid

from ftc.config import CELL_SIZE_IN, FIELD_SIZE_IN, FTC_GRID_SIZE, ROBOT_RADIUS_CELLS


@dataclass
class FieldElement:
    """One rectangular obstacle, in inches, positioned by its top-left
    corner (x_in, y_in) -- x along columns, y along rows, matching the
    grid's (row, col) indexing once converted."""
    x_in: float
    y_in: float
    w_in: float
    h_in: float


@dataclass
class TagSite:
    """An AprilTag mount point: (x_in, y_in) in field inches, and
    `heading_deg` -- the direction the tag *faces* (the outward normal
    of the wall/backdrop it's mounted on), in the same atan2(d_row,
    d_col) convention ftc/sensors.py uses for robot heading. A tag can
    only be read from roughly in front of it, never edge-on or from
    behind -- see ftc/sensors.py's AprilTagSuite for the FOV/range/line-
    of-sight check that uses this."""
    x_in: float
    y_in: float
    heading_deg: float


@dataclass
class FieldLayout:
    elements: list = dataclass_field(default_factory=list)
    # Defaults to the 4 wall-midpoint sites in DEFAULT_TAG_SITES if not
    # given -- most layouts don't need a custom tag placement, only a
    # custom obstacle arrangement.
    tags: list = None


def _default_tag_sites():
    """One AprilTag at the midpoint of each of the field's 4 walls,
    facing inward -- a domain-neutral generalization of how FTC seasons
    actually mount tags (on a backdrop or wall along the field
    perimeter), without hardcoding one season's specific backdrop
    location."""
    mid = FIELD_SIZE_IN / 2
    return [
        TagSite(mid, 0.0, 90.0),                 # top wall, faces +row (into field)
        TagSite(mid, FIELD_SIZE_IN, -90.0),       # bottom wall, faces -row
        TagSite(0.0, mid, 0.0),                   # left wall, faces +col
        TagSite(FIELD_SIZE_IN, mid, 180.0),       # right wall, faces -col
    ]


DEFAULT_TAG_SITES = _default_tag_sites()


# Three generic layouts, not tied to any specific season's game --
# element sizes/positions are representative of typical FTC field
# clutter (scoring structures, sample/prop zones) without claiming to
# model one exact game.

LAYOUTS = {
    # Minimal obstruction: two isolated elements off any direct route,
    # useful as a near-open-field baseline.
    "sparse": FieldLayout(elements=[
        FieldElement(96.0, 18.0, 18.0, 18.0),
        FieldElement(30.0, 96.0, 18.0, 18.0),
    ]),
    # A dense scatter of smaller elements across the whole field --
    # every route has to thread between several of them.
    "cluttered": FieldLayout(elements=[
        FieldElement(12.0, 12.0, 14.0, 14.0),
        FieldElement(60.0, 18.0, 12.0, 12.0),
        FieldElement(110.0, 24.0, 16.0, 16.0),
        FieldElement(24.0, 60.0, 12.0, 20.0),
        FieldElement(66.0, 66.0, 18.0, 18.0),
        FieldElement(108.0, 78.0, 14.0, 14.0),
        FieldElement(18.0, 108.0, 20.0, 12.0),
        FieldElement(78.0, 114.0, 16.0, 16.0),
    ]),
    # Two long walls with a single gap between them -- forces every
    # route through one deliberately narrow passage, the layout where
    # the robot's 3-cell footprint (vs. a point robot) matters the most.
    "corridor": FieldLayout(elements=[
        FieldElement(0.0, 66.0, 60.0, 12.0),
        FieldElement(84.0, 66.0, 60.0, 12.0),
    ]),
}


def _in_to_cell_range(pos_in, size_in, cell_size_in, grid_size):
    """Inclusive [lo, hi] cell index range covered by a span starting at
    `pos_in` and extending `size_in`, clamped to the grid."""
    lo = int(pos_in // cell_size_in)
    hi = int(math.ceil((pos_in + size_in) / cell_size_in)) - 1
    lo = max(0, min(lo, grid_size - 1))
    hi = max(0, min(hi, grid_size - 1))
    return lo, hi


def _hard_inflate(grid, radius):
    """Grow every obstacle cell by Chebyshev `radius` -- a free cell
    counts as blocked if a robot centered there would have any part of
    its footprint overlapping a real obstacle. Computed from the
    original obstacle set (collected up front) rather than growing the
    grid in place cell-by-cell, so inflation doesn't cascade past
    `radius` from any real obstacle."""
    size = grid.size
    sources = [(r, c) for r in range(size) for c in range(size) if grid.cells[r][c] == Grid.OBSTACLE]
    to_mark = set()
    for row, col in sources:
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                r, c = row + dr, col + dc
                if grid.is_valid(r, c) and grid.cells[r][c] == Grid.FREE:
                    to_mark.add((r, c))
    for r, c in to_mark:
        grid.cells[r][c] = Grid.OBSTACLE


def build_grid(layout_name_or_layout, robot_radius_cells=ROBOT_RADIUS_CELLS,
                soft_influence_radius=COST_INFLUENCE_RADIUS, soft_max_extra=COST_MAX_EXTRA,
                diagonal=True, size=FTC_GRID_SIZE, cell_size_in=CELL_SIZE_IN):
    """
    Build an FTC-scale Grid from a layout name (a key of LAYOUTS) or a
    FieldLayout directly. Obstacle cells come straight from the
    layout's elements; then every obstacle is hard-inflated by
    `robot_radius_cells` (see _hard_inflate) so the point-robot planner
    downstream is correct for a real 3-cell-wide robot, not just a
    point. A soft costmap (nav.grid.Grid.compute_cost_map, the same
    machinery pygame_app's K toggle uses) is layered on top of that
    hard boundary so a planner still prefers extra clearance beyond the
    minimum required, instead of grazing every hard-inflated edge.

    `diagonal=True` by default -- FTC drivetrains are commonly holonomic
    (mecanum), so 8-directional movement is a more honest model of what
    the robot can actually do than a 4-directional grid.

    Returns the Grid. Does not place a start/goal -- callers pick those
    (nav/uncertainty_benchmark.py's own convention: sample random free
    cells per trial, not bake fixed ones into the layout).
    """
    layout = LAYOUTS[layout_name_or_layout] if isinstance(layout_name_or_layout, str) else layout_name_or_layout

    grid = Grid(size=size)
    grid.diagonal = diagonal

    for el in layout.elements:
        row_lo, row_hi = _in_to_cell_range(el.y_in, el.h_in, cell_size_in, size)
        col_lo, col_hi = _in_to_cell_range(el.x_in, el.w_in, cell_size_in, size)
        for r in range(row_lo, row_hi + 1):
            for c in range(col_lo, col_hi + 1):
                grid.cells[r][c] = Grid.OBSTACLE

    _hard_inflate(grid, robot_radius_cells)

    grid.cost_map_enabled = True
    grid.compute_cost_map(influence_radius=soft_influence_radius, max_extra=soft_max_extra)

    return grid


def tag_sites_for(layout_name_or_layout):
    layout = LAYOUTS[layout_name_or_layout] if isinstance(layout_name_or_layout, str) else layout_name_or_layout
    return layout.tags if layout.tags else DEFAULT_TAG_SITES


def in_to_cell(x_in, y_in, cell_size_in=CELL_SIZE_IN):
    """A single (x_in, y_in) field point -> (row, col) grid cell --
    used to place AprilTag sites on the same grid the layout builds."""
    return (int(y_in // cell_size_in), int(x_in // cell_size_in))
