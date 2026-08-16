"""
Builds a nav.grid.Grid at FTC scale from a parameterized field layout --
deliberately not tied to any one season's game, since the actual game
elements change every year and hardcoding one would date this project
the moment the season rotates. A layout is just a list of rectangular
obstacles in inches (FieldElement) plus optional AprilTag mount sites;
a team can drop in their own season's surveyed layout as data without
touching any code here.

The obstacles get hard-inflated by the robot's radius (a Minkowski-sum
obstacle growth, applied here rather than in nav/ since it's this
module that first has a robot with a real footprint) before the point-
robot planner in nav/algorithms.py ever sees the grid -- an 18in robot
in a 6in cell is 3 cells wide, and planning it as a point (nav/'s
default assumption everywhere else) is optimistic: a route squeezed
directly between two obstacles one cell apart would actually clip the
robot's corners in reality.

This is the standard configuration-space (C-space) construction from
the motion-planning literature (Lozano-Perez, 1983), not an ad hoc
trick: the robot's true configuration is 2D (its center's (row, col)),
and its obstacle region in that configuration space is exactly the
Minkowski sum of every workspace obstacle with the robot's footprint
reflected through its own reference point -- which, for the disc-like
footprint a Chebyshev-radius inflation approximates here, collapses to
"grow every obstacle by the footprint's radius." Planning a point
robot through free *C-space* is provably equivalent to planning the
real, extended-footprint robot through free *workspace* -- which is
exactly what lets every existing nav/ algorithm (all written for a
point robot) run completely unmodified against a real 3-cell-wide one.
"""
import math
from dataclasses import dataclass, field as dataclass_field

from nav.config import COST_INFLUENCE_RADIUS, COST_MAX_EXTRA
from nav.grid import Grid

from ftc.config import CELL_SIZE_IN, FIELD_SIZE_IN, FTC_GRID_SIZE, ROBOT_RADIUS_CELLS, ROBOT_SIZE_IN


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
    its footprint overlapping a real obstacle. This is the actual
    C-space-obstacle construction (see module docstring): `to_mark` is
    the Minkowski sum of the real (workspace) obstacles with the
    robot's footprint, and the grid this function returns is the free/
    obstacle partition of *configuration* space, not workspace, even
    though it's stored in the exact same (row, col) Grid representation
    workspace obstacles were. Computed from the original obstacle set
    (collected up front) rather than growing the grid in place
    cell-by-cell, so inflation doesn't cascade past `radius` from any
    real obstacle."""
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


def _hard_inflate_border(grid, radius):
    """The field's own perimeter wall is a workspace obstacle exactly
    like any FieldElement, for the same C-space reason _hard_inflate
    exists (see module docstring) -- it's just one this module never
    stores as an explicit obstacle rectangle, since it's implicit in
    the grid's own edges. Without this, a free cell right on row/col 0
    (or the far edge) reads as perfectly legal for the robot's CENTER,
    but that cell is where an 18in-wide (3-cell) robot's footprint
    starts hanging a cell and a half off the field -- i.e. through the
    real wall. Blocks every cell within `radius` of any edge, the same
    "would a robot centered here have any part of its footprint outside
    a real boundary" test _hard_inflate applies to obstacles, so the
    point-robot planner treats the field edge with the same respect it
    already gives every interior obstacle."""
    size = grid.size
    for row in range(size):
        for col in range(size):
            if row < radius or col < radius or row >= size - radius or col >= size - radius:
                if grid.cells[row][col] == Grid.FREE:
                    grid.cells[row][col] = Grid.OBSTACLE


def build_grid(layout_name_or_layout, robot_radius_cells=ROBOT_RADIUS_CELLS,
                soft_influence_radius=COST_INFLUENCE_RADIUS, soft_max_extra=COST_MAX_EXTRA,
                diagonal=True, size=FTC_GRID_SIZE, cell_size_in=CELL_SIZE_IN):
    """
    Build an FTC-scale Grid from a layout name (a key of LAYOUTS) or a
    FieldLayout directly. Obstacle cells come straight from the
    layout's elements; then every obstacle is hard-inflated by
    `robot_radius_cells` (see _hard_inflate), and the field's own outer
    perimeter gets the identical treatment (see _hard_inflate_border) --
    so the point-robot planner downstream is correct for a real
    3-cell-wide robot, not just a point, against BOTH the game elements
    and the wall around them. A soft costmap (nav.grid.Grid.compute_
    cost_map, the same machinery pygame_app's K toggle uses) is layered
    on top of that hard boundary so a planner still prefers extra
    clearance beyond the minimum required, instead of grazing every
    hard-inflated edge.

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
    _hard_inflate_border(grid, robot_radius_cells)

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


def eroded_obstacle_cells(grid, radius=ROBOT_RADIUS_CELLS):
    """The actual game-element footprint a real robot could touch --
    approximately undoes this module's own Minkowski-sum hard-inflation
    (every obstacle grown by `radius` so a point-robot plan is safe for
    the real 3-cell-wide robot -- see `_hard_inflate`'s docstring) via
    morphological erosion, the standard inverse of dilation. `radius`
    should match whatever `build_grid` call produced `grid` -- pass the
    same value, not just the default, for a grid built with a
    non-default `robot_radius_cells`.

    Two callers, one obstacle set. `pygame_app/ftc_viz/field_view.py`
    draws exactly these cells (not the inflated ones) so a robot's
    drawn footprint reads as touching real game elements, not the
    planner's own safety buffer. `ftc/match.py`'s `footprint_overlaps_
    cells` collision check (see that function) tests the robot's actual
    ROTATED footprint against this same set, not the inflated grid --
    the two staying the same set is what makes "does this look like it's
    overlapping on screen" and "does this count as a collision" the same
    question, not two independently-maintained approximations of it.

    Center-cell inflation-clearance alone (the ORIGINAL reason this
    function exists) only ever proves an AXIS-ALIGNED 3-cell-wide
    footprint stays clear: a free center cell means no real (eroded)
    obstacle cell is within `radius` cells of it, so no eroded obstacle
    cell can be within `radius` of an axis-aligned footprint's own edge
    either. That guarantee does NOT extend to a footprint held at a
    non-cardinal heading (e.g. mid-diagonal-travel, or a mecanum
    drivetrain holding a fixed heading unrelated to its direction of
    travel) -- a square rotated 45 degrees reaches its own half-width
    times sqrt(2) (~2.12 cells at this project's 1.5-cell half-width)
    straight out from center, well past the ~1.5-cell axis-aligned
    clearance the inflation actually proves. `footprint_overlaps_cells`
    is the check that closes that gap; this function is what both it and
    the visualizer draw their notion of "real obstacle" from."""
    size = grid.size

    def is_obstacle(r, c):
        return grid.is_valid(r, c) and grid.cells[r][c] == Grid.OBSTACLE

    survivors = set()
    for row in range(size):
        for col in range(size):
            if not is_obstacle(row, col):
                continue
            if all(is_obstacle(row + dr, col + dc)
                    for dr in range(-radius, radius + 1) for dc in range(-radius, radius + 1)):
                survivors.add((row, col))
    return survivors


# Half-width, in grid cells, of the robot's actual ROBOT_SIZE_IN x
# ROBOT_SIZE_IN square footprint -- 1.5 cells at this project's default
# 6in cells, i.e. a full cell more than ROBOT_RADIUS_CELLS's rounded-
# down 1, and the number footprint_overlaps_cells rotates by heading
# rather than treating as a fixed axis-aligned bounding box.
ROBOT_HALF_WIDTH_CELLS = ROBOT_SIZE_IN / CELL_SIZE_IN / 2.0

# How far a footprint_overlaps_cells search needs to look from the
# robot's center cell to find every real-obstacle cell its rotated
# footprint could possibly touch, at ANY heading -- the worst case is a
# corner reaching straight out along a grid axis, sqrt(2) times the
# half-width (see ROBOT_HALF_WIDTH_CELLS), rounded up to a whole cell
# with one extra cell of slack rather than trusting a boundary float
# comparison to land exactly on an integer.
_FOOTPRINT_SEARCH_RADIUS_CELLS = int(math.ceil(ROBOT_HALF_WIDTH_CELLS * math.sqrt(2))) + 1


def _footprint_corners(position, heading_deg, half=ROBOT_HALF_WIDTH_CELLS):
    """The 4 (row, col) corners of the robot's actual footprint, in grid-
    cell units (not pixels -- see pygame_app/ftc_viz/field_view.py's
    `_robot_corners` for the pixel-space twin this mirrors), plus the
    footprint's own two edge-normal directions (forward, right) --
    `footprint_overlaps_cells` needs those as candidate separating
    axes. Uses the same directly-verified heading_deg convention,
    atan2(d_row, d_col), every other consumer in this project does:
    (cos, sin) = (d_col, d_row)."""
    row, col = position
    rad = math.radians(heading_deg)
    fwd_row, fwd_col = math.sin(rad), math.cos(rad)
    right_row, right_col = fwd_col, -fwd_row
    corners = [
        (row + half * fwd_row - half * right_row, col + half * fwd_col - half * right_col),
        (row + half * fwd_row + half * right_row, col + half * fwd_col + half * right_col),
        (row - half * fwd_row + half * right_row, col - half * fwd_col + half * right_col),
        (row - half * fwd_row - half * right_row, col - half * fwd_col - half * right_col),
    ]
    return corners, (fwd_row, fwd_col), (right_row, right_col)


def _project_onto_axis(points, axis):
    values = [p[0] * axis[0] + p[1] * axis[1] for p in points]
    return min(values), max(values)


# Two continuous ranges that merely TOUCH (share an endpoint, zero-width
# overlap) count as separated, not colliding -- otherwise the ordinary,
# already-proven-safe axis-aligned case (a free center cell's nearest
# real obstacle sits exactly ROBOT_HALF_WIDTH_CELLS away, edge to edge,
# at the worst-case Chebyshev-2 placement inflation guarantees) would
# spuriously flag as a collision on floating-point boundary noise alone.
_TOUCH_EPS = 1e-9


def _ranges_separated(a_lo, a_hi, b_lo, b_hi):
    return a_hi <= b_lo + _TOUCH_EPS or b_hi <= a_lo + _TOUCH_EPS


def footprint_overlaps_cells(position, heading_deg, real_obstacle_cells, grid_size,
                               half=ROBOT_HALF_WIDTH_CELLS):
    """Does the robot's ACTUAL footprint -- an 18in x 18in square
    (`ROBOT_HALF_WIDTH_CELLS`), centered at `position` and rotated to
    `heading_deg`, not an axis-aligned bounding box -- overlap any real
    (eroded, `eroded_obstacle_cells`) obstacle cell within reach.

    This is the check `_hard_inflate`'s own center-cell clearance
    doesn't cover (see `eroded_obstacle_cells`'s docstring): a robot
    whose CENTER sits on a cell the planner considers safe can still
    have its rotated body clip a real obstacle when it isn't facing a
    cardinal direction, which happens on every diagonal step this
    project's 8-directional grid allows, and on any step a holonomic
    drivetrain takes while holding a heading that doesn't match its
    direction of travel. `ftc/match.py` calls this once per successful
    step, at the heading the robot would actually be holding on
    arrival, and treats a hit exactly like arriving on an inflated-
    obstacle cell -- a rejected move, not a cosmetic footnote.

    Separating-axis test between the rotated footprint and each nearby
    axis-aligned unit-cell square: two convex quadrilaterals are
    disjoint iff their projections onto SOME candidate axis don't
    overlap, and for two rectangles the only candidate axes that can
    ever separate them are each rectangle's own two edge normals -- the
    grid's row/col axes for the cell, and the footprint's own forward/
    right axes for the rotated square. Only cells within
    `_FOOTPRINT_SEARCH_RADIUS_CELLS` of `position` that are actually in
    `real_obstacle_cells` get the full test; every other cell is
    rejected by a cheap set-membership check first."""
    row, col = position
    corners, fwd_axis, right_axis = _footprint_corners(position, heading_deg, half)
    reach = _FOOTPRINT_SEARCH_RADIUS_CELLS
    for dr in range(-reach, reach + 1):
        for dc in range(-reach, reach + 1):
            cell = (row + dr, col + dc)
            if cell not in real_obstacle_cells:
                continue
            cr, cc = cell
            if not (0 <= cr < grid_size and 0 <= cc < grid_size):
                continue
            cell_corners = [
                (cr - 0.5, cc - 0.5), (cr - 0.5, cc + 0.5),
                (cr + 0.5, cc - 0.5), (cr + 0.5, cc + 0.5),
            ]
            separated = False
            for axis in ((1.0, 0.0), (0.0, 1.0), fwd_axis, right_axis):
                a_lo, a_hi = _project_onto_axis(corners, axis)
                b_lo, b_hi = _project_onto_axis(cell_corners, axis)
                if _ranges_separated(a_lo, a_hi, b_lo, b_hi):
                    separated = True
                    break
            if not separated:
                return True
    return False
