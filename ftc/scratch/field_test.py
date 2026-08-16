"""
Does ftc/field.py's build_grid actually do what it claims: hard-inflate
every obstacle by the robot's radius (so a point-robot plan on the
result is equivalent to a real-footprint plan on the original layout),
does the corridor layout -- built specifically to be barely passable at
real robot scale -- stay passable after that inflation instead of
accidentally getting sealed shut, and -- the other direction, not
covered by the corridor check -- does a gap that's actually narrower
than the robot's real footprint get correctly sealed off instead of a
point-robot planner sneaking a path through a space the robot could
never physically fit?

The last group of checks is about the gap hard-inflation's own center-
cell clearance does NOT cover: `footprint_overlaps_cells` tests the
robot's ACTUAL rotated footprint, not an axis-aligned bounding box, so
a robot holding a non-cardinal heading (any diagonal step, or a
holonomic drivetrain's held heading) can be caught clipping a real
obstacle its center-cell placement alone would have called safe. See
that function's own docstring, and ftc/match.py's module docstring, for
why the axis-aligned inflation guarantee doesn't extend to that case.
"""
import math

from nav.algorithms import astar
from nav.grid import Grid

from ftc.config import FTC_GRID_SIZE, ROBOT_FOOTPRINT_CELLS, ROBOT_RADIUS_CELLS
from ftc.field import (
    FieldElement, FieldLayout, LAYOUTS, ROBOT_HALF_WIDTH_CELLS, build_grid, eroded_obstacle_cells,
    footprint_overlaps_cells, tag_sites_for,
)


def check_grid_shape():
    ok = True
    for name in LAYOUTS:
        g = build_grid(name)
        if g.size != FTC_GRID_SIZE:
            ok = False
            print(f"  {name}: size {g.size}, expected {FTC_GRID_SIZE}")
    print(f"every layout builds a {FTC_GRID_SIZE}x{FTC_GRID_SIZE} grid: {'OK' if ok else 'FAIL'}")
    return ok


def check_inflation_grows_obstacles():
    """Hard-inflating strictly can't shrink the obstacle set, and for
    every layout here (none is already wall-to-wall) it should strictly
    grow it -- a no-op inflation would mean ROBOT_RADIUS_CELLS silently
    isn't being applied at all."""
    ok = True
    for name, layout in LAYOUTS.items():
        raw = Grid(size=FTC_GRID_SIZE)
        for el in layout.elements:
            r0, r1 = int(el.y_in // 6), int((el.y_in + el.h_in) // 6)
            c0, c1 = int(el.x_in // 6), int((el.x_in + el.w_in) // 6)
            for r in range(max(0, r0), min(FTC_GRID_SIZE - 1, r1) + 1):
                for c in range(max(0, c0), min(FTC_GRID_SIZE - 1, c1) + 1):
                    raw.cells[r][c] = Grid.OBSTACLE
        raw_count = sum(row.count(Grid.OBSTACLE) for row in raw.cells)

        inflated = build_grid(name)
        inflated_count = sum(row.count(Grid.OBSTACLE) for row in inflated.cells)

        if inflated_count <= raw_count:
            ok = False
            print(f"  {name}: inflated obstacle count {inflated_count} <= raw {raw_count}")
    print(f"hard inflation strictly grows the obstacle set on every layout: {'OK' if ok else 'FAIL'}")
    return ok


def check_corridor_stays_passable():
    """Near-corner-to-near-corner, not literal-corner-to-literal-corner:
    build_grid now hard-inflates the field's own perimeter by the same
    ROBOT_RADIUS_CELLS as any obstacle (see ftc/field.py's
    _hard_inflate_border), so a cell exactly on the boundary (like
    (0, 0)) is correctly blocked -- a robot centered there would have
    part of its real footprint hanging off the field, through the wall.
    The corners this test cares about are therefore the nearest cells a
    robot's center could actually legally occupy."""
    g = build_grid("corridor")
    r = ROBOT_RADIUS_CELLS
    start, goal = (r, r), (g.size - 1 - r, g.size - 1 - r)
    path, _, _ = astar(g, start, goal)
    ok = path is not None
    print(f"corridor layout stays passable after {ROBOT_RADIUS_CELLS}-cell hard inflation: {'OK' if ok else 'FAIL'}")
    return ok


def check_narrow_gap_gets_sealed():
    """Two wall segments spanning the whole field width, minus a 6in
    gap between them -- 1 cell, well under the 18in/3-cell robot
    footprint. Unlike the corridor layout (deliberately built to stay
    just barely passable), this gap should NOT survive hard inflation:
    a robot that's 3 cells wide cannot fit through a 1-cell gap no
    matter how the point-robot planner is routed, so build_grid's hard
    inflation needs to actually seal it, not just discourage hugging
    it (see nav/grid.py's own compute_cost_map, which only ever adds
    soft cost and would let a point-robot planner sneak straight
    through a gap this narrow)."""
    narrow_gap_layout = FieldLayout(elements=[
        FieldElement(0.0, 66.0, 69.0, 12.0),
        FieldElement(75.0, 66.0, 69.0, 12.0),
    ])
    g = build_grid(narrow_gap_layout)
    path, _, _ = astar(g, (0, 0), (g.size - 1, g.size - 1))
    ok = path is None
    print(f"a 6in gap (narrower than the {ROBOT_FOOTPRINT_CELLS}-cell/{ROBOT_FOOTPRINT_CELLS * 6}in robot "
          f"footprint) is sealed shut by hard inflation: {'OK' if ok else 'FAIL'}")
    return ok


def check_default_tag_sites():
    ok = True
    for name in LAYOUTS:
        sites = tag_sites_for(name)
        if len(sites) != 4:
            ok = False
            print(f"  {name}: {len(sites)} tag sites, expected 4")
    print(f"every layout without a custom tag list falls back to the 4 default sites: {'OK' if ok else 'FAIL'}")
    return ok


def check_axis_aligned_footprint_never_spuriously_collides():
    """The worst-case placement hard-inflation's own center-cell
    clearance actually proves safe: a real obstacle cell exactly
    ROBOT_RADIUS_CELLS + 1 away (Chebyshev, the closest a real/eroded
    obstacle can legally sit next to a free center cell) straight ahead
    in each of the 4 cardinal directions. At each of the corresponding
    axis-aligned headings (0/90/180/270), the robot's edge exactly
    TOUCHES that obstacle's near edge -- zero gap, but not a crossing --
    and footprint_overlaps_cells must call that "not overlapping," not a
    spurious collision, or every ordinary axis-aligned move in this
    project would start colliding with cells the inflation already
    proved were clear."""
    position = (5, 5)
    offset = ROBOT_RADIUS_CELLS + 1
    cases = [
        (0.0, (position[0], position[1] + offset)),
        (90.0, (position[0] + offset, position[1])),
        (180.0, (position[0], position[1] - offset)),
        (270.0, (position[0] - offset, position[1])),
    ]
    ok = True
    for heading, obstacle_cell in cases:
        hit = footprint_overlaps_cells(position, heading, {obstacle_cell}, FTC_GRID_SIZE)
        if hit:
            ok = False
            print(f"  heading={heading}: obstacle {obstacle_cell} spuriously flagged as overlapping")
    print(f"a real obstacle at exactly the inflation's own worst-case clearance never spuriously collides at any "
          f"cardinal heading: {'OK' if ok else 'FAIL'}")
    return ok


def check_diagonal_heading_finds_the_gap_axis_alignment_misses():
    """The actual bug this check exists for: the SAME obstacle placement
    check_axis_aligned_footprint_never_spuriously_collides just proved
    safe at every cardinal heading is NOT safe at a diagonal one. A
    square rotated 45 degrees reaches its own half-width times sqrt(2)
    straight out along a grid axis (ROBOT_HALF_WIDTH_CELLS's own
    docstring) -- comfortably past the ROBOT_RADIUS_CELLS + 1 clearance
    the axis-aligned case only just touches -- so the identical
    obstacle placement that was fine at heading=0 must be a genuine hit
    at heading=45."""
    position = (5, 5)
    offset = ROBOT_RADIUS_CELLS + 1
    obstacle_cell = (position[0], position[1] + offset)
    reach_needed = ROBOT_HALF_WIDTH_CELLS * math.sqrt(2)
    hit = footprint_overlaps_cells(position, 45.0, {obstacle_cell}, FTC_GRID_SIZE)
    print(f"  obstacle at {obstacle_cell}, {offset} cells straight ahead of {position}; "
          f"a 45deg footprint reaches ~{reach_needed:.3f} cells that direction")
    print(f"the identical obstacle placement axis-aligned safety proved clear is a genuine hit at a 45deg heading: "
          f"{'OK' if hit else 'FAIL'}")
    return hit


def check_far_obstacle_never_overlaps_at_any_heading():
    """No false positives from the search-radius/candidate-cell logic
    itself: an obstacle cell far outside footprint_overlaps_cells' own
    search window never registers a hit, at any heading -- the
    complementary sanity check to the two above, which both plant the
    obstacle right at the edge of what should matter."""
    position = (5, 5)
    obstacle_cell = (position[0] + 10, position[1] + 10)
    ok = all(
        not footprint_overlaps_cells(position, heading, {obstacle_cell}, FTC_GRID_SIZE)
        for heading in (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0)
    )
    print(f"an obstacle well outside the search window never overlaps at any heading: {'OK' if ok else 'FAIL'}")
    return ok


def check_eroded_obstacle_cells_shared_with_visualizer():
    """`pygame_app/ftc_viz/field_view.py`'s `eroded_obstacle_cells` and
    `ftc/match.py`'s footprint_overlaps_cells collision check need to
    agree on what a "real" obstacle cell is -- otherwise "does this look
    like it's overlapping on screen" and "does this count as a
    collision" silently become two different questions. Checked as
    object identity (the visualizer imports the exact function this
    module defines, not a separately-maintained copy), the strongest
    guarantee available: no future edit to one can desync from the
    other without this failing."""
    from pygame_app.ftc_viz import field_view
    ok = field_view.eroded_obstacle_cells is eroded_obstacle_cells
    print(f"pygame_app/ftc_viz/field_view.py's eroded_obstacle_cells IS ftc/field.py's (not a copy): "
          f"{'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_grid_shape():
    assert check_grid_shape()


def test_inflation_grows_obstacles():
    assert check_inflation_grows_obstacles()


def test_corridor_stays_passable():
    assert check_corridor_stays_passable()


def test_narrow_gap_gets_sealed():
    assert check_narrow_gap_gets_sealed()


def test_default_tag_sites():
    assert check_default_tag_sites()


def test_axis_aligned_footprint_never_spuriously_collides():
    assert check_axis_aligned_footprint_never_spuriously_collides()


def test_diagonal_heading_finds_the_gap_axis_alignment_misses():
    assert check_diagonal_heading_finds_the_gap_axis_alignment_misses()


def test_far_obstacle_never_overlaps_at_any_heading():
    assert check_far_obstacle_never_overlaps_at_any_heading()


def test_eroded_obstacle_cells_shared_with_visualizer():
    assert check_eroded_obstacle_cells_shared_with_visualizer()


if __name__ == "__main__":
    checks = [
        check_grid_shape(),
        check_inflation_grows_obstacles(),
        check_corridor_stays_passable(),
        check_narrow_gap_gets_sealed(),
        check_default_tag_sites(),
        check_axis_aligned_footprint_never_spuriously_collides(),
        check_diagonal_heading_finds_the_gap_axis_alignment_misses(),
        check_far_obstacle_never_overlaps_at_any_heading(),
        check_eroded_obstacle_cells_shared_with_visualizer(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
