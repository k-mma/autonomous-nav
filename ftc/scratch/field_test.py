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
"""
from nav.algorithms import astar
from nav.grid import Grid

from ftc.config import FTC_GRID_SIZE, ROBOT_FOOTPRINT_CELLS, ROBOT_RADIUS_CELLS
from ftc.field import FieldElement, FieldLayout, LAYOUTS, build_grid, tag_sites_for


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
    g = build_grid("corridor")
    path, _, _ = astar(g, (0, 0), (g.size - 1, g.size - 1))
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


if __name__ == "__main__":
    checks = [
        check_grid_shape(),
        check_inflation_grows_obstacles(),
        check_corridor_stays_passable(),
        check_narrow_gap_gets_sealed(),
        check_default_tag_sites(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
