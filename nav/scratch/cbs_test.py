"""
Does nav/cbs.py actually produce a genuinely conflict-free joint plan,
not just "a plan," for more than two agents -- and does raising the
agent count past what pybullet_multi_robot_main.py's two-robot policy
can handle actually work? No UI: builds the same kind of 4-way
intersection (a 3-wide street crossing in the middle of four buildings)
pybullet_cbs_main.py drives robots through, runs CBS for 2, 3, and 4
agents on it, and checks every returned solution against
`first_conflict` directly -- the same conflict checker CBS itself uses
internally, so a passing result here means CBS's own termination
condition is trustworthy, not just that it returned *something*.
"""
from nav.cbs import cbs, first_conflict, solution_cost
from nav.grid import Grid

CENTER = 12
STREET_HALF_WIDTH = 1
BUILDING_SPANS = (range(5, 11), range(14, 20))


def build_intersection_grid():
    """Same layout as pybullet_multi_robot_main.py's build_intersection_grid
    (kept as a standalone copy here so this correctness test has no
    dependency on a pybullet-importing script) -- four square buildings,
    one per quadrant, leaving a 3-cell-wide "plus" of open street."""
    grid = Grid()
    street_lo, street_hi = CENTER - STREET_HALF_WIDTH, CENTER + STREET_HALF_WIDTH
    for row_span in BUILDING_SPANS:
        for col_span in BUILDING_SPANS:
            for row in row_span:
                if street_lo <= row <= street_hi:
                    continue
                for col in col_span:
                    if street_lo <= col <= street_hi:
                        continue
                    grid.cells[row][col] = Grid.OBSTACLE
    return grid


# One pair of start/goal per compass arm, each heading to the opposite
# arm -- a real 4-way crossing, all in contention at the same time.
ARMS = {
    "N->S": ((2, CENTER), (22, CENTER)),
    "S->N": ((22, CENTER), (2, CENTER)),
    "W->E": ((CENTER, 2), (CENTER, 22)),
    "E->W": ((CENTER, 22), (CENTER, 2)),
}


def check_solution(label, grid, agents):
    result = cbs(grid, agents)
    if result is None:
        print(f"{label}: FAILED to find a solution for {len(agents)} agents")
        return False

    conflict = first_conflict(result)
    ok = conflict is None
    cost = solution_cost(result)
    print(f"{label}: {len(agents)} agents, solved, cost={cost}, "
          f"conflict-free={ok}" + ("" if ok else f" -- CONFLICT: {conflict}"))

    for agent, (start, goal) in agents.items():
        path = result[agent]
        assert path[0] == start, f"{agent}: path doesn't start at {start}"
        assert path[-1] == goal, f"{agent}: path doesn't end at {goal}"
        for (r1, c1), (r2, c2) in zip(path, path[1:]):
            dr, dc = abs(r1 - r2), abs(c1 - c2)
            assert (dr, dc) in ((0, 0), (0, 1), (1, 0)), f"{agent}: illegal move {(r1,c1)}->{(r2,c2)}"
            assert not grid.is_obstacle(r2, c2), f"{agent}: stepped onto an obstacle at {(r2,c2)}"
    return ok


if __name__ == "__main__":
    grid = build_intersection_grid()

    all_ok = True
    for n, arm_names in [(2, ["N->S", "W->E"]), (3, ["N->S", "W->E", "S->N"]), (4, list(ARMS))]:
        agents = {name: ARMS[name] for name in arm_names}
        ok = check_solution(f"{n}-agent", grid, agents)
        all_ok = all_ok and ok

    print("\nALL CONFLICT-FREE" if all_ok else "\nSOME SOLUTIONS HAD CONFLICTS -- BUG")
