from nav.grid import Grid
from nav.algorithms import find_path


START, GOAL = (0, 0), (5, 5)


def wall_off(grid, cell):
    row, col = cell
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        r, c = row + dr, col + dc
        if grid.is_valid(r, c):
            grid.cells[r][c] = Grid.OBSTACLE


def run_case(name, grid, start, goal):
    path, explored, reason, _ = find_path(grid, "dijkstra", start, goal)
    print(f"--- {name} ---")
    print(f"start={start} goal={goal} -> path={path}")
    print(f"reason={reason} cells_explored={len(explored)}\n")
    return path, explored, reason


# --- pytest entry points --------------------------------------------------
# find_path's documented reason codes (nav/algorithms.py's validate_endpoints
# and find_path docstrings) are exact strings, so each edge case has one
# right answer, not just "didn't crash."

def test_start_on_obstacle_is_rejected():
    grid = Grid()
    grid.cells[START[0]][START[1]] = Grid.OBSTACLE
    path, _, reason = run_case("start on obstacle", grid, START, GOAL)
    assert reason == "start_blocked"
    assert path is None


def test_goal_on_obstacle_is_rejected():
    grid = Grid()
    grid.cells[GOAL[0]][GOAL[1]] = Grid.OBSTACLE
    path, _, reason = run_case("goal on obstacle", grid, START, GOAL)
    assert reason == "goal_blocked"
    assert path is None


def test_walled_off_goal_reports_no_path():
    grid = Grid()
    wall_off(grid, GOAL)
    path, _, reason = run_case("no path (goal walled off)", grid, START, GOAL)
    assert reason == "no_path"
    assert path is None


def test_start_equals_goal_is_a_trivial_one_cell_path():
    grid = Grid()
    path, _, reason = run_case("start == goal", grid, START, START)
    assert reason == "same_cell"
    assert path == [START]


if __name__ == "__main__":
    grid = Grid()
    grid.cells[START[0]][START[1]] = Grid.OBSTACLE
    run_case("start on obstacle", grid, START, GOAL)

    grid = Grid()
    grid.cells[GOAL[0]][GOAL[1]] = Grid.OBSTACLE
    run_case("goal on obstacle", grid, START, GOAL)

    grid = Grid()
    wall_off(grid, GOAL)
    run_case("no path (goal walled off)", grid, START, GOAL)

    grid = Grid()
    run_case("start == goal", grid, START, START)
