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
