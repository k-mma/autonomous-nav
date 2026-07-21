from nav.grid import Grid
from nav.rrt import rrt

START, GOAL = (0, 0), (20, 20)


def build_grid():
    grid = Grid()
    for row in range(5, 20):
        grid.cells[row][10] = Grid.OBSTACLE
    return grid


if __name__ == "__main__":
    grid = build_grid()
    path, tree_nodes, came_from = rrt(grid, START, GOAL)

    print(f"tree grew to {len(tree_nodes)} nodes, {len(came_from)} edges")

    if path is None:
        print("no path found within max_iters -- try again (RRT is randomized)")
    else:
        print(f"path found, {len(path)} waypoints:")
        for cell in path:
            print(f"  {cell}")
