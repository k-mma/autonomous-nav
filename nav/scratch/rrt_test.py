import random

from nav.grid import Grid
from nav.rrt import rrt

START, GOAL = (0, 0), (20, 20)


def build_grid():
    grid = Grid()
    for row in range(5, 20):
        grid.cells[row][10] = Grid.OBSTACLE
    return grid


# --- pytest entry points --------------------------------------------------
# RRT is randomized (no fixed neighbor graph like Dijkstra/A*, see the
# README's "The algorithms, briefly"), so this passes a seeded rng --
# rrt() exposes one for exactly this reason -- instead of asserting on an
# exact path/tree shape. 20/20 seeds (0-19) found a path on this layout in
# an ad hoc check before writing this, so seed=0 finding one isn't a
# coin flip; what's actually being verified is that the returned path is
# real: it starts/ends where asked and never crosses the wall.

def test_finds_a_path_that_avoids_the_wall():
    grid = build_grid()
    path, tree_nodes, came_from = rrt(grid, START, GOAL, rng=random.Random(0))
    assert path is not None, "RRT found no path within max_iters for seed=0 -- was RRT_MAX_ITERS lowered?"
    assert path[0] == START
    assert path[-1] == GOAL  # rrt() snaps the final node to GOAL once within goal_radius
    assert len(tree_nodes) > 1
    for cell in path:
        assert not grid.is_obstacle(*cell), f"path crosses the wall at {cell}"


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
