import random

from nav.grid import Grid
from nav.algorithms import find_path
from nav.obstacles import MovingObstacle

START, GOAL = (0, 0), (0, 9)
# Sits right on the straight-line path from START to GOAL.
OBSTACLE_START = (0, 4)
FRAMES_PER_MOVE = 5
TOTAL_FRAMES = 40
# MovingObstacle now random-walks instead of bouncing between two fixed
# cells (see nav/obstacles.py) -- seeded so this demo's printed output
# stays reproducible run to run.
SEED = 0


if __name__ == "__main__":
    grid = Grid()
    grid.place_start(*START)
    grid.place_goal(*GOAL)

    obstacle = MovingObstacle(OBSTACLE_START, period_ms=FRAMES_PER_MOVE, rng=random.Random(SEED))
    # FIX (Step 1 audit): the old bounce version never marked its own
    # starting cell as an obstacle at construction -- it stayed invisible
    # and unplannable-around until its first move. place() closes that gap.
    obstacle.place(grid)
    obstacle.start(0)

    path, _, reason, _ = find_path(grid, "astar", START, GOAL)
    print(f"initial path ({reason}): {path}\n")

    for frame in range(TOTAL_FRAMES):
        prev_pos = obstacle.position
        if not obstacle.tick(grid, frame):
            continue
        if obstacle.position == prev_pos:
            # Due to move but had no free cardinal neighbor -- stayed put.
            continue

        print(f"frame {frame}: obstacle moved to {obstacle.position}")

        if path is not None and obstacle.position not in path:
            print("  path still clear, no replan needed")
            continue

        print(f"  path blocked, replanning from {START}...")
        path, _, reason, _ = find_path(grid, "astar", START, GOAL)
        if path is None:
            print(f"  no path found (reason={reason})")
        else:
            print(f"  new path: {path}")

    print(f"\nfinal path: {path}")
