from nav.grid import Grid
from nav.algorithms import find_path
from nav.obstacles import MovingObstacle

START, GOAL = (0, 0), (0, 9)
# Sits right on the straight-line path from START to GOAL.
OBSTACLE_A, OBSTACLE_B = (0, 4), (0, 5)
FRAMES_PER_MOVE = 5
TOTAL_FRAMES = 40


if __name__ == "__main__":
    grid = Grid()
    grid.place_start(*START)
    grid.place_goal(*GOAL)

    obstacle = MovingObstacle(OBSTACLE_A, OBSTACLE_B, period_ms=FRAMES_PER_MOVE)
    obstacle.start(0)

    path, _, reason, _ = find_path(grid, "astar", START, GOAL)
    print(f"initial path ({reason}): {path}\n")

    for frame in range(TOTAL_FRAMES):
        if not obstacle.tick(grid, frame):
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
