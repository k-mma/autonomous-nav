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


def run_demo():
    """Same replan-on-block loop the old __main__ block ran directly, now
    returning what happened instead of just printing it. `violations`
    counts any replan that produced a path still passing through the
    obstacle's current cell -- the one outcome that would mean the
    replanning policy described in this module's docstring is broken."""
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

    replans = 0
    violations = 0
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
        replans += 1
        if path is None:
            print(f"  no path found (reason={reason})")
        else:
            print(f"  new path: {path}")
            if obstacle.position in path:
                violations += 1

    print(f"\nfinal path: {path}")
    return {"final_path": path, "replans": replans, "violations": violations}


# --- pytest entry points --------------------------------------------------

def test_replanning_never_routes_through_the_obstacles_current_cell():
    result = run_demo()
    assert result["violations"] == 0


def test_the_obstacle_starting_on_the_route_forces_at_least_one_replan():
    result = run_demo()
    assert result["replans"] >= 1


if __name__ == "__main__":
    run_demo()
