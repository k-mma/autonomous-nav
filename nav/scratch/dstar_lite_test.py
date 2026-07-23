"""
Does D* Lite (nav/dstar_lite.py) actually produce the same routes as
re-running A* from scratch, at every step of a live-replanning scenario
-- not just once, but continuously as the grid keeps changing under it?
Two checks, no UI:

1. One-shot: DStarLite's very first plan on a fresh grid should have the
   exact same path *cost* as `astar` (not necessarily the identical cell
   sequence -- ties can break differently -- but the same length).
2. Incremental: simulate a robot advancing one cell at a time while a
   random cell's obstacle state flips *every step* (a stress-test version
   of nav/obstacles.py's bouncing MovingObstacle) and D* Lite repairs its
   plan after each change. At every single step, its resulting path cost
   is compared against a completely fresh `astar` call from the robot's
   exact current position on the exact current grid -- the ground truth
   an incremental planner has to match to be trustworthy at all.
"""
import random

from nav.algorithms import astar, path_cost
from nav.dstar_lite import DStarLite
from nav.grid import Grid

GRID_SIZE = 25
DENSITY = 0.2
ONE_SHOT_TRIALS = 40
INCREMENTAL_TRIALS = 15
INCREMENTAL_STEPS = 15


def random_grid(rng, density=DENSITY):
    grid = Grid()
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            if rng.random() < density:
                grid.cells[row][col] = Grid.OBSTACLE
    free = [(r, c) for r in range(GRID_SIZE) for c in range(GRID_SIZE) if grid.cells[r][c] == Grid.FREE]
    if len(free) < 2:
        return None, None, None
    start, goal = rng.sample(free, 2)
    return grid, start, goal


def check_one_shot():
    rng = random.Random(11)
    checked, mismatches = 0, 0
    for _ in range(ONE_SHOT_TRIALS):
        grid, start, goal = random_grid(rng)
        if grid is None:
            continue
        a_path, _, _ = astar(grid, start, goal)
        d_path = DStarLite(grid, start, goal).extract_path()
        checked += 1
        if (a_path is None) != (d_path is None):
            mismatches += 1
            print(f"  MISMATCH (existence): astar={a_path is not None} dstar={d_path is not None}")
            continue
        if a_path is None:
            continue
        if abs(path_cost(grid, a_path) - path_cost(grid, d_path)) > 1e-9:
            mismatches += 1
            print(f"  MISMATCH (cost): astar={path_cost(grid, a_path):.3f} "
                  f"dstar={path_cost(grid, d_path):.3f}")
    print(f"one-shot vs astar: {checked - mismatches}/{checked} match")
    return mismatches


def check_incremental():
    rng = random.Random(99)
    checked, mismatches = 0, 0
    for trial in range(INCREMENTAL_TRIALS):
        grid, start, goal = random_grid(rng, density=0.15)
        if grid is None:
            continue
        a_path, _, _ = astar(grid, start, goal)
        if a_path is None:
            continue

        planner = DStarLite(grid, start, goal)
        current = start

        for step in range(INCREMENTAL_STEPS):
            d_path = planner.extract_path()
            if d_path is None or len(d_path) < 2:
                break
            current = d_path[1]
            planner.move_start(current)

            candidates = [
                (r, c) for r in range(GRID_SIZE) for c in range(GRID_SIZE)
                if (r, c) not in (current, goal)
            ]
            cell = rng.choice(candidates)
            grid.cells[cell[0]][cell[1]] = (
                Grid.FREE if grid.cells[cell[0]][cell[1]] == Grid.OBSTACLE else Grid.OBSTACLE
            )
            planner.update_edge_costs([cell])
            planner.compute_shortest_path()

            fresh_path, _, _ = astar(grid, current, goal)
            d_path_now = planner.extract_path()
            checked += 1

            if (fresh_path is None) != (d_path_now is None):
                mismatches += 1
                print(f"  trial {trial} step {step}: MISMATCH (existence) "
                      f"astar={fresh_path is not None} dstar={d_path_now is not None}")
                continue
            if fresh_path is None:
                continue
            fresh_cost = path_cost(grid, fresh_path)
            d_cost = path_cost(grid, d_path_now)
            if abs(fresh_cost - d_cost) > 1e-9:
                mismatches += 1
                print(f"  trial {trial} step {step}: MISMATCH (cost) "
                      f"astar={fresh_cost:.3f} dstar={d_cost:.3f}")

    print(f"incremental repair vs fresh astar, checked every step: {checked - mismatches}/{checked} match")
    return mismatches


if __name__ == "__main__":
    print(f"Checking {ONE_SHOT_TRIALS} random {GRID_SIZE}x{GRID_SIZE} grids (one-shot)...")
    m1 = check_one_shot()
    print(f"\nChecking {INCREMENTAL_TRIALS} trials x {INCREMENTAL_STEPS} incremental "
          f"obstacle-flip-and-move steps each...")
    m2 = check_incremental()

    total_mismatches = m1 + m2
    print(f"\n{'ALL MATCH -- D* Lite agrees with fresh astar every time.' if total_mismatches == 0 else f'{total_mismatches} MISMATCHES FOUND'}")
