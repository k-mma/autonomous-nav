import random

from nav.algorithms import astar, dijkstra, path_cost
from nav.config import GRID_SIZE
from nav.grid import Grid
from nav.heuristics import manhattan, octile

TRIALS = 30
DENSITY = 0.25
HEURISTICS = [("manhattan", manhattan), ("octile", octile)]


def random_diagonal_grid(rng):
    grid = Grid()
    grid.diagonal = True
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            if rng.random() < DENSITY:
                grid.cells[row][col] = Grid.OBSTACLE
    return grid


def run_trial(seed, verbose):
    rng = random.Random(seed)
    grid = random_diagonal_grid(rng)

    free = [
        (r, c)
        for r in range(len(grid.cells))
        for c in range(len(grid.cells))
        if grid.cells[r][c] == Grid.FREE
    ]
    if len(free) < 2:
        return {}
    start, goal = rng.sample(free, 2)

    optimal_path, _, _ = dijkstra(grid, start, goal)
    if optimal_path is None:
        return {}
    optimal_cost = path_cost(grid, optimal_path)

    if verbose:
        print(f"--- seed {seed} | start={start} goal={goal} | optimal cost: {optimal_cost:.3f} ---")

    suboptimal_by_name = {}
    for name, heuristic in HEURISTICS:
        path, explored, _ = astar(grid, start, goal, heuristic=heuristic)
        cost = path_cost(grid, path) if path else None
        is_suboptimal = cost is not None and cost > optimal_cost + 1e-9
        suboptimal_by_name[name] = is_suboptimal
        if verbose:
            flag = "  <-- SUBOPTIMAL" if is_suboptimal else ""
            cost_str = f"{cost:.3f}" if cost is not None else None
            print(f"  {name:10s} cost={cost_str}  explored={len(explored):4d}{flag}")
    if verbose:
        print()
    return suboptimal_by_name


# --- pytest entry points --------------------------------------------------
# octile is admissible for 8-directional movement (never overestimates true
# cost) so it must find the optimal path on every trial -- 0/N suboptimal
# isn't a statistical target, it's a correctness guarantee. manhattan is
# NOT admissible here (it overestimates a diagonal step's true cost), so
# unlike octile, seeing it actually produce a suboptimal path (not just
# "could in theory") is the point of this file -- confirmed empirically at
# TRIALS=30, seeds 0..29: manhattan was suboptimal on 2 of them.

def test_octile_is_always_optimal_being_admissible():
    counts = {name: 0 for name, _ in HEURISTICS}
    trial_count = 0
    for seed in range(TRIALS):
        result = run_trial(seed, verbose=False)
        if not result:
            continue
        trial_count += 1
        for name, was_suboptimal in result.items():
            counts[name] += was_suboptimal
    assert trial_count > 0
    assert counts["octile"] == 0


def test_manhattan_is_sometimes_suboptimal_being_inadmissible():
    counts = {name: 0 for name, _ in HEURISTICS}
    for seed in range(TRIALS):
        result = run_trial(seed, verbose=False)
        for name, was_suboptimal in result.items():
            counts[name] += was_suboptimal
    assert counts["manhattan"] > 0, (
        "manhattan produced zero suboptimal paths over TRIALS trials -- either the grid/density "
        "changed enough to stop exercising the inadmissible case, or heuristics.manhattan itself changed"
    )


if __name__ == "__main__":
    counts = {name: 0 for name, _ in HEURISTICS}
    trial_count = 0

    for seed in range(TRIALS):
        result = run_trial(seed, verbose=False)
        if not result:
            continue
        trial_count += 1
        if any(result.values()):
            run_trial(seed, verbose=True)
        for name, was_suboptimal in result.items():
            counts[name] += was_suboptimal

    print(f"Suboptimal path rate over {trial_count} random 8-directional "
          f"{int(DENSITY*100)}%-density grids:")
    for name, _ in HEURISTICS:
        print(f"  {name:10s} {counts[name]}/{trial_count}")
