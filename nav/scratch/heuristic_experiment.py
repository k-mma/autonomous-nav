import random

from nav.algorithms import astar, dijkstra, path_cost
from nav.config import GRID_SIZE
from nav.grid import Grid
from nav.heuristics import euclidean, manhattan, scaled

TRIALS = 30
DENSITY = 0.35
HEURISTICS = [
    ("manhattan", manhattan),
    ("euclidean", euclidean),
    ("inadmissible_1.5x", scaled(manhattan, 1.5)),
    ("inadmissible_3x", scaled(manhattan, 3.0)),
]

# A perfect maze (recursive backtracking, no loops) has exactly one route
# between any two cells -- there's nothing for a bad heuristic to get
# wrong. Suboptimality needs multiple candidate routes of different
# lengths, which is what a scattered obstacle field gives you.


def random_grid(rng):
    grid = Grid()
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            if rng.random() < DENSITY:
                grid.cells[row][col] = Grid.OBSTACLE
    return grid


def run_trial(seed, verbose):
    rng = random.Random(seed)
    grid = random_grid(rng)

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
            print(f"  {name:18s} cost={cost}  explored={len(explored):4d}{flag}")
    if verbose:
        print()
    return suboptimal_by_name


if __name__ == "__main__":
    counts = {name: 0 for name, _ in HEURISTICS}
    trial_count = 0

    for seed in range(TRIALS):
        # only print the trials where something actually goes wrong --
        # with 30 trials most are identical across every heuristic
        result = run_trial(seed, verbose=False)
        if not result:
            continue
        trial_count += 1
        if any(result.values()):
            run_trial(seed, verbose=True)
        for name, was_suboptimal in result.items():
            counts[name] += was_suboptimal

    print(f"Suboptimal path rate over {trial_count} random {int(DENSITY*100)}%-density grids:")
    for name, _ in HEURISTICS:
        print(f"  {name:18s} {counts[name]}/{trial_count}")
