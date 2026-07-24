"""
Does elevation-aware routing (Grid.elevation / Grid.elevation_aware, see
nav/grid.py: get_neighbors) actually stay admissible under A*, or does
charging extra for climbing quietly break the Manhattan/octile proof the
way an inflated heuristic does (see nav/scratch/heuristic_experiment.py)?

The argument (spelled out in WRITEUPS.md) is that the elevation surcharge
is always >= 0 and only ever added on top of the existing baseline step
cost -- exactly the same shape of argument that already justifies
compute_cost_map's terrain weighting staying admissible. This empirically
checks that argument the same way nav/scratch/diagonal_heuristic_test.py
checks octile: run Dijkstra (always optimal, no heuristic involved) and
A* on the identical elevation-aware grid, and confirm A* never comes back
with a higher-cost path than Dijkstra did, across enough random trials
and cost-model combinations (elevation alone, elevation + cost-map,
elevation + diagonal + cost-map together) that a break would show up if
there were one.
"""
import random

from nav.algorithms import astar, dijkstra, path_cost
from nav.grid import Grid
from nav.heuristics import manhattan, octile

GRID_SIZE = 25
DENSITY = 0.15
TRIALS = 100


def random_elevation_grid(rng, diagonal=False, cost_map=False, max_elevation=5.0):
    grid = Grid()
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            if rng.random() < DENSITY:
                grid.cells[row][col] = Grid.OBSTACLE
    free = [(r, c) for r in range(GRID_SIZE) for c in range(GRID_SIZE) if grid.cells[r][c] == Grid.FREE]
    if len(free) < 2:
        return None, None, None
    start, goal = rng.sample(free, 2)
    grid.place_start(*start)
    grid.place_goal(*goal)

    grid.diagonal = diagonal
    grid.cost_map_enabled = cost_map
    grid.refresh_cost_map()
    grid.elevation_aware = True
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            grid.elevation[row][col] = rng.uniform(0, max_elevation)

    return grid, grid.start, grid.goal


def run_suite(label, diagonal, cost_map, heuristic, seed):
    rng = random.Random(seed)
    checked = 0
    suboptimal = 0
    worst = 0.0
    for trial in range(TRIALS):
        grid, start, goal = random_elevation_grid(rng, diagonal=diagonal, cost_map=cost_map)
        if grid is None:
            continue
        d_path, _, _ = dijkstra(grid, start, goal)
        a_path, _, _ = astar(grid, start, goal, heuristic)
        if d_path is None and a_path is None:
            continue
        if (d_path is None) != (a_path is None):
            print(f"  [{label}] trial {trial}: EXISTENCE MISMATCH -- dijkstra found a path, astar didn't (or vice versa)")
            suboptimal += 1
            continue
        checked += 1
        d_cost, a_cost = path_cost(grid, d_path), path_cost(grid, a_path)
        if a_cost > d_cost + 1e-9:
            suboptimal += 1
            worst = max(worst, a_cost - d_cost)
            print(f"  [{label}] trial {trial}: astar {a_cost:.3f} > dijkstra {d_cost:.3f} "
                  f"(worse by {a_cost - d_cost:.3f}) -- INADMISSIBLE")
    print(f"{label}: {checked - suboptimal}/{checked} optimal ({suboptimal} suboptimal"
          + (f", worst overshoot {worst:.3f}" if suboptimal else "") + ")")
    return suboptimal


if __name__ == "__main__":
    total_suboptimal = 0
    total_suboptimal += run_suite("elevation only (4-directional, manhattan)",
                                   diagonal=False, cost_map=False, heuristic=manhattan, seed=1)
    total_suboptimal += run_suite("elevation + cost-map (4-directional, manhattan)",
                                   diagonal=False, cost_map=True, heuristic=manhattan, seed=2)
    total_suboptimal += run_suite("elevation + diagonal (octile)",
                                   diagonal=True, cost_map=False, heuristic=octile, seed=3)
    total_suboptimal += run_suite("elevation + diagonal + cost-map (octile)",
                                   diagonal=True, cost_map=True, heuristic=octile, seed=4)

    print(f"\n{'ADMISSIBLE: astar matched dijkstra in every case.' if total_suboptimal == 0 else f'{total_suboptimal} SUBOPTIMAL CASES -- see above'}")
