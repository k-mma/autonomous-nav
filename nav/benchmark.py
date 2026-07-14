import csv
import random
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from nav.algorithms import dijkstra, astar
from nav.config import GRID_SIZE
from nav.grid import Grid

TRIALS = 20
MIN_DENSITY = 0.10
MAX_DENSITY = 0.35
MAX_ATTEMPTS_PER_TRIAL = 50
OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

random.seed(42)


def random_grid(density):
    grid = Grid()
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            if random.random() < density:
                grid.cells[row][col] = Grid.OBSTACLE

    free_cells = [
        (r, c)
        for r in range(GRID_SIZE)
        for c in range(GRID_SIZE)
        if grid.cells[r][c] == Grid.FREE
    ]
    if len(free_cells) < 2:
        return None, None, None
    start, goal = random.sample(free_cells, 2)
    return grid, start, goal


def time_run(algo, grid, start, goal):
    t0 = time.perf_counter()
    path, explored, _ = algo(grid, start, goal)
    elapsed_ms = (time.perf_counter() - t0) * 1000
    return path, explored, elapsed_ms


def run_trial(trial_num):
    for _ in range(MAX_ATTEMPTS_PER_TRIAL):
        density = random.uniform(MIN_DENSITY, MAX_DENSITY)
        grid, start, goal = random_grid(density)
        if grid is None:
            continue
        d_path, d_explored, d_ms = time_run(dijkstra, grid, start, goal)
        if d_path is None:
            continue
        a_path, a_explored, a_ms = time_run(astar, grid, start, goal)
        return {
            "trial": trial_num,
            "obstacle_density": round(density, 3),
            "path_length": len(d_path) - 1,
            "dijkstra_cells_explored": len(d_explored),
            "astar_cells_explored": len(a_explored),
            "dijkstra_runtime_ms": round(d_ms, 4),
            "astar_runtime_ms": round(a_ms, 4),
        }
    raise RuntimeError(f"trial {trial_num}: no solvable grid after {MAX_ATTEMPTS_PER_TRIAL} attempts")


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def plot_results(rows, path):
    rows = sorted(rows, key=lambda r: r["obstacle_density"])
    densities = [r["obstacle_density"] for r in rows]
    dijkstra_cells = [r["dijkstra_cells_explored"] for r in rows]
    astar_cells = [r["astar_cells_explored"] for r in rows]
    gap_pct = [100 * (d - a) / d if d else 0 for d, a in zip(dijkstra_cells, astar_cells)]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 8), sharex=True)

    ax1.plot(densities, dijkstra_cells, "o-", label="Dijkstra")
    ax1.plot(densities, astar_cells, "o-", label="A*")
    ax1.set_ylabel("Cells explored")
    ax1.set_title(f"Dijkstra vs A* over {len(rows)} random {GRID_SIZE}x{GRID_SIZE} grids")
    ax1.legend()

    ax2.bar(densities, gap_pct, width=0.006, color="tab:green")
    ax2.axhline(0, color="black", linewidth=0.8)
    ax2.set_xlabel("Obstacle density")
    ax2.set_ylabel("A* advantage (%)")

    fig.tight_layout()
    fig.savefig(path, dpi=150)


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)
    rows = [run_trial(i + 1) for i in range(TRIALS)]

    write_csv(rows, OUTPUT_DIR / "results.csv")
    plot_results(rows, OUTPUT_DIR / "comparison.png")

    total_d = sum(r["dijkstra_cells_explored"] for r in rows)
    total_a = sum(r["astar_cells_explored"] for r in rows)
    print(f"Wrote {len(rows)} trials to {OUTPUT_DIR / 'results.csv'}")
    print(f"Plot saved to {OUTPUT_DIR / 'comparison.png'}")
    print(
        f"Total cells explored -- Dijkstra: {total_d}, A*: {total_a} "
        f"({100 * (total_d - total_a) / total_d:.1f}% fewer for A*)"
    )
