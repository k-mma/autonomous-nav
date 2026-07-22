"""
Does runtime actually scale the way the algorithms' theory predicts?
nav/benchmark.py holds grid size fixed and varies obstacle density; this
holds density fixed and varies grid size (20x20, 50x50, 100x100,
200x200) instead -- the direct answer to "how does this scale to a
bigger grid?"

Reuses nav.grid.Grid's `size` parameter (added for exactly this) and the
same dijkstra/astar/rrt functions, completely unmodified.
"""
import csv
import random
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from nav.algorithms import dijkstra, astar
from nav.grid import Grid
from nav.rrt import rrt

SIZES = [20, 50, 100, 200]
TRIALS_PER_SIZE = 8
DENSITY = 0.2
MAX_ATTEMPTS_PER_TRIAL = 50
OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"


def random_grid(size, rng):
    grid = Grid(size=size)
    for row in range(size):
        for col in range(size):
            if rng.random() < DENSITY:
                grid.cells[row][col] = Grid.OBSTACLE
    free = [(r, c) for r in range(size) for c in range(size) if grid.cells[r][c] == Grid.FREE]
    if len(free) < 2:
        return None, None, None
    start, goal = rng.sample(free, 2)
    return grid, start, goal


def time_call(fn, *args, **kwargs):
    t0 = time.perf_counter()
    result = fn(*args, **kwargs)
    return result, (time.perf_counter() - t0) * 1000


def run_trial(size, trial_num):
    rng = random.Random(size * 1000 + trial_num)
    for attempt in range(MAX_ATTEMPTS_PER_TRIAL):
        grid, start, goal = random_grid(size, rng)
        if grid is None:
            continue
        (d_path, d_explored, _), d_ms = time_call(dijkstra, grid, start, goal)
        if d_path is None:
            continue
        (a_path, a_explored, _), a_ms = time_call(astar, grid, start, goal)

        # RRT's step size and iteration budget need to scale with the
        # grid, or it would need proportionally far more steps to cross
        # a larger space -- see WRITEUPS.md for what happens without this.
        rrt_step = max(2, size // 12)
        rrt_iters = size * 30
        rrt_rng = random.Random(size * 1000 + trial_num)
        (r_path, r_explored, _), r_ms = time_call(
            rrt, grid, start, goal, max_iters=rrt_iters, step_size=rrt_step, rng=rrt_rng
        )

        return {
            "grid_size": size,
            "trial": trial_num,
            "total_cells": size * size,
            "path_length": len(d_path) - 1,
            "dijkstra_cells_explored": len(d_explored),
            "astar_cells_explored": len(a_explored),
            "rrt_tree_nodes": len(r_explored),
            "dijkstra_ms": round(d_ms, 4),
            "astar_ms": round(a_ms, 4),
            "rrt_ms": round(r_ms, 4),
            "rrt_found_path": r_path is not None,
        }
    raise RuntimeError(f"size {size} trial {trial_num}: no solvable grid after {MAX_ATTEMPTS_PER_TRIAL} attempts")


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def average(rows, key, size):
    matching = [r[key] for r in rows if r["grid_size"] == size]
    return sum(matching) / len(matching)


def plot_results(rows, path):
    sizes = sorted(set(r["grid_size"] for r in rows))
    d_ms = [average(rows, "dijkstra_ms", s) for s in sizes]
    a_ms = [average(rows, "astar_ms", s) for s in sizes]
    r_ms = [average(rows, "rrt_ms", s) for s in sizes]
    d_cells = [average(rows, "dijkstra_cells_explored", s) for s in sizes]
    a_cells = [average(rows, "astar_cells_explored", s) for s in sizes]
    total_cells = [s * s for s in sizes]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    ax1.plot(sizes, d_ms, "o-", label="Dijkstra")
    ax1.plot(sizes, a_ms, "o-", label="A*")
    ax1.plot(sizes, r_ms, "o-", label="RRT")
    ax1.set_xlabel("Grid size N (for an NxN grid)")
    ax1.set_ylabel("Average runtime (ms)")
    ax1.set_title(f"Runtime vs grid size ({TRIALS_PER_SIZE} trials/size, {int(DENSITY*100)}% density)")
    ax1.legend()

    ax2.plot(sizes, d_cells, "o-", label="Dijkstra cells explored")
    ax2.plot(sizes, a_cells, "o-", label="A* cells explored")
    ax2.plot(sizes, total_cells, "--", color="gray", label="Total cells (NxN)")
    ax2.set_xlabel("Grid size N (for an NxN grid)")
    ax2.set_ylabel("Cells")
    ax2.set_yscale("log")
    ax2.set_title("Search effort vs total grid size (log scale)")
    ax2.legend()

    fig.tight_layout()
    fig.savefig(path, dpi=150)


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)
    rows = []
    for size in SIZES:
        for trial in range(TRIALS_PER_SIZE):
            rows.append(run_trial(size, trial))
        print(f"size {size}x{size}: {TRIALS_PER_SIZE} trials done")

    write_csv(rows, OUTPUT_DIR / "scale_results.csv")
    plot_results(rows, OUTPUT_DIR / "scale_comparison.png")

    print(f"\nWrote {len(rows)} trials to {OUTPUT_DIR / 'scale_results.csv'}")
    print(f"Plot saved to {OUTPUT_DIR / 'scale_comparison.png'}")
    for size in SIZES:
        d = average(rows, "dijkstra_ms", size)
        a = average(rows, "astar_ms", size)
        r = average(rows, "rrt_ms", size)
        found = sum(1 for row in rows if row["grid_size"] == size and row["rrt_found_path"])
        print(f"  {size:4d}x{size:<4d} ({size*size:6d} cells): "
              f"Dijkstra {d:8.3f}ms | A* {a:8.3f}ms | RRT {r:8.3f}ms "
              f"(found path {found}/{TRIALS_PER_SIZE})")
