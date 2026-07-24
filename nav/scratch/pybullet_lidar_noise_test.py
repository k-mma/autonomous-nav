"""
Standalone demonstration of Lidar3D's noise model (nav/sim3d/lidar.py),
before it's wired into pybullet_main.py --sensor. Same wall-in-a-corridor
layout as nav/scratch/pybullet_lidar_test.py, scanned repeatedly from a
fixed position with `noisy=True`, mirroring nav/scratch/lidar_noise_test.py's
structure exactly so the two sensor models' noise can be compared
side by side. Answers the same question that one does: does
`confirmed_obstacles` (requiring a raycast to report the same cell more
than once) turn a noisy sensor's readings back into something a planner
can actually trust, and at what cost?

    python3 -m nav.scratch.pybullet_lidar_noise_test [--headless]
"""
import argparse
import random

import pybullet as p

from nav.algorithms import astar, find_path, path_cost
from nav.config import CONFIRMATION_THRESHOLD
from nav.grid import Grid
from nav.sensor import KnownGrid
from nav.sim3d.coords import grid_to_world
from nav.sim3d.lidar import Lidar3D
from nav.sim3d.world import connect, build_obstacles

START, GOAL = (10, 0), (10, 24)
WALL = {(row, 12) for row in range(8, 15)}
RADIUS = 6.0
SCAN_ROW, SCAN_COL = 10, 8
NUM_SCANS = 40


def build_grid():
    grid = Grid()
    for row, col in WALL:
        grid.cells[row][col] = Grid.OBSTACLE
    grid.place_start(*START)
    grid.place_goal(*GOAL)
    return grid


def plan_cost(known_obstacles, grid):
    known = KnownGrid(known_obstacles)
    path, _, reason, _ = find_path(known, "astar", START, GOAL)
    if path is None:
        return None, reason
    return path_cost(grid, path), None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    connect(gui=not args.headless)
    grid = build_grid()
    build_obstacles(grid)

    true_path, _, _ = astar(grid, START, GOAL)
    true_cost = path_cost(grid, true_path)
    print(f"ground truth: {len(WALL)} wall cells, optimal path cost = {true_cost:.2f}\n")

    scan_x, scan_y, _ = grid_to_world(SCAN_ROW, SCAN_COL)

    print("--- 1. perfect sensor (noisy=False), single scan ---")
    clean = Lidar3D(num_rays=48, max_range=RADIUS)
    seen = clean.scan((scan_x, scan_y), gui=False)
    print(f"detected: {sorted(seen)} (expect exactly the wall cells within range, no more, no less)\n")

    print(f"--- 2. noisy sensor, {NUM_SCANS} repeated scans from the same position ---")
    rng = random.Random(3)
    noisy = Lidar3D(num_rays=48, max_range=RADIUS, noisy=True, rng=rng)
    first_scan_seen = noisy.scan((scan_x, scan_y), gui=False)
    print(f"first scan alone: {sorted(first_scan_seen)} "
          f"({len(first_scan_seen & WALL)}/{len(WALL)} real wall cells caught on the very first try)")
    for _ in range(NUM_SCANS - 1):
        noisy.scan((scan_x, scan_y), gui=False)

    print(f"known_obstacles (every cell ever reported, right or wrong): {len(noisy.known_obstacles)} cells")
    print(f"  of which real: {len(noisy.known_obstacles & WALL)}/{len(WALL)}")
    print(f"  of which not real: {len(noisy.known_obstacles - WALL)}")

    print(f"\n--- 3. does confirming before replanning actually help? ---")
    print(f"(sweeping min_detections from 1 [= raw known_obstacles] upward)\n")
    for threshold in range(1, 6):
        view = noisy.confirmed_obstacles(threshold)
        fake = view - WALL
        cost, reason = plan_cost(view, grid)
        if cost is None:
            outcome = f"FAILED to find any path ({reason})"
        elif cost == true_cost:
            outcome = f"cost {cost:.2f} (= true optimal)"
        else:
            outcome = f"cost {cost:.2f} ({100 * (cost - true_cost) / true_cost:+.0f}% vs optimal)"
        label = "raw known_obstacles" if threshold == 1 else f"confirmed_obstacles(>={threshold})"
        print(f"  {label:28s}: {len(view):3d} cells ({len(fake):2d} fake) -> {outcome}")
    print(f"\ntrue optimal cost (perfect information): {true_cost:.2f}")
    print(f"(default CONFIRMATION_THRESHOLD in nav/config.py is {CONFIRMATION_THRESHOLD})")

    if not args.headless:
        input("Press Enter to close...")
    p.disconnect()


if __name__ == "__main__":
    main()
