"""
Standalone demonstration of Lidar3D's noise model (pybullet_app/sim3d/lidar.py),
before it's wired into pybullet_main.py --sensor. Same wall-in-a-corridor
layout as pybullet_app/scratch/pybullet_lidar_test.py, scanned repeatedly from a
fixed position with `noisy=True`, mirroring nav/scratch/lidar_noise_test.py's
structure exactly so the two sensor models' noise can be compared
side by side. Answers the same question that one does: does
`confirmed_obstacles` (requiring a raycast to report the same cell more
than once) turn a noisy sensor's readings back into something a planner
can actually trust, and at what cost?

    python3 -m pybullet_app.scratch.pybullet_lidar_noise_test [--headless]
"""
import argparse
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import pybullet as p

from nav.algorithms import astar, find_path, path_cost
from nav.config import CONFIRMATION_THRESHOLD
from nav.grid import Grid
from nav.sensor import KnownGrid
from pybullet_app.sim3d.coords import grid_to_world
from pybullet_app.sim3d.lidar import Lidar3D
from pybullet_app.sim3d.world import connect, build_obstacles

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


def run_demo(headless, seed=3):
    connect(gui=not headless)
    grid = build_grid()
    build_obstacles(grid)

    true_path, _, _ = astar(grid, START, GOAL)
    true_cost = path_cost(grid, true_path)
    print(f"ground truth: {len(WALL)} wall cells, optimal path cost = {true_cost:.2f}\n")

    scan_x, scan_y, _ = grid_to_world(SCAN_ROW, SCAN_COL)

    print("--- 1. perfect sensor (noisy=False), single scan ---")
    clean = Lidar3D(num_rays=48, max_range=RADIUS)
    clean_seen = clean.scan((scan_x, scan_y), gui=False)
    print(f"detected: {sorted(clean_seen)} (expect exactly the wall cells within range, no more, no less)\n")

    print(f"--- 2. noisy sensor, {NUM_SCANS} repeated scans from the same position ---")
    rng = random.Random(seed)
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
    by_threshold = {}
    for threshold in range(1, 6):
        view = noisy.confirmed_obstacles(threshold)
        fake = view - WALL
        cost, reason = plan_cost(view, grid)
        by_threshold[threshold] = {"cells": len(view), "fake": len(fake), "cost": cost, "reason": reason}
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
    return {"true_cost": true_cost, "clean_seen": clean_seen, "by_threshold": by_threshold}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    run_demo(args.headless)
    if not args.headless:
        input("Press Enter to close...")
    p.disconnect()


# --- pytest entry points --------------------------------------------------
# Mirrors nav/scratch/lidar_noise_test.py's pytest entry points: the module
# docstring's own framing is "measured, not asserted" for the noise itself,
# but the seed is fixed (seed=3, same as the demo above), so the two
# findings it's actually here to demonstrate -- a clean scan is exact, and
# confirming a detection before trusting it recovers the true-optimal path
# cost -- are fully reproducible and worth locking in as real regressions.

def test_clean_scan_sees_exactly_the_wall_within_range():
    try:
        result = run_demo(headless=True)
        assert result["clean_seen"] == WALL
    finally:
        p.disconnect()


def test_raw_known_obstacles_finds_a_path_but_pays_a_cost_penalty():
    try:
        result = run_demo(headless=True)
        raw = result["by_threshold"][1]
        assert raw["cost"] is not None
        assert raw["cost"] > result["true_cost"]
    finally:
        p.disconnect()


def test_confirming_detections_recovers_the_true_optimum():
    try:
        result = run_demo(headless=True)
        assert result["by_threshold"][2]["cost"] == result["true_cost"]
    finally:
        p.disconnect()


def test_false_positive_count_never_increases_as_the_threshold_rises():
    try:
        result = run_demo(headless=True)
        fake_counts = [result["by_threshold"][t]["fake"] for t in range(1, 6)]
        assert all(a >= b for a, b in zip(fake_counts, fake_counts[1:])), fake_counts
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
