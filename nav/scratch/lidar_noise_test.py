"""
Standalone demonstration of LidarSensor's noise model (nav/sensor.py),
before it's wired into nav/visualizer.py. Same corridor-with-hidden-
obstacles setup as nav/scratch/lidar_test.py, but scanned repeatedly from
a fixed position with `noisy=True` instead of moved through once, so the
three noise types actually show up in the output:

1. **False negatives**: does a real obstacle sometimes go undetected on
   a given scan, even though it's in range?
2. **Position noise**: does a detected obstacle sometimes get reported
   at the wrong (adjacent) cell?
3. **False positives**: does a free cell sometimes get "detected" as an
   obstacle that isn't there?

Then the actual planning-relevant question: does `confirmed_obstacles`
(requiring a cell to be reported more than once) actually produce a
*more trustworthy* belief than raw `known_obstacles`, or does it just
shrink the noise without meaningfully improving it? Answered by
replanning against both and comparing path cost to the true-grid
optimum, not just by eyeballing set sizes.
"""
import random

from nav.algorithms import astar, find_path, path_cost
from nav.config import CONFIRMATION_THRESHOLD
from nav.grid import Grid
from nav.sensor import KnownGrid, LidarSensor

START, GOAL = (0, 0), (0, 19)
HIDDEN_OBSTACLES = {(0, 9), (1, 9), (2, 9)}
RADIUS = 6
SCAN_POSITION = (0, 5)
NUM_SCANS = 40


def build_grid():
    grid = Grid()
    for row, col in HIDDEN_OBSTACLES:
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


if __name__ == "__main__":
    grid = build_grid()
    true_path, _, _ = astar(grid, START, GOAL)
    true_cost = path_cost(grid, true_path)
    print(f"ground truth: {len(HIDDEN_OBSTACLES)} hidden obstacles, optimal path cost = {true_cost:.2f}\n")

    print("--- 1. perfect sensor (noisy=False), single scan ---")
    clean = LidarSensor(RADIUS)
    seen = clean.sense(grid, SCAN_POSITION)
    print(f"detected: {sorted(seen)} (expect exactly the hidden obstacles within range, no more, no less)\n")

    print(f"--- 2. noisy sensor, {NUM_SCANS} repeated scans from the same position ---")
    rng = random.Random(7)
    noisy = LidarSensor(RADIUS, noisy=True, rng=rng)
    first_scan_seen = noisy.sense(grid, SCAN_POSITION)
    print(f"first scan alone: {sorted(first_scan_seen)} "
          f"({len(first_scan_seen & HIDDEN_OBSTACLES)}/{len(HIDDEN_OBSTACLES)} real obstacles caught "
          f"on the very first try -- a miss here isn't a bug, it's the miss rate)")
    for _ in range(NUM_SCANS - 1):
        noisy.sense(grid, SCAN_POSITION)

    print(f"known_obstacles (every cell ever reported, right or wrong): {len(noisy.known_obstacles)} cells")
    print(f"  of which real: {len(noisy.known_obstacles & HIDDEN_OBSTACLES)}/{len(HIDDEN_OBSTACLES)}")
    print(f"  of which not real (false positives + position-jittered ghosts): "
          f"{len(noisy.known_obstacles - HIDDEN_OBSTACLES)}")

    confirmed = noisy.confirmed_obstacles(CONFIRMATION_THRESHOLD)
    print(f"\nconfirmed_obstacles(min_detections={CONFIRMATION_THRESHOLD}): {len(confirmed)} cells")
    print(f"  of which real: {len(confirmed & HIDDEN_OBSTACLES)}/{len(HIDDEN_OBSTACLES)}")
    fake_confirmed = confirmed - HIDDEN_OBSTACLES
    print(f"  of which not real: {len(fake_confirmed)}")
    if fake_confirmed:
        near_real = {c for c in fake_confirmed if any(abs(c[0] - r) <= 1 and abs(c[1] - col) <= 1
                                                        for r, col in HIDDEN_OBSTACLES)}
        print(f"    ({len(near_real)} of those are position-jittered *neighbors* of a real obstacle -- "
              f"the wall's boundary blurred by about a cell, not a random phantom elsewhere; "
              f"{len(fake_confirmed) - len(near_real)} are unrelated false positives that happened "
              f"to get reported {CONFIRMATION_THRESHOLD}+ times by chance)")

    print("\n--- 3. does confirming before replanning actually help? ---")
    print(f"(sweeping min_detections from 1 [= raw known_obstacles] upward)\n")
    for threshold in range(1, 6):
        view = noisy.confirmed_obstacles(threshold)
        fake = view - HIDDEN_OBSTACLES
        cost, reason = plan_cost(view, grid)
        outcome = f"cost {cost:.2f}" + (" (= true optimal)" if cost == true_cost else
                                          f" ({100 * (cost - true_cost) / true_cost:+.0f}% vs optimal)") \
            if cost else f"FAILED to find any path ({reason})"
        label = "raw known_obstacles" if threshold == 1 else f"confirmed_obstacles(>={threshold})"
        print(f"  {label:28s}: {len(view):3d} cells ({len(fake):2d} fake) -> {outcome}")
    print(f"\ntrue optimal cost (perfect information): {true_cost:.2f}")
    print(
        "\nTakeaway: a single false-positive detection landing on the robot's own\n"
        "start cell (it happened in this exact run) makes raw known_obstacles fail\n"
        "outright -- not just suboptimal, unable to find *any* path. Requiring even\n"
        "one repeat (min_detections=2) avoids that outright failure, though enough\n"
        "surviving noise can still cost real path length. Requiring a couple more\n"
        "repeats closes the rest of the gap to the true optimum here -- at the\n"
        "obvious cost of needing more scans (more time near an obstacle) before\n"
        "trusting it enough to route around."
    )
