"""
Does nav/field_variance.py's generate_ground_truth actually respect its
own contract: zero deviation at variance_level=0.0, deviation capped at
the configured bound at variance_level=1.0, and exact reproducibility
given the same seed?
"""
import random

from nav.config import (
    FIELD_VARIANCE_MAX_OBSTACLE_DRIFT_COUNT, FIELD_VARIANCE_MAX_START_DRIFT_RADIUS, GRID_SIZE,
)
from nav.field_variance import generate_ground_truth
from nav.grid import Grid

DENSITY = 0.15
TRIALS = 30


def _random_grid(rng):
    grid = Grid(size=GRID_SIZE)
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            if rng.random() < DENSITY:
                grid.cells[row][col] = Grid.OBSTACLE
    free = [(r, c) for r in range(GRID_SIZE) for c in range(GRID_SIZE) if grid.cells[r][c] == Grid.FREE]
    start, goal = rng.sample(free, 2)
    return grid, start, goal


def _diff_count(a, b):
    return sum(1 for r in range(a.size) for c in range(a.size) if a.cells[r][c] != b.cells[r][c])


def _chebyshev(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def check_zero_variance_is_identity():
    rng = random.Random(7)
    mismatches = 0
    for t in range(TRIALS):
        grid, start, goal = _random_grid(rng)
        ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.0, seed=t)
        diff = _diff_count(grid, ground_truth)
        if diff != 0 or actual_start != start:
            mismatches += 1
            print(f"  trial {t}: variance_level=0.0 changed something "
                  f"(cell diffs={diff}, actual_start={actual_start}, start={start})")
    print(f"variance_level=0.0 is an exact identity: {TRIALS - mismatches}/{TRIALS} match")
    return mismatches == 0


def check_max_variance_within_bound():
    rng = random.Random(11)
    violations = 0
    # Obstacle drift can flip up to FIELD_VARIANCE_MAX_OBSTACLE_DRIFT_COUNT
    # cells each direction (free->obstacle and obstacle->free), plus one
    # more cell for the unplanned blocker.
    max_expected_diff = 2 * FIELD_VARIANCE_MAX_OBSTACLE_DRIFT_COUNT + 1
    for t in range(TRIALS):
        grid, start, goal = _random_grid(rng)
        ground_truth, actual_start = generate_ground_truth(grid, start, goal, 1.0, seed=t)
        diff = _diff_count(grid, ground_truth)
        drift = _chebyshev(start, actual_start)
        if diff > max_expected_diff:
            violations += 1
            print(f"  trial {t}: {diff} cells differ, expected <= {max_expected_diff}")
        if drift > FIELD_VARIANCE_MAX_START_DRIFT_RADIUS:
            violations += 1
            print(f"  trial {t}: start drifted {drift} cells, expected <= {FIELD_VARIANCE_MAX_START_DRIFT_RADIUS}")
    print(f"variance_level=1.0 stays within configured bounds: {TRIALS - violations}/{TRIALS} trials OK")
    return violations == 0


def check_deterministic():
    rng = random.Random(23)
    mismatches = 0
    levels = [0.0, 0.3, 0.6, 1.0]
    for t in range(TRIALS):
        grid, start, goal = _random_grid(rng)
        level = levels[t % len(levels)]
        gt1, s1 = generate_ground_truth(grid, start, goal, level, seed=999)
        gt2, s2 = generate_ground_truth(grid, start, goal, level, seed=999)
        if _diff_count(gt1, gt2) != 0 or s1 != s2:
            mismatches += 1
            print(f"  trial {t}: same seed produced different output at variance_level={level}")
    print(f"same seed is exactly reproducible: {TRIALS - mismatches}/{TRIALS} match")
    return mismatches == 0


if __name__ == "__main__":
    checks = [check_zero_variance_is_identity(), check_max_variance_within_bound(), check_deterministic()]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
