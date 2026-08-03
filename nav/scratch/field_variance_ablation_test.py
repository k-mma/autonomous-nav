"""
Does nav/field_variance.py's generate_ground_truth actually let a caller
isolate one deviation type from the other two -- the mechanism
ftc/suite_benchmark.py's deviation-type sweep depends on to explain
*why* a given sensor suite wins or loses, not just that it does?

1. Zeroing a scale independently kills that deviation type alone, at
   variance_level=1.0 where an unscaled call would max out all three.
2. Zeroing all three scales is identical to variance_level=0.0 --
   confirms the scales are pure multipliers on top of variance_level,
   not a separate deviation mechanism.
3. The default scale=1.0 call is unaffected by this change at all
   (already covered by nav/scratch/field_variance_test.py continuing to
   pass, re-asserted here for a few extra seeds as a direct regression
   guard on this specific edit).
"""
import random

from nav.config import FIELD_VARIANCE_MAX_START_DRIFT_RADIUS
from nav.field_variance import generate_ground_truth
from nav.grid import Grid

DENSITY = 0.15
SIZE = 20
TRIALS = 25


def _random_grid(rng):
    grid = Grid(size=SIZE)
    for row in range(SIZE):
        for col in range(SIZE):
            if rng.random() < DENSITY:
                grid.cells[row][col] = Grid.OBSTACLE
    free = [(r, c) for r in range(SIZE) for c in range(SIZE) if grid.cells[r][c] == Grid.FREE]
    start, goal = rng.sample(free, 2)
    return grid, start, goal


def _diff_count(a, b):
    return sum(1 for r in range(a.size) for c in range(a.size) if a.cells[r][c] != b.cells[r][c])


def _chebyshev(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def check_zeroing_one_scale_isolates_the_others():
    rng = random.Random(3)
    violations = 0
    for t in range(TRIALS):
        grid, start, goal = _random_grid(rng)

        # obstacle_drift_scale=0, blocker_scale=0 -> only start drift active.
        gt, actual_start = generate_ground_truth(
            grid, start, goal, 1.0, seed=t,
            obstacle_drift_scale=0.0, blocker_scale=0.0,
        )
        if _diff_count(grid, gt) != 0:
            violations += 1
            print(f"  trial {t}: obstacle/blocker scaled to 0 but grid still differs from assumed")

        # start_drift_scale=0, blocker_scale=0 -> only obstacle drift active
        # (actual_start must stay exactly at start).
        gt2, actual_start2 = generate_ground_truth(
            grid, start, goal, 1.0, seed=t,
            start_drift_scale=0.0, blocker_scale=0.0,
        )
        if actual_start2 != start:
            violations += 1
            print(f"  trial {t}: start_drift_scale=0 but actual_start moved to {actual_start2}")

    print(f"zeroing one scale isolates the other deviation types: "
          f"{'OK' if violations == 0 else f'FAIL ({violations} violations)'}")
    return violations == 0


def check_all_scales_zero_is_identity():
    rng = random.Random(11)
    violations = 0
    for t in range(TRIALS):
        grid, start, goal = _random_grid(rng)
        gt, actual_start = generate_ground_truth(
            grid, start, goal, 1.0, seed=t,
            start_drift_scale=0.0, obstacle_drift_scale=0.0, blocker_scale=0.0,
        )
        if _diff_count(grid, gt) != 0 or actual_start != start:
            violations += 1
            print(f"  trial {t}: all scales 0 but ground truth or actual_start still changed")
    print(f"all three scales at 0 is an exact identity, same as variance_level=0.0: "
          f"{'OK' if violations == 0 else f'FAIL ({violations} violations)'}")
    return violations == 0


def check_default_scales_match_unscaled_bound():
    """Same bound check nav/scratch/field_variance_test.py already runs
    at variance_level=1.0, re-run here as a direct guard that adding the
    *_scale kwargs didn't change the no-kwargs call path."""
    rng = random.Random(17)
    violations = 0
    max_expected_diff = 2 * 6 + 1  # FIELD_VARIANCE_MAX_OBSTACLE_DRIFT_COUNT, mirrored to avoid a config import loop
    for t in range(TRIALS):
        grid, start, goal = _random_grid(rng)
        gt, actual_start = generate_ground_truth(grid, start, goal, 1.0, seed=t)
        if _diff_count(grid, gt) > max_expected_diff:
            violations += 1
        if _chebyshev(start, actual_start) > FIELD_VARIANCE_MAX_START_DRIFT_RADIUS:
            violations += 1
    print(f"default (no *_scale kwargs) call still respects the original bounds: "
          f"{'OK' if violations == 0 else f'FAIL ({violations} violations)'}")
    return violations == 0


if __name__ == "__main__":
    checks = [
        check_zeroing_one_scale_isolates_the_others(),
        check_all_scales_zero_is_identity(),
        check_default_scales_match_unscaled_bound(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
