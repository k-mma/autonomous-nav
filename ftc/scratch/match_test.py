"""
Does ftc/match.py's run_match actually enforce the 30-second budget as
a real failure condition, not just a number reported alongside success
-- the whole reason ftc/match.py exists instead of reusing nav/
uncertainty_benchmark.py's execute_trial, which has no time budget at
all?

1. A run that would clearly reach the goal under the real budget, but
   is cut off by an artificially tiny one, must come back
   success=False, over_budget=True -- not success=True just because it
   was *making progress* toward the goal when time ran out.
2. The identical scenario, replayed under the real AUTONOMOUS_PERIOD_S,
   succeeds -- proving case 1 was genuinely "slow but on track," not a
   scenario that was broken/unsolvable regardless of budget.
3. Replanning costs real match time (PLANNING_OVERHEAD_S per replan,
   ftc/config.py) -- a suite forced to replan many times on an
   otherwise-identical path accrues more elapsed_s than one that
   doesn't, so the 30s budget is actually sensitive to replanning
   frequency the way ftc/match.py's own docstring claims.
"""
import random

import ftc.match as match_module
from ftc.field import build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import DeadReckoningSuite, DistanceSensorSuite
from nav.field_variance import generate_ground_truth

LAYOUT = "sparse"


def _zero_deviation_scenario(seed=1):
    grid = build_grid(LAYOUT)
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    rng = random.Random(seed)
    from nav.algorithms import astar
    for attempt in range(50):
        start, goal = rng.sample(free, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= 6:
            break
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.0, seed=seed)
    tag_sites = tag_sites_for(LAYOUT)
    return grid, start, goal, ground_truth, actual_start, tag_sites


def check_slow_success_is_recorded_as_failure_then_succeeds_at_real_budget():
    grid, start, goal, ground_truth, actual_start, tag_sites = _zero_deviation_scenario()

    original_budget = match_module.AUTONOMOUS_PERIOD_S
    try:
        # Smaller than a single cell's drive time at MAX_DRIVE_SPEED_MPS
        # -- the very first step should already blow this budget.
        match_module.AUTONOMOUS_PERIOD_S = 0.01
        tiny_budget_result = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start,
                                         tag_sites, random.Random(1))
    finally:
        match_module.AUTONOMOUS_PERIOD_S = original_budget

    real_budget_result = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start,
                                     tag_sites, random.Random(1))

    tiny_ok = (not tiny_budget_result.success) and tiny_budget_result.over_budget
    real_ok = real_budget_result.success and not real_budget_result.over_budget
    ok = tiny_ok and real_ok
    print(f"  tiny budget (0.01s): success={tiny_budget_result.success} over_budget={tiny_budget_result.over_budget}")
    print(f"  real budget ({original_budget}s): success={real_budget_result.success} "
          f"over_budget={real_budget_result.over_budget}")
    print(f"a run cut off by the budget is recorded as a failure, and the identical scenario succeeds under "
          f"the real budget: {'OK' if ok else 'FAIL'}")
    return ok


def check_replanning_costs_real_time():
    """Isolate PLANNING_OVERHEAD_S's actual effect on elapsed_s, rather
    than comparing elapsed_s *across different suites* (which take
    different paths and so aren't a clean comparison -- a suite that
    replans onto a shorter route can easily finish with a *lower*
    elapsed_s than one that didn't replan at all, replan overhead
    notwithstanding, which very nearly made this check flaky).

    Instead: run the identical (suite, scenario, seed) once with the
    real PLANNING_OVERHEAD_S and once with it patched to 0. Patching it
    doesn't change what any replan call decides (astar doesn't know
    about match time at all), so the path/replans/success/collisions
    all have to come out identical -- elapsed_s is the only thing that
    can differ, and it has to differ by exactly
    replans * PLANNING_OVERHEAD_S if replanning is really being charged
    to the clock and not just counted."""
    grid = build_grid("cluttered")
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for("cluttered")

    from nav.algorithms import astar
    for seed in range(20):
        rng = random.Random(seed)
        start, goal = rng.sample(free, 2)
        assumed_path, _, _ = astar(grid, start, goal)
        if assumed_path is None or len(assumed_path) < 6:
            continue
        deviated_ground_truth, deviated_actual_start = generate_ground_truth(
            grid, start, goal, 1.0, seed=seed, start_drift_scale=0.0, obstacle_drift_scale=1.0, blocker_scale=0.0,
        )
        with_overhead = run_match(DistanceSensorSuite(), grid, start, goal, deviated_ground_truth,
                                    deviated_actual_start, tag_sites, random.Random(seed))
        if with_overhead.replans == 0:
            continue

        original_overhead = match_module.PLANNING_OVERHEAD_S
        try:
            match_module.PLANNING_OVERHEAD_S = 0.0
            without_overhead = run_match(DistanceSensorSuite(), grid, start, goal, deviated_ground_truth,
                                           deviated_actual_start, tag_sites, random.Random(seed))
        finally:
            match_module.PLANNING_OVERHEAD_S = original_overhead

        expected_delta = with_overhead.replans * original_overhead
        actual_delta = with_overhead.elapsed_s - without_overhead.elapsed_s
        same_path = (with_overhead.replans == without_overhead.replans
                      and with_overhead.success == without_overhead.success
                      and with_overhead.steps == without_overhead.steps)
        ok = same_path and abs(actual_delta - expected_delta) < 1e-6
        print(f"  seed={seed}: replans={with_overhead.replans}, elapsed_s with overhead="
              f"{with_overhead.elapsed_s}, without={without_overhead.elapsed_s}, "
              f"delta={actual_delta:.4f} (expected {expected_delta:.4f})")
        print(f"replanning's PLANNING_OVERHEAD_S is actually charged to elapsed_s, not just counted: "
              f"{'OK' if ok else 'FAIL'}")
        return ok

    print("FAIL: no seed in range(20) triggered a distance_sensors replan on the 'cluttered' layout")
    return False


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_slow_success_is_recorded_as_failure_then_succeeds_at_real_budget():
    assert check_slow_success_is_recorded_as_failure_then_succeeds_at_real_budget()


def test_replanning_costs_real_time():
    assert check_replanning_costs_real_time()


if __name__ == "__main__":
    checks = [
        check_slow_success_is_recorded_as_failure_then_succeeds_at_real_budget(),
        check_replanning_costs_real_time(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
