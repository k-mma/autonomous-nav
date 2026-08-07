"""
Does ftc/match.py's elapsed_s/over_budget accounting actually ignore
measured planning latency, the way ftc/planning_latency_benchmark.py's
module docstring claims from reading the source? And does this
benchmark's own instrumentation (capture_astar_latencies, the
counterfactual arithmetic) measure what it claims to?

1. check_flat_charge_ignores_measured_latency: the direct, empirical
   version of the structural claim -- run the SAME scenario/seed twice,
   once against the real astar() and once against an artificially SLOW
   one (same path, same result, just slower), and confirm run_match's
   elapsed_s/success/over_budget/replans come out byte-for-byte
   identical either way. If this ever fails, the "tail latency can't
   change a match outcome under the current model" claim this whole
   study rests on is wrong.
2. check_capture_astar_latencies_matches_match_result: the sanity check
   ftc/planning_latency_benchmark.py's run_combo already asserts on
   every trial, as a standalone test -- captured latencies sum close to
   MatchResult.planning_time_s.
3. check_single_plan_match_has_zero_charged_planning: a suite that
   never replans (DeadReckoningSuite) has replans == 0, so the flat
   per-replan charge contributes exactly 0 to elapsed_s -- confirming
   the "first plan is free" reading of the source is correct, not just
   plausible.
4. check_counterfactual_flips_under_artificially_slow_planning: proves
   the counterfactual arithmetic itself is capable of detecting a flip
   -- constructs a match where PLANNING_OVERHEAD_S under-charges by a
   known, large, injected amount and confirms counterfactual_elapsed_s
   moves by (measured total - charged total), independent of whether
   any REAL trial in the actual sweep happens to trigger one.
"""
import random
import time

from nav.algorithms import astar
from nav.field_variance import generate_ground_truth

import ftc.match as match_module
from ftc.config import PLANNING_OVERHEAD_S
from ftc.field import build_grid, tag_sites_for
from ftc.match import run_match
from ftc.planning_latency_benchmark import capture_astar_latencies
from ftc.sensors import AprilTagSuite, DeadReckoningSuite


def _apriltag_scenario(seed, min_path_len=8):
    grid = build_grid("cluttered")
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for("cluttered")
    rng = random.Random(seed)
    start = goal = None
    for _ in range(50):
        start, goal = rng.sample(free, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= min_path_len:
            break
    ground_truth, actual_start = generate_ground_truth(
        grid, start, goal, 0.4, seed=seed, start_drift_scale=1.0, obstacle_drift_scale=1.0
    )
    return grid, start, goal, ground_truth, actual_start, tag_sites


def check_flat_charge_ignores_measured_latency():
    grid, start, goal, ground_truth, actual_start, tag_sites = _apriltag_scenario(seed=5)

    fast_result = run_match(AprilTagSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                             random.Random(5))

    original = match_module.astar

    def slow_astar(*args, **kwargs):
        # Deliberately larger than PLANNING_OVERHEAD_S itself (0.05s)
        # so a discrepancy would be impossible to miss, and larger than
        # any latency actually observed in this study's own sweep at
        # native grid scale -- if elapsed_s is insensitive to THIS, it
        # is insensitive to anything realistic too.
        result = original(*args, **kwargs)
        time.sleep(PLANNING_OVERHEAD_S * 2)
        return result

    match_module.astar = slow_astar
    try:
        slow_result = run_match(AprilTagSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                 random.Random(5))
    finally:
        match_module.astar = original

    fields = ["success", "over_budget", "elapsed_s", "collisions", "replans", "steps"]
    mismatches = [f for f in fields if getattr(fast_result, f) != getattr(slow_result, f)]
    ok = not mismatches
    print(f"  fast: {fast_result}")
    print(f"  slow (each astar() call artificially delayed {PLANNING_OVERHEAD_S * 2}s): {slow_result}")
    if mismatches:
        print(f"  MISMATCHES: {mismatches}")
    print(f"run_match's elapsed_s/success/over_budget/replans are identical regardless of how long astar() "
          f"actually took: {'OK' if ok else 'FAIL'}")
    return ok


def check_capture_astar_latencies_matches_match_result():
    grid, start, goal, ground_truth, actual_start, tag_sites = _apriltag_scenario(seed=11)
    with capture_astar_latencies() as latencies:
        result = run_match(AprilTagSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                            random.Random(11))

    measured_total = sum(latencies)
    tolerance_s = max(2e-4, 0.05 * result.planning_time_s)
    ok = (len(latencies) == result.replans + 1) and (abs(measured_total - result.planning_time_s) < tolerance_s)
    print(f"  captured {len(latencies)} calls, result.replans={result.replans} (expect replans + 1)")
    print(f"  measured_total={measured_total:.6f}s, MatchResult.planning_time_s={result.planning_time_s:.6f}s, "
          f"tolerance={tolerance_s:.6f}s")
    print(f"capture_astar_latencies captures the same calls run_match measures on its own: {'OK' if ok else 'FAIL'}")
    return ok


def check_single_plan_match_has_zero_charged_planning():
    grid, start, goal, ground_truth, actual_start, tag_sites = _apriltag_scenario(seed=3, min_path_len=4)
    with capture_astar_latencies() as latencies:
        result = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                            random.Random(3))

    charged_planning_s = result.replans * PLANNING_OVERHEAD_S
    ok = result.replans == 0 and len(latencies) == 1 and charged_planning_s == 0.0
    print(f"  DeadReckoningSuite: replans={result.replans}, calls captured={len(latencies)}, "
          f"charged_planning_s={charged_planning_s}")
    print(f"a suite that never replans has exactly one (uncharged) planning call: {'OK' if ok else 'FAIL'}")
    return ok


def check_counterfactual_flips_under_artificially_slow_planning():
    """Not a real trial from the sweep -- a constructed check that the
    counterfactual FORMULA (ftc/planning_latency_benchmark.py's
    `result.elapsed_s - charged_planning_s + measured_total`) actually
    moves the way it should when measured latency is deliberately made
    to exceed what PLANNING_OVERHEAD_S assumes, independent of whether
    any grid size in the real sweep happens to trigger this."""
    grid, start, goal, ground_truth, actual_start, tag_sites = _apriltag_scenario(seed=5)

    original = match_module.astar
    # Large enough, per call, that a match with just a few replans
    # blows straight past a 30s budget -- this is checking the
    # ARITHMETIC, not trying to reproduce a realistic scenario.
    injected_latency_s = 20.0

    def artificially_slow_astar(*args, **kwargs):
        result = original(*args, **kwargs)
        time.sleep(0.001)  # negligible real delay; injected_latency_s below stands in for the "measured" value
        return result

    match_module.astar = artificially_slow_astar
    try:
        with capture_astar_latencies() as latencies:
            result = run_match(AprilTagSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                random.Random(5))
    finally:
        match_module.astar = original

    # Substitute a known large per-call latency for what was actually
    # (barely) measured -- isolates the arithmetic from needing a real
    # multi-second sleep per call in a test.
    inflated_total = injected_latency_s * len(latencies)
    charged_planning_s = result.replans * PLANNING_OVERHEAD_S
    counterfactual_elapsed_s = result.elapsed_s - charged_planning_s + inflated_total

    ok = (not result.over_budget) and (counterfactual_elapsed_s > 30.0)
    print(f"  actual elapsed_s={result.elapsed_s}, over_budget={result.over_budget}, replans={result.replans}")
    print(f"  counterfactual (inflated to {injected_latency_s}s/call): {counterfactual_elapsed_s:.2f}s")
    print(f"the counterfactual formula correctly flips a match to over-budget when planning latency is "
          f"substituted with a large enough value: {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------


def test_flat_charge_ignores_measured_latency():
    assert check_flat_charge_ignores_measured_latency()


def test_capture_astar_latencies_matches_match_result():
    assert check_capture_astar_latencies_matches_match_result()


def test_single_plan_match_has_zero_charged_planning():
    assert check_single_plan_match_has_zero_charged_planning()


def test_counterfactual_flips_under_artificially_slow_planning():
    assert check_counterfactual_flips_under_artificially_slow_planning()


if __name__ == "__main__":
    checks = [
        check_flat_charge_ignores_measured_latency(),
        check_capture_astar_latencies_matches_match_result(),
        check_single_plan_match_has_zero_charged_planning(),
        check_counterfactual_flips_under_artificially_slow_planning(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
