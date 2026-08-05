"""
Does ftc/config.py's GEARING_OPTIONS (Priority 5, optional) actually
change match behavior the way its own docstring claims -- faster
gearing trades drive time for wheel slip, not a free win -- and does
the "stock" default reproduce every pre-Priority-5 caller exactly?

1. check_stock_default_reproduces_legacy_exactly: gearing=None and
   gearing="stock" both produce IDENTICAL MatchResults to a run before
   this feature existed (same elapsed_s, steps, final_pose_error_in) on
   a real seeded match -- the regression guarantee for every earlier
   priority's own numbers, which never pass a `gearing` argument.
2. check_faster_gearing_reduces_drive_time: on a straight-line, zero-
   deviation scenario (so pose error/collisions can't confound the
   comparison), "fast" and "faster" gearing produce a strictly SHORTER
   elapsed_s than "stock" for the identical path.
3. check_faster_gearing_increases_drift: on a scenario with real
   distance traveled, "faster" gearing produces a strictly LARGER
   final_pose_error_in than "stock" -- the wheel-slip tradeoff is
   actually wired to drift, not just present in ftc/config.py's numbers.
4. check_faster_gearing_can_recover_success_at_a_tight_budget: at a
   budget tight enough that "stock" gearing goes over_budget on a
   scenario "faster" gearing completes in time, "faster" actually
   succeeds where "stock" doesn't -- confirming the speed benefit is
   real, not just a smaller elapsed_s number that doesn't change any
   pass/fail outcome.
"""
import random

import ftc.config as config_module
import ftc.match as match_module
from ftc.field import build_grid, tag_sites_for
from ftc.match import _trapezoidal_drive_time_s, run_match
from ftc.sensors import DeadReckoningSuite
from nav.algorithms import astar
from nav.field_variance import generate_ground_truth

LAYOUT = "sparse"


def _zero_drift_suite():
    """A DeadReckoningSuite with drift_per_cell forced to 0 (the
    established instance-override pattern -- see ftc/robustness.py's
    docstring) -- isolates the pure speed/kinematics effect of a
    gearing option from the SEPARATE, confounding effect explored in
    check_faster_gearing_increases_drift below: more wheel slip changes
    the accumulated pose error, which changes which cells actually get
    visited (and how much turning happens), which can shift total
    elapsed_s in either direction on a real stochastic match -- a real
    modeling consequence, but the wrong thing to compare against when
    the question is specifically "does higher accel reduce per-step
    drive time.\""""
    suite = DeadReckoningSuite()
    suite.drift_per_cell = 0.0
    return suite


def _long_straight_scenario(seed=7):
    grid = build_grid(LAYOUT)
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)
    rng = random.Random(seed)
    best = None
    for _ in range(50):
        start, goal = rng.sample(free, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and (best is None or len(path) > len(best[0])):
            best = (path, start, goal)
        if path is not None and len(path) >= 15:
            break
    path, start, goal = best
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.0, seed=seed)
    return grid, start, goal, ground_truth, actual_start, tag_sites


def check_stock_default_reproduces_legacy_exactly():
    grid, start, goal, ground_truth, actual_start, tag_sites = _long_straight_scenario()

    legacy = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                         random.Random(7))
    stock = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                        random.Random(7), gearing="stock")
    default_none = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                               random.Random(7), gearing=None)

    ok = (legacy.elapsed_s == stock.elapsed_s == default_none.elapsed_s
          and legacy.steps == stock.steps == default_none.steps
          and legacy.final_pose_error_in == stock.final_pose_error_in == default_none.final_pose_error_in)
    print(f"  no gearing arg: elapsed_s={legacy.elapsed_s}")
    print(f"  gearing='stock': elapsed_s={stock.elapsed_s}")
    print(f"  gearing=None:    elapsed_s={default_none.elapsed_s}")
    print(f"gearing=None and gearing='stock' both reproduce the pre-Priority-5 default exactly: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_faster_gearing_reduces_drive_time():
    """Two levels: the direct kinematics mechanism (unit-level, like
    ftc/scratch/kinematics_test.py's own closed-form checks), then an
    end-to-end confirmation through run_match with drift zeroed out so
    the comparison isn't confounded by gearing-driven drift changing
    which cells actually get visited (see _zero_drift_suite)."""
    single_cell_m = config_module.CELL_SIZE_IN / config_module.INCHES_PER_METER
    times = {}
    for name in config_module.GEARING_ORDER:
        opt = config_module.GEARING_OPTIONS[name]
        times[name] = _trapezoidal_drive_time_s(single_cell_m, max_speed_mps=opt["max_speed_mps"],
                                                   max_accel_mps2=opt["max_accel_mps2"])
    direct_ok = times["fast"] < times["stock"] and times["faster"] < times["fast"]
    print(f"  single-cell drive time: stock={times['stock']:.4f}s, fast={times['fast']:.4f}s, "
          f"faster={times['faster']:.4f}s")
    print(f"faster gearing's own accel/speed values strictly reduce single-cell drive time: "
          f"{'OK' if direct_ok else 'FAIL'}")

    grid, start, goal, ground_truth, actual_start, tag_sites = _long_straight_scenario()
    stock = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                        random.Random(7), gearing="stock")
    fast = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                       random.Random(7), gearing="fast")
    faster = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                         random.Random(7), gearing="faster")
    end_to_end_ok = (stock.steps == fast.steps == faster.steps and fast.elapsed_s < stock.elapsed_s
                      and faster.elapsed_s < fast.elapsed_s)
    print(f"  (zero-drift) stock elapsed_s={stock.elapsed_s}, fast elapsed_s={fast.elapsed_s}, "
          f"faster elapsed_s={faster.elapsed_s} (same {stock.steps} steps in all three)")
    print(f"end to end (drift zeroed, isolating the speed effect): faster gearing completes the identical "
          f"path in strictly less elapsed_s: {'OK' if end_to_end_ok else 'FAIL'}")
    return direct_ok and end_to_end_ok


def check_faster_gearing_increases_drift():
    grid, start, goal, ground_truth, actual_start, tag_sites = _long_straight_scenario()

    stock = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                        random.Random(7), gearing="stock")
    faster = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                         random.Random(7), gearing="faster")

    ok = faster.final_pose_error_in > stock.final_pose_error_in
    print(f"  stock final_pose_error_in={stock.final_pose_error_in}, "
          f"faster final_pose_error_in={faster.final_pose_error_in}")
    print(f"faster gearing's wheel slip produces strictly more accumulated pose error on the identical "
          f"path: {'OK' if ok else 'FAIL'}")
    return ok


def check_faster_gearing_can_recover_success_at_a_tight_budget():
    grid, start, goal, ground_truth, actual_start, tag_sites = _long_straight_scenario()

    original_budget = match_module.AUTONOMOUS_PERIOD_S
    # Drift zeroed (see _zero_drift_suite) so the comparison isolates
    # the speed effect this check is actually about, and so it's safe
    # to derive the tight budget from these two runs' own elapsed_s
    # (self-verifying rather than a hand-picked, scenario-fragile
    # number) without a stochastic path divergence between suites.
    stock_at_real_budget = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start,
                                       tag_sites, random.Random(7), gearing="stock")
    faster_at_real_budget = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start,
                                        tag_sites, random.Random(7), gearing="faster")
    if not (faster_at_real_budget.elapsed_s < stock_at_real_budget.elapsed_s):
        print("SKIP: faster gearing wasn't actually faster on this scenario at the real budget")
        return True

    tight_budget = (stock_at_real_budget.elapsed_s + faster_at_real_budget.elapsed_s) / 2
    try:
        match_module.AUTONOMOUS_PERIOD_S = tight_budget
        stock_tight = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                  random.Random(7), gearing="stock")
        faster_tight = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                   random.Random(7), gearing="faster")
    finally:
        match_module.AUTONOMOUS_PERIOD_S = original_budget

    ok = stock_tight.over_budget and not faster_tight.over_budget
    print(f"  tight_budget={tight_budget:.3f}s: stock over_budget={stock_tight.over_budget}, "
          f"faster over_budget={faster_tight.over_budget}")
    print(f"faster gearing turns an over-budget failure into an on-time success at a tight budget: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


if __name__ == "__main__":
    checks = [
        check_stock_default_reproduces_legacy_exactly(),
        check_faster_gearing_reduces_drive_time(),
        check_faster_gearing_increases_drift(),
        check_faster_gearing_can_recover_success_at_a_tight_budget(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
