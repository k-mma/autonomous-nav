"""
Does ftc/config.py's GEARING_OPTIONS (Priority 5, optional) actually
change match behavior the way real, VERIFIED goBILDA 5203-series motor
specs say it should -- and does the "stock" default reproduce every
pre-Priority-5 caller exactly?

GEARING_OPTIONS was originally built from invented speed/accel
multipliers that both moved the same direction (faster AND harder-
accelerating), which is not how a real gearmotor works: torque falls as
RPM rises for a fixed motor, so a faster ratio has LESS acceleration
available, not more. Once that was corrected (see ftc/config.py's
GEARING_OPTIONS docstring, built directly off goBILDA's published
5203-series RPM/torque table), a second, non-obvious consequence
followed: at this project's actual cell scale (a single 6in step, or a
0.2155m diagonal one), EVERY gearing option's accel-to-cruise distance
(speed^2 / (2*accel)) is far larger than one cell, so a single step
always falls in _trapezoidal_drive_time_s's TRIANGULAR branch, whose
time is `2*sqrt(distance/accel)` -- a function of ACCELERATION alone,
with top speed never entering the formula at all. Since real motor
torque falls faster than RPM rises across the 5203 lineup, a faster
gearing choice is strictly SLOWER per short hop in this model, not
faster -- on top of costing more drift. This is a real, sourced finding
worth stating plainly, not a bug to route around: "buy a faster motor"
does not even buy the thing it promises at FTC's typical short-hop
distances.

1. check_stock_default_reproduces_legacy_exactly: gearing=None and
   gearing="stock" both produce IDENTICAL MatchResults to a run before
   this feature existed (same elapsed_s, steps, final_pose_error_in) on
   a real seeded match -- the regression guarantee for every earlier
   priority's own numbers, which never pass a `gearing` argument.
2. check_faster_gearing_is_slower_per_cell_not_faster: on a straight-
   line, zero-deviation scenario (so pose error/collisions can't
   confound the comparison), "fast" and "faster" gearing produce a
   strictly LONGER elapsed_s than "stock" for the identical path --
   both directly against _trapezoidal_drive_time_s's own closed-form
   formula (like ftc/scratch/kinematics_test.py's checks) and end to
   end through run_match.
3. check_faster_gearing_increases_drift: on a scenario with real
   distance traveled, "faster" gearing produces a strictly LARGER
   final_pose_error_in than "stock" -- the wheel-slip tradeoff is
   actually wired to drift, not just present in ftc/config.py's numbers.
4. check_no_gearing_option_rescues_a_tight_budget: at a budget tight
   enough that "stock" gearing goes over_budget on a scenario, "fast"
   and "faster" gearing don't rescue it -- they go over budget too
   (or worse), since neither actually reduces per-cell drive time at
   this project's grid scale.
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


def check_faster_gearing_is_slower_per_cell_not_faster():
    """Two levels: the direct kinematics mechanism (unit-level, like
    ftc/scratch/kinematics_test.py's own closed-form checks), then an
    end-to-end confirmation through run_match with drift zeroed out so
    the comparison isn't confounded by gearing-driven drift changing
    which cells actually get visited (see _zero_drift_suite). Also
    confirms the STRUCTURAL reason why: every option's accel-to-cruise
    distance is far larger than a single grid cell, so top speed never
    actually enters the triangular-branch time formula -- only
    acceleration does, and real motor acceleration falls as gearing
    speeds up (goBILDA 5203: 338 oz-in at 312 RPM, down to 109 oz-in at
    1150 RPM)."""
    single_cell_m = config_module.CELL_SIZE_IN / config_module.INCHES_PER_METER
    times = {}
    accel_distances = {}
    for name in config_module.GEARING_ORDER:
        opt = config_module.GEARING_OPTIONS[name]
        accel_distances[name] = opt["max_speed_mps"] ** 2 / (2 * opt["max_accel_mps2"])
        times[name] = _trapezoidal_drive_time_s(single_cell_m, max_speed_mps=opt["max_speed_mps"],
                                                   max_accel_mps2=opt["max_accel_mps2"])
    triangular_ok = all(single_cell_m < 2 * d for d in accel_distances.values())
    direct_ok = triangular_ok and times["fast"] > times["stock"] and times["faster"] > times["fast"]
    print(f"  accel-to-cruise distance: stock={accel_distances['stock']:.3f}m, "
          f"fast={accel_distances['fast']:.3f}m, faster={accel_distances['faster']:.3f}m "
          f"(single cell = {single_cell_m:.4f}m -- every option stays triangular)")
    print(f"  single-cell drive time: stock={times['stock']:.4f}s, fast={times['fast']:.4f}s, "
          f"faster={times['faster']:.4f}s")
    print(f"real motor torque loss makes faster gearing STRICTLY SLOWER per cell, not faster: "
          f"{'OK' if direct_ok else 'FAIL'}")

    grid, start, goal, ground_truth, actual_start, tag_sites = _long_straight_scenario()
    stock = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                        random.Random(7), gearing="stock")
    fast = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                       random.Random(7), gearing="fast")
    faster = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                         random.Random(7), gearing="faster")
    end_to_end_ok = (stock.steps == fast.steps == faster.steps and fast.elapsed_s > stock.elapsed_s
                      and faster.elapsed_s > fast.elapsed_s)
    print(f"  (zero-drift) stock elapsed_s={stock.elapsed_s}, fast elapsed_s={fast.elapsed_s}, "
          f"faster elapsed_s={faster.elapsed_s} (same {stock.steps} steps in all three)")
    print(f"end to end (drift zeroed, isolating the speed effect): faster gearing completes the identical "
          f"path in strictly MORE elapsed_s, never less: {'OK' if end_to_end_ok else 'FAIL'}")
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


def check_no_gearing_option_rescues_a_tight_budget():
    """The inverse of what an earlier (pre-motor-spec-correction) version
    of this check verified: with real goBILDA torque figures, faster
    gearing doesn't just fail to help at a tight budget, it actively
    makes an over-budget failure worse, since it's strictly slower per
    cell (check_faster_gearing_is_slower_per_cell_not_faster above) on
    top of drifting more. Drift zeroed (see _zero_drift_suite) so this
    isolates the speed effect and the tight budget can be self-derived
    from stock's own elapsed_s rather than a hand-picked, scenario-
    fragile number."""
    grid, start, goal, ground_truth, actual_start, tag_sites = _long_straight_scenario()

    original_budget = match_module.AUTONOMOUS_PERIOD_S
    stock_at_real_budget = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start,
                                       tag_sites, random.Random(7), gearing="stock")
    # A budget strictly between "never binds" and "stock already fails" --
    # tight enough that stock goes over_budget, self-derived from stock's
    # own real-budget elapsed_s.
    tight_budget = stock_at_real_budget.elapsed_s * 0.9
    try:
        match_module.AUTONOMOUS_PERIOD_S = tight_budget
        stock_tight = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                  random.Random(7), gearing="stock")
        fast_tight = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                 random.Random(7), gearing="fast")
        faster_tight = run_match(_zero_drift_suite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                   random.Random(7), gearing="faster")
    finally:
        match_module.AUTONOMOUS_PERIOD_S = original_budget

    ok = stock_tight.over_budget and fast_tight.over_budget and faster_tight.over_budget
    print(f"  tight_budget={tight_budget:.3f}s (90% of stock's real-budget elapsed_s={stock_at_real_budget.elapsed_s}): "
          f"stock over_budget={stock_tight.over_budget}, fast over_budget={fast_tight.over_budget}, "
          f"faster over_budget={faster_tight.over_budget}")
    print(f"no gearing option rescues a budget stock already fails -- 'fast'/'faster' don't reduce "
          f"per-cell drive time at this grid scale: {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_stock_default_reproduces_legacy_exactly():
    assert check_stock_default_reproduces_legacy_exactly()


def test_faster_gearing_is_slower_per_cell_not_faster():
    assert check_faster_gearing_is_slower_per_cell_not_faster()


def test_faster_gearing_increases_drift():
    assert check_faster_gearing_increases_drift()


def test_no_gearing_option_rescues_a_tight_budget():
    assert check_no_gearing_option_rescues_a_tight_budget()


if __name__ == "__main__":
    checks = [
        check_stock_default_reproduces_legacy_exactly(),
        check_faster_gearing_is_slower_per_cell_not_faster(),
        check_faster_gearing_increases_drift(),
        check_no_gearing_option_rescues_a_tight_budget(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
