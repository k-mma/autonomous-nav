"""
Does ftc/match.py's `scripted_auto` mode actually behave like a fixed,
hand-tuned auto routine -- planning exactly once and never again, a
pose-fixing suite still measurably helping despite never rerouting, and
an obstacle-sensing suite getting NO benefit from what it senses, since
the only thing that knowledge could ever do (trigger a reroute) is
exactly what this mode removes?

Why this isn't just nav/policies.py's OpenLoopPolicy reused directly:
that class's Policy interface (`step(true_grid, current_cell) ->
next_cell`) is nav/'s minimal grid abstraction -- no drivetrain
kinematics, no real elapsed-time accounting, no sensor suites, no pose/
heading error, no collision recovery. ftc/match.py's run_match already
owns all of that machinery for the non-scripted case; reimplementing a
second, parallel FTC-aware match loop around OpenLoopPolicy would have
duplicated it wholesale for one boolean's worth of behavior change.
`scripted_auto=True` reuses the CONCEPT (plan once, execute blind) by
narrowing run_match's own existing replan trigger to `not
planned_once`, unconditionally -- see run_match's own docstring.

1. check_scripted_auto_plans_exactly_once: `replans` is always 0 under
   scripted_auto=True, on a scenario (moving obstacles + an obstacle-
   sensing suite) that would ordinarily force several replans -- not
   just "usually low," exactly zero.
2. check_scripted_auto_default_is_false_and_a_no_op: omitting
   scripted_auto and passing scripted_auto=False produce byte-identical
   MatchResults -- the same "explicit default matches omitted default"
   guarantee every other run_match parameter already has to prove.
3. check_pose_fixing_suite_still_helps_under_scripted_auto: AprilTag's
   success rate under scripted_auto is measurably higher than Dead
   reckoning's, pooled over many trials -- pose correction still works
   even though the route itself never changes.
4. check_obstacle_sensing_buys_nothing_under_scripted_auto: Distance
   sensors' success rate under scripted_auto is NOT measurably better
   than Dead reckoning's (the two suites that never fix pose and never
   sense anything AT ALL under this mode's own replan trigger) -- the
   quantified version of "an obstacle-sensing suite has no avenue to
   help when rerouting is impossible," not just an assertion.
"""
import random

from ftc.field import build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import DeadReckoningSuite, DistanceSensorSuite, AprilTagSuite
from nav.field_variance import generate_ground_truth
from nav.obstacles import MovingObstacle

LAYOUT = "cluttered"
TRIALS = 60


def _scenario(seed):
    grid = build_grid(LAYOUT)
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)
    rng = random.Random(seed)
    start, goal = rng.sample(free, 2)
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.5, seed=seed,
                                                          start_drift_scale=0.5, obstacle_drift_scale=0.5)
    return grid, start, goal, ground_truth, actual_start, tag_sites


def check_scripted_auto_plans_exactly_once():
    grid, start, goal, ground_truth, actual_start, tag_sites = _scenario(5)
    moving = [MovingObstacle(cell=free_cell, period_ms=200.0, rng=random.Random(5))
              for free_cell in [(r, c) for r in range(grid.size) for c in range(grid.size)
                                 if grid.cells[r][c] == 0][:3]]
    result = run_match(DistanceSensorSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                        random.Random(5), moving_obstacles=moving, scripted_auto=True)
    ok = result.replans == 0
    print(f"scripted_auto=True with moving obstacles + DistanceSensorSuite: replans={result.replans} "
          f"(expected 0) -- {'OK' if ok else 'FAIL'}")
    return ok


def check_scripted_auto_default_is_false_and_a_no_op():
    grid, start, goal, ground_truth, actual_start, tag_sites = _scenario(6)

    def _tuple(r):
        return (r.success, r.over_budget, round(r.elapsed_s, 9), r.collisions, r.replans,
                round(r.final_pose_error_in, 9), r.steps)

    omitted = run_match(AprilTagSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                         random.Random(6))
    explicit_false = run_match(AprilTagSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                random.Random(6), scripted_auto=False)
    ok = _tuple(omitted) == _tuple(explicit_false)
    print(f"scripted_auto omitted vs. scripted_auto=False: identical -- {'OK' if ok else 'FAIL'}")
    return ok


def _pooled_rate(suite_factory, trials=TRIALS, base_seed=1000):
    successes = 0
    for t in range(trials):
        seed = base_seed + t
        grid, start, goal, ground_truth, actual_start, tag_sites = _scenario(seed)
        result = run_match(suite_factory(), grid, start, goal, ground_truth, actual_start, tag_sites,
                            random.Random(seed), scripted_auto=True)
        successes += int(result.success)
    return successes / trials


def check_pose_fixing_suite_still_helps_under_scripted_auto():
    dead_reckoning_rate = _pooled_rate(DeadReckoningSuite, base_seed=2000)
    apriltag_rate = _pooled_rate(AprilTagSuite, base_seed=2000)
    ok = apriltag_rate > dead_reckoning_rate
    print(f"scripted_auto: dead reckoning={dead_reckoning_rate:.0%}, AprilTag={apriltag_rate:.0%} -- "
          f"AprilTag still measurably ahead despite never rerouting -- {'OK' if ok else 'FAIL'}")
    return ok


def check_obstacle_sensing_buys_nothing_under_scripted_auto():
    dead_reckoning_rate = _pooled_rate(DeadReckoningSuite, base_seed=3000)
    distance_rate = _pooled_rate(DistanceSensorSuite, base_seed=3000)
    # Not asserting exact equality (both still drift identically per
    # DEAD_RECKONING_DRIFT_PER_CELL, but random noise across 60 trials
    # each could still separate them by a couple of points by chance) --
    # the claim is that distance sensing has no SYSTEMATIC advantage
    # here, so a generous tolerance band around zero is the right check,
    # not "greater than."
    diff = distance_rate - dead_reckoning_rate
    ok = abs(diff) <= 0.15
    print(f"scripted_auto: dead reckoning={dead_reckoning_rate:.0%}, distance sensors={distance_rate:.0%} "
          f"(diff={diff:+.0%}, expected close to 0 -- obstacle sensing has no avenue to help) -- "
          f"{'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------


def test_scripted_auto_plans_exactly_once():
    assert check_scripted_auto_plans_exactly_once()


def test_scripted_auto_default_is_false_and_a_no_op():
    assert check_scripted_auto_default_is_false_and_a_no_op()


def test_pose_fixing_suite_still_helps_under_scripted_auto():
    assert check_pose_fixing_suite_still_helps_under_scripted_auto()


def test_obstacle_sensing_buys_nothing_under_scripted_auto():
    assert check_obstacle_sensing_buys_nothing_under_scripted_auto()


if __name__ == "__main__":
    checks = [
        check_scripted_auto_plans_exactly_once(),
        check_scripted_auto_default_is_false_and_a_no_op(),
        check_pose_fixing_suite_still_helps_under_scripted_auto(),
        check_obstacle_sensing_buys_nothing_under_scripted_auto(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
