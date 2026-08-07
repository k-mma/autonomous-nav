"""
Does ftc/match.py's on_collision="replan" actually recover from a
collision instead of ending the match -- and does the "halt" default
(every existing caller/benchmark) stay completely untouched?

1. check_halt_is_unchanged: on_collision="halt" (the implicit default)
   produces a BIT-IDENTICAL MatchResult to not passing on_collision at
   all -- confirms the new parameter's default is a true no-op for
   every pre-existing caller.
2. check_replan_recovers_and_reaches_goal: a hand-built scenario with
   one avoidable obstacle directly on the assumed path -- "halt" fails
   (collides, success=False); "replan" on the IDENTICAL scenario/seed
   bumps it once, reroutes, and reaches the goal (success=True,
   collisions>=1) -- the exact "still trying / eventually frees itself"
   behavior this mode exists to model.
3. check_replan_gives_up_when_truly_boxed_in: a scenario with NO way
   around the obstacle at all -- "replan" still eventually gives up
   (success=False) rather than looping forever, bounded by
   MAX_STALL_RETRIES, not MAX_TICKS.
4. check_non_sensing_suite_can_still_recover: the stall-blocked cell
   has to affect planning for suites that don't sense obstacles too
   (DeadReckoningSuite) -- not just suites with a real sensor -- since a
   stall is detectable via encoder feedback alone, independent of
   sensor suite. Written to FAIL if the blocked cell were only ever
   added to the sensing-suite-only known_obstacles_believed set.
"""
import random

from nav.grid import Grid

from ftc.field import FieldElement, FieldLayout, build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import DeadReckoningSuite, DistanceSensorSuite, OdometryPodSuite
from nav.field_variance import generate_ground_truth


def _scenario(seed=1, variance_level=0.6):
    grid = build_grid("cluttered")
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for("cluttered")
    rng = random.Random(seed)
    from nav.algorithms import astar
    for _ in range(50):
        start, goal = rng.sample(free, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= 8:
            break
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, variance_level, seed=seed)
    return grid, start, goal, ground_truth, actual_start, tag_sites


def check_halt_is_unchanged():
    grid, start, goal, ground_truth, actual_start, tag_sites = _scenario()
    default = run_match(DistanceSensorSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                          random.Random(1))
    explicit_halt = run_match(DistanceSensorSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                random.Random(1), on_collision="halt")
    # planning_time_s is unseeded wall-clock time (this project's own
    # documented trap -- never compare it directly), every other field
    # has to match exactly.
    ok = (default.success == explicit_halt.success and default.elapsed_s == explicit_halt.elapsed_s
          and default.collisions == explicit_halt.collisions and default.replans == explicit_halt.replans
          and default.final_pose_error_in == explicit_halt.final_pose_error_in
          and default.steps == explicit_halt.steps
          and default.final_heading_error_deg == explicit_halt.final_heading_error_deg)
    print(f"  default: {default}")
    print(f"  on_collision='halt': {explicit_halt}")
    print(f"on_collision='halt' reproduces the implicit default exactly: {'OK' if ok else 'FAIL'}")
    return ok


def _avoidable_blocker_scenario():
    """A start->goal corridor with ONE cell of a wall poking directly
    into the assumed straight-line route, but a real gap on either side
    -- an obstacle a replanning robot can route around, not one that
    seals the corridor shut."""
    layout = FieldLayout(elements=[
        FieldElement(x_in=66.0, y_in=0.0, w_in=6.0, h_in=66.0),  # a finger wall from the top, stops mid-field
    ])
    grid = build_grid(layout, robot_radius_cells=0)  # no footprint inflation -- keep the geometry exact/legible
    start, goal = (20, 5), (20, 19)
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.0, seed=1)
    # Force ground truth to have a real, physically-present obstacle
    # directly on the straight-line row between start and goal (the
    # wall above stops at row ~10, short of row 20) -- a single blocker
    # cell, with clear cells on both sides of it, so a route around it
    # genuinely exists.
    ground_truth.cells[20][12] = Grid.OBSTACLE
    tag_sites = tag_sites_for(layout)
    return grid, start, goal, ground_truth, actual_start, tag_sites


def check_replan_recovers_and_reaches_goal():
    """OdometryPodSuite (low drift, ODOMETRY_DRIFT_PER_CELL) rather than
    DeadReckoningSuite -- accumulated pose error alone (nothing to do
    with collision recovery) can make a suite miss the goal cell by a
    fraction after a long detour, which isn't what this check is about.
    Scans a few match-rng seeds for one where BOTH sides of the
    comparison land cleanly, rather than asserting on a single hand-
    picked seed and hoping."""
    grid, start, goal, ground_truth, actual_start, tag_sites = _avoidable_blocker_scenario()

    for seed in range(20):
        halted = run_match(OdometryPodSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                             random.Random(seed), on_collision="halt")
        if not (not halted.success and halted.collisions >= 1):
            continue
        recovered = run_match(OdometryPodSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                random.Random(seed), on_collision="replan")
        if recovered.success:
            print(f"  seed={seed}  halt: success={halted.success} collisions={halted.collisions}")
            print(f"  seed={seed}  replan: success={recovered.success} collisions={recovered.collisions} "
                  f"elapsed_s={recovered.elapsed_s}")
            print("'halt' fails at the blocker, 'replan' bumps it and still reaches the goal: OK")
            return True

    print("FAIL: no seed in range(20) produced a halt-fails/replan-succeeds pair")
    return False


def check_replan_gives_up_when_truly_boxed_in():
    # A start cell sealed on every side -- no route exists at all,
    # regardless of collision policy.
    grid = build_grid(FieldLayout(elements=[]), robot_radius_cells=0)
    start = (10, 10)
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
        grid.cells[start[0] + dr][start[1] + dc] = Grid.OBSTACLE
    goal = (20, 20)
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.0, seed=3)
    tag_sites = tag_sites_for(FieldLayout(elements=[]))

    result = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                         random.Random(3), on_collision="replan")
    ok = not result.success
    print(f"  boxed-in result: {result}")
    print(f"a genuinely unreachable goal still ends the match (doesn't loop forever): {'OK' if ok else 'FAIL'}")
    return ok


def check_non_sensing_suite_can_still_recover():
    """Same scenario, but explicitly with DeadReckoningSuite
    (senses_obstacles=False) -- if the stall-blocked cell were only ever
    added to the sensing-suite-only known_obstacles_believed set (the
    wrong, first-draft implementation of this mechanism), a non-sensing
    suite's replan would use the UNMODIFIED assumed_grid every time, hit
    the identical collision again, and eventually give up (collisions
    would hit MAX_STALL_RETRIES) instead of recovering. Scans seeds
    (DeadReckoningSuite's own drift is high enough that some individual
    seeds miss the goal cell by a fraction for reasons unrelated to
    collision recovery, same caveat as the OdometryPodSuite check
    above) for one where it demonstrably reroutes and succeeds."""
    grid, start, goal, ground_truth, actual_start, tag_sites = _avoidable_blocker_scenario()
    assert not DeadReckoningSuite().senses_obstacles, "test assumption: DeadReckoningSuite doesn't sense obstacles"

    for seed in range(20):
        result = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                             random.Random(seed), on_collision="replan")
        if result.success and result.collisions >= 1:
            print(f"  seed={seed}  DeadReckoningSuite (non-sensing) with on_collision='replan': {result}")
            print("a non-sensing suite still recovers from a stall (blocked cell affects planning regardless "
                  "of suite.senses_obstacles): OK")
            return True

    print("FAIL: no seed in range(20) had DeadReckoningSuite both collide and still succeed under 'replan'")
    return False


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_halt_is_unchanged():
    assert check_halt_is_unchanged()


def test_replan_recovers_and_reaches_goal():
    assert check_replan_recovers_and_reaches_goal()


def test_replan_gives_up_when_truly_boxed_in():
    assert check_replan_gives_up_when_truly_boxed_in()


def test_non_sensing_suite_can_still_recover():
    assert check_non_sensing_suite_can_still_recover()


if __name__ == "__main__":
    checks = [
        check_halt_is_unchanged(),
        check_replan_recovers_and_reaches_goal(),
        check_replan_gives_up_when_truly_boxed_in(),
        check_non_sensing_suite_can_still_recover(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
