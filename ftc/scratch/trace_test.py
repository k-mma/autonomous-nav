"""
Does ftc/trace.py's record_match() -- and the on_tick hook in ftc/
match.py it's built on -- actually leave run_match()'s own simulation
untouched, the safety property both modules' docstrings claim? This is
the regression guarantee pygame_app/ftc_viz/'s animated visualizer
depends on: if recording a trace could perturb `rng`, it would silently
invalidate every number this project has already published.

1. check_trace_reproduces_plain_run_match_exactly: the identical
   (suite, scenario, seed) run once through plain run_match() and once
   through record_match() must return BIT-IDENTICAL MatchResults --
   proving on_tick's presence doesn't consume rng or otherwise change
   what gets computed.
2. check_tick_count_matches_steps: the number of "step" events in a
   trace equals MatchResult.steps -- the trace isn't silently dropping
   or duplicating ticks.
3. check_trace_covers_collision_and_success: on a scenario forced to
   collide and one that succeeds, the trace's last event is "end" and
   (for the collision case) a "collision" event with an
   attempted_position appears immediately before it.
4. check_on_tick_none_is_the_default: calling run_match without
   on_tick at all (every pre-existing caller) still works -- on_tick's
   own default is None, and _emit's internal guard means every new call
   site in the loop is a true no-op then, not just an unused callback.
"""
import random

from ftc.field import build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import DeadReckoningSuite, DistanceSensorSuite
from ftc.trace import record_match
from nav.algorithms import astar
from nav.field_variance import generate_ground_truth

LAYOUT = "cluttered"


def _scenario(seed, variance_level=0.6, **scale_kwargs):
    grid = build_grid(LAYOUT)
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)
    rng = random.Random(seed)
    for _ in range(50):
        start, goal = rng.sample(free, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= 8:
            break
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, variance_level, seed=seed, **scale_kwargs)
    return grid, start, goal, ground_truth, actual_start, tag_sites


def check_trace_reproduces_plain_run_match_exactly():
    grid, start, goal, ground_truth, actual_start, tag_sites = _scenario(11)

    plain = run_match(DistanceSensorSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                        random.Random(11))
    traced = record_match(DistanceSensorSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                            random.Random(11))

    ok = (plain.success == traced.result.success and plain.elapsed_s == traced.result.elapsed_s
          and plain.collisions == traced.result.collisions and plain.replans == traced.result.replans
          and plain.final_pose_error_in == traced.result.final_pose_error_in
          and plain.final_heading_error_deg == traced.result.final_heading_error_deg
          and plain.steps == traced.result.steps)
    print(f"  plain:  {plain}")
    print(f"  traced: {traced.result}")
    print(f"record_match's on_tick hook leaves run_match's own MatchResult byte-identical: {'OK' if ok else 'FAIL'}")
    return ok


def check_tick_count_matches_steps():
    # Scan for a scenario that actually takes several steps -- steps==0
    # (an immediate collision/stuck) would make len(step_events)==0==
    # steps trivially true for the wrong reason.
    traced = None
    for seed in range(30):
        grid, start, goal, ground_truth, actual_start, tag_sites = _scenario(seed)
        candidate = record_match(DistanceSensorSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                   random.Random(seed))
        if candidate.result.steps >= 4:
            traced = candidate
            break
    assert traced is not None, "no seed in range(30) produced a multi-step match"

    step_events = [t for t in traced.ticks if t["event"] == "step"]
    ok = len(step_events) == traced.result.steps
    print(f"  MatchResult.steps={traced.result.steps}, trace 'step' events={len(step_events)}")
    print(f"trace step-event count matches MatchResult.steps exactly: {'OK' if ok else 'FAIL'}")
    return ok


def check_trace_covers_collision_and_success():
    # A dense, zero-coverage-cone suite on a cluttered layout with heavy
    # obstacle drift reliably collides at least once across a handful of
    # seeds -- scan for one rather than assuming a specific seed does.
    collided = None
    for seed in range(30):
        grid, start, goal, ground_truth, actual_start, tag_sites = _scenario(
            seed, variance_level=1.0, start_drift_scale=0.0, obstacle_drift_scale=1.0, blocker_scale=0.0)
        traced = record_match(DistanceSensorSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                random.Random(seed))
        if traced.result.collisions > 0:
            collided = traced
            break
    collision_ok = False
    if collided is not None:
        last_two = [t["event"] for t in collided.ticks[-2:]]
        collision_ok = last_two == ["collision", "end"] and "attempted_position" in collided.ticks[-2]
    print(f"  collision case: last two events={[t['event'] for t in collided.ticks[-2:]] if collided else None}")
    print(f"a colliding match's trace ends with a 'collision' event (attempted_position set) then 'end': "
          f"{'OK' if collision_ok else 'FAIL'}")

    succeeded = None
    for seed in range(30):
        grid, start, goal, ground_truth, actual_start, tag_sites = _scenario(seed, variance_level=0.0)
        traced = record_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                random.Random(seed))
        if traced.result.success:
            succeeded = traced
            break
    success_ok = False
    if succeeded is not None:
        last = succeeded.ticks[-1]
        success_ok = last["event"] == "end" and last["success"] is True
    print(f"  success case: last event={succeeded.ticks[-1] if succeeded else None}")
    print(f"a successful match's trace ends with an 'end' event with success=True: {'OK' if success_ok else 'FAIL'}")

    return collision_ok and success_ok


def check_on_tick_none_is_the_default():
    grid, start, goal, ground_truth, actual_start, tag_sites = _scenario(13)
    try:
        result = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                             random.Random(13))
        ok = result is not None
    except Exception as e:
        ok = False
        print(f"  run_match without on_tick raised: {e!r}")
    print(f"run_match works with no on_tick argument at all (every pre-existing caller): {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_trace_reproduces_plain_run_match_exactly():
    assert check_trace_reproduces_plain_run_match_exactly()


def test_tick_count_matches_steps():
    assert check_tick_count_matches_steps()


def test_trace_covers_collision_and_success():
    assert check_trace_covers_collision_and_success()


def test_on_tick_none_is_the_default():
    assert check_on_tick_none_is_the_default()


if __name__ == "__main__":
    checks = [
        check_trace_reproduces_plain_run_match_exactly(),
        check_tick_count_matches_steps(),
        check_trace_covers_collision_and_success(),
        check_on_tick_none_is_the_default(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
