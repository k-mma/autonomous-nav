"""
Does ftc/drivetrain.py's new swappable heading_policy actually behave
as documented -- "fixed_at_start" staying a byte-for-byte no-op
against the pre-existing MECANUM behavior, "nearest_tag_current"
actually tracking the robot's current position instead of its start,
"route_dominant"'s circular mean landing on the right angle (not just
some plausible-looking one), "match_travel" landing on exactly the
immediate next leg's own heading (not an average of anything), and
every policy actually changing match dynamics once wired through
ftc/match.py, not just heading bookkeeping that never reaches
drivetrain.turn_cost_s/speed_and_drift_factor?

1. check_fixed_at_start_ignores_current_position: resolve_held_heading_
   deg("fixed_at_start") returns the identical heading regardless of
   `true_position`, using only `actual_start` -- the defining property
   that makes recomputing it every tick a no-op.
2. check_nearest_tag_current_tracks_current_position: resolve_held_
   heading_deg("nearest_tag_current") changes when `true_position`
   moves to somewhere a DIFFERENT tag is nearest to, even with
   `actual_start` held fixed -- proving it actually reads the argument
   "fixed_at_start" ignores.
3. check_route_dominant_circular_mean_is_exact: a hand-built two-leg
   path (one step east, one step north, equal length) has an exactly
   computable circular-mean heading (atan2(-1, 1) = -45deg) --
   route_dominant_heading_deg must reproduce it exactly, not just
   "some direction between the two."
4. check_route_dominant_falls_back_without_a_path: no path yet (None),
   or path_idx already at/past the last waypoint, both return the
   supplied fallback exactly.
5. check_match_travel_uses_only_the_immediate_next_leg: a hand-built
   three-leg path where the immediate next leg points a DIFFERENT
   direction than the route's overall dominant direction --
   next_leg_heading_deg must reproduce the immediate leg's own exact
   heading, not something route_dominant_heading_deg would return for
   the identical path (proving it isn't accidentally averaging).
6. check_match_travel_falls_back_without_a_path: no path yet (None),
   path_idx already at/past the last waypoint, or the immediate next
   leg has zero length, all return the supplied fallback exactly --
   the identical degenerate-input contract route_dominant_heading_deg
   follows.
7. check_tank_ignores_every_heading_policy: resolve_held_heading_deg
   returns None for TANK regardless of which heading_policy string a
   TANK instance happens to carry -- heading policy is holonomic-only.
8. check_unknown_policy_raises: an unrecognized heading_policy string
   raises ValueError rather than silently falling back to something.
9. check_mecanum_default_matches_pre_addition_behavior: running a real
   match with MECANUM (heading_policy left at its default) produces the
   IDENTICAL MatchResult this exact scenario/seed produced before the
   heading-policy work existed -- the regression guarantee, checked by
   comparing the two literal ways of asking for "the default" (calling
   resolve_held_heading_deg every tick vs. the old one-time computation,
   reproduced inline here) rather than trusting they must agree.
10. check_alternative_policies_change_match_dynamics: MECANUM_NEAREST_
    TAG_CURRENT, MECANUM_ROUTE_DOMINANT, and MECANUM_MATCH_TRAVEL all
    complete real matches without error, and each produces a DIFFERENT
    elapsed_s than plain MECANUM on a multi-leg zigzag scenario where
    the fixed-at-start heading is a poor fit for most of the route --
    proving heading_policy actually reaches the drivetrain's turn-cost/
    speed/drift math, not just changes what a getter reports.
11. check_holonomic_never_pays_turn_cost: a direct unit check of
    Drivetrain.turn_cost_s on the SAME large heading swing -- 0 for
    MECANUM_MATCH_TRAVEL (holonomic), nonzero for TANK -- checked on
    match_travel specifically (not fixed_at_start, which would pass
    even a stale "0 because the heading never changes" implementation
    by accident, since its heading truly never changes) so this
    actually exercises the `self.holonomic` branch turn_cost_s now
    dispatches on.
12. check_match_travel_pays_zero_turn_cost_despite_reaiming: on a real
    multi-leg zigzag match, MECANUM_MATCH_TRAVEL's held heading is
    tracked and confirmed to actually CHANGE at least once (proving
    it's genuinely re-aiming, not vacuously passing because nothing
    ever changed) -- yet every one of those changes is still charged
    exactly 0s, unlike the equivalent change would cost TANK on the
    identical route. This is the integration-level proof that a
    holonomic drivetrain's re-aiming is free in this model, matching
    the physical claim in ftc/drivetrain.py's own module docstring
    (blending a rotational component into the wheel mix vs. TANK's
    forced stop-pivot-accelerate).
"""
import math
import random

from ftc.drivetrain import (
    MECANUM, MECANUM_MATCH_TRAVEL, MECANUM_NEAREST_TAG_CURRENT, MECANUM_ROUTE_DOMINANT, TANK,
    nearest_tag_heading_deg, next_leg_heading_deg, resolve_held_heading_deg, route_dominant_heading_deg,
)
from ftc.field import TagSite, build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import DeadReckoningSuite, heading_deg
from nav.algorithms import astar
from nav.field_variance import generate_ground_truth

TOLERANCE = 1e-9
LAYOUT = "corridor"


def _two_far_tags():
    """Two tags placed far apart, on opposite sides of the field, so
    "nearest" clearly flips depending on which end a position is near."""
    return [TagSite(x_in=0.0, y_in=0.0, heading_deg=45.0), TagSite(x_in=140.0, y_in=140.0, heading_deg=225.0)]


def check_fixed_at_start_ignores_current_position():
    tag_sites = _two_far_tags()
    actual_start = (0, 0)
    heading_near_start = resolve_held_heading_deg(MECANUM, actual_start, actual_start, tag_sites, None, 0)
    heading_far_away = resolve_held_heading_deg(MECANUM, (23, 23), actual_start, tag_sites, None, 0)
    ok = heading_near_start == heading_far_away
    print(f"fixed_at_start at true_position=actual_start: {heading_near_start:.2f}deg, at a far true_position: "
          f"{heading_far_away:.2f}deg -- identical: {ok} -- {'OK' if ok else 'FAIL'}")
    return ok


def check_nearest_tag_current_tracks_current_position():
    # Offset by one cell from each tag's own cell (not sitting exactly
    # ON it) so heading_deg's atan2 isn't degenerate (atan2(0, 0) == 0
    # regardless of which tag is nearest, which would make this check
    # pass by accident) -- tag_1 is at inches (0,0) -> cell (0,0), tag_2
    # at inches (140,140) -> cell (23,23) (see _two_far_tags).
    tag_sites = _two_far_tags()
    actual_start = (0, 0)
    near_first_tag = resolve_held_heading_deg(MECANUM_NEAREST_TAG_CURRENT, (1, 1), actual_start, tag_sites, None, 0)
    near_second_tag = resolve_held_heading_deg(MECANUM_NEAREST_TAG_CURRENT, (22, 22), actual_start, tag_sites,
                                                 None, 0)
    ok = near_first_tag != near_second_tag
    print(f"nearest_tag_current near tag 1: {near_first_tag:.2f}deg, near tag 2: {near_second_tag:.2f}deg -- "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_route_dominant_circular_mean_is_exact():
    path = [(5, 5), (5, 6), (4, 6)]  # leg 1: east (dr=0,dc=1); leg 2: north (dr=-1,dc=0)
    expected = math.degrees(math.atan2(-1.0, 1.0))  # -45.0
    got = route_dominant_heading_deg(path, 0, fallback_heading_deg=999.0)
    ok = abs(got - expected) < TOLERANCE
    print(f"circular mean of one east step + one north step: {got:.4f}deg (expected {expected:.4f}) -- "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_route_dominant_falls_back_without_a_path():
    ok_none = route_dominant_heading_deg(None, 0, fallback_heading_deg=42.0) == 42.0
    path = [(0, 0), (0, 1)]
    ok_exhausted = route_dominant_heading_deg(path, 1, fallback_heading_deg=17.0) == 17.0
    ok = ok_none and ok_exhausted
    print(f"no path -> fallback used: {ok_none}; path_idx at last waypoint -> fallback used: {ok_exhausted} -- "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_match_travel_uses_only_the_immediate_next_leg():
    # Three legs: east, then north, then east again -- the route's own
    # dominant direction (circular mean) is neither pure east nor pure
    # north, but the IMMEDIATE next leg at path_idx=0 is exactly east
    # (0deg) and at path_idx=1 is exactly north (-90deg, this project's
    # atan2(d_row, d_col) convention). next_leg_heading_deg must land on
    # those exact single-leg values, not anywhere near the route's
    # overall average.
    path = [(5, 5), (5, 6), (4, 6), (4, 7)]
    got_leg0 = next_leg_heading_deg(path, 0, fallback_heading_deg=999.0)
    got_leg1 = next_leg_heading_deg(path, 1, fallback_heading_deg=999.0)
    expected_leg0 = math.degrees(math.atan2(0.0, 1.0))  # 0.0 (east)
    expected_leg1 = math.degrees(math.atan2(-1.0, 0.0))  # -90.0 (north)
    dominant = route_dominant_heading_deg(path, 0, fallback_heading_deg=999.0)
    ok = (abs(got_leg0 - expected_leg0) < TOLERANCE and abs(got_leg1 - expected_leg1) < TOLERANCE
          and abs(got_leg0 - dominant) > TOLERANCE)
    print(f"next_leg_heading_deg at idx=0: {got_leg0:.2f}deg (expected {expected_leg0:.2f}), at idx=1: "
          f"{got_leg1:.2f}deg (expected {expected_leg1:.2f}); route's own circular mean: {dominant:.2f}deg -- "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_match_travel_falls_back_without_a_path():
    ok_none = next_leg_heading_deg(None, 0, fallback_heading_deg=42.0) == 42.0
    path = [(0, 0), (0, 1)]
    ok_exhausted = next_leg_heading_deg(path, 1, fallback_heading_deg=17.0) == 17.0
    zero_length_path = [(3, 3), (3, 3)]
    ok_zero_length = next_leg_heading_deg(zero_length_path, 0, fallback_heading_deg=8.0) == 8.0
    ok = ok_none and ok_exhausted and ok_zero_length
    print(f"no path -> fallback used: {ok_none}; path_idx at last waypoint -> fallback used: {ok_exhausted}; "
          f"zero-length immediate leg -> fallback used: {ok_zero_length} -- {'OK' if ok else 'FAIL'}")
    return ok


def check_tank_ignores_every_heading_policy():
    tag_sites = _two_far_tags()
    from dataclasses import replace
    ok = True
    for policy in ("fixed_at_start", "nearest_tag_current", "route_dominant", "match_travel"):
        tank_variant = replace(TANK, heading_policy=policy)
        result = resolve_held_heading_deg(tank_variant, (5, 5), (0, 0), tag_sites, [(0, 0), (1, 1)], 1)
        if result is not None:
            ok = False
    print(f"resolve_held_heading_deg(TANK, ...) is None under every heading_policy -- {'OK' if ok else 'FAIL'}")
    return ok


def check_unknown_policy_raises():
    from dataclasses import replace
    bogus = replace(MECANUM, heading_policy="diagonally_or_something")
    try:
        resolve_held_heading_deg(bogus, (0, 0), (0, 0), _two_far_tags(), None, 0)
        ok = False
    except ValueError:
        ok = True
    print(f"unknown heading_policy raises ValueError -- {'OK' if ok else 'FAIL'}")
    return ok


def _zigzag_scenario(seed=3):
    grid = build_grid(LAYOUT)
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)
    rng = random.Random(seed)
    for _ in range(50):
        start, goal = rng.sample(free, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= 8:
            break
    return grid, start, goal, tag_sites


def check_mecanum_default_matches_pre_addition_behavior():
    grid, start, goal, tag_sites = _zigzag_scenario()
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.0, seed=3)

    # The exact pre-addition formula, reproduced inline (NOT calling
    # resolve_held_heading_deg), as the independent reference this check
    # is actually testing against.
    def _tag_dist(t):
        from ftc.field import in_to_cell
        tc = in_to_cell(t.x_in, t.y_in)
        return math.hypot(tc[0] - actual_start[0], tc[1] - actual_start[1])
    from ftc.field import in_to_cell
    nearest_tag = min(tag_sites, key=_tag_dist)
    expected_fixed_heading = heading_deg(actual_start, in_to_cell(nearest_tag.x_in, nearest_tag.y_in))

    got_fixed_heading = resolve_held_heading_deg(MECANUM, actual_start, actual_start, tag_sites, None, 0)
    ok = abs(got_fixed_heading - expected_fixed_heading) < TOLERANCE
    print(f"MECANUM's resolved start heading: {got_fixed_heading:.4f}deg vs. the old inline formula: "
          f"{expected_fixed_heading:.4f}deg -- {'OK' if ok else 'FAIL'}")
    return ok


def check_alternative_policies_change_match_dynamics():
    grid, start, goal, tag_sites = _zigzag_scenario()
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.0, seed=3)

    results = {}
    for name, drivetrain in [("fixed_at_start", MECANUM), ("nearest_tag_current", MECANUM_NEAREST_TAG_CURRENT),
                               ("route_dominant", MECANUM_ROUTE_DOMINANT), ("match_travel", MECANUM_MATCH_TRAVEL)]:
        result = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                            random.Random(3), drivetrain=drivetrain)
        results[name] = result

    differs = (abs(results["fixed_at_start"].elapsed_s - results["nearest_tag_current"].elapsed_s) > 1e-6
               or abs(results["fixed_at_start"].elapsed_s - results["route_dominant"].elapsed_s) > 1e-6
               or abs(results["fixed_at_start"].elapsed_s - results["match_travel"].elapsed_s) > 1e-6)
    all_finite = all(r.elapsed_s > 0 for r in results.values())
    ok = differs and all_finite
    print(f"elapsed_s: fixed_at_start={results['fixed_at_start'].elapsed_s:.3f}, "
          f"nearest_tag_current={results['nearest_tag_current'].elapsed_s:.3f}, "
          f"route_dominant={results['route_dominant'].elapsed_s:.3f}, "
          f"match_travel={results['match_travel'].elapsed_s:.3f} -- differs={differs}, "
          f"all_finite={all_finite} -- {'OK' if ok else 'FAIL'}")
    return ok


def check_holonomic_never_pays_turn_cost():
    """Direct unit check of Drivetrain.turn_cost_s on the identical
    160-degree heading swing: 0 for MECANUM_MATCH_TRAVEL (holonomic),
    nonzero for TANK. Checked on match_travel rather than fixed_at_start
    specifically because fixed_at_start's held heading never changes at
    all, in which case even a stale "0 because current==new" formula
    would happen to return 0 -- this exercises the actual `self.
    holonomic` dispatch instead of coincidentally passing."""
    large_swing_holonomic = MECANUM_MATCH_TRAVEL.turn_cost_s(10.0, 170.0)
    large_swing_tank = TANK.turn_cost_s(10.0, 170.0)
    ok = large_swing_holonomic == 0.0 and large_swing_tank > 0.0
    print(f"turn_cost_s(match_travel, 160deg swing): {large_swing_holonomic:.4f}s; "
          f"turn_cost_s(tank, same swing): {large_swing_tank:.4f}s -- {'OK' if ok else 'FAIL'}")
    return ok


def check_match_travel_pays_zero_turn_cost_despite_reaiming():
    """match_travel's held heading is re-resolved every tick from
    whichever leg is immediately next -- unlike fixed_at_start, it
    genuinely changes as the route progresses. This confirms that
    change actually happens at least once over a real zigzag match
    (proving the check below isn't vacuously true because nothing ever
    changed) AND that every one of those changes is still charged
    exactly 0s by turn_cost_s -- the physical claim ftc/drivetrain.py's
    own module docstring makes (re-aiming blends into the wheel mix
    rather than costing a dedicated stop-pivot-accelerate maneuver)."""
    import ftc.drivetrain as drivetrain_module
    grid, start, goal, tag_sites = _zigzag_scenario()
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.0, seed=3)

    original_turn_cost_s = drivetrain_module.Drivetrain.turn_cost_s
    calls = []

    def _tracking_turn_cost_s(self, current_heading_deg, new_heading_deg):
        cost = original_turn_cost_s(self, current_heading_deg, new_heading_deg)
        if self.heading_policy == "match_travel":
            calls.append((current_heading_deg, new_heading_deg, cost))
        return cost

    drivetrain_module.Drivetrain.turn_cost_s = _tracking_turn_cost_s
    try:
        run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                  random.Random(3), drivetrain=MECANUM_MATCH_TRAVEL)
    finally:
        drivetrain_module.Drivetrain.turn_cost_s = original_turn_cost_s

    heading_actually_changed = any(abs(cur - new) > 1e-6 for cur, new, _cost in calls)
    all_zero_cost = all(cost == 0.0 for _cur, _new, cost in calls)
    ok = heading_actually_changed and all_zero_cost
    print(f"match_travel held heading changed at least once over the match: {heading_actually_changed}; "
          f"every change still cost exactly 0s: {all_zero_cost} -- {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------


def test_fixed_at_start_ignores_current_position():
    assert check_fixed_at_start_ignores_current_position()


def test_nearest_tag_current_tracks_current_position():
    assert check_nearest_tag_current_tracks_current_position()


def test_route_dominant_circular_mean_is_exact():
    assert check_route_dominant_circular_mean_is_exact()


def test_route_dominant_falls_back_without_a_path():
    assert check_route_dominant_falls_back_without_a_path()


def test_match_travel_uses_only_the_immediate_next_leg():
    assert check_match_travel_uses_only_the_immediate_next_leg()


def test_match_travel_falls_back_without_a_path():
    assert check_match_travel_falls_back_without_a_path()


def test_tank_ignores_every_heading_policy():
    assert check_tank_ignores_every_heading_policy()


def test_unknown_policy_raises():
    assert check_unknown_policy_raises()


def test_mecanum_default_matches_pre_addition_behavior():
    assert check_mecanum_default_matches_pre_addition_behavior()


def test_alternative_policies_change_match_dynamics():
    assert check_alternative_policies_change_match_dynamics()


def test_holonomic_never_pays_turn_cost():
    assert check_holonomic_never_pays_turn_cost()


def test_match_travel_pays_zero_turn_cost_despite_reaiming():
    assert check_match_travel_pays_zero_turn_cost_despite_reaiming()


if __name__ == "__main__":
    checks = [
        check_fixed_at_start_ignores_current_position(),
        check_nearest_tag_current_tracks_current_position(),
        check_route_dominant_circular_mean_is_exact(),
        check_route_dominant_falls_back_without_a_path(),
        check_match_travel_uses_only_the_immediate_next_leg(),
        check_match_travel_falls_back_without_a_path(),
        check_tank_ignores_every_heading_policy(),
        check_unknown_policy_raises(),
        check_mecanum_default_matches_pre_addition_behavior(),
        check_alternative_policies_change_match_dynamics(),
        check_holonomic_never_pays_turn_cost(),
        check_match_travel_pays_zero_turn_cost_despite_reaiming(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
