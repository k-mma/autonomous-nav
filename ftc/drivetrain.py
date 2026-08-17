"""
A drivetrain (Priority 2 of the fidelity-tier expansion) is an axis
orthogonal to sensor suite -- ANY suite from ftc/sensors.py can run on
either drivetrain, so this lives as its own dimension, swept like
ftc/layout_benchmark.py's field layout or ftc/budget_benchmark.py's
budget rather than folded into ftc/sensors.py's SUITE_ORDER (see
ftc/drivetrain_benchmark.py).

ftc/field.py already defaults to diagonal=True "on the assumption of a
holonomic drivetrain," and README.md's "Threats to validity" names "no
mecanum-specific strafing advantage" as an open limitation this module
closes: TANK must physically rotate to face its direction of travel
before every direction change -- the flat TURN_TIME_PER_90DEG_S cost
ftc/match.py already charged, unconditionally, before this module
existed IS tank behavior, and stays the exact default when no
drivetrain is given. MECANUM can translate in any direction while
holding a fixed chassis heading, at the cost of a real drivetrain-price
premium (goBILDA's 96mm mecanum wheel set vs. its 96mm traction wheel
set -- see ftc/config.py's MECANUM_WHEEL_COST_USD/TANK_WHEEL_COST_USD
for the current sourced prices) and a speed/drift penalty on any step
that isn't roughly "forward" relative to whatever heading it's holding.

Heading policy, chosen and documented rather than left implicit --
and, as of the heading-policy work below, a SWAPPABLE choice rather
than one hardcoded shape: MECANUM's default policy
(`heading_policy="fixed_at_start"`) holds a fixed heading for the whole
match, aimed at the nearest AprilTag wall site FROM THE STARTING
POSITION, resolved once (ftc/match.py's own resolution just happens to
call the identical pure function every tick instead of caching it --
see resolve_held_heading_deg below, and its own docstring for why that
change is a byte-for-byte no-op for this policy specifically) -- the
natural choice for a robot that's investing in an AprilTag-reading
camera at all, and the specific interaction ftc/drivetrain_
benchmark.py's writeup is built around: holding that heading keeps the
camera aimed at tags for the entire match, which Priority 1's camera-
FOV gating should reward heavily -- while a tank robot rotates its
camera away from the tag wall every time it changes direction.
ftc/drivetrain_benchmark.py's own measured finding, though, is that
this specific policy LOSES to tank by a wide margin (see
benchmark_results/ftc_drivetrain_writeup.md): a route's actual travel
direction changes on almost every leg, so a heading fixed once at match
start ends up strafing on most steps, and the resulting drift/speed
penalty swamps the camera-alignment benefit the policy was chosen to
demonstrate. THREE alternative policies exist specifically to check
whether that's a property of mecanum drivetrains in general or of this
one particular (never re-aimed) policy -- see HEADING_POLICIES below
and ftc/drivetrain_benchmark.py's own policy x fidelity sweep for the
answer. Two of those three (`nearest_tag_current`, `route_dominant`)
still don't directly minimize per-step strafe against the route's own
IMMEDIATE next leg; the third, `match_travel`, does exactly that --
holding the heading TANK would already be facing on that leg, computed
fresh each tick from path[path_idx] -> path[path_idx + 1] (see
next_leg_heading_deg below).

That does NOT make `match_travel` a strictly-better-by-construction
policy, and in particular it does not reproduce "MECANUM never pays a
turn cost" the way `fixed_at_start` does: turn_cost_s (below) charges
for any change in the CHASSIS heading regardless of which policy
produced it, and `match_travel`'s held heading changes on almost every
leg -- about as often as TANK's own does -- so it pays a real, nonzero
turn cost too, just not necessarily the identical amount TANK pays for
the identical route (measured, not assumed: on the same scenario
`match_travel` has come out both cheaper AND more expensive than TANK's
turn cost across different seeds). What `match_travel` actually trades
away is the STRAFE penalty, not the turn cost -- it faces its direction
of travel on most legs (avoiding MECANUM_STRAFE_SPEED_FACTOR/_DRIFT_
MULTIPLIER the way TANK always does) while still being a holonomic
chassis that never needs to physically rotate before moving, so
whatever turn cost it does pay is the flat proportional cost of
updating a HELD heading, not a physical prerequisite to motion.
Whether that trade nets out ahead of tank is a measured result, not a
guarantee -- see ftc/drivetrain_benchmark.py's own writeup.

TANK always faces its current direction of travel, exactly matching
the existing (pre-Priority-2) behavior -- ftc/match.py's
`drivetrain=None` default reproduces that unchanged for every existing
caller, and an explicit TANK instance is functionally identical to
that default (see ftc/scratch/drivetrain_test.py's consistency check).
Heading policy only ever applies to a holonomic drivetrain (TANK's own
robot_heading_deg ignores it completely) so it has no effect on TANK at
all, regardless of what `heading_policy` a TANK instance happens to
carry.
"""
import math
from dataclasses import dataclass

from ftc.config import (
    MECANUM_STRAFE_DRIFT_MULTIPLIER, MECANUM_STRAFE_SPEED_FACTOR,
    MECANUM_WHEEL_COST_USD, TANK_WHEEL_COST_USD, TURN_TIME_PER_90DEG_S,
)
from ftc.field import in_to_cell
from ftc.sensors import angular_diff, heading_deg

# The four heading policies a holonomic (MECANUM) drivetrain can hold,
# see resolve_held_heading_deg below for what each one actually
# computes and module docstring above for why more than one exists.
HEADING_POLICIES = ["fixed_at_start", "nearest_tag_current", "route_dominant", "match_travel"]
HEADING_POLICY_LABELS = {
    "fixed_at_start": "Fixed at match start (nearest tag wall)",
    "nearest_tag_current": "Re-aim toward nearest tag from current position",
    "route_dominant": "Aim along the route's own dominant direction",
    "match_travel": "Match the immediate next leg's own direction",
}


def nearest_tag_heading_deg(position, tag_sites):
    """The heading FROM `position` TOWARD whichever tag_sites entry is
    geometrically closest to it (straight-line distance in cells,
    identical tie-breaking to Python's own min() -- first-in-list wins
    a tie), in the same atan2(d_row, d_col) convention every heading in
    this project uses (ftc/sensors.py's heading_deg). Returns None if
    tag_sites is empty -- nothing to aim at, so no held heading can be
    computed from this policy at all."""
    if not tag_sites:
        return None

    def _tag_dist(t):
        tc = in_to_cell(t.x_in, t.y_in)
        return math.hypot(tc[0] - position[0], tc[1] - position[1])

    nearest_tag = min(tag_sites, key=_tag_dist)
    return heading_deg(position, in_to_cell(nearest_tag.x_in, nearest_tag.y_in))


def route_dominant_heading_deg(path, path_idx, fallback_heading_deg):
    """The circular mean travel direction of `path` from `path_idx`
    onward -- a unit vector is taken for each remaining leg (so a long
    leg doesn't outweigh a short one) and summed, then converted back to
    an angle; this is the standard way to average angles without the
    wraparound error a plain mean of degree values would introduce
    (e.g. averaging 359deg and 1deg should give 0deg, not 180deg).
    Choosing THIS heading minimizes the route's own TOTAL strafe against
    a single fixed choice, which is exactly the quantity ftc/
    drivetrain.py's speed_and_drift_factor penalizes.

    Falls back to `fallback_heading_deg` when there's no usable route
    information yet (`path` is None/empty, `path_idx` is already at or
    past the last waypoint, or every remaining leg happens to have zero
    length -- degenerate, but not impossible, if the goal is reached in
    the same tick this is computed) rather than returning an arbitrary
    angle from a zero vector."""
    if not path or path_idx >= len(path) - 1:
        return fallback_heading_deg
    sum_dr, sum_dc = 0.0, 0.0
    for i in range(path_idx, len(path) - 1):
        dr = path[i + 1][0] - path[i][0]
        dc = path[i + 1][1] - path[i][1]
        norm = math.hypot(dr, dc)
        if norm > 1e-9:
            sum_dr += dr / norm
            sum_dc += dc / norm
    if abs(sum_dr) < 1e-9 and abs(sum_dc) < 1e-9:
        return fallback_heading_deg
    return math.degrees(math.atan2(sum_dr, sum_dc))


def next_leg_heading_deg(path, path_idx, fallback_heading_deg):
    """The direction of travel of ONLY the immediate next leg --
    heading_deg(path[path_idx], path[path_idx + 1]) -- the same rule
    TANK already applies every step (Drivetrain.robot_heading_deg's
    non-holonomic branch just returns step_heading_deg directly), but
    computed here as a HELD heading a holonomic drivetrain can adopt
    instead. Unlike route_dominant_heading_deg's circular mean over
    every remaining leg, this uses no averaging at all -- it only ever
    looks at the single leg about to be driven, so it changes every
    time path_idx advances onto a leg with a different direction,
    rather than drifting slowly as the route's remaining-leg average
    shifts.

    Falls back to `fallback_heading_deg` under the identical degenerate
    conditions route_dominant_heading_deg does (`path` is None/empty,
    `path_idx` already at or past the last waypoint, or the one leg in
    question happens to have zero length) rather than returning an
    arbitrary angle from a zero-length vector."""
    if not path or path_idx >= len(path) - 1:
        return fallback_heading_deg
    dr = path[path_idx + 1][0] - path[path_idx][0]
    dc = path[path_idx + 1][1] - path[path_idx][1]
    if math.hypot(dr, dc) < 1e-9:
        return fallback_heading_deg
    return heading_deg(path[path_idx], path[path_idx + 1])


def resolve_held_heading_deg(drivetrain, true_position, actual_start, tag_sites, path, path_idx):
    """The one entry point ftc/match.py calls, every tick, to get the
    CURRENT held heading for a holonomic drivetrain -- returns None
    immediately for a non-holonomic one (TANK), since a held heading is
    meaningless there. Dispatches on `drivetrain.heading_policy`:

    - "fixed_at_start": nearest_tag_heading_deg(actual_start, tag_sites)
      -- `actual_start` never changes during a match, so calling this
      EVERY tick instead of caching it once produces the exact same
      floating-point result every time (same inputs, same pure
      function, zero rng draws) -- a real behavior change for the other
      three policies below, a byte-for-byte no-op for this one. This is
      what lets MECANUM's existing default stay exactly as measured in
      benchmark_results/ftc_drivetrain_writeup.md while the other
      policies become real, live alternatives.
    - "nearest_tag_current": nearest_tag_heading_deg(true_position,
      tag_sites) -- re-evaluated against wherever the robot actually is
      RIGHT NOW, not where it started.
    - "route_dominant": route_dominant_heading_deg(path, path_idx,
      fallback), aimed along the route's own dominant direction instead
      of at any tag at all -- falls back to nearest_tag_heading_deg(
      true_position, tag_sites) when no path exists yet (before the
      first plan) or the path is exhausted, so this policy still
      produces a sensible heading at tick 0 instead of None.
    - "match_travel": next_leg_heading_deg(path, path_idx, fallback),
      aimed along ONLY the immediate next leg -- path[path_idx] ->
      path[path_idx + 1] -- the same rule TANK's own robot_heading_deg
      branch already applies every step, adopted here as a heading a
      HOLONOMIC drivetrain holds instead of merely turning to match.
      Identical fallback to "route_dominant" and for the identical
      reason: no path yet, or the path is exhausted.

    All four policies happen to agree at tick 0 (true_position ==
    actual_start, no path exists yet) -- they only diverge as the match
    actually progresses, which is exactly the property that makes them
    directly comparable rather than accidentally different from the
    very first step.
    """
    if not drivetrain.holonomic:
        return None
    policy = drivetrain.heading_policy
    if policy == "fixed_at_start":
        return nearest_tag_heading_deg(actual_start, tag_sites)
    if policy == "nearest_tag_current":
        return nearest_tag_heading_deg(true_position, tag_sites)
    if policy == "route_dominant":
        fallback = nearest_tag_heading_deg(true_position, tag_sites)
        return route_dominant_heading_deg(path, path_idx, fallback)
    if policy == "match_travel":
        fallback = nearest_tag_heading_deg(true_position, tag_sites)
        return next_leg_heading_deg(path, path_idx, fallback)
    raise ValueError(f"unknown heading_policy {policy!r} -- must be one of {HEADING_POLICIES}")


@dataclass
class Drivetrain:
    name: str
    cost_usd: float
    holonomic: bool  # can translate without turning to face the direction of travel
    heading_policy: str = "fixed_at_start"  # only read when holonomic -- see resolve_held_heading_deg

    def robot_heading_deg(self, current_heading_deg, step_heading_deg, fixed_heading_deg):
        """Chassis heading the robot faces while taking a step whose
        direction of travel is step_heading_deg. TANK always turns to
        match its direction of travel (the existing, pre-Priority-2
        behavior); MECANUM holds `fixed_heading_deg` -- the CURRENT
        held heading resolve_held_heading_deg computed this tick for
        whichever policy this instance carries, despite the parameter's
        name (kept as-is so every existing caller of this method is
        unaffected) -- regardless of where it's actually driving that
        tick."""
        if self.holonomic:
            return fixed_heading_deg if fixed_heading_deg is not None else current_heading_deg
        return step_heading_deg

    def turn_cost_s(self, current_heading_deg, new_heading_deg):
        """Time charged for changing the CHASSIS heading -- not the
        direction of travel. A holonomic drivetrain's chassis heading
        never changes mid-route (see robot_heading_deg above), so this
        is always 0 for MECANUM; TANK pays the existing flat per-
        90-degree cost every time its direction of travel (and
        therefore its chassis heading) changes, identical to ftc/
        match.py's pre-Priority-2 formula."""
        turn_deg = abs(angular_diff(new_heading_deg, current_heading_deg))
        return (turn_deg / 90.0) * TURN_TIME_PER_90DEG_S

    def speed_and_drift_factor(self, chassis_heading_deg, step_heading_deg):
        """(speed_factor, drift_multiplier) for a step whose travel
        direction is step_heading_deg while the chassis faces
        chassis_heading_deg. TANK's chassis heading always equals its
        travel direction by construction (robot_heading_deg above), so
        this is always (1.0, 1.0) for it -- driving "forward" is the
        only thing a tank drivetrain ever does. MECANUM can drive at any
        angle relative to its held heading; the further a step's travel
        direction is from straight-ahead (0deg offset, pure forward)
        toward straight-sideways (90deg+ offset, pure strafe), the more
        of a strafe it is -- linearly blended between (1.0, 1.0) at 0deg
        and (MECANUM_STRAFE_SPEED_FACTOR, MECANUM_STRAFE_DRIFT_
        MULTIPLIER) at a 90-degree-or-more offset."""
        if not self.holonomic:
            return 1.0, 1.0
        offset = abs(angular_diff(step_heading_deg, chassis_heading_deg))
        strafe_frac = min(offset, 90.0) / 90.0
        speed_factor = 1.0 - strafe_frac * (1.0 - MECANUM_STRAFE_SPEED_FACTOR)
        drift_multiplier = 1.0 + strafe_frac * (MECANUM_STRAFE_DRIFT_MULTIPLIER - 1.0)
        return speed_factor, drift_multiplier


TANK = Drivetrain(name="tank", cost_usd=TANK_WHEEL_COST_USD, holonomic=False)
# heading_policy left at its default ("fixed_at_start") -- MECANUM is
# the exact instance every existing caller/benchmark already uses, and
# must stay byte-for-byte identical to before this addition (see
# resolve_held_heading_deg's own docstring for why that's guaranteed,
# not just intended).
MECANUM = Drivetrain(name="mecanum", cost_usd=MECANUM_WHEEL_COST_USD, holonomic=True)
MECANUM_NEAREST_TAG_CURRENT = Drivetrain(name="mecanum_nearest_tag_current", cost_usd=MECANUM_WHEEL_COST_USD,
                                           holonomic=True, heading_policy="nearest_tag_current")
MECANUM_ROUTE_DOMINANT = Drivetrain(name="mecanum_route_dominant", cost_usd=MECANUM_WHEEL_COST_USD,
                                      holonomic=True, heading_policy="route_dominant")
MECANUM_MATCH_TRAVEL = Drivetrain(name="mecanum_match_travel", cost_usd=MECANUM_WHEEL_COST_USD,
                                    holonomic=True, heading_policy="match_travel")
DRIVETRAINS = {
    "tank": TANK,
    "mecanum": MECANUM,
    "mecanum_nearest_tag_current": MECANUM_NEAREST_TAG_CURRENT,
    "mecanum_route_dominant": MECANUM_ROUTE_DOMINANT,
    "mecanum_match_travel": MECANUM_MATCH_TRAVEL,
}
# The original two-entry axis (ftc/drivetrain_benchmark.py's original
# study) stays available as DRIVETRAIN_ORDER for any caller that only
# wants tank-vs-mecanum; MECANUM_HEADING_POLICY_ORDER is the new axis
# comparing MECANUM's four heading policies against each other and
# against TANK, used by ftc/drivetrain_benchmark.py's own policy sweep.
DRIVETRAIN_ORDER = ["tank", "mecanum"]
MECANUM_HEADING_POLICY_ORDER = ["tank", "mecanum", "mecanum_nearest_tag_current", "mecanum_route_dominant",
                                  "mecanum_match_travel"]
DRIVETRAIN_LABELS = {
    # "mecanum"'s label stays exactly "Mecanum" (not "Mecanum (fixed at
    # start)") deliberately -- ftc_drivetrain_writeup.md already cites
    # this exact label text, and the newer variants below are
    # distinguishable from it without changing what's already published.
    "tank": "Tank",
    "mecanum": "Mecanum",
    "mecanum_nearest_tag_current": "Mecanum (re-aim to nearest tag)",
    "mecanum_route_dominant": "Mecanum (aim along route)",
    "mecanum_match_travel": "Mecanum (match next leg)",
}
