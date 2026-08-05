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
premium (goBILDA-class mecanum ~$200 vs. traction ~$80, ftc/config.py)
and a speed/drift penalty on any step that isn't roughly "forward"
relative to whatever heading it's holding.

Heading policy, chosen and documented rather than left implicit:
MECANUM holds a fixed heading for the whole match, aimed at the nearest
AprilTag wall site (ftc/match.py picks this once, at the start, from
whatever `tag_sites` the match is given) -- the natural choice for a
robot that's investing in an AprilTag-reading camera at all, and the
specific interaction ftc/drivetrain_benchmark.py's writeup is built
around: holding that heading keeps the camera aimed at tags for the
entire match, which Priority 1's camera-FOV gating should reward
heavily -- while a tank robot rotates its camera away from the tag wall
every time it changes direction. TANK always faces its current
direction of travel, exactly matching the existing (pre-Priority-2)
behavior -- ftc/match.py's `drivetrain=None` default reproduces that
unchanged for every existing caller, and an explicit TANK instance is
functionally identical to that default (see ftc/scratch/
drivetrain_test.py's consistency check).
"""
from dataclasses import dataclass

from ftc.config import (
    MECANUM_STRAFE_DRIFT_MULTIPLIER, MECANUM_STRAFE_SPEED_FACTOR,
    MECANUM_WHEEL_COST_USD, TANK_WHEEL_COST_USD, TURN_TIME_PER_90DEG_S,
)
from ftc.sensors import angular_diff


@dataclass
class Drivetrain:
    name: str
    cost_usd: float
    holonomic: bool  # can translate without turning to face the direction of travel

    def robot_heading_deg(self, current_heading_deg, step_heading_deg, fixed_heading_deg):
        """Chassis heading the robot faces while taking a step whose
        direction of travel is step_heading_deg. TANK always turns to
        match its direction of travel (the existing, pre-Priority-2
        behavior); MECANUM holds fixed_heading_deg regardless of where
        it's actually driving that tick."""
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
MECANUM = Drivetrain(name="mecanum", cost_usd=MECANUM_WHEEL_COST_USD, holonomic=True)
DRIVETRAINS = {"tank": TANK, "mecanum": MECANUM}
DRIVETRAIN_ORDER = ["tank", "mecanum"]
DRIVETRAIN_LABELS = {"tank": "Tank", "mecanum": "Mecanum"}
