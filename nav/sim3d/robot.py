import math

import pybullet as p

DEFAULT_SPEED = 20.0
DEFAULT_TURN_SPEED = 22.0
ARRIVE_RADIUS = 0.2
# Below this heading error (radians), start driving forward as well as
# turning; above it, turn in place first -- keeps the robot from
# strafing sideways toward a target behind it, which resetBaseVelocity
# would otherwise happily do (it has no notion of "forward").
FACE_TARGET_TOLERANCE = 0.2
# Below this heading error, ease the turn rate down proportionally so it
# settles smoothly instead of overshooting; above it, turn at the full
# turn_speed. Pure proportional control (rate = error * gain) for *every*
# error, even large ones, decays too slowly to be worth it: for a full
# 90-degree turn it took over 100 simulation steps just to turn enough to
# start driving at all -- a robot whose very first waypoint happens to be
# behind or to the side of it would sit rotating in place noticeably
# longer than it should. Full-rate-until-close fixes that without giving
# up the smooth settle for small corrections.
TURN_EASE_THRESHOLD = 0.5


class Robot:
    """
    Wraps a loaded r2d2 body and drives it toward a sequence of (x, y)
    waypoints using base velocity control (pybullet.resetBaseVelocity) --
    not by teleporting its position/orientation. It won't look smooth
    driving raw A* waypoints (sharp turn, straight line, sharp turn...);
    it looks smooth automatically once it's handed the many closely
    spaced points a corner-cut or spline path produces, because the same
    controller then only ever has to make small heading corrections.
    That's the whole point of nav/sim3d/smoothing.py -- this class never
    changes between the two.
    """

    def __init__(self, body_id, speed=DEFAULT_SPEED, turn_speed=DEFAULT_TURN_SPEED,
                 arrive_radius=ARRIVE_RADIUS, allow_vertical_fall=False):
        self.body_id = body_id
        self.speed = speed
        self.turn_speed = turn_speed
        self.arrive_radius = arrive_radius
        # Off by default -- see _target_vz. Every existing demo runs on
        # flat ground, where forcing vertical velocity to exactly 0 every
        # control tick is already correct (there's nothing to fall into),
        # so this stays off for them and nothing about their behavior
        # changes. Only pybullet_main.py's --elevation demo (stepped
        # terrain, nav/sim3d/world.py: build_terrain) turns it on.
        self.allow_vertical_fall = allow_vertical_fall

    def position(self):
        pos, _ = p.getBasePositionAndOrientation(self.body_id)
        return pos

    def heading(self):
        _, orn = p.getBasePositionAndOrientation(self.body_id)
        _, _, yaw = p.getEulerFromQuaternion(orn)
        return yaw

    def _target_vz(self):
        """The Z component to command via resetBaseVelocity this tick.
        0.0 unless allow_vertical_fall is on, in which case it reads the
        robot's actual current vertical speed back from the physics
        engine instead. Why this needs to be conditional rather than
        just always reading it back: resetBaseVelocity replaces *all*
        of linearVelocity, including Z, every single control tick --
        harmless on flat ground (vertical velocity is already ~0 there,
        so hardcoding 0 changes nothing), but on stepped terrain,
        hardcoding 0 fights gravity, zeroing out whatever downward speed
        had just started building between ticks, over and over, forever.
        The robot can still climb a step under that regime (contact
        pushout from the collision solver doesn't depend on
        resetBaseVelocity at all), but it can't properly *fall* down
        one: found by watching a robot drive straight over a hill and
        land still 1.8m above the ground on the far side, barely
        descending at all long after "arriving" at the right (x, y).

        This is opt-in rather than the unconditional default specifically
        because turning it on unconditionally was tried and regressed
        pybullet_multi_robot_main.py -- two robots on flat ground,
        reading back and re-feeding a real (if tiny, near-zero) vertical
        velocity every tick instead of a hard 0 measurably destabilized
        their driving (the two robots' crossing that demo's WRITEUPS.md
        entry documents tuning to a specific closest-approach distance
        stopped reaching the goal reliably at all). Flat-ground demos
        never needed this fix in the first place -- only elevation does
        -- so it only applies where it's asked for."""
        if not self.allow_vertical_fall:
            return 0.0
        linear, _ = p.getBaseVelocity(self.body_id)
        return linear[2]

    def stop(self):
        p.resetBaseVelocity(self.body_id, linearVelocity=[0, 0, self._target_vz()],
                             angularVelocity=[0, 0, 0])

    def drive_toward(self, target_xy, steer_target=None):
        """One control step toward target_xy = (x, y). Returns True once
        the robot is within arrive_radius of target_xy (and stops it),
        False otherwise (call again next step).

        `steer_target`, if given, is aimed at *instead* of target_xy for
        the purposes of picking a heading this step -- lets a caller (see
        pybullet_multi_robot_main.py's local collision avoidance) nudge
        the robot's immediate direction without changing what actually
        counts as "arrived." Arrival is always judged against the real
        target_xy, never the steering override."""
        x, y, _ = self.position()
        dx, dy = target_xy[0] - x, target_xy[1] - y
        dist = math.hypot(dx, dy)
        if dist <= self.arrive_radius:
            self.stop()
            return True

        aim_x, aim_y = steer_target if steer_target is not None else target_xy
        desired_yaw = math.atan2(aim_y - y, aim_x - x)
        yaw_error = (desired_yaw - self.heading() + math.pi) % (2 * math.pi) - math.pi

        if abs(yaw_error) > FACE_TARGET_TOLERANCE:
            vx, vy = 0.0, 0.0
        else:
            vx = self.speed * math.cos(desired_yaw)
            vy = self.speed * math.sin(desired_yaw)

        if abs(yaw_error) > TURN_EASE_THRESHOLD:
            wz = self.turn_speed if yaw_error > 0 else -self.turn_speed
        else:
            wz = max(-self.turn_speed, min(self.turn_speed, yaw_error * 12))
        p.resetBaseVelocity(self.body_id, linearVelocity=[vx, vy, self._target_vz()],
                             angularVelocity=[0, 0, wz])
        return False
