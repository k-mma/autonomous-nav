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
    Wraps a loaded husky body and drives it toward a sequence of (x, y)
    waypoints using base velocity control (pybullet.resetBaseVelocity) --
    not by teleporting its position/orientation. It won't look smooth
    driving raw A* waypoints (sharp turn, straight line, sharp turn...);
    it looks smooth automatically once it's handed the many closely
    spaced points a corner-cut or spline path produces, because the same
    controller then only ever has to make small heading corrections.
    That's the whole point of pybullet_app/sim3d/smoothing.py -- this class never
    changes between the two.
    """

    def __init__(self, body_id, speed=DEFAULT_SPEED, turn_speed=DEFAULT_TURN_SPEED,
                 arrive_radius=ARRIVE_RADIUS):
        self.body_id = body_id
        self.speed = speed
        self.turn_speed = turn_speed
        self.arrive_radius = arrive_radius

    def position(self):
        pos, _ = p.getBasePositionAndOrientation(self.body_id)
        return pos

    def heading(self):
        _, orn = p.getBasePositionAndOrientation(self.body_id)
        _, _, yaw = p.getEulerFromQuaternion(orn)
        return yaw

    def stop(self):
        p.resetBaseVelocity(self.body_id, linearVelocity=[0, 0, 0],
                             angularVelocity=[0, 0, 0])

    def drive_toward(self, target_xy):
        """One control step toward target_xy = (x, y). Returns True once
        the robot is within arrive_radius of target_xy (and stops it),
        False otherwise (call again next step)."""
        x, y, _ = self.position()
        dx, dy = target_xy[0] - x, target_xy[1] - y
        dist = math.hypot(dx, dy)
        if dist <= self.arrive_radius:
            self.stop()
            return True

        desired_yaw = math.atan2(dy, dx)
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
        p.resetBaseVelocity(self.body_id, linearVelocity=[vx, vy, 0],
                             angularVelocity=[0, 0, wz])
        return False
