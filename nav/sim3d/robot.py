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
        p.resetBaseVelocity(self.body_id, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])

    def drive_toward(self, target_xy):
        """One control step toward target_xy = (x, y). Returns True once
        the robot is within arrive_radius (and stops it), False
        otherwise (call again next step)."""
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

        wz = max(-self.turn_speed, min(self.turn_speed, yaw_error * 4))
        p.resetBaseVelocity(self.body_id, linearVelocity=[vx, vy, 0], angularVelocity=[0, 0, wz])
        return False
