import math

import pybullet as p

from nav.sim3d.coords import world_to_grid

DEFAULT_NUM_RAYS = 36
DEFAULT_RANGE = 6.0
DEFAULT_HEIGHT = 0.5
# Rays start this far out from the robot's own center, in the ray's own
# direction, rather than dead center -- a ray literally originating inside
# the robot's own collision shape hits *itself* first every time, which
# would make it blind to everything else no matter what ignore_body_id says.
ORIGIN_OFFSET = 0.35
# A ray hit lands exactly on an obstacle box's surface -- e.g. x=8.5 for a
# 1.0m cell size, precisely the boundary between free cell 8 and obstacle
# cell 9. That's an inherently ambiguous point to round to a grid cell,
# and Python's round-half-to-even can even round it to the FREE cell in
# front of the wall instead of the wall itself, poisoning known_obstacles
# with a phantom obstacle in real free space. Nudging the hit point this
# far further along the ray, past the surface, before converting to a
# grid cell reliably lands inside the obstacle's own cell instead.
HIT_NUDGE = 0.1
HIT_COLOR = (0.95, 0.2, 0.2)
MISS_COLOR = (0.2, 0.55, 0.95)
RAY_LIFETIME = 0.25


class Lidar3D:
    """
    The pygame sensor model (nav/sensor.py's radius circle) ported to
    PyBullet using the real raycast API (pybullet.rayTestBatch) instead
    of "every obstacle within radius R" -- a wall can now block the view
    of what's behind it, which a radius circle has no way to represent.
    `known_obstacles` (grid cells) feeds the same `KnownGrid` class
    unchanged, so replanning-on-discovery logic doesn't change at all
    between the pygame and PyBullet sensor models -- only how a cell
    gets added to the set does.
    """

    def __init__(self, num_rays=DEFAULT_NUM_RAYS, max_range=DEFAULT_RANGE, ignore_body_id=None):
        self.num_rays = num_rays
        self.max_range = max_range
        self.ignore_body_id = ignore_body_id
        self.known_obstacles = set()

    def scan(self, position, height=DEFAULT_HEIGHT, gui=False):
        """Cast `num_rays` rays outward in a circle from `position`.
        Returns the set of grid cells newly discovered as obstacles this
        scan (already merged into self.known_obstacles -- knowledge only
        ever grows, same as the pygame sensor)."""
        x, y = position[0], position[1]
        froms, ends = [], []
        for i in range(self.num_rays):
            angle = 2 * math.pi * i / self.num_rays
            cos_a, sin_a = math.cos(angle), math.sin(angle)
            froms.append([x + ORIGIN_OFFSET * cos_a, y + ORIGIN_OFFSET * sin_a, height])
            ends.append([x + self.max_range * cos_a, y + self.max_range * sin_a, height])

        results = p.rayTestBatch(froms, ends)

        newly_seen = set()
        for (hit_id, _link, _frac, hit_pos, _normal), origin, end in zip(results, froms, ends):
            hit = hit_id >= 0 and hit_id != self.ignore_body_id
            if hit:
                dx, dy = hit_pos[0] - origin[0], hit_pos[1] - origin[1]
                ray_len = math.hypot(dx, dy) or 1.0
                nudged_x = hit_pos[0] + dx / ray_len * HIT_NUDGE
                nudged_y = hit_pos[1] + dy / ray_len * HIT_NUDGE
                cell = world_to_grid(nudged_x, nudged_y)
                if cell not in self.known_obstacles:
                    self.known_obstacles.add(cell)
                    newly_seen.add(cell)
            if gui:
                p.addUserDebugLine(
                    origin, hit_pos if hit else end,
                    lineColorRGB=HIT_COLOR if hit else MISS_COLOR,
                    lineWidth=1, lifeTime=RAY_LIFETIME,
                )
        return newly_seen
