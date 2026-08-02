import math
import random

import pybullet as p

from nav.config import NOISE_MISS_RATE, NOISE_POSITION_RATE, NOISE_FALSE_POSITIVE_RATE
from .coords import world_to_grid, WORLD_CELL_SIZE

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
# How far a noisy "position" reading can drift off the real hit point,
# in meters -- enough to sometimes land in an adjacent grid cell at
# WORLD_CELL_SIZE=1.0m, same idea as nav/sensor.py's _jitter but
# continuous instead of a discrete neighbor pick, since a raycast hit is
# a continuous point to begin with.
POSITION_JITTER_METERS = 0.6
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

    Perfect (every real raycast hit converted to its exact cell, nothing
    else) by default -- pass `noisy=True` for the same three independent
    imperfections nav.sensor.LidarSensor models, applied per-ray instead
    of per-cell:

    - **False negative** (`miss_rate`): a ray that actually hit
      something is reported as a miss instead.
    - **Position noise** (`position_rate`): a real hit's reported point
      is nudged by up to `position_jitter` meters in a random direction
      before being converted to a grid cell -- occasionally enough to
      land in a different cell than the true one.
    - **False positive** (`false_positive_rate`): a ray that hit nothing
      "detects" a phantom obstacle at a random point along its own
      length instead.

    Like LidarSensor, `known_obstacles` means every cell ever reported,
    right or wrong -- see `confirmed_obstacles` for a view that requires
    a cell to be reported more than once before trusting it.
    """

    def __init__(self, num_rays=DEFAULT_NUM_RAYS, max_range=DEFAULT_RANGE, ignore_body_id=None,
                 noisy=False, rng=None, miss_rate=NOISE_MISS_RATE, position_rate=NOISE_POSITION_RATE,
                 false_positive_rate=NOISE_FALSE_POSITIVE_RATE, position_jitter=POSITION_JITTER_METERS):
        self.num_rays = num_rays
        self.max_range = max_range
        self.ignore_body_id = ignore_body_id
        self.noisy = noisy
        self.rng = rng or random.Random()
        self.miss_rate = miss_rate
        self.position_rate = position_rate
        self.false_positive_rate = false_positive_rate
        self.position_jitter = position_jitter
        self.known_obstacles = set()
        # cell -> how many separate scans have (noisily) reported it --
        # see confirmed_obstacles.
        self.detection_counts = {}

    def _record(self, cell, newly_seen):
        self.detection_counts[cell] = self.detection_counts.get(cell, 0) + 1
        if cell not in self.known_obstacles:
            self.known_obstacles.add(cell)
            newly_seen.add(cell)

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
            reported = False

            if hit:
                if self.noisy and self.rng.random() < self.miss_rate:
                    hit = False  # false negative -- report this ray as a miss instead
                else:
                    dx, dy = hit_pos[0] - origin[0], hit_pos[1] - origin[1]
                    ray_len = math.hypot(dx, dy) or 1.0
                    nudged_x = hit_pos[0] + dx / ray_len * HIT_NUDGE
                    nudged_y = hit_pos[1] + dy / ray_len * HIT_NUDGE
                    if self.noisy and self.rng.random() < self.position_rate:
                        nudged_x += self.rng.uniform(-self.position_jitter, self.position_jitter)
                        nudged_y += self.rng.uniform(-self.position_jitter, self.position_jitter)
                    self._record(world_to_grid(nudged_x, nudged_y), newly_seen)
                    reported = True
            elif self.noisy and self.rng.random() < self.false_positive_rate:
                # Hallucinate a hit at a random point along this ray's
                # own (real, obstacle-free) length instead of at an
                # actual raycast hit -- there's no real geometry here to
                # perturb, so this is the phantom-detection equivalent of
                # nav.sensor.LidarSensor reporting a free cell as one.
                dx, dy = end[0] - origin[0], end[1] - origin[1]
                ray_len = math.hypot(dx, dy) or 1.0
                fake_dist = self.rng.uniform(WORLD_CELL_SIZE, ray_len)
                fake_x = origin[0] + dx / ray_len * fake_dist
                fake_y = origin[1] + dy / ray_len * fake_dist
                self._record(world_to_grid(fake_x, fake_y), newly_seen)
                reported = True

            if gui:
                p.addUserDebugLine(
                    origin, hit_pos if hit else end,
                    lineColorRGB=HIT_COLOR if (hit or reported) else MISS_COLOR,
                    lineWidth=1, lifeTime=RAY_LIFETIME,
                )
        return newly_seen

    def confirmed_obstacles(self, min_detections=1):
        """Cells reported at least `min_detections` times so far -- see
        nav.sensor.LidarSensor.confirmed_obstacles, same idea."""
        if min_detections <= 1:
            return set(self.known_obstacles)
        return {cell for cell, count in self.detection_counts.items() if count >= min_detections}
