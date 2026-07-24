import math
import random

from nav.config import NOISE_MISS_RATE, NOISE_POSITION_RATE, NOISE_FALSE_POSITIVE_RATE
from nav.grid import Grid


class LidarSensor:
    """
    Simulated lidar: reveals obstacles within `radius` cells of a given
    position. Mirrors a real range sensor -- the robot only learns about
    the world as it gets close enough to see it, instead of starting with
    a perfect map. `known_obstacles` accumulates everything sensed so far;
    it only ever grows (nothing already seen is ever un-sensed, even if it
    later moves -- see WRITEUPS.md for why that's an acceptable
    simplification here).

    Perfect (every real obstacle in range detected, at its exact cell,
    nothing else) by default -- pass `noisy=True` for a sensor that
    behaves like a real, imperfect one instead, in three independent
    ways (each an in-range obstacle can suffer every scan, at the given
    rate):

    - **False negative** (`miss_rate`): a real obstacle isn't detected
      this scan at all.
    - **Position noise** (`position_rate`): a detected obstacle is
      reported at a random *adjacent* cell instead of its true one.
    - **False positive** (`false_positive_rate`): a cell that's actually
      free gets "detected" as an obstacle anyway.

    `known_obstacles` still means exactly what it always did (every
    cell ever reported, right or wrong -- since noise can plant a wrong
    cell just as permanently as a correct one, a caller that wants some
    protection against a one-off bad reading should use
    `confirmed_obstacles` instead; see WRITEUPS.md for why requiring a
    cell to be reported more than once is a meaningfully different, more
    trustworthy signal than a single detection, and where that
    protection itself breaks down.
    """

    def __init__(self, radius, noisy=False, rng=None,
                 miss_rate=NOISE_MISS_RATE, position_rate=NOISE_POSITION_RATE,
                 false_positive_rate=NOISE_FALSE_POSITIVE_RATE):
        self.radius = radius
        self.noisy = noisy
        self.rng = rng or random.Random()
        self.miss_rate = miss_rate
        self.position_rate = position_rate
        self.false_positive_rate = false_positive_rate
        self.known_obstacles = set()
        # cell -> how many separate scans have (noisily) reported it as
        # an obstacle. A cell reported once always ends up with count 1,
        # whether that one report was right or wrong -- what makes this
        # useful is that a *repeated* false positive at the same cell is
        # far less likely by chance than a one-off, since each scan
        # re-rolls independently (see confirmed_obstacles).
        self.detection_counts = {}

    def _jitter(self, grid, row, col):
        """A random valid cell adjacent to (row, col), simulating range/
        position noise instead of a perfectly precise reading. Falls
        back to the original cell if every neighbor is out of bounds
        (only possible in a corner of a tiny grid)."""
        candidates = [
            (row + dr, col + dc)
            for dr in (-1, 0, 1) for dc in (-1, 0, 1)
            if (dr, dc) != (0, 0) and grid.is_valid(row + dr, col + dc)
        ]
        return self.rng.choice(candidates) if candidates else (row, col)

    def _record(self, cell, newly_seen):
        self.detection_counts[cell] = self.detection_counts.get(cell, 0) + 1
        if cell not in self.known_obstacles:
            self.known_obstacles.add(cell)
            newly_seen.add(cell)

    def sense(self, grid, position):
        """Look around `position` and add any newly-visible obstacles to
        known_obstacles. Returns the set of obstacles seen for the first
        time this call (empty if nothing new) -- with noise on, "seen"
        includes false positives and position-jittered ghosts of real
        obstacles, exactly as a real caller would receive them (it has
        no way to tell a true reading from a noisy one except by asking
        again -- see confirmed_obstacles)."""
        row, col = position
        newly_seen = set()
        for dr in range(-self.radius, self.radius + 1):
            for dc in range(-self.radius, self.radius + 1):
                if math.hypot(dr, dc) > self.radius:
                    continue
                r, c = row + dr, col + dc
                if not grid.is_valid(r, c):
                    continue

                if grid.cells[r][c] == Grid.OBSTACLE:
                    if self.noisy and self.rng.random() < self.miss_rate:
                        continue  # false negative
                    cell = (r, c)
                    if self.noisy and self.rng.random() < self.position_rate:
                        cell = self._jitter(grid, r, c)
                    self._record(cell, newly_seen)
                elif self.noisy and self.rng.random() < self.false_positive_rate:
                    self._record((r, c), newly_seen)
        return newly_seen

    def confirmed_obstacles(self, min_detections=1):
        """Cells reported at least `min_detections` times so far.
        `min_detections=1` (the default) is exactly `known_obstacles` --
        trusting a cell the very first time it's reported, which is only
        safe because a perfect (non-noisy) sensor is never wrong. With
        noise on, a caller that wants to avoid replanning on a one-off
        false positive or a stray jittered reading should ask for
        `min_detections=2` or higher instead."""
        if min_detections <= 1:
            return set(self.known_obstacles)
        return {cell for cell, count in self.detection_counts.items() if count >= min_detections}


class KnownGrid(Grid):
    """
    A Grid built only from what a LidarSensor has discovered so far --
    every cell the sensor hasn't seen is assumed free, since that's the
    only honest assumption a robot without a perfect map can make. This
    turns planning into planning-under-uncertainty: a route can look clear
    right up until the sensor reveals otherwise.

    Implements the same interface as Grid (is_obstacle, get_neighbors,
    ...), so astar/dijkstra/rrt run against it completely unmodified.
    Cost-map weighting is intentionally not carried over -- combining
    "unknown terrain cost" with "unknown obstacles" is its own can of
    worms this project doesn't open.
    """

    def __init__(self, known_obstacles, diagonal=False, size=None):
        # `size` defaults to Grid's own default (nav.config.GRID_SIZE),
        # same as Grid itself -- every existing caller (the pygame
        # sensor mode, the pybullet sensor demo) always runs on that
        # default-size grid and is unaffected. It exists so
        # nav/replan_benchmark.py can exercise the sensor-discovery
        # scenario at other grid sizes too, the same reason Grid grew a
        # `size` parameter for nav/scale_benchmark.py.
        super().__init__(size=size)
        self.diagonal = diagonal
        for row, col in known_obstacles:
            self.cells[row][col] = Grid.OBSTACLE
