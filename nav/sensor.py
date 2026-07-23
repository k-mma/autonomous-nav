import math

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
    """

    def __init__(self, radius):
        self.radius = radius
        self.known_obstacles = set()

    def sense(self, grid, position):
        """Look around `position` and add any newly-visible obstacles to
        known_obstacles. Returns the set of obstacles seen for the first
        time this call (empty if nothing new)."""
        row, col = position
        newly_seen = set()
        for dr in range(-self.radius, self.radius + 1):
            for dc in range(-self.radius, self.radius + 1):
                if math.hypot(dr, dc) > self.radius:
                    continue
                r, c = row + dr, col + dc
                if not grid.is_valid(r, c):
                    continue
                if grid.cells[r][c] == Grid.OBSTACLE and (r, c) not in self.known_obstacles:
                    self.known_obstacles.add((r, c))
                    newly_seen.add((r, c))
        return newly_seen


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
