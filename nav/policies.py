"""
Three planning policies behind one shared interface, so
nav/uncertainty_benchmark.py can drive any of them through the exact
same execution loop. Each models a different answer to "how much does
the robot look at the world while it's moving":

- OpenLoopPolicy: never looks. Plans once against the assumed map,
  executes that plan blindly. This is what a standard FTC autonomous
  routine does today -- plan from the field's measured/assumed layout
  before the match, then run the recorded path with no feedback.
- ReactivePolicy: looks, but only reacts to a hard binary signal --
  replans the moment a sensed obstacle actually blocks its route.
  Reuses nav.sensor.blocks_remaining_path, the same trigger
  pygame_app/visualizer.py's live replanning already uses.
- BeliefPolicy: looks continuously and plans against expected cost
  (nav/occupancy.py's per-cell occupancy probability) instead of a
  binary obstacle/free split, replanning every step since its cost map
  changes with every sensor sweep, not just when a cell crosses the
  hard-obstacle threshold.

A policy is constructed once per trial (its very first plan happens
inside __init__, against the assumed map only -- see each subclass) and
then stepped once per simulated tick via `.step(true_grid, current_cell)`.
`replans` only counts planning calls *after* that first one; a policy
that never plans again after its initial plan keeps `replans == 0`.
"""
import time

from nav.algorithms import astar
from nav.config import LIDAR_RADIUS
from nav.occupancy import BeliefGrid, OccupancyGrid
from nav.sensor import KnownGrid, LidarSensor, blocks_remaining_path


class Policy:
    """Common base every policy below extends. Not meant to be
    instantiated directly."""

    def __init__(self, assumed_grid, start, goal):
        self.assumed_grid = assumed_grid
        self.start = start
        self.goal = goal
        self.replans = 0
        self.planning_time_s = 0.0

    def step(self, true_grid, current_cell):
        """Given where the robot actually is right now (`current_cell`,
        ground truth), return the next cell to move to, or None if the
        policy has no route forward at all (stuck). Subclasses override
        this."""
        raise NotImplementedError


class OpenLoopPolicy(Policy):
    """Plans once against the assumed map, then executes that path
    blindly -- no sensing, no replanning, even if a step drives straight
    through a cell that's actually blocked in ground truth.

    `current_cell` is only ever looked at once, on the very first call,
    to work out the offset between where this policy planned from
    (`start`) and where the robot actually starts (which can differ --
    see nav/field_variance.py's start drift). It has no way to sense or
    correct for that offset, so it keeps executing the exact same
    relative motions it always would have: the whole executed
    trajectory ends up shifted by that one initial offset rather than
    jumping discontinuously onto cells from a path measured from a start
    it was never actually standing on. That offset is also the only way
    start drift can make an open-loop run miss the goal outright -- with
    a nonzero offset, `path[-1] + offset != goal` unless the offset
    happens to be zero.
    """

    def __init__(self, assumed_grid, start, goal, rng=None):
        super().__init__(assumed_grid, start, goal)
        t0 = time.perf_counter()
        path, _, _ = astar(assumed_grid, start, goal)
        self.planning_time_s += time.perf_counter() - t0
        self.path = path
        self._idx = 0
        self._offset = None

    def step(self, true_grid, current_cell):
        if self.path is None:
            return None
        if self._offset is None:
            self._offset = (current_cell[0] - self.path[0][0], current_cell[1] - self.path[0][1])
        if self._idx >= len(self.path) - 1:
            return None
        self._idx += 1
        row, col = self.path[self._idx]
        return (row + self._offset[0], col + self._offset[1])


class ReactivePolicy(Policy):
    """Senses locally every step with LidarSensor, replanning with astar
    the moment a newly sensed obstacle blocks the remaining path (see
    nav.sensor.blocks_remaining_path) -- the same trigger
    pygame_app/visualizer.py's live replanning and nav/replan_benchmark.py's
    sensor-discovery scenario both already use, packaged behind this
    project's shared Policy interface instead of being inlined in an
    event loop or duplicated into a third benchmark helper."""

    def __init__(self, assumed_grid, start, goal, sensor_radius=LIDAR_RADIUS, rng=None):
        super().__init__(assumed_grid, start, goal)
        self.size = assumed_grid.size
        self.diagonal = assumed_grid.diagonal
        self.sensor = LidarSensor(sensor_radius, rng=rng)
        self.path = None
        self._idx = 0
        self._planned_once = False

    def _replan(self, current_cell, count_as_replan):
        known = KnownGrid(self.sensor.known_obstacles, diagonal=self.diagonal, size=self.size)
        t0 = time.perf_counter()
        path, _, _ = astar(known, current_cell, self.goal)
        self.planning_time_s += time.perf_counter() - t0
        if count_as_replan:
            self.replans += 1
        self.path = path
        self._idx = 0

    def step(self, true_grid, current_cell):
        newly_seen = self.sensor.sense(true_grid, current_cell)
        if not self._planned_once:
            self._replan(current_cell, count_as_replan=False)
            self._planned_once = True
        # Stuck (no known path) retries every step, same as
        # nav/obstacles.py's moving-obstacle replanning: "if no path
        # exists ... the replan is retried on every later [event] until
        # a path opens back up." Each retry is a genuine extra planning
        # call, so -- unlike the bootstrap plan above -- it counts.
        elif self.path is None or blocks_remaining_path(newly_seen, current_cell, self.path[self._idx:]):
            self._replan(current_cell, count_as_replan=True)

        if self.path is None or self._idx >= len(self.path) - 1:
            return None
        self._idx += 1
        return self.path[self._idx]


class BeliefPolicy(Policy):
    """Senses locally every step, folds each reading into an
    OccupancyGrid (nav/occupancy.py) instead of a binary known/unknown
    split, and plans against the resulting expected-cost map -- unknown
    cells cost something between free and blocked, proportional to how
    likely the belief thinks they are to be occupied, rather than being
    silently assumed free the way KnownGrid (and ReactivePolicy) treats
    them.

    Replans every single step rather than waiting for a discrete
    obstacle-on-path trigger: the belief's expected-cost map changes with
    every sensor sweep, not just when a cell crosses the hard-obstacle
    threshold, so "replan as the belief updates" means replanning
    continuously. See nav/uncertainty_benchmark.py's writeup for what
    that continuous replanning actually costs in practice against
    ReactivePolicy's event-triggered approach."""

    def __init__(self, assumed_grid, start, goal, sensor_radius=LIDAR_RADIUS, rng=None):
        super().__init__(assumed_grid, start, goal)
        self.size = assumed_grid.size
        self.diagonal = assumed_grid.diagonal
        self.sensor = LidarSensor(sensor_radius, rng=rng)
        self.occupancy = OccupancyGrid(self.size)
        self.path = None
        self._idx = 0
        self._planned_once = False

    def _replan(self, current_cell, count_as_replan):
        belief = BeliefGrid(self.occupancy, diagonal=self.diagonal, size=self.size)
        t0 = time.perf_counter()
        path, _, _ = astar(belief, current_cell, self.goal)
        self.planning_time_s += time.perf_counter() - t0
        if count_as_replan:
            self.replans += 1
        self.path = path
        self._idx = 0

    def step(self, true_grid, current_cell):
        self.sensor.sense(true_grid, current_cell)
        self.occupancy.update_from_sensor(self.sensor, true_grid, current_cell)
        self._replan(current_cell, count_as_replan=self._planned_once)
        self._planned_once = True

        if self.path is None or self._idx >= len(self.path) - 1:
            return None
        self._idx += 1
        return self.path[self._idx]
