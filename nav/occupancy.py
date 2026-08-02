"""
Occupancy belief model: replaces nav/sensor.py's binary known_obstacles
set with a per-cell *probability* of being occupied, updated via
log-odds (Thrun/Burgard/Fox's standard occupancy-grid-mapping update)
instead of a single "seen it once, believe it forever" flag. A cell
nobody has ever sensed stays at OCCUPANCY_PRIOR (nav/config.py); a cell
seen free repeatedly gets driven toward probability 0, a cell seen
occupied repeatedly gets driven toward probability 1 -- and because
log-odds are additive, a single stray misreading only nudges the
estimate instead of flipping it outright the way a single false positive
permanently corrupts known_obstacles.

This is an additional consumer of LidarSensor.sense()'s output, not a
replacement for it -- nav/sensor.py's KnownGrid, known_obstacles, and
confirmed_obstacles all keep behaving exactly as before for every
existing caller. See nav/policies.py's BeliefPolicy for the planner that
actually plans against this instead of a binary known/unknown split.
"""
import math

from nav.config import (
    OCCUPANCY_PRIOR, OCCUPANCY_LOGODDS_FREE, OCCUPANCY_LOGODDS_OCCUPIED,
    OCCUPANCY_LOGODDS_CLAMP, OCCUPANCY_OBSTACLE_THRESHOLD,
    OCCUPANCY_FREE_COST_MULT, OCCUPANCY_BLOCKED_COST_MULT,
)
from nav.grid import Grid


def _prob_to_logodds(p):
    return math.log(p / (1 - p))


def _logodds_to_prob(l):
    return 1 - 1 / (1 + math.exp(l))


_PRIOR_LOGODDS = _prob_to_logodds(OCCUPANCY_PRIOR)


class OccupancyGrid:
    """
    size x size field of occupied-probability estimates, one per cell,
    all starting at OCCUPANCY_PRIOR. Stored internally as log-odds (so
    repeated observations just add) and only converted to a probability
    on read, in `probability()`.
    """

    def __init__(self, size):
        self.size = size
        self._logodds = [[_PRIOR_LOGODDS for _ in range(size)] for _ in range(size)]

    def _is_valid(self, row, col):
        return 0 <= row < self.size and 0 <= col < self.size

    def observe_free(self, row, col):
        if not self._is_valid(row, col):
            return
        l = self._logodds[row][col] + OCCUPANCY_LOGODDS_FREE
        self._logodds[row][col] = max(l, -OCCUPANCY_LOGODDS_CLAMP)

    def observe_occupied(self, row, col):
        if not self._is_valid(row, col):
            return
        l = self._logodds[row][col] + OCCUPANCY_LOGODDS_OCCUPIED
        self._logodds[row][col] = min(l, OCCUPANCY_LOGODDS_CLAMP)

    def probability(self, row, col):
        """1.0 (certain-occupied) for an out-of-bounds cell -- there's
        nothing to plan through past the edge of the grid, so treating
        it the same as a fully-confirmed obstacle is the correct belief,
        not a missing value."""
        if not self._is_valid(row, col):
            return 1.0
        return _logodds_to_prob(self._logodds[row][col])

    def update_from_sensor(self, sensor, grid, position):
        """Sweep the same disc LidarSensor.sense() just looked at
        (`sensor.radius` around `position`) and update every in-range
        cell: occupied if it's in `sensor.known_obstacles` (whatever the
        sensor has reported there, right or wrong -- an occupancy grid
        is supposed to accumulate exactly this kind of noisy evidence
        over repeated looks, not silently trust or distrust a single
        reading), free otherwise. Cells outside the swept disc are left
        untouched, staying at whatever they were before (or the prior,
        if this is the first time they've ever been in range). Call this
        right after `sensor.sense(grid, position)` -- it doesn't call
        `sense` itself, since a caller may want to inspect the newly-seen
        set first (see nav/policies.py's BeliefPolicy)."""
        row, col = position
        for dr in range(-sensor.radius, sensor.radius + 1):
            for dc in range(-sensor.radius, sensor.radius + 1):
                if math.hypot(dr, dc) > sensor.radius:
                    continue
                r, c = row + dr, col + dc
                if not grid.is_valid(r, c):
                    continue
                if (r, c) in sensor.known_obstacles:
                    self.observe_occupied(r, c)
                else:
                    self.observe_free(r, c)


class BeliefGrid(Grid):
    """
    A Grid whose per-cell traversal cost is derived from an
    OccupancyGrid instead of a fixed obstacle layout -- the "plan
    against expected cost" counterpart to nav/sensor.py's KnownGrid
    (which plans against a binary known/unknown split instead).

    A cell's cost multiplier interpolates linearly from
    OCCUPANCY_FREE_COST_MULT at probability 0 up to
    OCCUPANCY_BLOCKED_COST_MULT as probability approaches
    OCCUPANCY_OBSTACLE_THRESHOLD -- so a planner routing across this grid
    prefers cells it believes are more likely clear over ones it isn't
    sure about, without needing to have actually confirmed either way.
    Only cells the belief considers virtually certain to be occupied
    (probability >= OCCUPANCY_OBSTACLE_THRESHOLD) become hard
    Grid.OBSTACLE cells; every other cell stays traversable, just at a
    probability-weighted cost. That's what makes this "planning under
    uncertainty" instead of "planning on a slightly-different known map"
    -- the never-observed cells KnownGrid would treat as free and the
    almost-certainly-occupied cells it would treat as identically free
    are no longer indistinguishable to the planner.

    Implements the same interface as Grid, so astar/dijkstra run against
    it unmodified, exactly like KnownGrid.
    """

    def __init__(self, occupancy, diagonal=False, size=None):
        super().__init__(size=size or occupancy.size)
        self.diagonal = diagonal
        for row in range(self.size):
            for col in range(self.size):
                p = occupancy.probability(row, col)
                if p >= OCCUPANCY_OBSTACLE_THRESHOLD:
                    self.cells[row][col] = Grid.OBSTACLE
                else:
                    frac = min(p / OCCUPANCY_OBSTACLE_THRESHOLD, 1.0)
                    self.cost[row][col] = (
                        OCCUPANCY_FREE_COST_MULT
                        + frac * (OCCUPANCY_BLOCKED_COST_MULT - OCCUPANCY_FREE_COST_MULT)
                    )
