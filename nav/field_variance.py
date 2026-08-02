"""
Turns "how much the real world deviates from the robot's assumed map"
into a controllable, sweepable parameter -- the thing a physical
competition field can never give you (one noisy, unrepeatable trial, no
control over the variance, no ground truth to compare against). See
nav/uncertainty_benchmark.py for the sweep that actually uses this.
"""
import random

from nav.algorithms import astar
from nav.config import (
    FIELD_VARIANCE_MAX_START_DRIFT_RADIUS, FIELD_VARIANCE_MAX_OBSTACLE_DRIFT_COUNT,
)
from nav.grid import Grid


def _clone_grid(grid):
    """Copy just the obstacle layout and movement mode -- the pieces
    generate_ground_truth actually needs to mutate independently of the
    assumed grid it was built from. Mirrors nav/replan_benchmark.py's
    _clone_grid; terrain/cost-map state isn't copied since nothing in
    this module reads or writes it."""
    g = Grid(size=grid.size)
    g.cells = [row[:] for row in grid.cells]
    g.diagonal = grid.diagonal
    return g


def _drift_obstacles(grid, start, goal, count, rng):
    """Flip up to `count` currently-free cells to obstacles, and free up
    to `count` currently-obstacle cells, in place on `grid` -- modeling
    field elements measured or placed slightly differently than assumed,
    in both directions at once (a real field can have both new clutter
    the assumed map didn't have, and open space the assumed map wrongly
    thought was blocked)."""
    if count <= 0:
        return
    free_cells = [
        (r, c) for r in range(grid.size) for c in range(grid.size)
        if grid.cells[r][c] == Grid.FREE and (r, c) not in (start, goal)
    ]
    obstacle_cells = [
        (r, c) for r in range(grid.size) for c in range(grid.size)
        if grid.cells[r][c] == Grid.OBSTACLE
    ]
    rng.shuffle(free_cells)
    for row, col in free_cells[:count]:
        grid.cells[row][col] = Grid.OBSTACLE
    rng.shuffle(obstacle_cells)
    for row, col in obstacle_cells[:count]:
        grid.cells[row][col] = Grid.FREE


def _place_unplanned_blocker(grid, assumed_grid, start, goal, probability, rng):
    """With probability `probability`, drop one extra obstacle directly
    on the assumed start->goal route -- modeling an alliance/opponent
    robot sitting in the way, the one deviation type an open-loop policy
    has no way to avoid short of never having planned through that cell
    at all. No-op if the assumed map has no path, or if that path is too
    short to have an interior cell to block."""
    if rng.random() >= probability:
        return
    path, _, _ = astar(assumed_grid, start, goal)
    if not path or len(path) < 3:
        return
    row, col = rng.choice(path[1:-1])
    grid.cells[row][col] = Grid.OBSTACLE


def _drift_start(grid, start, radius, rng):
    """A free cell within Chebyshev distance `radius` of `start` (`start`
    itself is always a candidate, so radius == 0 always returns `start`
    unchanged) -- modeling odometry/placement error between where the
    autonomous routine assumes it's starting and where the robot actually
    sits at the start of the run. Falls back to `start` itself if every
    candidate in range turned out to be an obstacle after obstacle drift
    (only possible with a large radius on a dense grid)."""
    if radius <= 0:
        return start
    candidates = [
        (start[0] + dr, start[1] + dc)
        for dr in range(-radius, radius + 1)
        for dc in range(-radius, radius + 1)
        if grid.is_valid(start[0] + dr, start[1] + dc) and grid.is_free(start[0] + dr, start[1] + dc)
    ]
    return rng.choice(candidates) if candidates else start


def generate_ground_truth(assumed_grid, start, goal, variance_level, seed):
    """
    Build a "ground truth" Grid that models how far the real world can
    diverge from `assumed_grid` -- the map a policy plans against -- at a
    single scalar knob, plus an `actual_start` cell for where the robot
    really begins (which can differ from `start`, the cell the plan was
    computed from).

    `variance_level` in [0, 1] scales three independent deviation types
    at once, each linearly between 0 at 0.0 and its configured bound
    (nav/config.py) at 1.0:

    - **Start drift**: the robot's real starting cell is displaced from
      `start` by up to `round(variance_level * FIELD_VARIANCE_MAX_START_
      DRIFT_RADIUS)` cells (Chebyshev radius).
    - **Obstacle drift**: up to `round(variance_level *
      FIELD_VARIANCE_MAX_OBSTACLE_DRIFT_COUNT)` free cells are flipped to
      obstacles, and the same number of assumed-obstacle cells are freed.
    - **Unplanned blocker**: with probability exactly `variance_level`,
      one additional obstacle is placed on a random interior cell of the
      assumed start->goal path (found via nav.algorithms.astar on
      `assumed_grid`) -- never the start or goal cell itself, and skipped
      entirely if the assumed map has no path.

    At variance_level == 0.0 every deviation's magnitude/probability is
    exactly 0, so `ground_truth` is cell-for-cell identical to
    `assumed_grid` and `actual_start == start`. At variance_level == 1.0
    every deviation is at its configured maximum.

    Deterministic given `seed`: the same (assumed_grid, start, goal,
    variance_level, seed) always produces the exact same
    (ground_truth, actual_start) pair, via a `random.Random(seed)`
    private to this call -- it never touches the global `random` module,
    so a specific trial in nav/uncertainty_benchmark.py is exactly
    reproducible independent of call order.

    Returns (ground_truth_grid, actual_start).
    """
    rng = random.Random(seed)
    ground_truth = _clone_grid(assumed_grid)

    obstacle_count = round(variance_level * FIELD_VARIANCE_MAX_OBSTACLE_DRIFT_COUNT)
    _drift_obstacles(ground_truth, start, goal, obstacle_count, rng)
    _place_unplanned_blocker(ground_truth, assumed_grid, start, goal, variance_level, rng)

    start_radius = round(variance_level * FIELD_VARIANCE_MAX_START_DRIFT_RADIUS)
    actual_start = _drift_start(ground_truth, start, start_radius, rng)

    return ground_truth, actual_start
