import random

from nav.grid import Grid

CARDINAL_DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]


class MovingObstacle:
    """
    Replanning policy
    ------------------
    - A moving obstacle does a random walk: each tick it looks at its
      four cardinal neighbors, and if any are free (not an obstacle, not
      out of bounds, not the start or goal cell -- Grid.is_free already
      excludes all three), steps to one of them at random. If none are
      free this tick, it stays put.
    - After an obstacle moves, if its new cell lies on the remaining portion
      of the robot's current path (from the robot's position to the goal),
      the visualizer immediately replans from the robot's current cell to
      the goal with the active algorithm.
    - If an obstacle moves directly onto the robot's current cell, the robot
      does not step onto it: it holds its position and tries to replan from
      where it stands. If no path exists, the robot stays put, the
      visualizer shows a "no path" indicator, and the replan is retried on
      every later obstacle move until a path opens back up.
    - An obstacle moving away (freeing a cell) never forces a replan by
      itself -- only a move that newly blocks the robot's current path does.
    """

    def __init__(self, cell, period_ms=700, rng=None):
        self.cell = cell
        self.period_ms = period_ms
        self.next_move_time = 0
        # Each obstacle gets its own RNG instance rather than sharing
        # `random`'s global one -- random.Random() with no seed pulls
        # fresh entropy from the OS per instance, so obstacles created
        # in the same frame still end up on independent walks instead
        # of all making the same "random" choice in lockstep.
        self.rng = rng or random.Random()

    @property
    def position(self):
        return self.cell

    def place(self, grid):
        """Mark this obstacle's starting cell as an obstacle right away.
        FIX (Step 1 audit): the old two-cell bounce version never marked
        its own starting cell -- __init__ just recorded cell_a/cell_b,
        and _move() only ever set the *destination* cell to OBSTACLE.
        That left a freshly placed moving obstacle invisible (still
        drawn as plain free/terrain color) and unplannable-around (grid
        cell still FREE) for a full period_ms, until its first tick
        happened to move it. Calling this immediately after construction
        closes that gap.
        """
        row, col = self.cell
        if grid.is_free(row, col):
            grid.cells[row][col] = Grid.OBSTACLE

    def start(self, now_ms):
        self.next_move_time = now_ms + self.period_ms

    def tick(self, grid, now_ms, blocked=frozenset()):
        """Advance the obstacle if it's due. `blocked` is cells to treat
        as unavailable even though the grid marks them free -- the
        visualizer passes every other moving obstacle's current cell so
        two obstacles can't step onto each other in the same tick.
        Returns True if the obstacle was due to move this call (whether
        or not it actually had anywhere free to go), False if it wasn't
        due yet.
        """
        if now_ms < self.next_move_time:
            return False
        self.next_move_time = now_ms + self.period_ms
        self._move(grid, blocked)
        return True

    def _move(self, grid, blocked=frozenset()):
        row, col = self.cell
        candidates = []
        for dr, dc in CARDINAL_DIRS:
            r, c = row + dr, col + dc
            if grid.is_free(r, c) and (r, c) not in blocked:
                candidates.append((r, c))

        if not candidates:
            return  # No free neighbor this tick -- stay put.

        new_cell = self.rng.choice(candidates)
        if grid.cells[row][col] == Grid.OBSTACLE:
            grid.cells[row][col] = Grid.FREE
        grid.cells[new_cell[0]][new_cell[1]] = Grid.OBSTACLE
        self.cell = new_cell


def find_free_neighbor(grid, cell):
    """First free, non start/goal neighbor of cell, or None."""
    row, col = cell
    for dr, dc in CARDINAL_DIRS:
        r, c = row + dr, col + dc
        if grid.is_free(r, c):
            return (r, c)
    return None
