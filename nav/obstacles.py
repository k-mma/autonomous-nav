from nav.grid import Grid


class MovingObstacle:
    """
    Replanning policy
    ------------------
    - A moving obstacle bounces between two adjacent free cells, swapping
      position every `period_ms` milliseconds.
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

    def __init__(self, cell_a, cell_b, period_ms=700):
        self.cell_a = cell_a
        self.cell_b = cell_b
        self.at_a = True
        self.period_ms = period_ms
        self.next_move_time = 0

    @property
    def position(self):
        return self.cell_a if self.at_a else self.cell_b

    def start(self, now_ms):
        self.next_move_time = now_ms + self.period_ms

    def tick(self, grid, now_ms):
        """Advance the obstacle if it's due. Returns True if it moved this call."""
        if now_ms < self.next_move_time:
            return False
        self.next_move_time = now_ms + self.period_ms
        self._move(grid)
        return True

    def _move(self, grid):
        old_row, old_col = self.position
        self.at_a = not self.at_a
        new_row, new_col = self.position

        if grid.cells[old_row][old_col] == Grid.OBSTACLE:
            grid.cells[old_row][old_col] = Grid.FREE
        if grid.cells[new_row][new_col] == Grid.FREE:
            grid.cells[new_row][new_col] = Grid.OBSTACLE


def find_free_neighbor(grid, cell):
    """First free, non start/goal neighbor of cell, or None."""
    row, col = cell
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        r, c = row + dr, col + dc
        if grid.is_free(r, c):
            return (r, c)
    return None
