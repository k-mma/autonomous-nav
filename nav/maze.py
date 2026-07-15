import random

from nav.grid import Grid


def generate_maze(grid, rng=None):
    """
    Carve a maze into `grid` with recursive backtracking.

    Fills the grid with obstacles, then walks a random depth-first path
    from a random cell, knocking down the wall between the current cell
    and a randomly chosen unvisited cell two steps away, backtracking when
    a cell has no unvisited neighbors. Produces a "perfect" maze -- exactly
    one route between any two open cells, no loops.

    "Cells" live on even row/col coordinates with a wall cell between each
    pair of neighbors, so this requires an odd-sized grid (GRID_SIZE is 25).
    """
    rng = rng or random
    size = len(grid.cells)
    grid.clear()
    for row in range(size):
        for col in range(size):
            grid.cells[row][col] = Grid.OBSTACLE

    def cells():
        return [(r, c) for r in range(0, size, 2) for c in range(0, size, 2)]

    start = rng.choice(cells())
    grid.cells[start[0]][start[1]] = Grid.FREE
    visited = {start}
    stack = [start]

    while stack:
        row, col = stack[-1]
        unvisited = []
        for dr, dc in [(-2, 0), (2, 0), (0, -2), (0, 2)]:
            r, c = row + dr, col + dc
            if 0 <= r < size and 0 <= c < size and (r, c) not in visited:
                unvisited.append((r, c))

        if not unvisited:
            stack.pop()
            continue

        next_row, next_col = rng.choice(unvisited)
        wall = ((row + next_row) // 2, (col + next_col) // 2)
        grid.cells[wall[0]][wall[1]] = Grid.FREE
        grid.cells[next_row][next_col] = Grid.FREE
        visited.add((next_row, next_col))
        stack.append((next_row, next_col))
