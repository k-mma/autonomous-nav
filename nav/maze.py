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
    pair of neighbors, which only tiles evenly across an odd-sized grid
    (GRID_SIZE is 25). On an even-sized grid (e.g. DEMO_GRID_SIZE, used by
    --demo and every scenario_*.py), the carving below runs on the largest
    odd sub-grid that fits, and the leftover last row/column is opened as
    plain free space instead of being left as a permanently uncarvable
    wall along two edges of the grid.
    """
    rng = rng or random
    size = len(grid.cells)
    grid.clear()
    for row in range(size):
        for col in range(size):
            grid.cells[row][col] = Grid.OBSTACLE

    maze_size = size if size % 2 == 1 else size - 1

    def cells():
        return [(r, c) for r in range(0, maze_size, 2) for c in range(0, maze_size, 2)]

    start = rng.choice(cells())
    grid.cells[start[0]][start[1]] = Grid.FREE
    visited = {start}
    stack = [start]

    while stack:
        row, col = stack[-1]
        unvisited = []
        for dr, dc in [(-2, 0), (2, 0), (0, -2), (0, 2)]:
            r, c = row + dr, col + dc
            if 0 <= r < maze_size and 0 <= c < maze_size and (r, c) not in visited:
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

    if maze_size != size:
        for i in range(size):
            grid.cells[size - 1][i] = Grid.FREE
            grid.cells[i][size - 1] = Grid.FREE

    # FIX (Step 1 audit): every wall/passage above is carved by writing
    # grid.cells directly, bypassing toggle_obstacle (which keeps
    # grid.cost in sync on every change). Without this, generating a
    # maze while cost-map mode (K) was already on left grid.cost at
    # its pre-maze values -- the cost tint and cost-aware A*/Dijkstra
    # silently ignored the maze's walls entirely.
    grid.refresh_cost_map()
