import random

from nav.grid import Grid
from nav.obstacles import find_free_neighbor

CARDINAL_DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def scatter_obstacles(grid, rng, density):
    """Randomly toggle free cells into obstacles at roughly `density`
    fraction of the grid -- skips start/goal automatically (toggle_
    obstacle already no-ops on those) since scenarios place start/goal
    before scattering."""
    for row in range(grid.size):
        for col in range(grid.size):
            if grid.is_free(row, col) and rng.random() < density:
                grid.toggle_obstacle(row, col)


def find_dead_ends(grid):
    """Free cells with exactly one free cardinal neighbor -- the tips of
    dead-end branches in a carved maze."""
    dead_ends = []
    for row in range(grid.size):
        for col in range(grid.size):
            if grid.cells[row][col] != Grid.FREE:
                continue
            free_neighbors = sum(
                1 for dr, dc in CARDINAL_DIRS
                if grid.is_valid(row + dr, col + dc) and grid.cells[row + dr][col + dc] != Grid.OBSTACLE
            )
            if free_neighbors == 1:
                dead_ends.append((row, col))
    return dead_ends


def paint_branch(grid, start_cell, terrain_type, max_len=4):
    """Paint `start_cell` and up to `max_len - 1` cells back along its
    single-path corridor (stopping at a junction or another dead end),
    so a dead-end branch reads as a filled-in stretch of terrain rather
    than a single isolated cell."""
    prev = None
    cur = start_cell
    for _ in range(max_len):
        grid.paint_terrain(cur[0], cur[1], terrain_type)
        neighbors = [
            (cur[0] + dr, cur[1] + dc) for dr, dc in CARDINAL_DIRS
            if grid.is_valid(cur[0] + dr, cur[1] + dc)
            and grid.cells[cur[0] + dr][cur[1] + dc] != Grid.OBSTACLE
            and (cur[0] + dr, cur[1] + dc) != prev
        ]
        if len(neighbors) != 1:
            break
        prev, cur = cur, neighbors[0]


def pick_moving_obstacle_cells(grid, count, seed):
    """`count` free cells, each guaranteed to have at least one free
    neighbor (so it actually has somewhere to random-walk to -- see
    nav/obstacles.py), chosen deterministically from `seed`."""
    rng = random.Random(seed)
    candidates = [
        (row, col) for row in range(grid.size) for col in range(grid.size)
        if grid.is_free(row, col) and find_free_neighbor(grid, (row, col)) is not None
    ]
    rng.shuffle(candidates)
    return candidates[:count]
