# Meters per grid cell in the PyBullet world -- unrelated to nav.config's
# CELL_SIZE, which is a pixel size for the pygame window. Same grid, two
# different physical units depending which renderer is looking at it.
WORLD_CELL_SIZE = 1.0


def grid_to_world(row, col, cell_size=WORLD_CELL_SIZE):
    """(row, col) -> (x, y, z) at ground level, matching the pygame
    convention of col -> horizontal, row -> vertical, just with row now
    mapped to world y instead of screen y."""
    return (col * cell_size, row * cell_size, 0.0)


def world_to_grid(x, y, cell_size=WORLD_CELL_SIZE):
    return (round(y / cell_size), round(x / cell_size))
