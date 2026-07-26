"""Forest Corridor -- two solid obstacle walls running almost the full
grid height, each with a single narrow gap at the vertical midpoint,
with bush terrain flanking both gaps. Start left-center, goal
right-center, so every algorithm is forced through the same two
chokepoints -- this is meant to make exploration-count and path-cost
differences between the three algorithms obvious, since they can't just
take three different routes around open ground.

Shows: how all three algorithms behave when forced through the same
narrow chokepoints instead of being free to spread out.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from nav.config import TERRAIN_BUSH
from nav.scenario import ScenarioConfig
from nav.visualizer import main

GAP_HALF_HEIGHT = 1  # rows within this distance of the midpoint stay open


def build(grid):
    size = grid.size
    mid_row = size // 2
    wall_cols = [size // 3, (2 * size) // 3]

    grid.place_start(mid_row, 1)
    grid.place_goal(mid_row, size - 2)

    for col in wall_cols:
        for row in range(size):
            if abs(row - mid_row) <= GAP_HALF_HEIGHT:
                continue  # leave the gap open
            if grid.is_free(row, col):
                grid.toggle_obstacle(row, col)

    # Bush terrain flanking each gap's entrance and exit.
    for wall_col in wall_cols:
        for dc in (-1, 1):
            col = wall_col + dc
            for row in range(mid_row - 2, mid_row + 3):
                grid.paint_terrain(row, col, TERRAIN_BUSH)


CONFIG = ScenarioConfig(
    title="Forest Corridor",
    build=build,
    auto_run=True,
)

if __name__ == "__main__":
    main(CONFIG)
