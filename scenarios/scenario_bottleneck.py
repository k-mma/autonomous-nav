"""Forest Corridor -- two solid obstacle walls running almost the full
grid height, each with a single narrow gap at the vertical midpoint,
with bush terrain flanking both gaps and a broader scatter of bush/mud/
water across the rest of the map. Start left-center, goal right-center,
so every algorithm is forced through the same two chokepoints -- this
is meant to make exploration-count and path-cost differences between
the three algorithms obvious, since they can't just take three
different routes around open ground.

Shows: how all three algorithms behave when forced through the same
narrow chokepoints instead of being free to spread out.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import random

from nav.config import TERRAIN_BUSH, TERRAIN_MUD, TERRAIN_WATER
from nav.scenario import ScenarioConfig
from nav.scenario_helpers import scatter_terrain
from nav.visualizer import main

GAP_HALF_HEIGHT = 1  # rows within this distance of the midpoint stay open
SEED = 20260731
BUSH_DENSITY = 0.10
MUD_DENSITY = 0.08
WATER_DENSITY = 0.05


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

    # Broad scatter first, so it's the "forest floor" -- then the
    # deliberate gap-flanking bush below paints last and always wins at
    # the chokepoints regardless of what the random scatter did there.
    scatter_terrain(grid, random.Random(SEED), TERRAIN_BUSH, BUSH_DENSITY)
    scatter_terrain(grid, random.Random(SEED + 1), TERRAIN_MUD, MUD_DENSITY)
    scatter_terrain(grid, random.Random(SEED + 2), TERRAIN_WATER, WATER_DENSITY)

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
