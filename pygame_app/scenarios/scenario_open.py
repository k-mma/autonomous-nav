"""Open Field -- sparse fixed obstacles scattered across otherwise open
ground, with bush/mud/water terrain scattered across the whole field
plus a denser bush/mud patch near the center. Start near top-left, goal
near bottom-right, so there's nothing forcing a particular route -- this
is meant to show Dijkstra's full radial exploration blob against A*'s
directed search and RRT's random sampling, all with (mostly) nothing in
the way.

Shows: how Dijkstra, A*, and RRT each explore the same open, mostly
unobstructed ground differently.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import random

from nav.config import TERRAIN_BUSH, TERRAIN_MUD, TERRAIN_WATER
from pygame_app.scenario import ScenarioConfig
from nav.scenario_helpers import scatter_obstacles, scatter_terrain
from pygame_app.visualizer import main

SEED = 20260725
OBSTACLE_DENSITY = 0.08
BUSH_DENSITY = 0.12
MUD_DENSITY = 0.08
WATER_DENSITY = 0.05


def build(grid):
    size = grid.size
    grid.place_start(1, 1)
    grid.place_goal(size - 2, size - 2)

    scatter_obstacles(grid, random.Random(SEED), OBSTACLE_DENSITY)
    scatter_terrain(grid, random.Random(SEED + 1), TERRAIN_BUSH, BUSH_DENSITY)
    scatter_terrain(grid, random.Random(SEED + 2), TERRAIN_MUD, MUD_DENSITY)
    scatter_terrain(grid, random.Random(SEED + 3), TERRAIN_WATER, WATER_DENSITY)

    # A denser bush/mud patch near the center on top of the broad
    # scatter -- two adjacent blocks so the two terrain types are
    # clearly next to each other in one shot, not just wherever chance
    # placed them.
    mid = size // 2
    for row in range(mid - 2, mid + 1):
        for col in range(mid - 3, mid):
            grid.paint_terrain(row, col, TERRAIN_BUSH)
        for col in range(mid + 1, mid + 4):
            grid.paint_terrain(row, col, TERRAIN_MUD)


CONFIG = ScenarioConfig(
    title="Open Field",
    build=build,
    auto_run=True,
)

if __name__ == "__main__":
    main(CONFIG)
