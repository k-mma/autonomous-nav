"""Costed Clearing -- none of the four core scenarios turn on the
obstacle-inflation cost map (K) or diagonal movement (X), so neither the
cost-map tint nor an 8-directional path is visible in any of them. This
one enables both: a scattered-obstacle field with cost-map inflation on
(so the peach halo around every obstacle is visible) and diagonal
movement on (so Dijkstra/A*'s paths cut corners instead of
stair-stepping), plus a water crossing so the terrain-cost and
inflation-cost tints can be told apart in the same shot (see
nav/visualizer.py's draw_grid_panel -- the tint blends from the cell's
own terrain color, dividing the terrain multiplier back out first,
specifically so the two don't get confused with each other).

Shows: the obstacle-inflation cost-map tint and 8-directional diagonal
movement, side by side with terrain cost.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import random

from nav.config import TERRAIN_WATER
from nav.scenario import ScenarioConfig
from nav.scenario_helpers import scatter_obstacles
from nav.visualizer import main

SEED = 20260727
OBSTACLE_DENSITY = 0.10


def build(grid):
    size = grid.size
    grid.place_start(1, 1)
    grid.place_goal(size - 2, size - 2)

    scatter_obstacles(grid, random.Random(SEED), OBSTACLE_DENSITY)

    # A water strip across the middle, away from start/goal, so a
    # diagonal path has an obvious reason to route around rather than
    # through it, and the terrain-cost tint (water) sits next to the
    # inflation-cost tint (K, around obstacles) in the same screenshot.
    mid = size // 2
    for col in range(2, size - 2):
        grid.paint_terrain(mid, col, TERRAIN_WATER)

    grid.diagonal = True
    grid.cost_map_enabled = True
    grid.refresh_cost_map()


CONFIG = ScenarioConfig(
    title="Costed Clearing",
    build=build,
    auto_run=True,
)

if __name__ == "__main__":
    main(CONFIG)
