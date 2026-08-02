"""Search Snapshot -- none of the four core scenarios demonstrate the
step-by-step replay mode; all of them show a fully completed search.
This one launches straight into step mode, already stepped partway
through, so a screenshot captures the mid-expansion state across all
three panels at once (Dijkstra's wavefront, A*'s narrower directed
frontier, RRT's partial tree) -- exactly what step mode's screenshots of
mid-expansion states are for.

Shows: the search algorithms mid-expansion, one step at a time, instead
of a finished result.

Bush/mud/water terrain is scattered in too -- previously this scenario
painted none at all, and a frozen mid-expansion state is exactly where
terrain peeking out ahead of the wavefront (revealed cells now leave a
terrain-colored border instead of covering it, see pygame_app/visualizer.py:
draw_grid_panel) is easiest to actually see.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import random

from nav.config import TERRAIN_BUSH, TERRAIN_MUD, TERRAIN_WATER
from pygame_app.scenario import ScenarioConfig
from nav.scenario_helpers import scatter_obstacles, scatter_terrain
from pygame_app.visualizer import main

SEED = 20260728
OBSTACLE_DENSITY = 0.08
BUSH_DENSITY = 0.10
MUD_DENSITY = 0.09
WATER_DENSITY = 0.06
STEP_SNAPSHOT = 60  # cells/nodes revealed per panel


def build(grid):
    size = grid.size
    grid.place_start(1, 1)
    grid.place_goal(size - 2, size - 2)
    scatter_obstacles(grid, random.Random(SEED), OBSTACLE_DENSITY)
    scatter_terrain(grid, random.Random(SEED + 1), TERRAIN_BUSH, BUSH_DENSITY)
    scatter_terrain(grid, random.Random(SEED + 2), TERRAIN_MUD, MUD_DENSITY)
    scatter_terrain(grid, random.Random(SEED + 3), TERRAIN_WATER, WATER_DENSITY)


CONFIG = ScenarioConfig(
    title="Search Snapshot (step mode)",
    build=build,
    auto_run=False,  # step_snapshot below takes over instead
    step_snapshot=STEP_SNAPSHOT,
)

if __name__ == "__main__":
    main(CONFIG)
