"""Search Snapshot -- none of the four core scenarios demonstrate the
step-by-step replay mode; all of them show a fully completed search.
This one launches straight into step mode, already stepped partway
through, so a screenshot captures the mid-expansion state across all
three panels at once (Dijkstra's wavefront, A*'s narrower directed
frontier, RRT's partial tree) -- exactly what step mode's screenshots of
mid-expansion states are for.

Shows: the search algorithms mid-expansion, one step at a time, instead
of a finished result.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import random

from nav.scenario import ScenarioConfig
from nav.scenario_helpers import scatter_obstacles
from nav.visualizer import main

SEED = 20260728
OBSTACLE_DENSITY = 0.08
STEP_SNAPSHOT = 60  # cells/nodes revealed per panel


def build(grid):
    size = grid.size
    grid.place_start(1, 1)
    grid.place_goal(size - 2, size - 2)
    scatter_obstacles(grid, random.Random(SEED), OBSTACLE_DENSITY)


CONFIG = ScenarioConfig(
    title="Search Snapshot (step mode)",
    build=build,
    auto_run=False,  # step_snapshot below takes over instead
    step_snapshot=STEP_SNAPSHOT,
)

if __name__ == "__main__":
    main(CONFIG)
