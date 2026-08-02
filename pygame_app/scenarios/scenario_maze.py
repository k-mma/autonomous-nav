"""Deep Forest -- the existing recursive-backtracking maze generator
(nav/maze.py), fixed-seed for reproducibility, with mud and bush terrain
filling most dead-end branches (alternating the two so they're easy to
tell apart at a glance). Start and goal at opposite corners -- a maze is
a "perfect" one (exactly one route between any two cells), so this is
meant to show RRT's random-sampling tree fighting its way through a
winding, mostly-single-path environment against Dijkstra/A*'s systematic
wavefront.

Shows: RRT's random sampling struggling through narrow single-path
corridors, compared to Dijkstra/A*'s systematic search.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import random

from nav.config import TERRAIN_BUSH, TERRAIN_MUD
from nav.maze import generate_maze
from pygame_app.scenario import ScenarioConfig
from nav.scenario_helpers import find_dead_ends, paint_branch
from pygame_app.visualizer import main

MAZE_SEED = 7
DEAD_END_SEED = 8
DEAD_ENDS_TO_FILL = 14


def build(grid):
    generate_maze(grid, rng=random.Random(MAZE_SEED))

    size = grid.size
    # Maze "cells" only live on even row/col coordinates -- the largest
    # in-bounds even coordinate is the opposite corner from (0, 0).
    last = size - 1 if (size - 1) % 2 == 0 else size - 2
    grid.place_start(0, 0)
    grid.place_goal(last, last)

    dead_ends = find_dead_ends(grid)
    random.Random(DEAD_END_SEED).shuffle(dead_ends)
    for i, cell in enumerate(dead_ends[:DEAD_ENDS_TO_FILL]):
        terrain_type = TERRAIN_MUD if i % 2 == 0 else TERRAIN_BUSH
        paint_branch(grid, cell, terrain_type)


CONFIG = ScenarioConfig(
    title="Deep Forest",
    build=build,
    auto_run=True,
)

if __name__ == "__main__":
    main(CONFIG)
