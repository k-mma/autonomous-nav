"""Deep Forest -- the existing recursive-backtracking maze generator
(nav/maze.py), fixed-seed for reproducibility, with mud terrain filling
several dead-end branches. Start and goal at opposite corners -- a maze
is a "perfect" one (exactly one route between any two cells), so this is
meant to show RRT's random-sampling tree fighting its way through a
winding, mostly-single-path environment against Dijkstra/A*'s systematic
wavefront.

Shows: RRT's random sampling struggling through narrow single-path
corridors, compared to Dijkstra/A*'s systematic search.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import random

from nav.config import TERRAIN_MUD
from nav.maze import generate_maze
from nav.scenario import ScenarioConfig
from nav.scenario_helpers import find_dead_ends, paint_branch
from nav.visualizer import main

MAZE_SEED = 7
DEAD_END_SEED = 8
DEAD_ENDS_TO_FILL = 6


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
    for cell in dead_ends[:DEAD_ENDS_TO_FILL]:
        paint_branch(grid, cell, TERRAIN_MUD)


CONFIG = ScenarioConfig(
    title="Deep Forest",
    build=build,
    auto_run=True,
)

if __name__ == "__main__":
    main(CONFIG)
