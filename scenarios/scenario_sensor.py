"""Hidden Animals -- a moderate fixed obstacle layout with 5 moving
obstacles (animals), lidar sensor enabled (perfect, not noisy), and the
robot auto-walking as soon as the algorithms have run. Meant to
demonstrate the sensor radius ring, the visible distinction between
known (solid) and hidden (outlined) obstacles, and a live replan when an
animal wanders into the robot's path mid-walk.

Shows: the lidar sensor radius, known vs. hidden obstacles, and the
robot replanning around a moving animal.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import random

from nav.scenario import ScenarioConfig
from nav.scenario_helpers import pick_moving_obstacle_cells, scatter_obstacles
from nav.visualizer import main

SEED = 20260726
OBSTACLE_DENSITY = 0.10
NUM_ANIMALS = 5
ANIMAL_SEED = SEED + 1000


def build(grid):
    size = grid.size
    grid.place_start(1, 1)
    grid.place_goal(size - 2, size - 2)
    scatter_obstacles(grid, random.Random(SEED), OBSTACLE_DENSITY)


def moving_obstacles(grid):
    return pick_moving_obstacle_cells(grid, NUM_ANIMALS, ANIMAL_SEED)


CONFIG = ScenarioConfig(
    title="Hidden Animals",
    build=build,
    moving_obstacles=moving_obstacles,
    sensor_enabled=True,
    noisy_sensor=False,
    auto_run=True,
    auto_walk=True,
)

if __name__ == "__main__":
    main(CONFIG)
