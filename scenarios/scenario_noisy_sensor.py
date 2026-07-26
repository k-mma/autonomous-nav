"""Unreliable Sensor -- scenario_sensor.py explicitly runs with a perfect
(non-noisy) lidar; noisy sensing (missed detections, position-jittered
readings, occasional false positives, and the confirmed_obstacles()
vote-threshold that guards against trusting any single noisy reading --
see nav/sensor.py and WRITEUPS.md) is a genuinely different behavior
that none of the four core scenarios show. Same moderate obstacle field
and moving obstacles as "Hidden Animals", but noisy=True, so the status
line's "raw / confirmed" sensed counts diverge and occasional
ghost/missed readings become visible as the robot walks.

Shows: how a noisy, imperfect sensor differs from a perfect one --
missed detections, jittered positions, and occasional false readings.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import random

from nav.scenario import ScenarioConfig
from nav.scenario_helpers import pick_moving_obstacle_cells, scatter_obstacles
from nav.visualizer import main

SEED = 20260729
OBSTACLE_DENSITY = 0.10
NUM_ANIMALS = 4
ANIMAL_SEED = SEED + 1000


def build(grid):
    size = grid.size
    grid.place_start(1, 1)
    grid.place_goal(size - 2, size - 2)
    scatter_obstacles(grid, random.Random(SEED), OBSTACLE_DENSITY)


def moving_obstacles(grid):
    return pick_moving_obstacle_cells(grid, NUM_ANIMALS, ANIMAL_SEED)


CONFIG = ScenarioConfig(
    title="Unreliable Sensor",
    build=build,
    moving_obstacles=moving_obstacles,
    sensor_enabled=True,
    noisy_sensor=True,
    auto_run=True,
    auto_walk=True,
)

if __name__ == "__main__":
    main(CONFIG)
