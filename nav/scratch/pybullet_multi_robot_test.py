"""
Standalone sanity check: two robots in the same PyBullet world, each
running its own independent A* plan on non-conflicting paths (opposite ends of
the grid, far enough apart that neither route ever comes close to the
other). No priority scheme, no dynamic replanning -- that's what
pybullet_multi_robot_main.py's forced-corridor-conflict scenario is for.
This just confirms the basics: two robots can be loaded and driven
simultaneously in one simulation without interfering with each other.

    python3 -m nav.scratch.pybullet_multi_robot_test [--headless]
"""
import argparse
import math
import time

import pybullet as p

from nav.algorithms import find_path
from nav.grid import Grid
from pybullet_main import build_drive_waypoints
from nav.sim3d.coords import grid_to_world
from nav.sim3d.robot import Robot
from nav.sim3d.world import connect, build_obstacles

SIM_HZ = 240
ROBOT_A = ((2, 2), (2, 22))
ROBOT_B = ((22, 2), (22, 22))
MIN_SEPARATION_SEEN = float("inf")


def plan_waypoints(grid, start, goal):
    path, _, reason, _ = find_path(grid, "astar", start, goal)
    if path is None:
        raise RuntimeError(f"no path found (reason={reason})")
    return build_drive_waypoints(path, "spline")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=30.0)
    args = parser.parse_args()

    connect(gui=not args.headless)
    grid = Grid()
    # A modest obstacle far from both lanes, just so this isn't a
    # trivially empty world -- neither robot's route goes near it.
    for row in range(10, 16):
        grid.cells[row][12] = Grid.OBSTACLE
    build_obstacles(grid)

    a_start, a_goal = ROBOT_A
    b_start, b_goal = ROBOT_B
    a_waypoints = plan_waypoints(grid, a_start, a_goal)
    b_waypoints = plan_waypoints(grid, b_start, b_goal)

    ax, ay, _ = grid_to_world(*a_start)
    bx, by, _ = grid_to_world(*b_start)
    robot_a = Robot(p.loadURDF("r2d2.urdf", basePosition=[ax, ay, 0.4]))
    robot_b = Robot(p.loadURDF("r2d2.urdf", basePosition=[bx, by, 0.4]))

    print(f"robot A: {len(a_waypoints)} waypoints, {a_start} -> {a_goal}")
    print(f"robot B: {len(b_waypoints)} waypoints, {b_start} -> {b_goal}")

    idx_a = idx_b = 0
    arrived_a = arrived_b = False
    min_separation = float("inf")
    steps = 0
    max_steps = int(args.max_seconds * SIM_HZ)

    while steps < max_steps and not (arrived_a and arrived_b):
        if not arrived_a:
            if robot_a.drive_toward(a_waypoints[idx_a]):
                idx_a += 1
                if idx_a >= len(a_waypoints):
                    arrived_a = True
                    robot_a.stop()
        if not arrived_b:
            if robot_b.drive_toward(b_waypoints[idx_b]):
                idx_b += 1
                if idx_b >= len(b_waypoints):
                    arrived_b = True
                    robot_b.stop()

        p.stepSimulation()
        if not args.headless:
            time.sleep(1 / SIM_HZ)
        steps += 1

        pa, pb = robot_a.position(), robot_b.position()
        min_separation = min(min_separation, math.hypot(pa[0] - pb[0], pa[1] - pb[1]))

    print(f"robot A arrived: {arrived_a}, robot B arrived: {arrived_b}")
    print(f"minimum separation observed between the two robots: {min_separation:.2f}m")
    assert arrived_a and arrived_b, "both robots should reach their goals with no conflict"
    assert min_separation > 2.0, "these paths were supposed to be non-conflicting"
    print("both robots navigated simultaneously with no interference -- sanity check passed.")

    if not args.headless:
        input("Press Enter to close...")
    p.disconnect()


if __name__ == "__main__":
    main()
