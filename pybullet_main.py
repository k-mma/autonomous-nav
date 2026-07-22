"""
A robot navigating a 3D environment using the same A* code as the pygame
visualizer, plus a lidar-sensor mode: the robot plans against only what
it's actually seen via real raycasts, and replans as it discovers more.
Everything planning-related here is imported unchanged from
nav/algorithms.py, nav/grid.py, and nav/sensor.py -- the only new code is
the PyBullet physics interface (nav/sim3d/).

    python3 pybullet_main.py                     # cost-map path, spline-smoothed
    python3 pybullet_main.py --smooth raw         # raw A* waypoints, sharp turns, no smoothing
    python3 pybullet_main.py --smooth corner_cut  # Chaikin corner-cutting instead of a spline
    python3 pybullet_main.py --smooth spline      # Catmull-Rom spline (default)
    python3 pybullet_main.py --no-cost-map        # binary obstacles only, no clearance routing
    python3 pybullet_main.py --sensor             # lidar-limited knowledge, replans on discovery
    python3 pybullet_main.py --headless           # DIRECT mode, no GUI window, for automated runs
"""
import argparse
import time

import pybullet as p

from nav.algorithms import find_path
from nav.grid import Grid
from nav.sensor import KnownGrid
from nav.sim3d.coords import grid_to_world, world_to_grid, WORLD_CELL_SIZE
from nav.sim3d.lidar import Lidar3D
from nav.sim3d.robot import Robot
from nav.sim3d.smoothing import simplify_collinear, chaikin_smooth, catmull_rom_spline
from nav.sim3d.world import (
    connect, build_obstacles, mark_cell, draw_path, draw_waypoints,
    BINARY_PATH_COLOR, COST_MAP_PATH_COLOR, START_COLOR, GOAL_COLOR,
)

START = (12, 1)
GOAL = (12, 23)
SIM_HZ = 240
SENSOR_SCAN_PERIOD_S = 0.3
SENSOR_NUM_RAYS = 48
SENSOR_RANGE = 6.0


def build_demo_grid():
    """The same Grid class the pygame visualizer uses, with a hand-placed
    layout: start and goal sit on the same row, with an obstacle block
    directly between them, blocking the straight route entirely and
    forcing a detour around one edge. The block is deliberately placed
    off-center on that row (row 12 sits only 2 rows below the block's top
    edge but 6 rows above its bottom edge) so going around the top is
    unambiguously the shorter detour -- both a binary-obstacle search and
    a cost-map search are forced to solve the *same* detour, which is
    what makes the difference in how closely each one hugs the block
    worth looking at (a start/goal pair that can route around the block
    entirely, e.g. via a far corner, would make the two searches pick the
    identical path and there'd be nothing to compare -- an earlier
    version of this demo did exactly that by accident)."""
    grid = Grid()
    for row in range(10, 19):
        for col in range(9, 18):
            grid.cells[row][col] = Grid.OBSTACLE
    grid.place_start(*START)
    grid.place_goal(*GOAL)
    return grid


def plan(grid, use_cost_map):
    grid.cost_map_enabled = use_cost_map
    grid.refresh_cost_map()
    path, _, reason, _ = find_path(grid, "astar", grid.start, grid.goal)
    if path is None:
        raise RuntimeError(f"no path found (reason={reason})")
    return path


def to_world_xy(path_cells, cell_size=WORLD_CELL_SIZE):
    return [grid_to_world(row, col, cell_size)[:2] for row, col in path_cells]


def build_drive_waypoints(path_cells, method):
    waypoints = simplify_collinear(to_world_xy(path_cells))
    if method == "raw":
        return waypoints
    if method == "corner_cut":
        return chaikin_smooth(waypoints, iterations=3)
    return catmull_rom_spline(waypoints, samples_per_segment=8)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--smooth", choices=["raw", "corner_cut", "spline"], default="spline")
    parser.add_argument("--no-cost-map", action="store_true")
    parser.add_argument("--sensor", action="store_true",
                         help="plan against lidar-discovered knowledge only, replanning as it explores")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=60.0,
                         help="safety cap so a headless/automated run can't hang forever")
    return parser.parse_args()


def run_static_demo(args, grid, gui):
    """Plan the whole grid twice (binary vs cost-map) up front, since the
    robot already has a perfect map -- there's nothing to discover, so
    nothing to replan."""
    binary_path = plan(grid, use_cost_map=False)
    cost_map_path = plan(grid, use_cost_map=True)
    draw_path(binary_path, BINARY_PATH_COLOR, z=0.03, gui=gui)
    draw_path(cost_map_path, COST_MAP_PATH_COLOR, z=0.06, gui=gui)
    print(f"binary-obstacle path:  {len(binary_path)} cells")
    print(f"cost-map path:         {len(cost_map_path)} cells (drawn in blue, red = binary)")

    chosen_path = binary_path if args.no_cost_map else cost_map_path
    drive_waypoints = build_drive_waypoints(chosen_path, args.smooth)
    corner_waypoints = set(simplify_collinear(to_world_xy(chosen_path)))
    draw_waypoints(drive_waypoints, gui=gui)

    start_xy = drive_waypoints[0]
    robot_id = p.loadURDF("r2d2.urdf", basePosition=[start_xy[0], start_xy[1], 0.4])
    robot = Robot(robot_id)

    print(f"driving {len(drive_waypoints)} waypoints "
          f"(smoothing={args.smooth}, cost_map={not args.no_cost_map})")

    idx = 0
    steps = 0
    max_steps = int(args.max_seconds * SIM_HZ)
    while idx < len(drive_waypoints) and steps < max_steps:
        arrived = robot.drive_toward(drive_waypoints[idx])
        p.stepSimulation()
        if not args.headless:
            time.sleep(1 / SIM_HZ)
        steps += 1
        if arrived:
            if drive_waypoints[idx] in corner_waypoints:
                print(f"  reached waypoint {tuple(round(v, 2) for v in drive_waypoints[idx])}")
            idx += 1

    if idx >= len(drive_waypoints):
        print("reached goal")
    else:
        print(f"stopped after {args.max_seconds}s safety cap ({idx}/{len(drive_waypoints)} waypoints)")


def run_sensor_demo(args, grid, gui):
    """The robot only knows about obstacles nav/sim3d/lidar.py has
    actually raycast-hit. It plans against a KnownGrid (the exact same
    class the pygame sensor mode uses -- unseen cells assumed
    free), drives toward that plan, and rescans every SENSOR_SCAN_PERIOD_S
    seconds; any newly discovered obstacle triggers a fresh plan from
    wherever it currently is. Cost-map mode is skipped here for the same
    reason it is in pygame's sensor mode: KnownGrid doesn't carry the
    real grid's terrain weights, so mixing "unknown obstacles" with
    "unknown terrain cost" is a second, separate problem this demo
    doesn't try to solve at the same time."""
    start_xy = grid_to_world(*START)[:2]
    robot_id = p.loadURDF("r2d2.urdf", basePosition=[start_xy[0], start_xy[1], 0.4])
    robot = Robot(robot_id)
    lidar = Lidar3D(num_rays=SENSOR_NUM_RAYS, max_range=SENSOR_RANGE, ignore_body_id=robot_id)

    def replan_from(position):
        cell = world_to_grid(position[0], position[1])
        known = KnownGrid(lidar.known_obstacles)
        path, _, reason, _ = find_path(known, "astar", cell, GOAL)
        if path is None:
            print(f"  no known path to goal yet (reason={reason}) -- waiting for more of the map")
            return None
        return build_drive_waypoints(path, args.smooth)

    lidar.scan(start_xy, gui=gui)
    drive_waypoints = replan_from(start_xy)
    print(f"initial scan: {len(lidar.known_obstacles)} obstacle cells sensed")

    idx = 0
    steps = 0
    max_steps = int(args.max_seconds * SIM_HZ)
    next_scan_step = int(SENSOR_SCAN_PERIOD_S * SIM_HZ)
    reached_goal = False

    while steps < max_steps:
        if drive_waypoints is None:
            # No known route to the goal yet -- hold position and keep
            # scanning until enough of the map has been discovered.
            p.stepSimulation()
        elif idx >= len(drive_waypoints):
            reached_goal = True
            break
        else:
            if robot.drive_toward(drive_waypoints[idx]):
                idx += 1
            p.stepSimulation()

        if not args.headless:
            time.sleep(1 / SIM_HZ)
        steps += 1

        if steps >= next_scan_step:
            next_scan_step += int(SENSOR_SCAN_PERIOD_S * SIM_HZ)
            newly_seen = lidar.scan(robot.position(), gui=gui)
            if newly_seen:
                print(f"  sensed {len(newly_seen)} new obstacle cell(s) at t={steps / SIM_HZ:.1f}s -- replanning")
                drive_waypoints = replan_from(robot.position())
                idx = 0

    if reached_goal:
        print("reached goal")
    else:
        print(f"stopped after {args.max_seconds}s safety cap")
    print(f"final known map: {len(lidar.known_obstacles)} obstacle cells sensed out of "
          f"{sum(row.count(Grid.OBSTACLE) for row in grid.cells)} actual")


def main():
    args = parse_args()
    gui = not args.headless
    connect(gui=gui)

    grid = build_demo_grid()
    build_obstacles(grid)
    mark_cell(*START, color=START_COLOR)
    mark_cell(*GOAL, color=GOAL_COLOR)

    if args.sensor:
        run_sensor_demo(args, grid, gui)
    else:
        run_static_demo(args, grid, gui)

    if not args.headless:
        input("Press Enter to close...")
    p.disconnect()


if __name__ == "__main__":
    main()
