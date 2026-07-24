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
    python3 pybullet_main.py --elevation          # sloped/stepped terrain, elevation-aware routing
    python3 pybullet_main.py --headless           # DIRECT mode, no GUI window, for automated runs
"""
import argparse
import math
import time

import pybullet as p

from nav.algorithms import find_path, path_cost
from nav.config import CONFIRMATION_THRESHOLD, ELEVATION_COST_FACTOR
from nav.grid import Grid
from nav.sensor import KnownGrid
from nav.sim3d.coords import grid_to_world, world_to_grid, WORLD_CELL_SIZE
from nav.sim3d.hud import Hud
from nav.sim3d.lidar import Lidar3D
from nav.sim3d.robot import Robot, DEFAULT_SPEED
from nav.sim3d.smoothing import simplify_collinear, chaikin_smooth, catmull_rom_spline
from nav.sim3d.world import (
    connect, build_obstacles, build_terrain, hide_obstacles, reveal_obstacles, mark_cell, draw_path,
    draw_xy_path, draw_waypoints, draw_cost_map_tint, remove_debug_items,
    BINARY_PATH_COLOR, COST_MAP_PATH_COLOR, SENSOR_PATH_COLOR, START_COLOR, GOAL_COLOR,
    ELEVATION_UNAWARE_PATH_COLOR, ELEVATION_AWARE_PATH_COLOR,
)

START = (12, 1)
GOAL = (12, 23)
SIM_HZ = 240
SENSOR_SCAN_PERIOD_S = 0.3
SENSOR_NUM_RAYS = 48
SENSOR_RANGE = 6.0
# A hill straddling the direct route from START to GOAL, tall/steep
# enough that elevation-aware routing prefers going around it entirely
# to climbing it (see WRITEUPS.md for the parameter search behind these
# specific numbers, and why the margin matters physically, not just
# numerically -- a route that clips even the hill's lower slope tipped
# a driven robot over in testing, one that avoids it completely never did).
HILL_CENTER = (12, 13)
HILL_PEAK = 1.5
HILL_RADIUS = 6
# How often the HUD text / debug-parameter sliders actually get read and
# redrawn -- doing it every physics step (240/s) would spam PyBullet's
# debug-item pipeline for no visible benefit; a human can't perceive HUD
# updates faster than this anyway.
HUD_UPDATE_PERIOD_S = 0.1
# How long the "REPLANNING..." indicator stays lit after a real replan,
# mirroring nav/visualizer.py's REPLAN_FLASH_MS.
REPLAN_FLASH_S = 0.7
HUD_POSITION = (4, 4, 6)


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


def build_elevation_grid():
    """No obstacles at all -- a single round hill (see HILL_CENTER/
    HILL_PEAK/HILL_RADIUS) straddling the direct route between START
    and GOAL, radially symmetric so its per-cell grade is the same in
    every direction (max HILL_PEAK / HILL_RADIUS, comfortably within
    what a driven robot can actually climb -- see WRITEUPS.md)."""
    grid = Grid()
    for row in range(grid.size):
        for col in range(grid.size):
            dist = math.hypot(row - HILL_CENTER[0], col - HILL_CENTER[1])
            if dist <= HILL_RADIUS:
                grid.elevation[row][col] = HILL_PEAK * (1 - dist / HILL_RADIUS)
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
    parser.add_argument("--noisy-sensor", action="store_true",
                         help="(with --sensor) make the lidar imperfect -- misses, position noise, and "
                              "false positives (see nav/sim3d/lidar.py) -- and require "
                              f"{CONFIRMATION_THRESHOLD}+ repeated detections before trusting a cell "
                              "enough to replan on")
    parser.add_argument("--elevation", action="store_true",
                         help="sloped/stepped terrain instead of a flat plane -- drives the "
                              "elevation-aware route around a hill instead of climbing straight over it "
                              "(see nav/grid.py: elevation_aware, WRITEUPS.md)")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=60.0,
                         help="safety cap so a headless/automated run can't hang forever")
    return parser.parse_args()


def run_static_demo(args, grid, gui):
    """Plan the whole grid twice (binary vs cost-map) up front, since the
    robot already has a perfect map -- there's nothing to discover, so
    nothing to replan."""
    binary_path = plan(grid, use_cost_map=False)
    binary_cost = path_cost(grid, binary_path)
    cost_map_path = plan(grid, use_cost_map=True)
    cost_map_cost = path_cost(grid, cost_map_path)
    draw_path(binary_path, BINARY_PATH_COLOR, z=0.03, gui=gui)
    draw_path(cost_map_path, COST_MAP_PATH_COLOR, z=0.06, gui=gui)
    if not args.no_cost_map:
        draw_cost_map_tint(grid, gui=gui)
    print(f"binary-obstacle path:  {len(binary_path)} cells, cost {binary_cost:.2f}")
    print(f"cost-map path:         {len(cost_map_path)} cells, cost {cost_map_cost:.2f} "
          f"(drawn in blue, red = binary)")

    chosen_path = binary_path if args.no_cost_map else cost_map_path
    chosen_cost = binary_cost if args.no_cost_map else cost_map_cost
    drive_waypoints = build_drive_waypoints(chosen_path, args.smooth)
    corner_waypoints = set(simplify_collinear(to_world_xy(chosen_path)))
    draw_waypoints(drive_waypoints, gui=gui)

    start_xy = drive_waypoints[0]
    robot_id = p.loadURDF("r2d2.urdf", basePosition=[start_xy[0], start_xy[1], 0.4])
    robot = Robot(robot_id)

    print(f"driving {len(drive_waypoints)} waypoints "
          f"(smoothing={args.smooth}, cost_map={not args.no_cost_map})")

    hud = Hud(HUD_POSITION, gui)
    speed_param = p.addUserDebugParameter("robot speed", 5.0, 40.0, DEFAULT_SPEED) if gui else None

    idx = 0
    steps = 0
    max_steps = int(args.max_seconds * SIM_HZ)
    next_hud_step = 0
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

        if steps >= next_hud_step:
            next_hud_step = steps + int(HUD_UPDATE_PERIOD_S * SIM_HZ)
            if speed_param is not None:
                robot.speed = p.readUserDebugParameter(speed_param)
            status = "reached goal" if idx >= len(drive_waypoints) else "driving"
            hud.update([
                f"[static] A* | cost_map={'on' if not args.no_cost_map else 'off'} | smoothing={args.smooth}",
                f"path: {len(chosen_path)} cells, cost {chosen_cost:.2f} | waypoint {idx}/{len(drive_waypoints)}",
                f"{status} | t={steps / SIM_HZ:.1f}s",
            ])

    if idx >= len(drive_waypoints):
        print("reached goal")
    else:
        print(f"stopped after {args.max_seconds}s safety cap ({idx}/{len(drive_waypoints)} waypoints)")


def run_elevation_demo(args, grid, gui):
    """Plan the hill crossing twice -- once ignoring elevation entirely
    (a plain 4-directional search finds the straight line, since nothing
    here is an actual obstacle to route around), once charging for
    climbing (Grid.elevation_aware) -- and draw both, the same red-vs-
    green comparison run_static_demo draws red-vs-blue for binary-vs-
    cost-map routing.

    Only the elevation-aware route is actually driven. The unaware one
    would send the robot straight over the hill's steepest point, and
    that's not just numerically worse -- driving it was tried, and it
    reliably tipped the robot over, at every speed tested, on both the
    full hill crossing and its own (otherwise fine) elevation-aware
    detour before the hill was made tall/steep enough to route around
    entirely instead of just clipping its lower slope (see WRITEUPS.md).
    A route that never touches a non-flat cell is exactly as drivable as
    this project's other, flat-ground demos; one that still climbs
    partway up a stepped slope turned out not to be, regardless of how
    slowly it's driven."""
    grid.elevation_aware = False
    unaware_path, _, reason, _ = find_path(grid, "astar", grid.start, grid.goal)
    if unaware_path is None:
        raise RuntimeError(f"no elevation-unaware path found (reason={reason})")

    grid.elevation_aware = True
    aware_path, _, reason, _ = find_path(grid, "astar", grid.start, grid.goal)
    if aware_path is None:
        raise RuntimeError(f"no elevation-aware path found (reason={reason})")

    # Cost both paths under the *same* (true, elevation-charging) cost
    # model, now that grid.elevation_aware is left on -- unaware_path was
    # found ignoring elevation, but its real cost, climb surcharge
    # included, is the actual point of comparison; costing it with
    # elevation_aware still off would just report its flat step count
    # again and hide exactly what "ignoring elevation" costs.
    unaware_cost = path_cost(grid, unaware_path)
    aware_cost = path_cost(grid, aware_path)

    draw_path(unaware_path, ELEVATION_UNAWARE_PATH_COLOR, z=0.05, gui=gui)
    draw_path(aware_path, ELEVATION_AWARE_PATH_COLOR, z=0.08, gui=gui)
    peak_unaware = max(grid.elevation[r][c] for r, c in unaware_path)
    peak_aware = max(grid.elevation[r][c] for r, c in aware_path)
    print(f"elevation-unaware path (straight over the hill): {len(unaware_path)} cells, cost {unaware_cost:.2f} "
          f"(peak elevation crossed: {peak_unaware:.2f})")
    print(f"elevation-aware path (routes around it):         {len(aware_path)} cells, cost {aware_cost:.2f} "
          f"(peak elevation crossed: {peak_aware:.2f}) (drawn in green, red = elevation-unaware)")

    drive_waypoints = build_drive_waypoints(aware_path, args.smooth)
    draw_waypoints(drive_waypoints, gui=gui)

    start_xy = drive_waypoints[0]
    start_z = grid.elevation[grid.start[0]][grid.start[1]] + 0.6
    robot_id = p.loadURDF("r2d2.urdf", basePosition=[start_xy[0], start_xy[1], start_z])
    # allow_vertical_fall=True: this route crosses stepped terrain (even
    # though, by construction, only flat cells -- see the docstring
    # above), so the robot needs to be able to actually settle onto the
    # ground plane's true height rather than have gravity fought every
    # control tick. See nav/sim3d/robot.py's Robot.allow_vertical_fall.
    robot = Robot(robot_id, allow_vertical_fall=True)

    print(f"driving the elevation-aware route: {len(drive_waypoints)} waypoints (smoothing={args.smooth})")

    hud = Hud(HUD_POSITION, gui)
    speed_param = p.addUserDebugParameter("robot speed", 5.0, 40.0, DEFAULT_SPEED) if gui else None

    idx = 0
    steps = 0
    max_steps = int(args.max_seconds * SIM_HZ)
    next_hud_step = 0
    while idx < len(drive_waypoints) and steps < max_steps:
        arrived = robot.drive_toward(drive_waypoints[idx])
        p.stepSimulation()
        if not args.headless:
            time.sleep(1 / SIM_HZ)
        steps += 1
        if arrived:
            idx += 1

        if steps >= next_hud_step:
            next_hud_step = steps + int(HUD_UPDATE_PERIOD_S * SIM_HZ)
            if speed_param is not None:
                robot.speed = p.readUserDebugParameter(speed_param)
            status = "reached goal" if idx >= len(drive_waypoints) else "driving"
            hud.update([
                f"[elevation] A* | elevation-aware | smoothing={args.smooth}",
                f"path: {len(aware_path)} cells, cost {aware_cost:.2f} (unaware would cost "
                f"{unaware_cost:.2f}) | waypoint {idx}/{len(drive_waypoints)}",
                f"{status} | t={steps / SIM_HZ:.1f}s",
            ])

    if idx >= len(drive_waypoints):
        print("reached goal")
    else:
        print(f"stopped after {args.max_seconds}s safety cap ({idx}/{len(drive_waypoints)} waypoints)")


def run_sensor_demo(args, grid, gui, obstacle_bodies):
    """The robot only knows about obstacles nav/sim3d/lidar.py has
    actually raycast-hit. It plans against a KnownGrid (the exact same
    class the pygame sensor mode uses -- unseen cells assumed
    free), drives toward that plan, and rescans every SENSOR_SCAN_PERIOD_S
    seconds; any newly discovered obstacle triggers a fresh plan from
    wherever it currently is. Cost-map mode is skipped here for the same
    reason it is in pygame's sensor mode: KnownGrid doesn't carry the
    real grid's terrain weights, so mixing "unknown obstacles" with
    "unknown terrain cost" is a second, separate problem this demo
    doesn't try to solve at the same time.

    Real obstacles start dimmed to near-invisible (see
    hide_obstacles/reveal_obstacles) and only turn solid once the lidar
    actually raycasts them -- matching nav/visualizer.py's sensor mode,
    which draws an undiscovered obstacle as a free cell with just a faint
    outline instead of its real fill color."""
    total_obstacles = sum(row.count(Grid.OBSTACLE) for row in grid.cells)
    hide_obstacles(obstacle_bodies)

    start_xy = grid_to_world(*START)[:2]
    robot_id = p.loadURDF("r2d2.urdf", basePosition=[start_xy[0], start_xy[1], 0.4])
    robot = Robot(robot_id)
    lidar = Lidar3D(num_rays=SENSOR_NUM_RAYS, max_range=SENSOR_RANGE, ignore_body_id=robot_id,
                     noisy=args.noisy_sensor)
    # With noise off, confirmed_cells() is exactly lidar.known_obstacles
    # (trusting a perfect sensor's first reading is safe). With noise on,
    # a cell only counts once it's been reported CONFIRMATION_THRESHOLD+
    # times -- a single false positive or jittered reading of a real
    # obstacle isn't enough to replan on by itself. See WRITEUPS.md.
    confirm_threshold = CONFIRMATION_THRESHOLD if args.noisy_sensor else 1

    def confirmed_cells():
        return lidar.confirmed_obstacles(confirm_threshold)

    hud = Hud(HUD_POSITION, gui)
    speed_param = p.addUserDebugParameter("robot speed", 5.0, 40.0, DEFAULT_SPEED) if gui else None
    full_map_param = p.addUserDebugParameter("reveal full map (visual only)", 0, 1, 0) if gui else None
    path_line_ids = []

    def replan_from(position):
        cell = world_to_grid(position[0], position[1])
        known = KnownGrid(confirmed_cells())
        path, _, reason, _ = find_path(known, "astar", cell, GOAL)
        if path is None:
            print(f"  no known path to goal yet (reason={reason}) -- waiting for more of the map")
            return None
        return build_drive_waypoints(path, args.smooth)

    lidar.scan(start_xy, gui=gui)
    reveal_obstacles(obstacle_bodies, lidar.known_obstacles)
    drive_waypoints = replan_from(start_xy)
    if drive_waypoints:
        path_line_ids = draw_xy_path(drive_waypoints, SENSOR_PATH_COLOR, gui=gui)
    print(f"initial scan: {len(lidar.known_obstacles)} obstacle cells sensed")

    idx = 0
    steps = 0
    max_steps = int(args.max_seconds * SIM_HZ)
    next_scan_step = int(SENSOR_SCAN_PERIOD_S * SIM_HZ)
    next_hud_step = 0
    replan_flash_until_step = 0
    reached_goal = False
    full_map_revealed = False

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
            before_confirmed = confirmed_cells()
            newly_seen = lidar.scan(robot.position(), gui=gui)
            if newly_seen:
                reveal_obstacles(obstacle_bodies, newly_seen)
            # Replan on newly-*confirmed* cells, not raw sensor output --
            # with noise on, a single stray false positive or jittered
            # reading shouldn't be enough to trigger a replan by itself.
            newly_confirmed = confirmed_cells() - before_confirmed
            if newly_confirmed:
                print(f"  confirmed {len(newly_confirmed)} new obstacle cell(s) at "
                      f"t={steps / SIM_HZ:.1f}s -- replanning")
                drive_waypoints = replan_from(robot.position())
                idx = 0
                replan_flash_until_step = steps + int(REPLAN_FLASH_S * SIM_HZ)
                if drive_waypoints:
                    path_line_ids = draw_xy_path(
                        drive_waypoints, SENSOR_PATH_COLOR, gui=gui, existing_ids=path_line_ids
                    )
                else:
                    remove_debug_items(path_line_ids)
                    path_line_ids = []

        if steps >= next_hud_step:
            next_hud_step = steps + int(HUD_UPDATE_PERIOD_S * SIM_HZ)
            if speed_param is not None:
                robot.speed = p.readUserDebugParameter(speed_param)
            if full_map_param is not None:
                want_full_map = p.readUserDebugParameter(full_map_param) > 0.5
                if want_full_map != full_map_revealed:
                    if want_full_map:
                        reveal_obstacles(obstacle_bodies, obstacle_bodies.keys())
                    else:
                        hide_obstacles(obstacle_bodies)
                        reveal_obstacles(obstacle_bodies, lidar.known_obstacles)
                    full_map_revealed = want_full_map
            if steps < replan_flash_until_step:
                status = "REPLANNING..."
            elif drive_waypoints is None:
                status = "waiting for map"
            elif reached_goal or idx >= len(drive_waypoints):
                status = "reached goal"
            else:
                status = f"driving waypoint {idx}/{len(drive_waypoints)}"
            sensed_line = (
                f"sensed {len(lidar.known_obstacles)} raw / {len(confirmed_cells())} confirmed "
                f"(of {total_obstacles} real) | t={steps / SIM_HZ:.1f}s"
                if args.noisy_sensor else
                f"sensed {len(lidar.known_obstacles)}/{total_obstacles} obstacles | t={steps / SIM_HZ:.1f}s"
            )
            hud.update([
                f"[sensor] A* | lidar-limited{' (noisy)' if args.noisy_sensor else ''} | smoothing={args.smooth}",
                status,
                sensed_line,
            ])

    if reached_goal:
        print("reached goal")
    else:
        print(f"stopped after {args.max_seconds}s safety cap")
    print(f"final known map: {len(lidar.known_obstacles)} obstacle cells sensed out of {total_obstacles} actual")
    if args.noisy_sensor:
        print(f"  of which confirmed (>={confirm_threshold} detections, what the planner actually trusted): "
              f"{len(confirmed_cells())}")


def main():
    args = parse_args()
    gui = not args.headless
    # No flat plane for the elevation demo -- it builds its own terrain
    # (build_terrain) matched to the hill's actual shape instead.
    connect(gui=gui, load_ground_plane=not args.elevation)

    if args.elevation:
        grid = build_elevation_grid()
        build_terrain(grid)
        mark_cell(*START, color=START_COLOR)
        mark_cell(*GOAL, color=GOAL_COLOR)
        run_elevation_demo(args, grid, gui)
    else:
        grid = build_demo_grid()
        obstacle_bodies = build_obstacles(grid)
        mark_cell(*START, color=START_COLOR)
        mark_cell(*GOAL, color=GOAL_COLOR)
        if args.sensor:
            run_sensor_demo(args, grid, gui, obstacle_bodies)
        else:
            run_static_demo(args, grid, gui)

    if not args.headless:
        input("Press Enter to close...")
    p.disconnect()


if __name__ == "__main__":
    main()
