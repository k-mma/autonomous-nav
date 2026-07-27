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
    python3 pybullet_main.py --terrain            # scattered forest terrain, naive vs cost-aware routing
    python3 pybullet_main.py --headless           # DIRECT mode, no GUI window, for automated runs
"""
import argparse
import random
import time

import pybullet as p

from nav.algorithms import find_path, path_cost
from nav.config import CONFIRMATION_THRESHOLD, TERRAIN_BUSH, TERRAIN_GRASS, TERRAIN_MUD, TERRAIN_WATER
from nav.grid import Grid
from nav.scenario_helpers import scatter_obstacles, scatter_terrain
from nav.sensor import KnownGrid
from nav.sim3d.coords import grid_to_world, world_to_grid, WORLD_CELL_SIZE
from nav.sim3d.hud import Hud
from nav.sim3d.lidar import Lidar3D
from nav.sim3d.robot import Robot, DEFAULT_SPEED
from nav.sim3d.smoothing import simplify_collinear, chaikin_smooth, catmull_rom_spline
from nav.sim3d.world import (
    connect, build_obstacles, hide_obstacles, reveal_obstacles, mark_cell,
    draw_waypoints, draw_cost_map_tint, draw_terrain, draw_trigger_marker, LivePath,
    BINARY_PATH_COLOR, COST_MAP_PATH_COLOR, SENSOR_PATH_COLOR, START_COLOR, GOAL_COLOR,
    TERRAIN_NAIVE_PATH_COLOR, TERRAIN_AWARE_PATH_COLOR,
)

START = (12, 1)
GOAL = (12, 23)
SIM_HZ = 240
SENSOR_SCAN_PERIOD_S = 0.3
SENSOR_NUM_RAYS = 48
SENSOR_RANGE = 6.0
# Small, discrete obstacle blocks for the sensor demo (see
# build_scattered_grid) -- two sit within SENSOR_RANGE of START and are
# revealed by the very first scan, two sit well outside it and are only
# discovered once the robot drives closer. One of those far blocks
# straddles the direct row-12 route from START to GOAL, so discovering it
# forces a genuine mid-drive replan rather than a cosmetic-only reveal.
SCATTERED_OBSTACLES = [
    (9, 10, 4, 5),
    (14, 15, 6, 7),
    (11, 13, 13, 14),
    (9, 10, 19, 20),
]
# Forest-floor scattering for the terrain demo (see build_terrain_grid)
# -- each layer gets its own Random(seed) so tuning one density doesn't
# reshuffle the others. Water is kept sparsest since it's both the most
# expensive terrain (see TERRAIN_COST) and the most visually dominant
# color; obstacles are kept sparser than pygame's own scatter_obstacles
# scenarios (0.08-0.10) since a 3D box reads as "more obstacle" per cell
# than a flat 2D square does at this camera distance.
# Picked (out of a search over nearby seeds) for a naive-vs-aware
# comparison that's actually dramatic once terrain is scattered instead
# of one solid block: a bad seed can land naive and aware on nearly the
# same route by chance, which is a fine outcome for the planner but a
# poor one for a screenshot meant to show they differ.
FOREST_SEED = 20260769
FOREST_OBSTACLE_DENSITY = 0.05
FOREST_BUSH_DENSITY = 0.12
FOREST_MUD_DENSITY = 0.08
FOREST_WATER_DENSITY = 0.05
# Terrain scattering for the sensor/lidar demo (see build_scattered_grid)
# -- a different seed from FOREST_SEED so the two terrain-scattered demos
# don't happen to paint the same pattern, same density values as the
# terrain demo otherwise (no reason for this one to look sparser/denser,
# it's the same "forest floor" ground either way).
SENSOR_TERRAIN_SEED = 20260827
SENSOR_BUSH_DENSITY = 0.12
SENSOR_MUD_DENSITY = 0.08
SENSOR_WATER_DENSITY = 0.05
# How often the HUD text / debug-parameter sliders actually get read and
# redrawn -- doing it every physics step (240/s) would spam PyBullet's
# debug-item pipeline for no visible benefit; a human can't perceive HUD
# updates faster than this anyway.
HUD_UPDATE_PERIOD_S = 0.1
# How long the "REPLANNING..." HUD indicator stays lit after a real
# replan, mirroring nav/visualizer.py's REPLAN_FLASH_MS -- matched to
# nav/sim3d/world.py's PATH_LINGER_SECONDS so the HUD text and the old
# path's on-screen linger both clear at roughly the same moment.
REPLAN_FLASH_S = 1.2
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


def build_scattered_grid():
    """Several small, discrete obstacle blocks (see SCATTERED_OBSTACLES)
    instead of build_demo_grid's single wall -- mirrors pygame's
    scenario_maze.py/scenario_bottleneck.py approach of scattering
    obstacles rather than one contiguous block. Used only by the
    sensor/lidar demo (see main()): a single wall gives the lidar exactly
    one discovery event, not much to actually watch happen, whereas
    several separated blocks -- some inside the robot's initial scan
    radius, at least one well outside it and squarely on the direct
    route -- produce multiple distinct reveal moments and a real
    mid-drive replan. build_demo_grid's single wall stays exactly as it
    was for the binary-vs-cost-map comparison, which depends on one clean
    shared detour rather than several scattered ones.

    Also scatters bush/mud/water across the whole grid (see SENSOR_*_
    DENSITY), mirroring pygame's own sensor scenarios (hidden_animals.py/
    unreliable_sensor.py), which are terrain-scattered the same way. This
    is purely cosmetic here, not a second thing being tested alongside
    sensing: run_sensor_demo plans against nav.sensor.KnownGrid, which
    (per its own docstring) never carries cost-map/terrain weighting over
    from the real grid at all, so the routes driven are exactly as
    terrain-blind as they were before -- only the rendered world (and the
    real, if unused-by-planning, cost field on `grid` itself) changes."""
    grid = Grid()
    for row_start, row_end, col_start, col_end in SCATTERED_OBSTACLES:
        for row in range(row_start, row_end + 1):
            for col in range(col_start, col_end + 1):
                grid.cells[row][col] = Grid.OBSTACLE
    grid.place_start(*START)
    grid.place_goal(*GOAL)
    scatter_terrain(grid, random.Random(SENSOR_TERRAIN_SEED), TERRAIN_BUSH, SENSOR_BUSH_DENSITY)
    scatter_terrain(grid, random.Random(SENSOR_TERRAIN_SEED + 1), TERRAIN_MUD, SENSOR_MUD_DENSITY)
    scatter_terrain(grid, random.Random(SENSOR_TERRAIN_SEED + 2), TERRAIN_WATER, SENSOR_WATER_DENSITY)
    return grid


def build_terrain_grid():
    """A varied forest floor, not one painted patch: scattered trees/
    rocks (obstacles) and three different terrain types -- bush, mud,
    water, each costing progressively more per step (see nav/config.py:
    TERRAIN_COST) -- spread across the whole grid, mirroring how
    scatter_obstacles/scatter_terrain already build pygame's own
    scenario_open.py/scenario_costmap.py rather than one hand-placed
    block. The question this demo asks is "does terrain-aware routing
    still pay off once the whole map is uneven," not just "does it
    detour around one bad patch."

    Start/goal are placed first -- scatter_obstacles and scatter_terrain
    both already skip whatever cell a start/goal occupies (via
    Grid.toggle_obstacle/paint_terrain's own guards), so placing them
    first just means neither scatter pass ever has a chance to land on
    top of one."""
    grid = Grid()
    grid.place_start(*START)
    grid.place_goal(*GOAL)

    scatter_obstacles(grid, random.Random(FOREST_SEED), FOREST_OBSTACLE_DENSITY)
    scatter_terrain(grid, random.Random(FOREST_SEED + 1), TERRAIN_BUSH, FOREST_BUSH_DENSITY)
    scatter_terrain(grid, random.Random(FOREST_SEED + 2), TERRAIN_MUD, FOREST_MUD_DENSITY)
    scatter_terrain(grid, random.Random(FOREST_SEED + 3), TERRAIN_WATER, FOREST_WATER_DENSITY)
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
    parser.add_argument("--terrain", action="store_true",
                         help="scattered forest floor (bush/mud/water and obstacles) instead of a single "
                              "hard-obstacle wall -- compares a route that ignores terrain cost against "
                              "one that routes around it (see nav/grid.py: paint_terrain, TERRAIN_COST)")
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
    # LivePath, even for a path that's only ever drawn once: it still
    # gets the flash-in reveal (bright, then settles to its real color a
    # moment later) instead of simply appearing -- a small thing, but it
    # means every path in every demo announces itself onscreen the same
    # way, which matters more for a screenshot/video than it does live.
    binary_live = LivePath(BINARY_PATH_COLOR, gui, SIM_HZ, z=0.03)
    cost_map_live = LivePath(COST_MAP_PATH_COLOR, gui, SIM_HZ, z=0.06)
    binary_live.set_path(to_world_xy(binary_path), 0)
    cost_map_live.set_path(to_world_xy(cost_map_path), 0)
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
    robot_id = p.loadURDF("husky/husky.urdf", basePosition=[start_xy[0], start_xy[1], 0.15])
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
            binary_live.tick(steps)
            cost_map_live.tick(steps)
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


def run_terrain_demo(args, grid, gui):
    """Plan the forest crossing twice -- once blind to terrain cost
    (temporarily flattening every cell to TERRAIN_GRASS, so the search
    still routes around every real obstacle -- trees/rocks stay
    impassable in both searches -- but no longer prefers grass over
    bush/mud/water among the cells that are open), once respecting it
    -- and draw both, the same red-vs-green naive/aware comparison.

    Both routes are physically flat ground (bush/mud/water are cost
    penalties, not elevation), so there's no drivability reason to only
    drive one of them -- the cost-aware route is driven because it's the
    one the demo is actually about, not because the naive one is
    unsafe."""
    # cost_map_enabled=True for *both* searches, not just a naive/aware
    # toggle -- obstacles are real, scattered geometry now (see
    # build_terrain_grid), not the empty flat ground this demo used to
    # have, so both routes need real clearance from them or the driven
    # spline can round a corner close enough to actually clip an
    # obstacle's collision box (confirmed: the husky reliably got stuck
    # against one without this). This only adds clearance -- it doesn't
    # touch what's actually being compared (terrain cost on vs. off).
    real_terrain = [row[:] for row in grid.terrain]
    grid.terrain = [[TERRAIN_GRASS for _ in range(grid.size)] for _ in range(grid.size)]
    grid.cost_map_enabled = True
    grid.refresh_cost_map()
    naive_path, _, reason, _ = find_path(grid, "astar", grid.start, grid.goal)
    if naive_path is None:
        raise RuntimeError(f"no terrain-naive path found (reason={reason})")

    grid.terrain = real_terrain
    grid.refresh_cost_map()
    aware_path, _, reason, _ = find_path(grid, "astar", grid.start, grid.goal)
    if aware_path is None:
        raise RuntimeError(f"no terrain-aware path found (reason={reason})")

    # Cost both paths under the *same* (true, terrain-charging) cost
    # model, now that grid.terrain is back to the real patch -- naive_path
    # was found ignoring terrain, but its real cost, mud/water surcharge
    # included, is the actual point of comparison.
    naive_cost = path_cost(grid, naive_path)
    aware_cost = path_cost(grid, aware_path)

    draw_terrain(grid, gui=gui)
    # z=0.09/0.14, not the smaller offsets other paths use elsewhere in
    # this file -- draw_terrain's own ground overlay sits right beneath
    # these (up to z=0.04) and needs a real gap, not just a nonzero one,
    # to avoid shadow-map z-fighting at this camera distance (see
    # nav/sim3d/world.py: draw_terrain).
    naive_live = LivePath(TERRAIN_NAIVE_PATH_COLOR, gui, SIM_HZ, z=0.09)
    aware_live = LivePath(TERRAIN_AWARE_PATH_COLOR, gui, SIM_HZ, z=0.14)
    naive_live.set_path(to_world_xy(naive_path), 0)
    aware_live.set_path(to_world_xy(aware_path), 0)
    print(f"terrain-naive path (ignores bush/mud/water cost): {len(naive_path)} cells, cost {naive_cost:.2f}")
    print(f"terrain-aware path (routes around costly terrain): {len(aware_path)} cells, cost {aware_cost:.2f} "
          f"(drawn in green, red = terrain-naive)")

    drive_waypoints = build_drive_waypoints(aware_path, args.smooth)
    draw_waypoints(drive_waypoints, gui=gui)

    start_xy = drive_waypoints[0]
    robot_id = p.loadURDF("husky/husky.urdf", basePosition=[start_xy[0], start_xy[1], 0.15])
    robot = Robot(robot_id)

    print(f"driving the terrain-aware route: {len(drive_waypoints)} waypoints (smoothing={args.smooth})")

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
            naive_live.tick(steps)
            aware_live.tick(steps)
            if speed_param is not None:
                robot.speed = p.readUserDebugParameter(speed_param)
            status = "reached goal" if idx >= len(drive_waypoints) else "driving"
            hud.update([
                f"[terrain] A* | terrain-aware | smoothing={args.smooth}",
                f"path: {len(aware_path)} cells, cost {aware_cost:.2f} (naive would cost "
                f"{naive_cost:.2f}) | waypoint {idx}/{len(drive_waypoints)}",
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
    robot_id = p.loadURDF("husky/husky.urdf", basePosition=[start_xy[0], start_xy[1], 0.15])
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
    # z=0.09, not LivePath's z=0.05 default -- draw_terrain's own overlay
    # (called in main() before this) sits at z=0.04, and its own
    # docstring documents needing a ~3cm+ gap from anything drawn above
    # it to avoid shadow-map z-fighting at this project's usual camera
    # distance; run_terrain_demo's paths clear the same overlay the same
    # way.
    live_path = LivePath(SENSOR_PATH_COLOR, gui, SIM_HZ, z=0.09)

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
    live_path.set_path(drive_waypoints or [], 0)
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
            # scanning until enough of the map has been discovered. Must
            # call stop() explicitly: resetBaseVelocity persists whatever
            # velocity the last drive_toward() call set, so a robot that
            # was moving when a replan lost the route would otherwise keep
            # coasting instead of actually holding position.
            robot.stop()
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
                for cell in newly_confirmed:
                    draw_trigger_marker(*cell, gui=gui)
                drive_waypoints = replan_from(robot.position())
                idx = 0
                replan_flash_until_step = steps + int(REPLAN_FLASH_S * SIM_HZ)
                live_path.set_path(drive_waypoints or [], steps)

        if steps >= next_hud_step:
            next_hud_step = steps + int(HUD_UPDATE_PERIOD_S * SIM_HZ)
            live_path.tick(steps)
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
    connect(gui=gui)

    # try/finally so a RuntimeError from a demo finding no path (see
    # run_static_demo/run_terrain_demo above) still disconnects the
    # pybullet client instead of leaking it -- matters for --headless,
    # which is meant for repeated automated runs.
    try:
        if args.terrain:
            grid = build_terrain_grid()
            build_obstacles(grid)
            mark_cell(*START, color=START_COLOR)
            mark_cell(*GOAL, color=GOAL_COLOR)
            run_terrain_demo(args, grid, gui)
        elif args.sensor:
            grid = build_scattered_grid()
            obstacle_bodies = build_obstacles(grid)
            mark_cell(*START, color=START_COLOR)
            mark_cell(*GOAL, color=GOAL_COLOR)
            draw_terrain(grid, gui=gui)
            run_sensor_demo(args, grid, gui, obstacle_bodies)
        else:
            grid = build_demo_grid()
            obstacle_bodies = build_obstacles(grid)
            mark_cell(*START, color=START_COLOR)
            mark_cell(*GOAL, color=GOAL_COLOR)
            run_static_demo(args, grid, gui)

        if not args.headless:
            input("Press Enter to close...")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
