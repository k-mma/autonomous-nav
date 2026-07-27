"""Captures the fixed poster screenshots for all three pybullet_main.py
demos, headless (DIRECT mode has no debug-visualizer camera to read
back from, so this actually opens a real GUI connection, same as
pybullet_main.py does by default -- getCameraImage still works exactly
the same way, it's just reading the camera nav.sim3d.world.connect()
already set up instead of a hand-rolled one, so every screenshot uses
the exact same framing (distance 22, yaw 45, pitch -55, target
[12, 12, 0]) a live viewer would see.

    python3 screenshots/pybullet/capture.py     # writes screenshots/pybullet/*.png
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import numpy as np
import pybullet as p
from PIL import Image

import pybullet_main as pm
from nav.algorithms import find_path
from nav.config import TERRAIN_GRASS
from nav.sim3d.coords import grid_to_world, WORLD_CELL_SIZE
from nav.sim3d.hud import Hud
from nav.sim3d.lidar import Lidar3D
from nav.sim3d.robot import Robot
from nav.sim3d.world import (
    connect, build_obstacles, hide_obstacles, reveal_obstacles, mark_cell, draw_terrain,
    draw_cost_map_tint, draw_path, LivePath, _path_segment_body,
    BINARY_PATH_COLOR, COST_MAP_PATH_COLOR, SENSOR_PATH_COLOR, START_COLOR, GOAL_COLOR,
    TERRAIN_NAIVE_PATH_COLOR, TERRAIN_AWARE_PATH_COLOR, TRIGGER_MARKER_COLOR,
)

OUT_DIR = Path(__file__).resolve().parent
ROBOT_SPAWN_Z = 0.15


def screenshot(name, width=1600, height=1200):
    """Settle a few physics steps (resetDebugVisualizerCamera's target
    doesn't take effect in getDebugVisualizerCamera() until at least one
    step has run), then grab the debug visualizer's own camera matrices
    so every shot is framed identically to what connect() set up, and
    save it.

    Explicitly ER_BULLET_HARDWARE_OPENGL, not the ER_TINY_RENDERER
    default -- verified empirically that TinyRenderer ignores alpha
    entirely (a rgbaColor=(..., 0.05) box renders exactly as opaque as
    one at alpha=1.0), which would silently break both
    HIDDEN_OBSTACLE_COLOR's near-invisible undiscovered obstacles and
    draw_cost_map_tint/draw_terrain's translucent overlays -- the
    hardware renderer blends them correctly.

    Re-enables GUI rendering first -- capture_sensor_pair() turns it off
    for its long drive-forward loop (see its own comment) to avoid
    syncing the GUI window on every one of several thousand physics
    steps, which is far slower wall-clock than headless DIRECT mode even
    with no time.sleep in the loop; nothing in that run-up is ever
    actually looked at, only the moment this function captures."""
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    for _ in range(5):
        p.stepSimulation()
    cam = p.getDebugVisualizerCamera()
    view_matrix, proj_matrix = cam[2], cam[3]
    img = p.getCameraImage(width, height, view_matrix, proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgba = np.reshape(img[2], (height, width, 4))
    out_path = OUT_DIR / name
    Image.fromarray(rgba[:, :, :3].astype("uint8")).save(out_path)
    print(f"wrote {out_path}")


def draw_trigger_x(row, col, z=1.15, size=1.3, thickness=0.35, height=0.1):
    """A mesh 'X' over a grid cell, standing in for
    nav.sim3d.world.draw_trigger_marker for screenshot purposes only --
    that function draws its X with addUserDebugLine, which (like the old
    line-based path renderer -- see PATH_WIDTH's comment in
    nav/sim3d/world.py) never appears in a getCameraImage() capture at
    any lineWidth. Left un-ported in the shared module since the fix
    there was scoped to draw_xy_path/draw_path specifically, and
    draw_trigger_marker's debug-line auto-expiry (lifeTime=1.2s) is
    exactly right for a *live* demo session, where it renders fine in
    the interactive viewport -- it just can't be captured by this
    script. This screenshot-only version has no lifetime; the capture
    script never needs one removed.

    z defaults above OBSTACLE_HEIGHT (1.0m, see nav/sim3d/world.py), not
    just above the husky (~0.4m) -- the cell being marked is the one
    that just turned solid (see reveal_obstacles in the caller), so the
    marker has to clear the obstacle box itself, not only the robot.
    size/thickness were first tried at 0.9/0.12 (comparable to
    draw_trigger_marker's own size=0.9, width=16px), and *did* create
    real geometry (confirmed via getVisualShapeData) -- it just rendered
    as a barely-there sliver a couple pixels wide once placed among the
    obstacle block and husky, the same lesson PATH_WIDTH's own history
    already taught (see its comment): a debug-line pixel width and a
    mesh's real-world size are not interchangeable, and porting a value
    tuned for one to the other under-sizes it badly."""
    x, y, _ = grid_to_world(row, col, WORLD_CELL_SIZE)
    rgba = (*TRIGGER_MARKER_COLOR[:3], 1.0)
    half = size / 2
    # World-axis-aligned (not corner-to-corner diagonal) on purpose: this
    # capture's camera sits at cameraYaw=45 (see nav.sim3d.world.connect),
    # which rotates the view exactly enough that a world-space "+"
    # (horizontal/vertical) reads as a screen-space "X" to the viewer,
    # while a world-space corner-to-corner "X" (what
    # draw_trigger_marker's own two debug lines are, for what it's
    # worth -- same effect there, just never visible in a screenshot to
    # notice) reads as a screen-space "+" instead. Confirmed by
    # rendering both in isolation against this exact camera.
    _path_segment_body(x - half, y, x + half, y, rgba, z, thickness, height)
    _path_segment_body(x, y - half, x, y + half, rgba, z, thickness, height)


def spawn_husky(xy):
    robot_id = p.loadURDF("husky/husky.urdf", basePosition=[xy[0], xy[1], ROBOT_SPAWN_Z])
    for _ in range(60):
        p.stepSimulation()
    return robot_id


def capture_binary_vs_costmap():
    """Scenario 1: build_demo_grid's single wall, full map, both the
    binary-obstacle and cost-map paths drawn together with the
    clearance-cost tint. Legend elements: Paths (binary-obstacle path,
    cost-map path), Obstacles & Cost (obstacle, clearance cost tint),
    Markers (start, goal)."""
    connect(gui=True)
    grid = pm.build_demo_grid()
    build_obstacles(grid)
    mark_cell(*pm.START, color=START_COLOR)
    mark_cell(*pm.GOAL, color=GOAL_COLOR)

    binary_path = pm.plan(grid, use_cost_map=False)
    cost_map_path = pm.plan(grid, use_cost_map=True)
    draw_cost_map_tint(grid, gui=True)
    draw_path(binary_path, BINARY_PATH_COLOR, z=0.03, gui=True)
    draw_path(cost_map_path, COST_MAP_PATH_COLOR, z=0.06, gui=True)

    start_xy = grid_to_world(*pm.START)[:2]
    spawn_husky(start_xy)

    screenshot("binary_vs_costmap.png")
    p.disconnect()


def capture_terrain():
    """Scenario 2: build_terrain_grid's scattered forest floor
    (obstacles plus bush/mud/water terrain), terrain-naive vs.
    terrain-aware paths. Legend elements: Paths (terrain-aware path,
    naive/binary baseline), Terrain (grass/bush/mud/water), Obstacles &
    Cost (obstacle), Markers (start, goal)."""
    connect(gui=True)
    grid = pm.build_terrain_grid()
    build_obstacles(grid)
    mark_cell(*pm.START, color=START_COLOR)
    mark_cell(*pm.GOAL, color=GOAL_COLOR)

    # cost_map_enabled=True -- matches pybullet_main.py's run_terrain_demo;
    # obstacles are real scattered geometry now, so both paths need
    # clearance from them or the driven route can clip one.
    real_terrain = [row[:] for row in grid.terrain]
    grid.terrain = [[TERRAIN_GRASS for _ in range(grid.size)] for _ in range(grid.size)]
    grid.cost_map_enabled = True
    grid.refresh_cost_map()
    naive_path, _, reason, _ = find_path(grid, "astar", grid.start, grid.goal)
    assert naive_path is not None, reason

    grid.terrain = real_terrain
    grid.refresh_cost_map()
    aware_path, _, reason, _ = find_path(grid, "astar", grid.start, grid.goal)
    assert aware_path is not None, reason

    draw_terrain(grid, gui=True)
    # z=0.09/0.14 -- matches pybullet_main.py's run_terrain_demo; see
    # nav/sim3d/world.py's draw_terrain docstring for why these need a
    # real gap above its z=0.04, not just a nonzero one.
    draw_path(naive_path, TERRAIN_NAIVE_PATH_COLOR, z=0.09, gui=True)
    draw_path(aware_path, TERRAIN_AWARE_PATH_COLOR, z=0.14, gui=True)

    start_xy = grid_to_world(*pm.START)[:2]
    spawn_husky(start_xy)

    screenshot("terrain_routing.png")
    p.disconnect()


def capture_sensor_pair():
    """Scenario 3: build_scattered_grid's discrete obstacle blocks, a
    before/after pair.

    (a) sensor_before.png -- shortly after the initial scan: two nearby
        obstacle blocks revealed, a tentative path already planned
        around them, the two far blocks (including the one dead ahead
        on the direct route) still dimmed/undiscovered.
    (b) sensor_after.png -- captured right after the robot, driving
        that tentative path, gets close enough to confirm the obstacle
        block squarely on its route: the old path is still lingering
        (dim gray) next to the new flash-colored replan, with a trigger
        marker "X" on the newly-confirmed cell.

    Legend elements: Paths (sensor-limited path), Replan Signals (just
    replanned, superseded path, discovery trigger), Obstacles & Cost
    (obstacle, undiscovered obstacle), Markers (start, goal)."""
    connect(gui=True)
    grid = pm.build_scattered_grid()
    obstacle_bodies = build_obstacles(grid)
    mark_cell(*pm.START, color=START_COLOR)
    mark_cell(*pm.GOAL, color=GOAL_COLOR)
    draw_terrain(grid, gui=True)
    hide_obstacles(obstacle_bodies)

    start_xy = grid_to_world(*pm.START)[:2]
    robot_id = p.loadURDF("husky/husky.urdf", basePosition=[start_xy[0], start_xy[1], ROBOT_SPAWN_Z])
    robot = Robot(robot_id)
    lidar = Lidar3D(num_rays=pm.SENSOR_NUM_RAYS, max_range=pm.SENSOR_RANGE, ignore_body_id=robot_id)
    # z=0.09, matching pybullet_main.py's run_sensor_demo -- draw_terrain's
    # own overlay sits at z=0.04 and needs a real (~3cm+) gap from
    # anything drawn above it to avoid shadow-map z-fighting at this
    # project's usual camera distance (see nav/sim3d/world.py: draw_terrain).
    live_path = LivePath(SENSOR_PATH_COLOR, True, pm.SIM_HZ, z=0.09)
    hud = Hud(pm.HUD_POSITION, True)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

    # The scattered block squarely on the direct row-12 route (see
    # SCATTERED_OBSTACLES) -- the one this capture drives toward and
    # waits to see confirmed, rather than whichever block happens to be
    # discovered first.
    target_block = (11, 13, 13, 14)
    target_cells = {
        (row, col)
        for row in range(target_block[0], target_block[1] + 1)
        for col in range(target_block[2], target_block[3] + 1)
    }

    class DummyArgs:
        smooth = "spline"
        noisy_sensor = False

    args = DummyArgs()

    from nav.sensor import KnownGrid

    def replan_from(position):
        cell = pm.world_to_grid(position[0], position[1])
        known = KnownGrid(set(lidar.known_obstacles))
        path, _, reason, _ = find_path(known, "astar", cell, pm.GOAL)
        if path is None:
            return None
        return pm.build_drive_waypoints(path, args.smooth)

    lidar.scan(start_xy, gui=True)
    reveal_obstacles(obstacle_bodies, lidar.known_obstacles)
    drive_waypoints = replan_from(start_xy)
    live_path.set_path(drive_waypoints or [], 0)
    hud.update([
        "[sensor] A* | lidar-limited | smoothing=spline",
        f"driving waypoint 0/{len(drive_waypoints or [])}",
        f"sensed {len(lidar.known_obstacles)}/{sum(row.count(1) for row in grid.cells)} obstacles | t=0.0s",
    ])

    # 110 steps, not just a handful -- long enough to clear
    # PATH_FLASH_SECONDS (0.4s = 96 steps at SIM_HZ=240) so this "early
    # state" shot shows the tentative path settled into its real
    # SENSOR_PATH_COLOR green, not still mid-flash.
    for _ in range(110):
        p.stepSimulation()
    live_path.tick(110)
    screenshot("sensor_before.png")
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

    # Drive forward, rescanning on the usual cadence, until the target
    # block is actually confirmed.
    idx = 0
    steps = 110
    max_steps = int(20 * pm.SIM_HZ)
    next_scan_step = steps + int(pm.SENSOR_SCAN_PERIOD_S * pm.SIM_HZ)
    trigger_step = None
    while steps < max_steps:
        if drive_waypoints and idx < len(drive_waypoints):
            if robot.drive_toward(drive_waypoints[idx]):
                idx += 1
        p.stepSimulation()
        steps += 1

        if steps >= next_scan_step:
            next_scan_step += int(pm.SENSOR_SCAN_PERIOD_S * pm.SIM_HZ)
            before_known = set(lidar.known_obstacles)
            newly_seen = lidar.scan(robot.position(), gui=True)
            if newly_seen:
                reveal_obstacles(obstacle_bodies, newly_seen)
            newly_confirmed = set(lidar.known_obstacles) - before_known
            if newly_confirmed & target_cells:
                for cell in newly_confirmed & target_cells:
                    draw_trigger_x(*cell)
                drive_waypoints = replan_from(robot.position())
                idx = 0
                live_path.set_path(drive_waypoints or [], steps)
                trigger_step = steps
                break
            elif newly_confirmed:
                drive_waypoints = replan_from(robot.position())
                idx = 0
                live_path.set_path(drive_waypoints or [], steps)

    assert trigger_step is not None, "target obstacle block was never confirmed -- capture setup is broken"

    # A few steps after the replan -- still inside the flash/linger
    # windows (PATH_FLASH_SECONDS=0.4s, PATH_LINGER_SECONDS=1.2s at
    # SIM_HZ=240) so the new path reads as freshly flashed and the old
    # one is still visibly lingering.
    for _ in range(48):
        p.stepSimulation()
        steps += 1
    live_path.tick(steps)
    hud.update([
        "[sensor] A* | lidar-limited | smoothing=spline",
        "REPLANNING...",
        f"sensed {len(lidar.known_obstacles)}/{sum(row.count(1) for row in grid.cells)} obstacles | "
        f"t={steps / pm.SIM_HZ:.1f}s",
    ])
    screenshot("sensor_after.png")
    p.disconnect()


def main():
    capture_binary_vs_costmap()
    capture_terrain()
    capture_sensor_pair()


if __name__ == "__main__":
    main()
