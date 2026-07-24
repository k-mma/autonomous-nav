import math

import pybullet as p
import pybullet_data

from nav.config import COST_MAX_EXTRA
from nav.grid import Grid
from nav.sim3d.coords import grid_to_world, WORLD_CELL_SIZE

OBSTACLE_HEIGHT = 1.0
OBSTACLE_COLOR = (0.15, 0.15, 0.15, 1.0)
# Real obstacle the lidar hasn't raycast-hit yet -- nearly invisible
# instead of fully removed, since the collision shape underneath still has
# to exist (both for the robot to physically bump into and for lidar's own
# raycasts to hit), so "hidden" here means "dim", not "absent". Mirrors
# nav/visualizer.py's sensor mode drawing an undiscovered obstacle as a
# free cell with a faint outline.
HIDDEN_OBSTACLE_COLOR = (0.15, 0.15, 0.15, 0.05)
BINARY_PATH_COLOR = (0.9, 0.15, 0.15)
COST_MAP_PATH_COLOR = (0.1, 0.5, 0.95)
SENSOR_PATH_COLOR = (0.2, 0.75, 0.35)
# The route that ignores elevation (red, same "naive baseline" role
# BINARY_PATH_COLOR plays for the cost-map comparison) vs the one that
# charges for climbing (green, distinguishable from COST_MAP_PATH_COLOR's
# blue since a demo could in principle show both comparisons at once).
ELEVATION_UNAWARE_PATH_COLOR = (0.9, 0.15, 0.15)
ELEVATION_AWARE_PATH_COLOR = (0.15, 0.85, 0.35)
WAYPOINT_MARKER_COLOR = (1.0, 0.85, 0.1, 1.0)
START_COLOR = (0.2, 0.8, 0.4, 1.0)
GOAL_COLOR = (0.9, 0.25, 0.25, 1.0)
ROBOT_A_COLOR = (0.1, 0.7, 0.9, 1.0)
ROBOT_B_COLOR = (0.95, 0.55, 0.1, 1.0)
# Top color of the cost-map ground tint at maximum extra cost; blends
# toward invisible (alpha 0) as cost drops to 1.0. Same hue family as
# nav/visualizer.py's COST_TINT, just expressed as a translucent overlay
# instead of an opaque cell fill since it sits on top of the ground plane
# rather than replacing it.
COST_TINT_COLOR = (0.95, 0.55, 0.25)
TERRAIN_COLOR = (0.55, 0.45, 0.32, 1.0)
# Bottom of every terrain column build_terrain builds (see below) --
# comfortably below any elevation this project's demos actually use, so
# there's never a visible gap under a column regardless of how tall its
# neighbors are.
TERRAIN_BASE_Z = -2.0


def connect(gui=True, load_ground_plane=True):
    """Open a PyBullet connection and load the ground plane. This is the
    only new "physics interface" code the 3D port needed -- the grid
    model and A* itself (nav/grid.py, nav/algorithms.py) are unchanged
    from pygame, imported and reused as-is.

    `load_ground_plane=False` skips `plane.urdf` -- for a demo building
    its own elevation-matched terrain instead (see build_terrain), which
    would otherwise sit half-buried in or floating above a separate flat
    plane at z=0."""
    p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.resetDebugVisualizerCamera(
        cameraDistance=22, cameraYaw=45, cameraPitch=-55, cameraTargetPosition=[12, 12, 0]
    )
    if load_ground_plane:
        return p.loadURDF("plane.urdf")
    return None


def build_terrain(grid, cell_size=WORLD_CELL_SIZE, base_z=TERRAIN_BASE_Z, color=TERRAIN_COLOR):
    """One static box "column" per grid cell, its top surface at
    `grid.elevation[row][col]` -- the elevation analogue of
    build_obstacles' one-box-per-cell approach, reusing the exact same
    grid_to_world coordinate convention every other piece of this file
    already relies on (obstacles, markers, paths) instead of introducing
    PyBullet's separate heightfield coordinate/scaling conventions and
    having to keep two systems in sync.

    This produces genuinely *stepped* terrain -- a vertical face
    wherever two adjacent cells differ in elevation -- rather than a
    smoothly interpolated slope. That's an explicit, deliberate choice,
    not a shortcut: a real heightfield collision shape would need its
    own coordinate system reconciled against grid_to_world's, and one
    box per cell is both simpler to verify correct (its footprint is
    exactly one grid cell, exactly like every other body in this file)
    and, for a wheeled robot climbing it, easier to reason about the
    physics of (a short, wheel-height step per cell rather than a
    continuous incline whose steepness varies with elevation data).

    Replaces the flat ground plane entirely -- pass
    `connect(gui, load_ground_plane=False)` first so there's no separate
    flat plane underneath it. Collision *and* visual shapes are cached
    per distinct column height (many cells commonly share the same
    elevation, e.g. a multi-cell-wide ramp step), the same sharing
    build_obstacles already does for collision shapes -- safe here for
    visual shapes too since, unlike hide_obstacles/reveal_obstacles,
    nothing ever calls changeVisualShape on a terrain body afterward."""
    collision_cache = {}
    visual_cache = {}
    body_ids = []
    for row in range(len(grid.cells)):
        for col in range(len(grid.cells[row])):
            top = grid.elevation[row][col]
            half_height = max((top - base_z) / 2, cell_size * 0.01)
            half_extents = [cell_size / 2, cell_size / 2, half_height]

            collision_shape = collision_cache.get(half_height)
            if collision_shape is None:
                collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
                collision_cache[half_height] = collision_shape

            visual_shape = visual_cache.get(half_height)
            if visual_shape is None:
                visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
                visual_cache[half_height] = visual_shape

            x, y, _ = grid_to_world(row, col, cell_size)
            body_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=[x, y, base_z + half_height],
            )
            body_ids.append(body_id)
    return body_ids


def build_obstacles(grid, cell_size=WORLD_CELL_SIZE, height=OBSTACLE_HEIGHT):
    """Create one static box body per obstacle cell in `grid`, positioned
    to line up 1:1 with the pygame grid layout -- just projected from
    pixels into meters instead of redesigning the map. Returns a
    {(row, col): body_id} map (rather than a bare list) so sensor mode can
    look a body up by grid cell to dim/reveal it as the lidar discovers it
    -- see hide_obstacles/reveal_obstacles below.

    The collision shape is shared across every cell (collision geometry
    has no per-instance color, so sharing it is free), but each cell gets
    its own visual shape rather than sharing one -- calling
    changeVisualShape per-body on a *shared* visual shape index doesn't
    reliably keep instances' colors independent in PyBullet's renderer
    (verified empirically: hiding/revealing a mix of cells on a shared
    shape rendered the whole contiguous block as solid regardless of
    which cells were actually marked opaque). One shape per body avoids
    that entirely."""
    half_extents = [cell_size / 2, cell_size / 2, height / 2]
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)

    obstacle_bodies = {}
    for row in range(len(grid.cells)):
        for col in range(len(grid.cells[row])):
            if grid.cells[row][col] != Grid.OBSTACLE:
                continue
            x, y, _ = grid_to_world(row, col, cell_size)
            visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=OBSTACLE_COLOR)
            body_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=[x, y, height / 2],
            )
            obstacle_bodies[(row, col)] = body_id
    return obstacle_bodies


def hide_obstacles(obstacle_bodies):
    """Dim every real obstacle to near-invisible -- called once up front in
    sensor mode, before anything has been sensed."""
    for body_id in obstacle_bodies.values():
        p.changeVisualShape(body_id, -1, rgbaColor=HIDDEN_OBSTACLE_COLOR)


def reveal_obstacles(obstacle_bodies, cells):
    """Restore full opacity for `cells` (e.g. lidar.known_obstacles) --
    called after each scan so newly-sensed obstacles turn solid."""
    for cell in cells:
        body_id = obstacle_bodies.get(cell)
        if body_id is not None:
            p.changeVisualShape(body_id, -1, rgbaColor=OBSTACLE_COLOR)


def mark_cell(row, col, color, cell_size=WORLD_CELL_SIZE, height=0.05):
    """A flat marker disc for a start cell -- purely visual, no collision
    shape, so it never interferes with planning or driving."""
    x, y, _ = grid_to_world(row, col, cell_size)
    visual_shape = p.createVisualShape(
        p.GEOM_CYLINDER, radius=cell_size * 0.4, length=height, rgbaColor=color
    )
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape, basePosition=[x, y, height / 2])


def mark_goal_cell(row, col, color, cell_size=WORLD_CELL_SIZE, height=0.05):
    """A flat diamond marker (a box rotated 45 degrees about Z) for a
    goal cell -- deliberately a different shape from mark_cell's disc so
    a start and a goal sitting on the *same* cell (as they do in the
    two-robot swap-sides scenario) are both visible and distinguishable
    rather than one marker silently overwriting the other. Raised
    slightly higher in z than a co-located start disc to avoid z-fighting."""
    x, y, _ = grid_to_world(row, col, cell_size)
    half = cell_size * 0.3
    visual_shape = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[half, half, height / 2], rgbaColor=color
    )
    orientation = p.getQuaternionFromEuler([0, 0, math.pi / 4])
    z = height * 1.5
    return p.createMultiBody(
        baseMass=0, baseVisualShapeIndex=visual_shape, basePosition=[x, y, z], baseOrientation=orientation
    )


def label_cell(row, col, text, color, cell_size=WORLD_CELL_SIZE, height=1.2):
    """Floating text above a cell (e.g. "A start", "B goal") -- the
    unambiguous fallback when shape/color alone might not read clearly at
    a glance, especially with two markers sharing one cell."""
    x, y, _ = grid_to_world(row, col, cell_size)
    return p.addUserDebugText(text, [x, y, height], textColorRGB=color[:3], textSize=1.3)


def remove_debug_items(item_ids):
    """Remove every debug item in `item_ids` (as returned by draw_xy_path /
    draw_path). No-op for an empty/None list, and safe to call in headless
    mode since the ids are already -1 there and PyBullet just ignores the
    removal."""
    for item_id in item_ids or []:
        p.removeUserDebugItem(item_id)


def draw_xy_path(points_xy, color, z=0.05, width=3, gui=True, existing_ids=None):
    """Draw a world-space (x, y) polyline as a sequence of debug line
    segments, returning their ids. Pass the ids from a previous call back
    in as `existing_ids` to erase the old line first -- a replan can
    change the number of segments, so (unlike Hud/FollowLabel) this can't
    just replaceItemUniqueId a fixed set of items in place; it has to tear
    down and rebuild. No-op in DIRECT/headless mode: debug lines are a
    GUI-only visualization aid, not part of the planning or driving logic."""
    if not gui:
        return []
    remove_debug_items(existing_ids)
    ids = []
    for (x1, y1), (x2, y2) in zip(points_xy, points_xy[1:]):
        ids.append(p.addUserDebugLine([x1, y1, z], [x2, y2, z], lineColorRGB=color, lineWidth=width))
    return ids


def draw_path(path_cells, color, cell_size=WORLD_CELL_SIZE, z=0.05, width=3, gui=True, existing_ids=None):
    """Draw a grid-cell path (list of (row, col)) as a debug polyline --
    used to visually compare the binary-obstacle route against the
    cost-map route on the same grid (see pybullet_main.py). See
    draw_xy_path for the `existing_ids` redraw-on-replan contract."""
    points = [grid_to_world(r, c, cell_size)[:2] for r, c in path_cells]
    return draw_xy_path(points, color, z=z, width=width, gui=gui, existing_ids=existing_ids)


def draw_waypoints(waypoints_xy, z=0.05, gui=True):
    if not gui:
        return
    for x, y in waypoints_xy:
        p.addUserDebugLine([x, y, z], [x, y, z + 0.3], lineColorRGB=WAYPOINT_MARKER_COLOR[:3], lineWidth=1)


def draw_cost_map_tint(grid, cell_size=WORLD_CELL_SIZE, max_extra=COST_MAX_EXTRA, z=0.02, gui=True):
    """Flat, semi-transparent quads over every cell with grid.cost > 1.0
    (see Grid.compute_cost_map), colored by magnitude -- the 3D analogue
    of nav/visualizer.py's orange cost tint, drawn as an overlay on the
    ground plane instead of an opaque cell fill since there's real
    geometry underneath it here. Static: call once after the cost map is
    computed, not per frame -- the cost field doesn't change without a
    grid edit, which none of these demos do mid-run."""
    if not gui:
        return []
    half = cell_size / 2
    body_ids = []
    for row in range(len(grid.cells)):
        for col in range(len(grid.cells[row])):
            cost = grid.cost[row][col]
            if cost <= 1.0:
                continue
            t = min(1.0, (cost - 1.0) / max_extra)
            x, y, _ = grid_to_world(row, col, cell_size)
            rgba = (*COST_TINT_COLOR, 0.1 + 0.5 * t)
            visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[half, half, 0.001], rgbaColor=rgba)
            body_ids.append(p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape, basePosition=[x, y, z]))
    return body_ids
