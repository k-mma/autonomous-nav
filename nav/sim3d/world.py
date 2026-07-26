import math

import pybullet as p
import pybullet_data

from nav.config import COST_MAX_EXTRA, TERRAIN_COLORS, TERRAIN_GRASS
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
# Darker/more saturated than a first attempt at this color (0.1, 0.5,
# 0.95) -- plane.urdf's own ground is pale blue/white, and that lighter
# blue was close enough in both hue *and* lightness to blend into it at
# a glance. This one reads as clearly "blue" without competing with the
# tiles underneath it.
COST_MAP_PATH_COLOR = (0.05, 0.15, 0.85)
SENSOR_PATH_COLOR = (0.2, 0.75, 0.35)
# The route that ignores terrain cost (red, same "naive baseline" role
# BINARY_PATH_COLOR plays for the cost-map comparison) vs the one that
# charges for crossing mud/water (green, distinguishable from
# COST_MAP_PATH_COLOR's blue since a demo could in principle show both
# comparisons at once).
TERRAIN_NAIVE_PATH_COLOR = (0.9, 0.15, 0.15)
TERRAIN_AWARE_PATH_COLOR = (0.15, 0.85, 0.35)
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
# A path redraw used to be instantaneous: the old line vanished and the
# new one appeared in the same frame, easy to miss watching live and
# invisible entirely in a single screenshot. FLASH_COLOR is what a freshly
# (re)drawn path renders in for PATH_FLASH_SECONDS before settling into
# its real color (see LivePath below) -- bright and shared across every
# demo specifically so "this just changed" reads as one consistent visual
# signal regardless of which path/robot/color it's attached to.
#
# Hot magenta, not white: plane.urdf's default ground is itself pale
# blue/white, so a white flash was landing almost exactly on top of the
# lightest tiles and the sun-glare highlight -- close to zero contrast
# right where it needed the *most* contrast. Magenta doesn't occur
# anywhere else in this file's palette (ground, obstacles, or any path/
# robot color), so it can't blend into anything by coincidence.
FLASH_COLOR = (1.0, 0.0, 0.85)
PATH_FLASH_SECONDS = 0.4
# How long a *superseded* path keeps rendering before being removed --
# long enough that a viewer (live, or scrubbing a video) can actually
# compare "here's what it was" against the new one flashing in next to
# it, instead of the change happening in one blink-and-you-miss-it frame.
# It doesn't linger in its *old* color/width, though (see LivePath) --
# it gets redrawn dimmer and thinner the moment it's superseded, so
# "old, fading out" and "new, bold and flashing" read as two different
# things at a glance instead of two identical-looking lines.
PATH_LINGER_SECONDS = 1.2
# Dark charcoal, not mid-gray -- (0.55, 0.55, 0.55) turned out to still
# have weak contrast against plane.urdf's pale blue/white tiles (a mid
# gray is roughly as light as the lighter tiles themselves). The obstacle
# block's near-black already reads clearly against this same ground in
# every screenshot, which is the actual benchmark this needs to clear.
LINGER_COLOR = (0.2, 0.2, 0.2)
# Path thickness, in world-space meters (a fraction of WORLD_CELL_SIZE),
# not pixels. Paths used to be drawn with addUserDebugLine, whose
# lineWidth is a screen-space pixel count -- pushing that as high as 200
# was tried and confirmed, empirically, to make no visible difference:
# on this Mac's Metal-translated GL driver, debug lines rasterize at a
# fixed ~1px regardless of the requested lineWidth (a driver-side
# core-profile glLineWidth cap, not a PyBullet bug), AND -- the more
# load-bearing finding -- addUserDebugLine items never show up in
# getCameraImage() at all, in either DIRECT or GUI mode (verified with a
# lineWidth=200 line producing zero matching pixels in the rendered
# image). Since every screenshot in this project is captured via
# getCameraImage, no lineWidth value could ever have worked. Paths are
# now real GEOM_BOX geometry (see draw_xy_path) sized in world units
# instead, which solves both problems at once: genuinely tile-width, and
# actually visible to a synthetic camera.
PATH_WIDTH = 0.55
PATH_FLASH_WIDTH = 0.75
PATH_LINGER_WIDTH = 0.4
# Vertical extent of the path mesh -- thin enough to read as a flat
# ribbon on the ground rather than a wall, thick enough to catch light
# and cast a visible edge at the demo's oblique camera pitch (-55).
PATH_HEIGHT = 0.05
# Color for draw_trigger_marker's short-lived "X" -- calls out *why* a
# redraw just happened (the cell a sensor discovery or another robot's
# position triggered it from), distinct from FLASH_COLOR so the two don't
# read as the same signal.
TRIGGER_MARKER_COLOR = (1.0, 0.8, 0.0)


def connect(gui=True):
    """Open a PyBullet connection and load the ground plane. This is the
    only new "physics interface" code the 3D port needed -- the grid
    model and A* itself (nav/grid.py, nav/algorithms.py) are unchanged
    from pygame, imported and reused as-is."""
    p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.resetDebugVisualizerCamera(
        cameraDistance=22, cameraYaw=45, cameraPitch=-55, cameraTargetPosition=[12, 12, 0]
    )
    return p.loadURDF("plane.urdf")


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
    """Remove every body in `item_ids` (as returned by draw_xy_path /
    draw_path -- real GEOM_BOX multibodies, not PyBullet "debug items"
    despite the name kept here for callers). No-op for an empty/None
    list, and safe to call in headless mode since draw_xy_path never
    creates any bodies there in the first place."""
    for item_id in item_ids or []:
        p.removeBody(item_id)


def _path_segment_body(x1, y1, x2, y2, rgba, z, width, height):
    """One flat GEOM_BOX spanning (x1, y1) -> (x2, y2), oriented so its
    long axis follows the segment and its short axis is `width` wide --
    the mesh analogue of a single addUserDebugLine call, sized in world
    units instead of screen-space pixels (see PATH_WIDTH's comment for
    why: debug lines never rasterize into getCameraImage at any pixel
    width, so a real box is the only way for a path to actually show up
    in a screenshot). Returns None for a zero-length segment (two
    identical consecutive points) rather than creating a degenerate,
    zero-extent box."""
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return None
    yaw = math.atan2(dy, dx)
    half_extents = [length / 2, width / 2, height / 2]
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba)
    return p.createMultiBody(
        baseMass=0, baseVisualShapeIndex=visual_shape,
        basePosition=[(x1 + x2) / 2, (y1 + y2) / 2, z],
        baseOrientation=p.getQuaternionFromEuler([0, 0, yaw]),
    )


def _path_joint_body(x, y, rgba, z, width, height):
    """A small square GEOM_BOX centered on one interior waypoint --
    fills the gap two angled _path_segment_body boxes would otherwise
    leave at a turn (their corners don't meet flush the way two
    infinitely-thin lines would), so a path with direction changes
    (every grid diagonal, every spline sample) still reads as one
    continuous ribbon instead of a dashed one."""
    half_extents = [width / 2, width / 2, height / 2]
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba)
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape, basePosition=[x, y, z])


def draw_xy_path(points_xy, color, z=0.05, width=PATH_WIDTH, gui=True, existing_ids=None, height=PATH_HEIGHT):
    """Draw a world-space (x, y) polyline as a sequence of flat mesh
    boxes (see _path_segment_body/_path_joint_body), returning their
    body ids. Pass the ids from a previous call back in as `existing_ids`
    to erase the old path first -- a replan can change the number of
    segments, so (unlike Hud/FollowLabel) this can't just
    replaceItemUniqueId a fixed set of items in place; it has to tear
    down and rebuild. No-op in DIRECT/headless mode: a driven path's
    visualization is a GUI-only aid, not part of the planning or driving
    logic, same as every other draw_* helper in this file."""
    if not gui:
        return []
    remove_debug_items(existing_ids)
    rgba = (*color[:3], 1.0)
    ids = []
    for (x1, y1), (x2, y2) in zip(points_xy, points_xy[1:]):
        body_id = _path_segment_body(x1, y1, x2, y2, rgba, z, width, height)
        if body_id is not None:
            ids.append(body_id)
    for x, y in points_xy[1:-1]:
        ids.append(_path_joint_body(x, y, rgba, z, width, height))
    return ids


def draw_path(path_cells, color, cell_size=WORLD_CELL_SIZE, z=0.05, width=PATH_WIDTH, gui=True, existing_ids=None):
    """Draw a grid-cell path (list of (row, col)) as a mesh polyline --
    used to visually compare the binary-obstacle route against the
    cost-map route on the same grid (see pybullet_main.py). See
    draw_xy_path for the `existing_ids` redraw-on-replan contract."""
    points = [grid_to_world(r, c, cell_size)[:2] for r, c in path_cells]
    return draw_xy_path(points, color, z=z, width=width, gui=gui, existing_ids=existing_ids)


class LivePath:
    """A path debug-line whose redraws are actually visible to someone
    watching, instead of an instant, easy-to-miss swap.

    Call `set_path(points_xy, sim_step)` whenever the path actually
    changes (a real replan -- not every frame) and `tick(sim_step)`
    every frame (or every few frames; see the callers' HUD-update
    cadence, which is plenty precise for this). Two things happen on a
    change:

    - The newly drawn path renders in FLASH_COLOR, at PATH_FLASH_WIDTH
      (thicker than its steady width, not just a different color), for
      PATH_FLASH_SECONDS, then `tick` transitions it to its real `color`
      at `width` -- a redraw now visibly *pops*, rather than silently
      having always looked the way it does.
    - The path it replaced doesn't just disappear -- the instant it's
      superseded it gets redrawn in LINGER_COLOR (a neutral gray) at the
      thinner PATH_LINGER_WIDTH, so it reads as "the old route" rather
      than looking like a second, equally-current path, and keeps
      rendering in that dimmed style for PATH_LINGER_SECONDS before
      `tick` removes it -- long enough that old and new are both legible
      on screen at once, long enough to actually compare them.

    Only one lingering path is ever kept at a time (a new change
    discards whatever was still lingering from before) -- under a fast
    replan cadence (pybullet_multi_robot_main.py's is 0.05s) this
    self-limits to "current path + at most one recent previous one"
    rather than an ever-growing trail, and the active path itself can
    end up re-flashing before it ever settles. That's read as a feature,
    not a bug: rapid flashing *is* an accurate picture of an area with a
    lot of active replanning (e.g. two robots contesting a crossing),
    and it settles to a steady color the moment replans actually stop.
    """

    def __init__(self, color, gui, sim_hz, z=0.05, width=PATH_WIDTH,
                 flash_color=FLASH_COLOR, flash_width=PATH_FLASH_WIDTH, flash_seconds=PATH_FLASH_SECONDS,
                 linger_color=LINGER_COLOR, linger_width=PATH_LINGER_WIDTH, linger_seconds=PATH_LINGER_SECONDS):
        self.color = color
        self.gui = gui
        self.z = z
        self.width = width
        self.flash_color = flash_color
        self.flash_width = flash_width
        self.flash_steps = max(1, int(flash_seconds * sim_hz))
        self.linger_color = linger_color
        self.linger_width = linger_width
        self.linger_steps = max(1, int(linger_seconds * sim_hz))

        self.points = []
        self.active_ids = []
        self.flash_until_step = -1
        self.settled = True

        self.lingering_ids = []
        self.lingering_remove_step = -1

    def set_path(self, points_xy, sim_step):
        """Make `points_xy` the active path, effective now. The
        previously active path (if any, and if actually different)
        becomes the lingering one -- redrawn dimmer and thinner, not
        just relabeled -- instead of being removed outright."""
        points_xy = list(points_xy)
        if points_xy == self.points:
            return
        if self.active_ids:
            remove_debug_items(self.lingering_ids)
            self.lingering_ids = draw_xy_path(
                self.points, self.linger_color, z=self.z, width=self.linger_width,
                gui=self.gui, existing_ids=self.active_ids
            )
            self.lingering_remove_step = sim_step + self.linger_steps

        self.points = points_xy
        self.active_ids = draw_xy_path(points_xy, self.flash_color, z=self.z, width=self.flash_width, gui=self.gui)
        self.flash_until_step = sim_step + self.flash_steps
        self.settled = False

    def clear(self, sim_step):
        """No active path right now (e.g. a replan found nothing) -- the
        current active path lingers and fades exactly as it would if
        replaced by a real new one."""
        self.set_path([], sim_step)

    def tick(self, sim_step):
        """Advance the flash->settle and linger->removal timers. Cheap
        and safe to call every frame; callers use their existing
        HUD-update cadence (~10Hz) since that's plenty of resolution for
        multi-tenths-of-a-second windows."""
        if not self.gui:
            return
        if not self.settled and sim_step >= self.flash_until_step:
            self.active_ids = draw_xy_path(
                self.points, self.color, z=self.z, width=self.width, gui=self.gui, existing_ids=self.active_ids
            )
            self.settled = True
        if self.lingering_ids and sim_step >= self.lingering_remove_step:
            remove_debug_items(self.lingering_ids)
            self.lingering_ids = []


def draw_trigger_marker(row, col, cell_size=WORLD_CELL_SIZE, color=TRIGGER_MARKER_COLOR,
                         z=0.2, size=0.9, width=16, lifetime=1.2, gui=True):
    """A short-lived 'X' over a grid cell -- calls out *why* a redraw
    just happened (a newly-sensed obstacle, or another robot's current
    cell) instead of leaving a viewer to infer it from the path alone.
    Uses PyBullet's own `lifeTime` (unlike LivePath's lingering path,
    this never needs to be replaced or extended, so there's no reason
    not to let PyBullet auto-remove it instead of tracking it by hand)."""
    if not gui:
        return
    x, y, _ = grid_to_world(row, col, cell_size)
    half = size / 2
    p.addUserDebugLine([x - half, y - half, z], [x + half, y + half, z],
                        lineColorRGB=color, lineWidth=width, lifeTime=lifetime)
    p.addUserDebugLine([x - half, y + half, z], [x + half, y - half, z],
                        lineColorRGB=color, lineWidth=width, lifeTime=lifetime)


def draw_waypoints(waypoints_xy, z=0.05, gui=True):
    if not gui:
        return
    for x, y in waypoints_xy:
        p.addUserDebugLine([x, y, z], [x, y, z + 0.3], lineColorRGB=WAYPOINT_MARKER_COLOR[:3], lineWidth=1)


def draw_terrain(grid, cell_size=WORLD_CELL_SIZE, z=0.04, base_z=0.002, gui=True):
    """One big grass-colored quad covering the whole grid footprint,
    plus one opaque quad per cell whose terrain isn't the default
    TERRAIN_GRASS layered on top of it (see nav/config.py:
    TERRAIN_COLORS/TERRAIN_COST) -- the flat-ground analogue of
    build_terrain's old stepped height columns, and the same "colored
    overlay on top of real geometry" approach draw_cost_map_tint already
    uses for obstacle-inflation cost, just opaque and keyed by terrain
    type instead of translucent and keyed by cost magnitude.

    The base quad is what makes this "the whole grid is terrain" rather
    than "one patch of terrain floating on the checkered ground plane" --
    an earlier version skipped grass cells individually (grass being the
    cost-1.0 default, same as bare ground, so a same-colored quad seemed
    redundant), but that left plane.urdf's own checker pattern visible
    everywhere except the painted patch, which reads as "one square of
    terrain," not "a terrain demo." One large quad covers the same area
    far more cheaply than 625 individual grass quads would, and avoids
    seams between adjacent same-colored cells entirely.

    z/base_z default to a 0.038 gap, not just-barely-above -- a first
    pass at z=0.005/base_z=0.003 (a bare few mm apart) turned out to
    z-fight visibly at this project's usual camera distance (22, see
    connect()): not simple color-flicker z-fighting but shadow-map
    self-shadowing (banded, mirror-like artifacts), which needed a much
    larger gap to clear than plain co-planar color z-fighting would have
    (confirmed empirically -- a few mm still showed banding, ~3cm+ was
    clean). z (0.04) is set to clear that same threshold above base_z
    (0.002) while still sitting below every caller's path z (see
    pybullet_main.py's run_terrain_demo, which keeps its own paths at
    >= 0.09 for the same reason -- terrain, cost tint, and path all need
    that ~3cm+ separation from *each other*, not just a nonzero one, to
    stay clean at this camera distance). Static, like draw_cost_map_tint:
    call once after the terrain is painted, not per frame."""
    if not gui:
        return []
    size = len(grid.cells)
    half = cell_size / 2
    body_ids = []

    base_r, base_g, base_b = TERRAIN_COLORS[TERRAIN_GRASS]
    base_extent = size * cell_size / 2
    base_center = (size - 1) * cell_size / 2
    base_visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[base_extent, base_extent, 0.001],
        rgbaColor=(base_r / 255, base_g / 255, base_b / 255, 1.0),
    )
    body_ids.append(p.createMultiBody(
        baseMass=0, baseVisualShapeIndex=base_visual, basePosition=[base_center, base_center, base_z]
    ))

    for row in range(size):
        for col in range(size):
            if grid.cells[row][col] == Grid.OBSTACLE:
                continue
            terrain_type = grid.terrain[row][col]
            if terrain_type == TERRAIN_GRASS:
                continue
            r, g, b = TERRAIN_COLORS[terrain_type]
            x, y, _ = grid_to_world(row, col, cell_size)
            visual_shape = p.createVisualShape(
                p.GEOM_BOX, halfExtents=[half, half, 0.001], rgbaColor=(r / 255, g / 255, b / 255, 1.0)
            )
            body_ids.append(p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape, basePosition=[x, y, z]))
    return body_ids


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
