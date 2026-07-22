import pybullet as p
import pybullet_data

from nav.grid import Grid
from nav.sim3d.coords import grid_to_world, WORLD_CELL_SIZE

OBSTACLE_HEIGHT = 1.0
OBSTACLE_COLOR = (0.15, 0.15, 0.15, 1.0)
BINARY_PATH_COLOR = (0.9, 0.15, 0.15)
COST_MAP_PATH_COLOR = (0.1, 0.5, 0.95)
WAYPOINT_MARKER_COLOR = (1.0, 0.85, 0.1, 1.0)
START_COLOR = (0.2, 0.8, 0.4, 1.0)
GOAL_COLOR = (0.9, 0.25, 0.25, 1.0)
ROBOT_A_COLOR = (0.1, 0.7, 0.9, 1.0)
ROBOT_B_COLOR = (0.95, 0.55, 0.1, 1.0)


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
    pixels into meters instead of redesigning the map."""
    half_extents = [cell_size / 2, cell_size / 2, height / 2]
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=OBSTACLE_COLOR)

    body_ids = []
    for row in range(len(grid.cells)):
        for col in range(len(grid.cells[row])):
            if grid.cells[row][col] != Grid.OBSTACLE:
                continue
            x, y, _ = grid_to_world(row, col, cell_size)
            body_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=[x, y, height / 2],
            )
            body_ids.append(body_id)
    return body_ids


def mark_cell(row, col, color, cell_size=WORLD_CELL_SIZE, height=0.05):
    """A flat marker disc for a start/goal cell -- purely visual, no
    collision shape, so it never interferes with planning or driving."""
    x, y, _ = grid_to_world(row, col, cell_size)
    visual_shape = p.createVisualShape(
        p.GEOM_CYLINDER, radius=cell_size * 0.4, length=height, rgbaColor=color
    )
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape, basePosition=[x, y, height / 2])


def draw_path(path_cells, color, cell_size=WORLD_CELL_SIZE, z=0.05, width=3, gui=True):
    """Draw a grid-cell path (list of (row, col)) as a debug polyline --
    used to visually compare the binary-obstacle route against the
    cost-map route on the same grid (see pybullet_main.py). No-op in
    DIRECT/headless mode: debug lines are a GUI-only visualization aid,
    not part of the planning or driving logic."""
    if not gui:
        return
    for (r1, c1), (r2, c2) in zip(path_cells, path_cells[1:]):
        x1, y1, _ = grid_to_world(r1, c1, cell_size)
        x2, y2, _ = grid_to_world(r2, c2, cell_size)
        p.addUserDebugLine([x1, y1, z], [x2, y2, z], lineColorRGB=color, lineWidth=width)


def draw_waypoints(waypoints_xy, z=0.05, gui=True):
    if not gui:
        return
    for x, y in waypoints_xy:
        p.addUserDebugLine([x, y, z], [x, y, z + 0.3], lineColorRGB=WAYPOINT_MARKER_COLOR[:3], lineWidth=1)
