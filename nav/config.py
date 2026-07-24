# Grid dimensions

GRID_SIZE = 25
CELL_SIZE = 28          # Pixels per cell
WINDOW_WIDTH = GRID_SIZE * CELL_SIZE
STATUS_LINE_HEIGHT = 21
STATUS_LINES = 4
STATUS_BAR_HEIGHT = STATUS_LINES * STATUS_LINE_HEIGHT + 12
WINDOW_HEIGHT = WINDOW_WIDTH + STATUS_BAR_HEIGHT


# Cell colors

# Free cell
WHITE = (255, 255, 255)
# Obstacle
BLACK = (30, 30, 30)
# Grid lines
GRAY = (200, 200, 200)
# Start cell
GREEN = (50, 200, 100)
# Goal cell
RED = (220, 60, 60)
# Explored cells, Dijkstra
LIGHT_BLUE = (100, 180, 255)
# Explored cells, A*
LIGHT_PURPLE = (190, 160, 255)
# Path cells
YELLOW = (255, 210, 50)
# Unreachable goal
DARK_RED = (160, 30, 30)
# Moving obstacle cells
ORANGE = (255, 140, 0)
# Robot marker
CYAN = (0, 190, 190)

STATUS_BG = (245, 245, 245)
STATUS_TEXT = (60, 60, 60)

# Moving obstacles + robot animation

OBSTACLE_PERIOD_MS = 700
ROBOT_STEP_MS = 300
REPLAN_FLASH_MS = 700


# RRT

RRT_MAX_ITERS = 5000
RRT_STEP_SIZE = 2.0
RRT_GOAL_SAMPLE_RATE = 0.1
RRT_GOAL_RADIUS = 1.5
# Tree edges/nodes
RRT_TREE_COLOR = (0, 150, 130)


# Cost map (weighted terrain / obstacle inflation, like Nav2's costmap)

COST_INFLUENCE_RADIUS = 3
COST_MAX_EXTRA = 4.0
# Free-cell tint at maximum cost; blends toward WHITE as cost drops to 1.0
COST_TINT = (255, 205, 150)

# Elevation-aware routing (Grid.elevation / Grid.elevation_aware, see
# nav/grid.py: get_neighbors). Extra cost charged per unit of *uphill*
# elevation gain when entering a cell -- climbing costs more, descending
# or staying level doesn't cost any extra over the baseline. Off by
# default (elevation_aware=False), same opt-in pattern cost_map_enabled
# uses.
#
# Why 10.0 and not something closer to COST_MAX_EXTRA's 4.0: a hill tall
# and steep enough to matter but still climbable (a real robot can only
# climb a bounded per-cell grade) has a physical footprint wide enough
# that detouring around it costs roughly 2x its footprint radius -- and
# for *any* climbable hill shape, that radius works out to be roughly
# (peak height / max climbable grade), which puts a hard floor under
# how large this factor has to be before a detour ever beats climbing
# straight over. Empirically (see pybullet_main.py's --elevation demo
# and WRITEUPS.md) that floor is around 6-7 for this project's terrain;
# 10.0 clears it with enough margin that the elevation-aware route goes
# all the way *around* the demo hill rather than merely clipping its
# lower slope -- which matters physically, not just for a bigger
# number: a route that never touches a non-flat cell at all is
# guaranteed exactly as drivable as this project's existing flat-ground
# demos, where one that still climbs partway up a stepped slope turned
# out not to be (see WRITEUPS.md -- r2d2 reliably tipped over combining
# a turn with a climb, regardless of speed).
ELEVATION_COST_FACTOR = 10.0


# Lidar sensor model

LIDAR_RADIUS = 5
# Outline drawn around a real obstacle the robot hasn't sensed yet
HIDDEN_OBSTACLE_OUTLINE = (170, 170, 170)
SENSOR_RING_COLOR = (0, 140, 200)

# Sensor noise -- both LidarSensor (nav/sensor.py) and Lidar3D
# (nav/sim3d/lidar.py) are perfect by default (every real obstacle in
# range is detected, at its exact cell, and nothing else is); passing
# noisy=True to either makes them imperfect in three independent ways,
# each governed by one of these rates:
#   - a real obstacle in range can go undetected this scan (false
#     negative), governed by NOISE_MISS_RATE;
#   - a detected obstacle's reported position can be off by a cell (2D)
#     or a bit of physical distance (3D) instead of exact, governed by
#     NOISE_POSITION_RATE;
#   - a free cell/empty ray can be "detected" as an obstacle that isn't
#     really there (false positive), governed by NOISE_FALSE_POSITIVE_RATE.
# Off by default (both sensors default to noisy=False) so every existing
# caller and test keeps its current, deterministic behavior unchanged.
NOISE_MISS_RATE = 0.15
NOISE_POSITION_RATE = 0.15
NOISE_FALSE_POSITIVE_RATE = 0.02
# How many separate scans have to (noisily) report the same cell before
# a caller that's using confirmed_obstacles() should trust it enough to
# replan on -- see LidarSensor.confirmed_obstacles / Lidar3D.confirmed_obstacles
# and WRITEUPS.md for why a single noisy reading isn't enough on its own.
CONFIRMATION_THRESHOLD = 2
