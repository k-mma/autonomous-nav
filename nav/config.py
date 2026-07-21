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


# Lidar sensor model

LIDAR_RADIUS = 5
# Outline drawn around a real obstacle the robot hasn't sensed yet
HIDDEN_OBSTACLE_OUTLINE = (170, 170, 170)
SENSOR_RING_COLOR = (0, 140, 200)
