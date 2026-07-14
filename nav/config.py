# Grid dimensions

GRID_SIZE = 25
CELL_SIZE = 28          # Pixels per cell
WINDOW_WIDTH = GRID_SIZE * CELL_SIZE
STATUS_BAR_HEIGHT = 72
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
