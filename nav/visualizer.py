import sys
import pygame

from nav.config import (
    GRID_SIZE, CELL_SIZE, WINDOW_WIDTH, WINDOW_HEIGHT, STATUS_BAR_HEIGHT,
    WHITE, BLACK, GRAY, GREEN, RED, LIGHT_BLUE, YELLOW, DARK_RED,
    STATUS_BG, STATUS_TEXT,
)
from nav.grid import Grid
from nav.algorithms import dijkstra


# DRAWING


def draw_grid(screen, grid, explored, path):
    if path:
        path_set = set(path)
    else:
        path_set = set()
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            cell = grid.cells[row][col]
            pos = (row, col)
            
            if cell == Grid.START:
                color = GREEN
            elif cell == Grid.GOAL:
                if path is None and explored:
                    color = DARK_RED
                else:
                    color = RED
            elif pos in path_set:
                color = YELLOW
            elif pos in explored:
                color = LIGHT_BLUE
            elif cell == Grid.OBSTACLE:
                color = BLACK
            else:
                color = WHITE

            rect = pygame.Rect(col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, GRAY, rect, 1)


def draw_status(screen, font, message):
    bar_rect = pygame.Rect(0, WINDOW_WIDTH, WINDOW_WIDTH, STATUS_BAR_HEIGHT)
    pygame.draw.rect(screen, STATUS_BG, bar_rect)
    lines = message.split("\n")
    for i, line in enumerate(lines):
        text_surf = font.render(line, True, STATUS_TEXT)
        screen.blit(text_surf, (10, WINDOW_WIDTH + 6 + i * 18))


# INPUT


def pixel_to_cell(pos):
    x, y = pos
    return y // CELL_SIZE, x // CELL_SIZE


# MAIN LOOP


def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Autonomous Nav")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 13)

    grid = Grid()

    # Reset state w/ grid change
    explored = set()
    path = None
    # T/F for Dijkstra running since grid change
    ran = False

    def reset_algorithm():
        nonlocal explored, path, ran
        explored = set()
        path = None
        ran = False

    def run_dijkstra():
        nonlocal explored, path, ran
        if grid.start is None or grid.goal is None:
            return
        path, explored, _ = dijkstra(grid, grid.start, grid.goal)
        ran = True

    CONTROLS = "LClick: obstacle  |  RClick: start  |  Shift+RClick: goal  |  Space: run  |  C: clear"

    def status_message():
        if grid.start is None and grid.goal is None:
            return f"Place a start and goal to begin.\n{CONTROLS}"
        if grid.start is None:
            return f"Right-click to place start.\n{CONTROLS}"
        if grid.goal is None:
            return f"Shift+right-click to place goal.\n{CONTROLS}"
        if not ran:
            return f"Start and goal set — press Space to run Dijkstra.\n{CONTROLS}"
        if path is None:
            return f"No path found — goal is unreachable.  Cells explored: {len(explored)}\n{CONTROLS}"
        return f"Path length: {len(path) - 1} steps  |  Cells explored: {len(explored)}\n{CONTROLS}"

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                row, col = pixel_to_cell(event.pos)
                if not grid.is_valid(row, col):
                    continue
                    
                if event.button == 1:
                    grid.toggle_obstacle(row, col)
                    reset_algorithm()
                elif event.button == 3:
                    mods = pygame.key.get_mods()
                    if mods & pygame.KMOD_SHIFT:
                        grid.place_goal(row, col)
                    else:
                        grid.place_start(row, col)
                    reset_algorithm()
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    run_dijkstra()
                elif event.key == pygame.K_c:
                    grid.clear()
                    reset_algorithm()

        screen.fill(WHITE)
        draw_grid(screen, grid, explored, path)
        draw_status(screen, font, status_message())
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()