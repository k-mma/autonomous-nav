import sys
import pygame

from nav.config import (
    GRID_SIZE, CELL_SIZE, WINDOW_WIDTH, WINDOW_HEIGHT, STATUS_BAR_HEIGHT,
    WHITE, BLACK, GRAY, GREEN, RED, LIGHT_BLUE, LIGHT_PURPLE, YELLOW, DARK_RED,
    STATUS_BG, STATUS_TEXT
)
from nav.grid import Grid
from nav.algorithms import dijkstra, astar


# DRAWING


def draw_grid(screen, grid, explored, path, active_algo):
    if path:
        path_set = set(path)
    else:
        path_set = set()
    if active_algo == "astar":
        explored_color = LIGHT_PURPLE
    else:
        explored_color = LIGHT_BLUE
    
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
                color = explored_color
            elif cell == Grid.OBSTACLE:
                color = BLACK
            else:
                color = WHITE

            rect = pygame.Rect(col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, GRAY, rect, 1)


def draw_status(screen, font, lines):
    bar_rect = pygame.Rect(0, WINDOW_WIDTH, WINDOW_WIDTH, STATUS_BAR_HEIGHT)
    pygame.draw.rect(screen, STATUS_BG, bar_rect)
    for i, line in enumerate(lines):
        text_surf = font.render(line, True, STATUS_TEXT)
        screen.blit(text_surf, (10, WINDOW_WIDTH + 6 + i * 21))


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
    font = pygame.font.SysFont("monospace", 12)

    grid = Grid()

    # Reset state w/ grid change
    # Controls algorithm running and color of explored cells
    active_algo = "dijkstra"
    explored = set()
    path = None
    # T/F for Dijkstra running since grid change
    ran = False
    # Most recent output from both algorithms
    last_results = {}

    def reset_algorithm():
        nonlocal explored, path, ran, last_results
        explored = set()
        path = None
        ran = False
        last_results = {}

    def run_active():
        nonlocal explored, path, ran
        if grid.start is None or grid.goal is None:
            return
        if active_algo == "dijkstra":
            algo = dijkstra
        else:
            algo = astar
        path, explored, _ = algo(grid, grid.start, grid.goal)
        last_results[active_algo] = (path, explored)
        ran = True
    
    def switch_algo(name):
        nonlocal active_algo, explored, path, ran
        active_algo = name
        if name in last_results:
            path, explored = last_results[name]
            ran = True
        else:
            explored = set()
            path = None
            ran = False

    CONTROLS_1 = "D: Dijkstra  A: A*  |  Space: run  |  C: clear"
    CONTROLS_2 = "LClick: obstacle  |  RClick: start  |  Shift+RClick: goal"

    def status_lines():
        if active_algo == "dijkstra":
            algo_label = "Dijkstra"
        else:
            algo_label = "A*"
        
        if grid.start is None and grid.goal is None:
            return f"Place a start (RClick) and goal (Shift+RClick) to begin."
        elif not ran:
            line_1 = f"[{algo_label}] ready. Press space to run."
        elif path is None:
            line_1 = f"[{algo_label}] No path found. Cells explored: {len(explored)}"
        else:
            line_1 = (
                f"[{algo_label}] Path: {len(path) - 1} steps | "
                f"Cells explored: {len(explored)}"
            )
        
        if "dijkstra" in last_results and "astar" in last_results:
            d_explored = last_results["dijkstra"][1]
            a_explored = last_results["astar"][1]
            diff = len(d_explored) - len(a_explored)
            if diff > 0:
                sign = "fewer"
            else:
                sign = "more"
            line_2 = (
                f"Dijkstra: {len(d_explored)} cells | "
                f"A*: {len(a_explored)} cells | "
                f"A* explored {abs(diff)} {sign}"
            )
        else:
            line_2 = CONTROLS_1
        
        return [line_1, line_2, CONTROLS_2]

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
                    run_active()
                elif event.key == pygame.K_d:
                    switch_algo("dijkstra")
                elif event.key == pygame.K_a:
                    switch_algo("astar")
                elif event.key == pygame.K_c:
                    grid.clear()
                    reset_algorithm()

        screen.fill(WHITE)
        draw_grid(screen, grid, explored, path, active_algo)
        draw_status(screen, font, status_lines())
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()