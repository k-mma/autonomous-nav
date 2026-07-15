import sys
import pygame

from nav.config import (
    GRID_SIZE, CELL_SIZE, WINDOW_WIDTH, WINDOW_HEIGHT, STATUS_BAR_HEIGHT, STATUS_LINE_HEIGHT,
    WHITE, BLACK, GRAY, GREEN, RED, LIGHT_BLUE, LIGHT_PURPLE, YELLOW, DARK_RED,
    ORANGE, CYAN, STATUS_BG, STATUS_TEXT,
    OBSTACLE_PERIOD_MS, ROBOT_STEP_MS, REPLAN_FLASH_MS
)
from nav.grid import Grid
from nav.algorithms import find_path, path_cost
from nav.heuristics import euclidean, manhattan, octile, scaled
from nav.maze import generate_maze
from nav.obstacles import MovingObstacle, find_free_neighbor

# Cycled through with the H key while A* is active
HEURISTICS = [
    ("manhattan", manhattan),
    ("euclidean", euclidean),
    ("octile", octile),
    ("inadmissible (1.5x manhattan)", scaled(manhattan, 1.5)),
]


# DRAWING


def draw_grid(screen, grid, explored, path, active_algo, reason, moving_cells):
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
                color = DARK_RED if reason == "start_blocked" else GREEN
            elif cell == Grid.GOAL:
                if reason in ("goal_blocked", "no_path"):
                    color = DARK_RED
                else:
                    color = RED
            elif pos in path_set:
                color = YELLOW
            elif pos in explored:
                color = explored_color
            elif cell == Grid.OBSTACLE:
                color = ORANGE if pos in moving_cells else BLACK
            else:
                color = WHITE

            rect = pygame.Rect(col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, GRAY, rect, 1)


def draw_robot(screen, robot_pos):
    if robot_pos is None:
        return
    row, col = robot_pos
    center = (col * CELL_SIZE + CELL_SIZE // 2, row * CELL_SIZE + CELL_SIZE // 2)
    pygame.draw.circle(screen, CYAN, center, CELL_SIZE // 3)


def draw_status(screen, font, lines):
    bar_rect = pygame.Rect(0, WINDOW_WIDTH, WINDOW_WIDTH, STATUS_BAR_HEIGHT)
    pygame.draw.rect(screen, STATUS_BG, bar_rect)
    for i, line in enumerate(lines):
        text_surf = font.render(line, True, STATUS_TEXT)
        screen.blit(text_surf, (10, WINDOW_WIDTH + 6 + i * STATUS_LINE_HEIGHT))


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
    # Why the last run didn't produce a normal path, or None
    reason = None
    # T/F for Dijkstra running since grid change
    ran = False
    # Most recent output from both algorithms
    last_results = {}

    # Which heuristic astar uses (irrelevant to dijkstra)
    heuristic_idx = 0

    # Obstacles that bounce between two cells
    moving_obstacles = []
    # Robot animation along the current path
    robot_running = False
    robot_pos = None
    robot_index = 0
    robot_blocked = False
    next_robot_step = 0
    replan_flash_until = 0

    def reset_algorithm():
        nonlocal explored, path, ran, last_results, reason
        explored = set()
        path = None
        reason = None
        ran = False
        last_results = {}

    def stop_robot():
        nonlocal robot_running, robot_pos, robot_index, robot_blocked
        robot_running = False
        robot_pos = None
        robot_index = 0
        robot_blocked = False

    def start_robot():
        nonlocal robot_running, robot_pos, robot_index, robot_blocked, next_robot_step
        if path is None or grid.start is None or grid.goal is None:
            return
        robot_running = True
        robot_pos = path[0]
        robot_index = 0
        robot_blocked = False
        next_robot_step = pygame.time.get_ticks() + ROBOT_STEP_MS

    def toggle_robot():
        if robot_running:
            stop_robot()
        else:
            start_robot()

    def toggle_moving_obstacle(row, col):
        cell = (row, col)
        for obs in moving_obstacles:
            if cell in (obs.cell_a, obs.cell_b):
                for r, c in (obs.cell_a, obs.cell_b):
                    if grid.cells[r][c] == Grid.OBSTACLE:
                        grid.cells[r][c] = Grid.FREE
                moving_obstacles.remove(obs)
                reset_algorithm()
                stop_robot()
                return

        if not grid.is_free(row, col):
            return
        neighbor = find_free_neighbor(grid, cell)
        if neighbor is None:
            return
        obstacle = MovingObstacle(cell, neighbor, period_ms=OBSTACLE_PERIOD_MS)
        obstacle.start(pygame.time.get_ticks())
        moving_obstacles.append(obstacle)
        reset_algorithm()
        stop_robot()

    def is_moving_obstacle_cell(row, col):
        cell = (row, col)
        return any(cell in (obs.cell_a, obs.cell_b) for obs in moving_obstacles)

    def current_heuristic():
        return HEURISTICS[heuristic_idx][1] if active_algo == "astar" else None

    def cycle_heuristic():
        nonlocal heuristic_idx
        heuristic_idx = (heuristic_idx + 1) % len(HEURISTICS)
        if "astar" in last_results:
            del last_results["astar"]
        if active_algo == "astar":
            reset_algorithm()

    def toggle_diagonal():
        grid.diagonal = not grid.diagonal
        reset_algorithm()
        stop_robot()

    def generate_new_maze():
        generate_maze(grid)
        moving_obstacles.clear()
        reset_algorithm()
        stop_robot()

    def run_active():
        nonlocal explored, path, ran, reason
        if grid.start is None or grid.goal is None:
            return
        path, explored, reason = find_path(grid, active_algo, grid.start, grid.goal, current_heuristic())
        last_results[active_algo] = (path, explored, reason)
        ran = True

    def switch_algo(name):
        nonlocal active_algo, explored, path, ran, reason
        active_algo = name
        if name in last_results:
            path, explored, reason = last_results[name]
            ran = True
        else:
            explored = set()
            path = None
            reason = None
            ran = False

    def replan_from_robot():
        nonlocal path, explored, reason, robot_index, robot_blocked, replan_flash_until
        new_path, new_explored, new_reason = find_path(
            grid, active_algo, robot_pos, grid.goal, current_heuristic()
        )
        explored = new_explored
        reason = new_reason
        if new_path is None:
            robot_blocked = True
        else:
            path = new_path
            robot_index = 0
            robot_blocked = False
            last_results[active_algo] = (path, explored, reason)
        replan_flash_until = pygame.time.get_ticks() + REPLAN_FLASH_MS

    CONTROLS_1 = "D: Dijkstra  A: A*  |  Space: run  |  C: clear  |  R: robot"
    CONTROLS_2 = "LClick: obstacle (Shift: moving)  |  RClick: start (Shift: goal)"
    CONTROLS_3 = "H: heuristic  |  X: diagonal move  |  M: new maze"

    def status_lines():
        if active_algo == "dijkstra":
            algo_label = "Dijkstra"
        else:
            algo_label = f"A* ({HEURISTICS[heuristic_idx][0]})"

        if grid.start is None and grid.goal is None:
            return [f"Place a start (RClick) and goal (Shift+RClick) to begin."]
        elif pygame.time.get_ticks() < replan_flash_until:
            line_1 = "REPLANNING..."
        elif robot_blocked:
            line_1 = "Robot blocked -- no path. Waiting for a route to open..."
        elif robot_pos is not None and not robot_running and robot_pos == grid.goal:
            line_1 = "Robot arrived at goal!"
        elif not ran:
            line_1 = f"[{algo_label}] ready. Press space to run."
        elif reason == "same_cell":
            line_1 = f"[{algo_label}] Start and goal are the same cell."
        elif reason == "start_blocked":
            line_1 = f"[{algo_label}] Start is blocked by an obstacle."
        elif reason == "goal_blocked":
            line_1 = f"[{algo_label}] Goal is blocked by an obstacle."
        elif reason == "no_path":
            line_1 = f"[{algo_label}] No path found. Cells explored: {len(explored)}"
        else:
            distance = f"cost {path_cost(path):.2f}" if grid.diagonal else f"{len(path) - 1} steps"
            line_1 = f"[{algo_label}] Path: {distance} | Cells explored: {len(explored)}"

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

        return [line_1, line_2, CONTROLS_2, CONTROLS_3]

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
                    mods = pygame.key.get_mods()
                    if mods & pygame.KMOD_SHIFT:
                        toggle_moving_obstacle(row, col)
                    elif not is_moving_obstacle_cell(row, col):
                        grid.toggle_obstacle(row, col)
                        reset_algorithm()
                        stop_robot()

                elif event.button == 3:
                    mods = pygame.key.get_mods()
                    if mods & pygame.KMOD_SHIFT:
                        grid.place_goal(row, col)
                    else:
                        grid.place_start(row, col)
                    reset_algorithm()
                    stop_robot()

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    run_active()
                elif event.key == pygame.K_d:
                    switch_algo("dijkstra")
                elif event.key == pygame.K_a:
                    switch_algo("astar")
                elif event.key == pygame.K_r:
                    toggle_robot()
                elif event.key == pygame.K_h:
                    cycle_heuristic()
                elif event.key == pygame.K_x:
                    toggle_diagonal()
                elif event.key == pygame.K_m:
                    generate_new_maze()
                elif event.key == pygame.K_c:
                    grid.clear()
                    moving_obstacles.clear()
                    reset_algorithm()
                    stop_robot()

        now = pygame.time.get_ticks()
        moved_cells = [obs.position for obs in moving_obstacles if obs.tick(grid, now)]

        if robot_running and moved_cells:
            remaining = set(path[robot_index:]) if path else set()
            if robot_blocked or robot_pos in moved_cells or remaining & set(moved_cells):
                replan_from_robot()

        if robot_running and not robot_blocked and path and now >= next_robot_step:
            next_robot_step = now + ROBOT_STEP_MS
            if robot_index < len(path) - 1:
                robot_index += 1
                robot_pos = path[robot_index]
            else:
                robot_running = False

        moving_cells = {obs.position for obs in moving_obstacles}
        screen.fill(WHITE)
        draw_grid(screen, grid, explored, path, active_algo, reason, moving_cells)
        draw_robot(screen, robot_pos)
        draw_status(screen, font, status_lines())
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()