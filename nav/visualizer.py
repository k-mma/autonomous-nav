import sys
import pygame

from nav.config import (
    GRID_SIZE, CELL_SIZE, DEMO_GRID_SIZE, DEMO_CELL_SIZE,
    STATUS_BAR_HEIGHT, STATUS_LINE_HEIGHT,
    PANEL_DIVIDER_WIDTH, PANEL_DIVIDER_COLOR, PANEL_LABEL_HEIGHT, PANEL_LABEL_BG, PANEL_LABEL_TEXT,
    WHITE, BLACK, GRAY, GREEN, RED, EXPLORED_COLOR, YELLOW, DARK_RED,
    ORANGE, CYAN, STATUS_BG, STATUS_TEXT,
    OBSTACLE_PERIOD_MS, ROBOT_STEP_MS, REPLAN_FLASH_MS,
    RRT_TREE_COLOR, COST_MAX_EXTRA, COST_TINT, LIDAR_RADIUS,
    HIDDEN_OBSTACLE_OUTLINE, SENSOR_RING_COLOR, CONFIRMATION_THRESHOLD,
    TERRAIN_CYCLE, TERRAIN_PAINT_CYCLE, TERRAIN_NAMES, TERRAIN_COLORS, TERRAIN_COST,
)
from nav.grid import Grid
from nav.algorithms import find_path, path_cost, weighted_path_length
from nav.heuristics import euclidean, manhattan, octile, scaled
from nav.maze import generate_maze
from nav.obstacles import MovingObstacle
from nav.sensor import LidarSensor, KnownGrid

# Step 5: the three panels always run together now -- there's no more
# "active algorithm" you switch between. RRT* is dropped from this
# interactive tool entirely (it still exists for the benchmarks/scratch
# scripts); only these three get a panel.
PANEL_ALGOS = ["dijkstra", "astar", "rrt"]
ALGO_LABELS = {"dijkstra": "Dijkstra", "astar": "A*", "rrt": "RRT"}

# Cycled through with the H key -- affects the A* panel only
HEURISTICS = [
    ("manhattan", manhattan),
    ("euclidean", euclidean),
    ("octile", octile),
    ("inadmissible (1.5x manhattan)", scaled(manhattan, 1.5)),
]

# Step 4's legend (L key) -- every color currently in the palette, not
# just the required subset the spec called out by name, since the goal
# is a viewer being able to decode literally anything they see on screen.
LEGEND_ENTRIES = [
    (GREEN, "Start"),
    (RED, "Goal"),
    (DARK_RED, "Unreachable start/goal"),
    (BLACK, "Tree / Rock (obstacle)"),
    (ORANGE, "Animal (moving obstacle)"),
    (YELLOW, "Path"),
    (EXPLORED_COLOR, "Explored cells (Dijkstra / A* / RRT)"),
    (RRT_TREE_COLOR, "RRT tree edges"),
    (CYAN, "Robot"),
    (SENSOR_RING_COLOR, "Sensor radius ring"),
    (COST_TINT, "Cost-map inflation tint (K)"),
] + [
    (TERRAIN_COLORS[t], f"{TERRAIN_NAMES[t]} x{TERRAIN_COST[t]:g}")
    for t in TERRAIN_CYCLE
]


# DRAWING


def lerp_color(c1, c2, t):
    t = max(0.0, min(1.0, t))
    return tuple(round(a + (b - a) * t) for a, b in zip(c1, c2))


def draw_grid_panel(screen, grid, explored, path, algo, reason, moving_cells,
                     sensor_enabled, known_obstacles, x_off, y_off, grid_size, cell_size):
    """Draw one algorithm's panel -- same per-cell priority order as the
    single-panel version had: start/goal, path, explored, obstacle,
    cost-map tint, terrain background (see Step 3/4)."""
    path_set = set(path) if path else set()

    for row in range(grid_size):
        for col in range(grid_size):
            cell = grid.cells[row][col]
            pos = (row, col)
            hidden = sensor_enabled and cell == Grid.OBSTACLE and pos not in known_obstacles
            terrain_color = TERRAIN_COLORS[grid.terrain[row][col]]

            if cell == Grid.START:
                color = DARK_RED if reason == "start_blocked" else GREEN
            elif cell == Grid.GOAL:
                color = DARK_RED if reason in ("goal_blocked", "no_path") else RED
            elif pos in path_set:
                color = YELLOW
            # RRT's tree is drawn separately as edges -- filling every
            # tree node here would look like Dijkstra's solid explored blob.
            elif pos in explored and algo != "rrt":
                color = EXPLORED_COLOR
            elif cell == Grid.OBSTACLE and not hidden:
                color = ORANGE if pos in moving_cells else BLACK
            elif grid.cost_map_enabled and not hidden and grid.cost[row][col] > 1.0:
                inflation_cost = grid.cost[row][col] / TERRAIN_COST[grid.terrain[row][col]]
                t = (inflation_cost - 1.0) / COST_MAX_EXTRA
                color = lerp_color(terrain_color, COST_TINT, t)
            else:
                color = terrain_color

            rect = pygame.Rect(x_off + col * cell_size, y_off + row * cell_size, cell_size, cell_size)
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, GRAY, rect, 1)
            if hidden:
                pygame.draw.rect(screen, HIDDEN_OBSTACLE_OUTLINE, rect, 2)


def draw_rrt_tree(screen, came_from, visible_nodes, path, x_off, y_off, cell_size):
    """Draw every tree edge whose child has been revealed so far (all of
    them outside step mode) as a thin line, with path edges highlighted
    once `path` is non-None (Step 6: only once the RRT panel has been
    fully stepped through)."""
    path_edges = set(zip(path, path[1:])) if path else set()
    for child, parent in came_from.items():
        if child not in visible_nodes:
            continue
        p1 = (x_off + parent[1] * cell_size + cell_size // 2, y_off + parent[0] * cell_size + cell_size // 2)
        p2 = (x_off + child[1] * cell_size + cell_size // 2, y_off + child[0] * cell_size + cell_size // 2)
        on_path = (parent, child) in path_edges or (child, parent) in path_edges
        color = YELLOW if on_path else RRT_TREE_COLOR
        width = 3 if on_path else 1
        pygame.draw.line(screen, color, p1, p2, width)
    for row, col in visible_nodes:
        center = (x_off + col * cell_size + cell_size // 2, y_off + row * cell_size + cell_size // 2)
        pygame.draw.circle(screen, RRT_TREE_COLOR, center, 2)


def draw_robot(screen, robot_pos, x_off, y_off, cell_size):
    if robot_pos is None:
        return
    row, col = robot_pos
    center = (x_off + col * cell_size + cell_size // 2, y_off + row * cell_size + cell_size // 2)
    pygame.draw.circle(screen, CYAN, center, cell_size // 3)


def draw_sensor_radius(screen, position, radius, x_off, y_off, cell_size):
    if position is None:
        return
    row, col = position
    center = (x_off + col * cell_size + cell_size // 2, y_off + row * cell_size + cell_size // 2)
    pygame.draw.circle(screen, SENSOR_RING_COLOR, center, round(radius * cell_size), 1)


def draw_panel_label(screen, font, x_off, panel_px, label_height, text):
    rect = pygame.Rect(x_off, 0, panel_px, label_height)
    pygame.draw.rect(screen, PANEL_LABEL_BG, rect)
    text_surf = font.render(text, True, PANEL_LABEL_TEXT)
    tx = x_off + (panel_px - text_surf.get_width()) // 2
    ty = (label_height - text_surf.get_height()) // 2
    screen.blit(text_surf, (tx, ty))


def draw_status(screen, font, lines, window_width, bar_y):
    bar_rect = pygame.Rect(0, bar_y, window_width, STATUS_BAR_HEIGHT)
    pygame.draw.rect(screen, STATUS_BG, bar_rect)
    for i, line in enumerate(lines):
        text_surf = font.render(line, True, STATUS_TEXT)
        screen.blit(text_surf, (10, bar_y + 6 + i * STATUS_LINE_HEIGHT))


def draw_legend(screen, font, window_width, window_height):
    swatch, pad, row_h, width = 16, 10, 22, 360
    height = pad * 2 + row_h * len(LEGEND_ENTRIES)
    x = (window_width - width) // 2
    y = max(0, (window_height - height) // 2)
    overlay = pygame.Surface((width, height), pygame.SRCALPHA)
    overlay.fill((255, 255, 255, 235))
    pygame.draw.rect(overlay, (60, 60, 60, 255), overlay.get_rect(), 2)
    for i, (color, label) in enumerate(LEGEND_ENTRIES):
        row_y = pad + i * row_h
        pygame.draw.rect(overlay, color, (pad, row_y, swatch, swatch))
        pygame.draw.rect(overlay, (90, 90, 90), (pad, row_y, swatch, swatch), 1)
        text_surf = font.render(label, True, (30, 30, 30))
        overlay.blit(text_surf, (pad + swatch + 8, row_y + 1))
    screen.blit(overlay, (x, y))


# MAIN LOOP


def main(scenario=None):
    """`scenario`, if given, is a nav.scenario.ScenarioConfig -- it's
    applied to the grid right after setup, before the event loop starts,
    so a scenario_*.py file can launch straight into a preset,
    already-run (or already-walking) result with no interaction needed.
    Duck-typed rather than imported/isinstance-checked here so this
    module doesn't need to depend on nav.scenario at all.
    """
    # --demo (Step 5): smaller grid, bigger cells, for screenshot-friendly
    # output. Every drawing function takes grid_size/cell_size explicitly
    # rather than reading module-level constants, so this is the only
    # place the choice is made. The scenario_*.py files pass a
    # ScenarioConfig instead of relying on the command line, and always
    # want demo sizing (screenshot-ready output is the whole point).
    demo = scenario is not None or "--demo" in sys.argv[1:]
    grid_size = DEMO_GRID_SIZE if demo else GRID_SIZE
    desired_cell_size = DEMO_CELL_SIZE if demo else CELL_SIZE

    pygame.init()

    # Fix: three full-size panels side by side (2108px wide at the
    # non-demo default) is wider than a lot of laptop screens, so the
    # window got created bigger than the display and simply ran off the
    # edge -- cut off mid-panel with no way to see the rest, since this
    # app never resizes or scrolls. Shrink cell_size (never grow it) just
    # enough that the whole 3-panel window plus status bar fits within
    # the actual display, before the window is ever created. Leaves a
    # margin for OS window chrome (title bar, dock/taskbar) rather than
    # using the full reported resolution.
    info = pygame.display.Info()
    cell_size = desired_cell_size
    if info.current_w > 0 and info.current_h > 0:
        max_w = int(info.current_w * 0.95)
        max_h = int(info.current_h * 0.85)
        cell_w_limit = (max_w - 2 * PANEL_DIVIDER_WIDTH) // (3 * grid_size)
        cell_h_limit = (max_h - PANEL_LABEL_HEIGHT - STATUS_BAR_HEIGHT) // grid_size
        # Never shrink below 8px/cell -- smaller than that and individual
        # cells stop being distinguishable at all.
        cell_size = max(8, min(desired_cell_size, cell_w_limit, cell_h_limit))

    panel_px = grid_size * cell_size
    window_width = 3 * panel_px + 2 * PANEL_DIVIDER_WIDTH
    window_height = PANEL_LABEL_HEIGHT + panel_px + STATUS_BAR_HEIGHT
    panel_y_off = PANEL_LABEL_HEIGHT

    def panel_x_off(i):
        return i * (panel_px + PANEL_DIVIDER_WIDTH)

    screen = pygame.display.set_mode((window_width, window_height))
    pygame.display.set_caption("Autonomous Nav" + (" (demo)" if demo else ""))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 12)
    label_font = pygame.font.SysFont("monospace", 14, bold=True)

    grid = Grid(size=grid_size)

    def blank_panel():
        return {"ran": False, "path": None, "explored": set(), "came_from": {}, "reason": None, "order": []}

    panels = {a: blank_panel() for a in PANEL_ALGOS}

    # Which heuristic astar uses
    heuristic_idx = 0

    # Step 6: step-by-step replay across all three panels, sharing one
    # step index -- each panel's own `order` list may be a different
    # length, so a panel that's exhausted just stops changing while the
    # others keep advancing.
    step_mode = False
    step_idx = 0

    moving_obstacles = []
    robot_running = False
    robot_pos = None
    robot_index = 0
    robot_blocked = False
    # Which panel's path the robot is currently following (Step 5: the
    # lowest-cost path found across the three panels) -- None when the
    # robot isn't running.
    robot_algo = None
    next_robot_step = 0
    replan_flash_until = 0

    sensor_enabled = False
    lidar = None
    noisy_sensor = False

    # Terrain painting is the default left-click action now -- there's no
    # separate mode to enter/exit. T always cycles which terrain type
    # plain left-click paints; Ctrl+LClick toggles a static obstacle and
    # Ctrl+Shift+LClick toggles a moving obstacle instead. Indexes into
    # TERRAIN_PAINT_CYCLE, not TERRAIN_CYCLE -- grass isn't paintable
    # (it's just the default tile), so it's left out of what T cycles.
    terrain_idx = 0

    legend_visible = False

    def reset_panels():
        nonlocal panels, step_mode, step_idx
        panels = {a: blank_panel() for a in PANEL_ALGOS}
        step_mode = False
        step_idx = 0

    def reset_sensor():
        nonlocal lidar
        if sensor_enabled:
            lidar = LidarSensor(LIDAR_RADIUS, noisy=noisy_sensor)
            origin = robot_pos if robot_pos is not None else grid.start
            if origin is not None:
                lidar.sense(grid, origin)

    def confirmed_cells():
        if lidar is None:
            return set()
        return lidar.confirmed_obstacles(CONFIRMATION_THRESHOLD if noisy_sensor else 1)

    def planning_grid():
        if sensor_enabled and lidar is not None:
            # size=grid.size, not KnownGrid's own default (nav.config.
            # GRID_SIZE) -- omitting it here used to silently plan
            # against a 25x25 KnownGrid even in --demo mode's 20x20 grid
            # (every scenario_*.py runs in demo mode), letting a search
            # wander into KnownGrid's assumed-free cells past the real
            # grid's actual boundary and return a path with a cell index
            # weighted_path_length then couldn't look up in the real,
            # smaller grid.cost -- see nav/replan_benchmark.py, which
            # already passes size explicitly for the same reason.
            return KnownGrid(confirmed_cells(), diagonal=grid.diagonal, size=grid.size)
        return grid

    def stop_robot():
        nonlocal robot_running, robot_pos, robot_index, robot_blocked, robot_algo
        robot_running = False
        robot_pos = None
        robot_index = 0
        robot_blocked = False
        robot_algo = None

    def current_robot_path():
        return panels[robot_algo]["path"] if robot_algo is not None else None

    def start_robot():
        nonlocal robot_running, robot_pos, robot_index, robot_blocked, next_robot_step, robot_algo
        # Walking only makes sense once you have full paths to compare --
        # step mode is for inspecting search progress, not execution.
        if step_mode or grid.start is None or grid.goal is None:
            return
        candidates = [(a, panels[a]["path"]) for a in PANEL_ALGOS if panels[a]["path"]]
        if not candidates:
            return
        best_algo, best_path = min(candidates, key=lambda kv: weighted_path_length(grid, kv[1]))
        robot_algo = best_algo
        robot_running = True
        robot_pos = best_path[0]
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
            if cell == obs.cell:
                if grid.cells[cell[0]][cell[1]] == Grid.OBSTACLE:
                    grid.cells[cell[0]][cell[1]] = Grid.FREE
                moving_obstacles.remove(obs)
                grid.refresh_cost_map()
                reset_sensor()
                reset_panels()
                stop_robot()
                return

        if not grid.is_free(row, col):
            return
        obstacle = MovingObstacle(cell, period_ms=OBSTACLE_PERIOD_MS)
        obstacle.place(grid)
        obstacle.start(pygame.time.get_ticks())
        moving_obstacles.append(obstacle)
        grid.refresh_cost_map()
        reset_sensor()
        reset_panels()
        stop_robot()

    def is_moving_obstacle_cell(row, col):
        return any((row, col) == obs.cell for obs in moving_obstacles)

    def cycle_heuristic():
        # H only invalidates the A* panel (Step 5) -- Dijkstra/RRT are
        # unaffected by heuristic choice and keep showing their last result.
        # FIX: previously just blanked the astar panel and left it that
        # way until a manual Space press -- pressing H looked like it did
        # nothing. Re-run astar immediately (when there's a start/goal)
        # so the new heuristic's effect on its explored set/path is
        # visible right away, not just the heuristic name in its label.
        nonlocal heuristic_idx, step_mode
        heuristic_idx = (heuristic_idx + 1) % len(HEURISTICS)
        step_mode = False
        if grid.start is not None and grid.goal is not None:
            run_one("astar", grid.start)
        else:
            panels["astar"] = blank_panel()
        stop_robot()

    def toggle_diagonal():
        grid.diagonal = not grid.diagonal
        # FIX: previously just reset_panels()+stop_robot(), leaving all
        # three panels blank until the user remembered to press Space
        # again -- toggling X looked like it did nothing. Re-run
        # immediately (when there's a start/goal) so the new setting's
        # effect on the explored set/path is visible right away. Note:
        # with start/goal aligned on a single row or column, diagonal
        # movement genuinely makes no difference to the result -- that's
        # not a bug, there's just nothing to gain from cutting a corner
        # on an already-straight line.
        if grid.start is not None and grid.goal is not None:
            run_active()
        else:
            reset_panels()
            stop_robot()

    def toggle_cost_map():
        grid.cost_map_enabled = not grid.cost_map_enabled
        grid.refresh_cost_map()
        reset_panels()
        stop_robot()

    def toggle_sensor():
        nonlocal sensor_enabled
        sensor_enabled = not sensor_enabled
        reset_sensor()
        reset_panels()
        stop_robot()

    def toggle_noisy_sensor():
        nonlocal noisy_sensor
        noisy_sensor = not noisy_sensor
        reset_sensor()
        reset_panels()
        stop_robot()

    def cycle_terrain():
        # Just changes what the next paint uses -- nothing on the grid
        # changes yet, so unlike the grid-mutating actions below, this
        # doesn't need to reset panels or stop the robot.
        nonlocal terrain_idx
        terrain_idx = (terrain_idx + 1) % len(TERRAIN_PAINT_CYCLE)

    def paint_terrain_cell(row, col):
        grid.paint_terrain(row, col, TERRAIN_PAINT_CYCLE[terrain_idx])
        reset_panels()
        stop_robot()

    def generate_new_maze():
        generate_maze(grid)
        moving_obstacles.clear()
        reset_sensor()
        reset_panels()
        stop_robot()

    def run_one(algo, start_cell):
        """Run a single algorithm's panel from start_cell to grid.goal,
        recording its settle/insertion order (Step 6) for step-mode
        replay. Shared by run_all (all three) and cycle_heuristic
        (astar only, since heuristic choice doesn't affect the others)."""
        order = []
        heuristic = HEURISTICS[heuristic_idx][1] if algo == "astar" else None
        path, explored, reason, came_from = find_path(
            planning_grid(), algo, start_cell, grid.goal, heuristic, order_out=order
        )
        panels[algo] = {
            "ran": True, "path": path, "explored": explored,
            "came_from": came_from, "reason": reason, "order": order,
        }

    def run_all(start_cell):
        """Run Dijkstra/A*/RRT from start_cell to grid.goal."""
        for algo in PANEL_ALGOS:
            run_one(algo, start_cell)

    def run_active():
        nonlocal step_idx, step_mode
        if grid.start is None or grid.goal is None:
            return
        run_all(grid.start)
        step_idx = 0
        step_mode = False
        stop_robot()

    def enter_step_mode():
        nonlocal step_mode, step_idx
        if grid.start is None or grid.goal is None:
            return
        run_all(grid.start)
        step_mode = True
        step_idx = 0
        stop_robot()

    def exit_step_mode():
        nonlocal step_mode
        step_mode = False

    def step_forward():
        nonlocal step_idx
        if not step_mode:
            return
        max_len = max((len(panels[a]["order"]) for a in PANEL_ALGOS), default=0)
        step_idx = min(step_idx + 1, max_len)

    def step_backward():
        nonlocal step_idx
        if not step_mode:
            return
        step_idx = max(step_idx - 1, 0)

    def replan_from_robot():
        nonlocal robot_index, robot_blocked, replan_flash_until, robot_algo
        run_all(robot_pos)
        candidates = [(a, panels[a]["path"]) for a in PANEL_ALGOS if panels[a]["path"]]
        if not candidates:
            robot_blocked = True
            robot_algo = None
        else:
            best_algo, best_path = min(candidates, key=lambda kv: weighted_path_length(grid, kv[1]))
            robot_algo = best_algo
            robot_index = 0
            robot_blocked = False
        replan_flash_until = pygame.time.get_ticks() + REPLAN_FLASH_MS

    CONTROLS_1 = "Space: run all  |  Shift+Space: step mode  |  W: walk best path  |  C: clear"

    def controls_3():
        # Diagonal's ON/OFF state is shown here so toggling X has a
        # persistent, always-visible readout -- on top of X now
        # re-running immediately (see toggle_diagonal) so its effect on
        # the explored set/path is visible right away too, instead of
        # silently going blank until the user remembered to press Space.
        diag_state = "ON" if grid.diagonal else "OFF"
        return (f"H: A* heuristic  X: diagonal ({diag_state})  M: maze  "
                f"K: cost map  S: sensor  N: noisy  L: legend")

    def controls_2():
        # Terrain painting is the default left-click action (no mode to
        # enter/exit) -- this line is computed per-frame rather than a
        # fixed string since it names whichever terrain type T last
        # selected.
        terrain_name = TERRAIN_NAMES[TERRAIN_PAINT_CYCLE[terrain_idx]]
        return (f"LClick: paint {terrain_name} (T: cycle terrain)  |  "
                f"Ctrl+LClick: obstacle  |  Ctrl+Shift+LClick: moving obstacle  |  "
                f"RClick: start (Shift: goal)")

    def format_panel_summary(algo):
        panel = panels[algo]
        noun = "nodes" if algo == "rrt" else "cells"
        if panel["path"] is None:
            return f"{ALGO_LABELS[algo]}: no path ({len(panel['explored'])} {noun})"
        cost = weighted_path_length(grid, panel["path"]) if algo == "rrt" else path_cost(grid, panel["path"])
        return f"{ALGO_LABELS[algo]}: {len(panel['explored'])} {noun}, cost {cost:.1f}"

    def status_lines():
        if grid.start is None and grid.goal is None:
            return ["Place a start (RClick) and goal (Shift+RClick) to begin."]

        all_ran = all(panels[a]["ran"] for a in PANEL_ALGOS)

        if step_mode:
            progress = "  |  ".join(
                f"{ALGO_LABELS[a]} {min(step_idx, len(panels[a]['order']))}/{len(panels[a]['order'])}"
                for a in PANEL_ALGOS
            )
            line_1 = f"STEP MODE: {progress}"
            line_2 = "Right arrow / Space: next step  |  Left arrow: back  |  Space at end: show full result & exit"
            return [line_1, line_2, controls_2(), controls_3()]

        if pygame.time.get_ticks() < replan_flash_until:
            line_1 = "REPLANNING..."
        elif robot_blocked:
            line_1 = "Robot blocked -- no path from any algorithm. Waiting for a route to open..."
        elif robot_pos is not None and not robot_running and robot_pos == grid.goal:
            line_1 = f"Robot arrived at goal! (followed {ALGO_LABELS[robot_algo]}'s path)"
        elif robot_running:
            line_1 = f"Robot walking {ALGO_LABELS[robot_algo]}'s path..."
        elif not all_ran:
            line_1 = "Press Space to run Dijkstra/A*/RRT (Shift+Space: step mode)."
        else:
            reasons = {panels[a]["reason"] for a in PANEL_ALGOS}
            if reasons == {"same_cell"}:
                line_1 = "Start and goal are the same cell."
            elif reasons == {"start_blocked"}:
                line_1 = "Start is blocked by an obstacle."
            elif reasons == {"goal_blocked"}:
                line_1 = "Goal is blocked by an obstacle."
            else:
                line_1 = "  |  ".join(format_panel_summary(a) for a in PANEL_ALGOS)
                if panels["dijkstra"]["path"] and panels["astar"]["path"]:
                    diff = len(panels["dijkstra"]["explored"]) - len(panels["astar"]["explored"])
                    if diff > 0:
                        line_1 += f"  |  A* explored {diff} fewer cells"
                    elif diff < 0:
                        line_1 += f"  |  A* explored {-diff} more cells"
                    else:
                        line_1 += "  |  A* explored the same number of cells"

        line_2 = CONTROLS_1
        if sensor_enabled and lidar is not None:
            if noisy_sensor:
                line_2 += f"  |  sensed: {len(lidar.known_obstacles)} raw / {len(confirmed_cells())} confirmed"
            else:
                line_2 += f"  |  sensed obstacles: {len(lidar.known_obstacles)}"

        return [line_1, line_2, controls_2(), controls_3()]

    def pixel_to_cell(pos):
        x, y = pos
        y -= panel_y_off
        if y < 0:
            return None
        row = y // cell_size
        if not (0 <= row < grid_size):
            return None
        for i in range(3):
            x0 = panel_x_off(i)
            if x0 <= x < x0 + panel_px:
                col = (x - x0) // cell_size
                return (row, col) if 0 <= col < grid_size else None
        return None

    if scenario is not None:
        # Apply the preset scenario to the freshly built grid, then
        # optionally run/step/walk immediately -- reuses exactly
        # the same setup (grid, obstacles, terrain) and closures
        # (run_active, enter_step_mode, start_robot) the interactive
        # app itself uses, rather than duplicating any of this logic in
        # each scenario_*.py file.
        pygame.display.set_caption(f"Autonomous Nav -- {scenario.title}")
        scenario.build(grid)
        for cell in scenario.moving_obstacles(grid):
            obstacle = MovingObstacle(cell, period_ms=OBSTACLE_PERIOD_MS)
            obstacle.place(grid)
            obstacle.start(pygame.time.get_ticks())
            moving_obstacles.append(obstacle)
        if moving_obstacles:
            grid.refresh_cost_map()
        sensor_enabled = scenario.sensor_enabled
        noisy_sensor = scenario.noisy_sensor
        reset_sensor()

        if scenario.step_snapshot is not None:
            enter_step_mode()
            max_len = max((len(panels[a]["order"]) for a in PANEL_ALGOS), default=0)
            step_idx = min(scenario.step_snapshot, max_len)
        elif scenario.auto_run:
            run_active()
            if scenario.auto_walk:
                start_robot()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                cell = pixel_to_cell(event.pos)
                if cell is None:
                    continue
                row, col = cell
                if event.button == 1:
                    # Terrain is the default left-click action -- Ctrl
                    # switches to obstacle editing, Ctrl+Shift to moving
                    # obstacles, instead of a separate mode to enter/exit.
                    mods = pygame.key.get_mods()
                    if mods & pygame.KMOD_CTRL and mods & pygame.KMOD_SHIFT:
                        toggle_moving_obstacle(row, col)
                    elif mods & pygame.KMOD_CTRL:
                        if not is_moving_obstacle_cell(row, col):
                            grid.toggle_obstacle(row, col)
                            reset_sensor()
                            reset_panels()
                            stop_robot()
                    else:
                        paint_terrain_cell(row, col)

                elif event.button == 3:
                    mods = pygame.key.get_mods()
                    if mods & pygame.KMOD_SHIFT:
                        grid.place_goal(row, col)
                    else:
                        grid.place_start(row, col)
                    reset_sensor()
                    reset_panels()
                    stop_robot()

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    if step_mode:
                        # Space jumps straight to the end and exits step
                        # mode -- right/left arrows are the actual
                        # step-at-a-time controls.
                        exit_step_mode()
                    elif pygame.key.get_mods() & pygame.KMOD_SHIFT:
                        enter_step_mode()
                    else:
                        run_active()
                elif event.key == pygame.K_RIGHT:
                    step_forward()
                elif event.key == pygame.K_LEFT:
                    step_backward()
                elif event.key == pygame.K_w:
                    toggle_robot()
                elif event.key == pygame.K_h:
                    cycle_heuristic()
                elif event.key == pygame.K_x:
                    toggle_diagonal()
                elif event.key == pygame.K_m:
                    generate_new_maze()
                elif event.key == pygame.K_k:
                    toggle_cost_map()
                elif event.key == pygame.K_s:
                    toggle_sensor()
                elif event.key == pygame.K_n:
                    toggle_noisy_sensor()
                elif event.key == pygame.K_t:
                    cycle_terrain()
                elif event.key == pygame.K_l:
                    legend_visible = not legend_visible
                elif event.key == pygame.K_c:
                    grid.clear()
                    moving_obstacles.clear()
                    reset_sensor()
                    reset_panels()
                    stop_robot()

        now = pygame.time.get_ticks()
        moved_cells = []
        for obs in moving_obstacles:
            prev_pos = obs.position
            if obs.tick(grid, now) and obs.position != prev_pos:
                moved_cells.append(obs.position)
        if moved_cells:
            grid.refresh_cost_map()

        if robot_running and moved_cells:
            rp = current_robot_path()
            remaining = set(rp[robot_index:]) if rp else set()
            if robot_blocked or robot_pos in moved_cells or remaining & set(moved_cells):
                replan_from_robot()

        if robot_running and not robot_blocked and current_robot_path() and now >= next_robot_step:
            next_robot_step = now + ROBOT_STEP_MS
            rp = current_robot_path()
            if robot_index < len(rp) - 1:
                robot_index += 1
                robot_pos = rp[robot_index]
                if sensor_enabled and lidar is not None:
                    before = confirmed_cells()
                    lidar.sense(grid, robot_pos)
                    newly_confirmed = confirmed_cells() - before
                    remaining = set(rp[robot_index:])
                    if newly_confirmed and (newly_confirmed & remaining or robot_pos in newly_confirmed):
                        replan_from_robot()
            else:
                robot_running = False

        moving_cells = {obs.position for obs in moving_obstacles}
        known_obstacles = lidar.known_obstacles if (sensor_enabled and lidar is not None) else set()

        screen.fill(WHITE)
        for i, algo in enumerate(PANEL_ALGOS):
            x_off = panel_x_off(i)
            panel = panels[algo]

            if step_mode:
                revealed = set(panel["order"][:step_idx])
                show_path = panel["path"] if step_idx >= len(panel["order"]) else None
            else:
                revealed = set(panel["order"])
                show_path = panel["path"]

            label = ALGO_LABELS[algo]
            if algo == "astar":
                label += f" ({HEURISTICS[heuristic_idx][0]})"
            draw_panel_label(screen, label_font, x_off, panel_px, PANEL_LABEL_HEIGHT, label)

            draw_grid_panel(screen, grid, revealed, show_path, algo, panel["reason"], moving_cells,
                             sensor_enabled, known_obstacles, x_off, panel_y_off, grid_size, cell_size)
            if algo == "rrt":
                draw_rrt_tree(screen, panel["came_from"], revealed, show_path, x_off, panel_y_off, cell_size)
            if sensor_enabled and lidar is not None:
                draw_sensor_radius(screen, robot_pos if robot_pos is not None else grid.start,
                                    lidar.radius, x_off, panel_y_off, cell_size)
            draw_robot(screen, robot_pos, x_off, panel_y_off, cell_size)

        for i in (1, 2):
            divider_rect = pygame.Rect(panel_x_off(i) - PANEL_DIVIDER_WIDTH, 0,
                                        PANEL_DIVIDER_WIDTH, PANEL_LABEL_HEIGHT + panel_px)
            pygame.draw.rect(screen, PANEL_DIVIDER_COLOR, divider_rect)

        draw_status(screen, font, status_lines(), window_width, PANEL_LABEL_HEIGHT + panel_px)
        if legend_visible:
            draw_legend(screen, font, window_width, window_height)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()
