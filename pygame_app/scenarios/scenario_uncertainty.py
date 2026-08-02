"""
Live, presentable demo of nav/uncertainty_benchmark.py's headline result:
the same wall-detour layout pybullet_app/pybullet_main.py's
build_demo_grid uses, turned into a comparison you can actually watch
instead of just reading off a success-rate table. Runs one of
nav.policies.py's three policies at a time against a single fixed
(assumed grid, ground truth, actual start) triple built once by
nav/field_variance.py -- ground truth shown on the left, the policy's
current belief about the world shown on the right, so the gap between
"what's really there" and "what the robot thinks is there" stays visible
the whole run, not just in the final success/fail outcome.

Not built on pygame_app.scenario.ScenarioConfig / pygame_app.visualizer.
main() -- those exist to run a search algorithm once against a single
displayed grid, not to step one of nav.policies.py's policies against a
ground truth grid that's deliberately different from what's shown. This
is a small, self-contained pygame loop instead of a launcher into the
shared visualizer.

    python3 pygame_app/scenarios/scenario_uncertainty.py

Keys:
  1 / 2 / 3   run OpenLoopPolicy / ReactivePolicy / BeliefPolicy against
              the exact same scenario, from the start
  R           restart the currently selected policy
  Esc / close window to quit
"""
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import pygame

from nav.algorithms import astar
from nav.config import (
    GRID_SIZE, WHITE, BLACK, GREEN, RED, GRAY, YELLOW, CYAN,
    STATUS_BG, STATUS_TEXT, PANEL_LABEL_BG, PANEL_LABEL_TEXT,
    PANEL_DIVIDER_WIDTH, PANEL_DIVIDER_COLOR, OCCUPANCY_OBSTACLE_THRESHOLD,
)
from nav.field_variance import generate_ground_truth
from nav.grid import Grid
from nav.policies import BeliefPolicy, OpenLoopPolicy, ReactivePolicy

# Same single-wall detour layout as pybullet_app/pybullet_main.py's
# build_demo_grid -- start and goal on the same row, one solid block
# directly between them, forcing a detour around one edge. Familiar
# shape, and its long detour edge is exactly what a one-cell start-drift
# offset (see VARIANCE_LEVEL/SCENARIO_SEED below) can clip.
START, GOAL = (12, 1), (12, 23)
WALL_ROWS, WALL_COLS = range(9, 18), range(9, 16)

# Picked (out of a search over nearby (variance_level, seed) pairs) so
# this exact scenario visibly demonstrates the headline result:
# OpenLoopPolicy's blind path -- shifted by the offset between where it
# planned from (START) and where the robot actually starts (actual_start,
# one cell off per field_variance's start drift) -- clips the wall corner
# its unshifted plan used to clear cleanly, so it collides partway
# through. ReactivePolicy and BeliefPolicy both sense the wall as they
# approach and route around it, reaching the goal instead. See
# nav/uncertainty_benchmark.py for the aggregate version of this same
# comparison across many trials and variance levels.
VARIANCE_LEVEL = 0.25
SCENARIO_SEED = 1

CELL = 20
GRID_PX = GRID_SIZE * CELL
LABEL_H = 28
STATUS_H = 110
DIVIDER = PANEL_DIVIDER_WIDTH
WINDOW_W = GRID_PX * 2 + DIVIDER
WINDOW_H = LABEL_H + GRID_PX + STATUS_H
STEP_MS = 250

STATUS_COLOR = {
    "running": STATUS_TEXT,
    "success": GREEN,
    "collided": RED,
    "stuck": (200, 120, 0),
    "timed_out": (200, 120, 0),
}
STATUS_LABEL = {
    "running": "running...",
    "success": "reached the goal",
    "collided": "collided with an unsensed obstacle",
    "stuck": "stuck -- no known route to the goal",
    "timed_out": "timed out before reaching the goal",
}


def build_assumed_grid():
    grid = Grid()
    for row in WALL_ROWS:
        for col in WALL_COLS:
            grid.cells[row][col] = Grid.OBSTACLE
    return grid


class Trial:
    """One (policy, fixed scenario) run, stepped one tick at a time by
    the pygame event loop below instead of all at once the way
    nav/uncertainty_benchmark.py's execute_trial does -- so each step can
    actually be drawn before the next one happens."""

    def __init__(self, name, policy_cls, assumed_grid, ground_truth, actual_start, max_steps):
        self.name = name
        self.ground_truth = ground_truth
        self.max_steps = max_steps
        self.current = actual_start
        self.trail = [actual_start]
        self.collision_cell = None
        self.steps = 0
        self.status = "running"
        self.policy = policy_cls(assumed_grid, START, GOAL, rng=random.Random(SCENARIO_SEED))

    def tick(self):
        if self.status != "running":
            return
        if self.current == GOAL:
            self.status = "success"
            return
        if self.steps >= self.max_steps:
            self.status = "timed_out"
            return
        next_cell = self.policy.step(self.ground_truth, self.current)
        if next_cell is None:
            self.status = "stuck"
            return
        if not self.ground_truth.is_valid(*next_cell) or self.ground_truth.is_obstacle(*next_cell):
            self.collision_cell = next_cell
            self.status = "collided"
            return
        self.current = next_cell
        self.trail.append(next_cell)
        self.steps += 1
        if self.current == GOAL:
            self.status = "success"

    def known_obstacles(self):
        """Cells this trial's policy currently believes are obstacles --
        nothing for OpenLoopPolicy (it never senses at all), whatever
        LidarSensor has reported for ReactivePolicy/BeliefPolicy (both
        carry a `.sensor`, see nav/policies.py)."""
        if isinstance(self.policy, (ReactivePolicy, BeliefPolicy)):
            return set(self.policy.sensor.known_obstacles)
        return set()

    def occupancy(self):
        return self.policy.occupancy if isinstance(self.policy, BeliefPolicy) else None


def lerp_color(c0, c1, t):
    t = max(0.0, min(1.0, t))
    return tuple(round(c0[i] + (c1[i] - c0[i]) * t) for i in range(3))


def draw_panel(screen, font, x0, y0, title, is_obstacle_fn, prob_fn, trial):
    pygame.draw.rect(screen, PANEL_LABEL_BG, (x0, 0, GRID_PX, LABEL_H))
    screen.blit(font.render(title, True, PANEL_LABEL_TEXT), (x0 + 8, 6))

    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            rect = (x0 + col * CELL, y0 + row * CELL, CELL, CELL)
            if is_obstacle_fn(row, col):
                color = BLACK
            elif prob_fn is not None:
                # Capped below 1.0 so a "confirmed" cell (drawn solid
                # black via is_obstacle_fn above) stays visually distinct
                # from a merely very-likely-occupied one.
                color = lerp_color(WHITE, BLACK, min(prob_fn(row, col), 0.95))
            else:
                color = WHITE
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, GRAY, rect, 1)

    sr, sc = START
    gr, gc = GOAL
    pygame.draw.rect(screen, GREEN, (x0 + sc * CELL, y0 + sr * CELL, CELL, CELL))
    pygame.draw.rect(screen, RED, (x0 + gc * CELL, y0 + gr * CELL, CELL, CELL))

    if len(trial.trail) > 1:
        points = [(x0 + c * CELL + CELL // 2, y0 + r * CELL + CELL // 2) for r, c in trial.trail]
        pygame.draw.lines(screen, YELLOW, False, points, 3)

    cr, cc = trial.current
    pygame.draw.circle(screen, CYAN, (x0 + cc * CELL + CELL // 2, y0 + cr * CELL + CELL // 2), CELL // 2 - 2)

    if trial.collision_cell is not None:
        r, c = trial.collision_cell
        cx, cy = x0 + c * CELL + CELL // 2, y0 + r * CELL + CELL // 2
        pygame.draw.line(screen, RED, (cx - 7, cy - 7), (cx + 7, cy + 7), 4)
        pygame.draw.line(screen, RED, (cx - 7, cy + 7), (cx + 7, cy - 7), 4)


def draw_ground_truth_panel(screen, font, trial):
    draw_panel(screen, font, 0, LABEL_H, "Ground truth", trial.ground_truth.is_obstacle, None, trial)


def draw_belief_panel(screen, font, trial):
    x0 = GRID_PX + DIVIDER
    if trial.name == "Open-loop":
        draw_panel(screen, font, x0, LABEL_H, "Belief (no sensing)", lambda r, c: False, None, trial)
    elif trial.name == "Reactive":
        known = trial.known_obstacles()
        draw_panel(screen, font, x0, LABEL_H, "Belief (known / unknown)",
                   lambda r, c: (r, c) in known, None, trial)
    else:
        occ = trial.occupancy()
        draw_panel(screen, font, x0, LABEL_H, "Belief (occupancy probability)",
                   lambda r, c: occ.probability(r, c) >= OCCUPANCY_OBSTACLE_THRESHOLD,
                   lambda r, c: occ.probability(r, c), trial)


def draw_status_bar(screen, font, trial):
    y0 = LABEL_H + GRID_PX
    pygame.draw.rect(screen, STATUS_BG, (0, y0, WINDOW_W, STATUS_H))

    line1 = f"[{trial.name}]  step {trial.steps}/{trial.max_steps}  replans {trial.policy.replans}  " \
            f"planning {trial.policy.planning_time_s * 1000:.1f}ms"
    line2 = STATUS_LABEL[trial.status]
    line3 = "[1] Open-loop   [2] Reactive   [3] Belief   [R] Restart   [Esc] Quit"

    screen.blit(font.render(line1, True, STATUS_TEXT), (10, y0 + 8))
    screen.blit(font.render(line2, True, STATUS_COLOR[trial.status]), (10, y0 + 32))
    screen.blit(font.render(line3, True, STATUS_TEXT), (10, y0 + 64))


def main():
    pygame.init()
    pygame.display.set_caption("Planning Under Uncertainty")
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 20)

    assumed_grid = build_assumed_grid()
    ground_truth, actual_start = generate_ground_truth(assumed_grid, START, GOAL, VARIANCE_LEVEL, seed=SCENARIO_SEED)
    assumed_path, _, _ = astar(assumed_grid, START, GOAL)
    max_steps = min(4 * len(assumed_path), len(assumed_path) + 60)

    policies = {
        pygame.K_1: ("Open-loop", OpenLoopPolicy),
        pygame.K_2: ("Reactive", ReactivePolicy),
        pygame.K_3: ("Belief", BeliefPolicy),
    }

    def new_trial(key):
        name, cls = policies[key]
        return Trial(name, cls, assumed_grid, ground_truth, actual_start, max_steps)

    current_key = pygame.K_1
    trial = new_trial(current_key)
    next_step_time = pygame.time.get_ticks() + STEP_MS

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key in policies:
                    current_key = event.key
                    trial = new_trial(current_key)
                    next_step_time = pygame.time.get_ticks() + STEP_MS
                elif event.key == pygame.K_r:
                    trial = new_trial(current_key)
                    next_step_time = pygame.time.get_ticks() + STEP_MS

        now = pygame.time.get_ticks()
        if trial.status == "running" and now >= next_step_time:
            trial.tick()
            next_step_time = now + STEP_MS

        screen.fill(WHITE)
        draw_ground_truth_panel(screen, font, trial)
        draw_belief_panel(screen, font, trial)
        pygame.draw.rect(screen, PANEL_DIVIDER_COLOR, (GRID_PX, 0, DIVIDER, LABEL_H + GRID_PX))
        draw_status_bar(screen, font, trial)
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
