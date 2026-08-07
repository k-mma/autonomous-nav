"""
The RoadRunner/MeepMeep-style demo: N sensor suites (ftc/sensors.py),
replaying the IDENTICAL seeded scenario in REAL TIME, side by side --
the same "share the random scenario across everything being compared"
rule ftc/suite_benchmark.py itself already runs on, just watchable
instead of only ever summarized as a success-rate table. Every match is
recorded once, up front, via ftc/trace.py's record_match (built on
ftc/match.py's on_tick hook -- purely additive, see that module's own
docstring for why recording a trace can't perturb the simulation
itself); playback then advances a shared match-time clock (seconds,
scaled by --speed) and pygame_app/ftc_viz/field_view.interp_snapshot
linearly interpolates each panel's robot between whichever two recorded
ticks bracket that moment -- so a suite whose steps take 0.3s each and
one whose steps take 1.2s each (MAX_DRIVE_SPEED_MPS/MAX_ACCEL_MPS2/
TURN_TIME_PER_90DEG_S, ftc/config.py) both move at the actual pace the
model says they do, not one tick per animation frame regardless of what
that tick cost.

Collisions default to ftc/match.py's on_collision="replan" here (NOT
the "halt" every existing benchmark uses, and NOT the default if you
call run_match directly) -- a robot that bumps something keeps trying
to finish the course, stuck-and-struggling for a beat, rather than the
match ending on first contact. This only changes what THIS visualizer
shows; no benchmark's own numbers are affected (see ftc/match.py's own
docstring for why "halt" stays the untouched default everywhere else).

Not built on pygame_app.scenario.ScenarioConfig / pygame_app.visualizer
.main() -- same reason pygame_app/scenarios/scenario_uncertainty.py
isn't: this steps N synchronized match traces against a shared field,
not a single search algorithm against a single displayed grid.

    python3 pygame_app/scenarios/scenario_ftc_suites.py
        # all 5 headline suites, side by side, real-time playback

    python3 pygame_app/scenarios/scenario_ftc_suites.py --suite apriltag
    python3 pygame_app/scenarios/scenario_ftc_suites.py --suites apriltag,full_suite
    python3 pygame_app/scenarios/scenario_ftc_suites.py --suites all

    python3 pygame_app/scenarios/scenario_ftc_suites.py --opponent moving
        # a second robot (an alliance partner or the opposing alliance)
        # on the field, driving its OWN pre-planned auto path from its
        # own starting wall to its own goal (OpponentRobot below) -- not
        # a single randomly-wandering cell. Each suite panel gets its
        # own fresh copy of the identical opponent trajectory (a
        # deliberate departure from ftc/opponent_benchmark.py's own
        # run_combo, which reuses one mutable object across every suite
        # in its inner loop and leaves each suite starting from wherever
        # the PREVIOUS suite's run left it -- fine for that module's
        # aggregate success-rate statistics, but wrong for a side-by-
        # side replay where every panel needs to show the identical path)
    python3 pygame_app/scenarios/scenario_ftc_suites.py --opponent static
        # a second, stationary robot -- one that's already finished its
        # own auto and is parked somewhere on the field

    python3 pygame_app/scenarios/scenario_ftc_suites.py --layout corridor \\
        --deviation-type obstacle_drift --level 0.9 --fidelity pessimistic --seed 7 --speed 2

Keys:
  SPACE       play / pause
  Left/Right  step half a second back / forward (while paused)
  Up/Down     playback speed faster / slower
  R           reroll -- new random seed, same layout/deviation/level/fidelity/opponent
  N           cycle deviation type (start_drift / obstacle_drift / unplanned_blocker)
  [ / ]       decrease / increase variance_level by 0.1
  F           cycle MODEL_FIDELITY tier (optimistic / realistic / pessimistic)
  O           cycle opponent (none / static / moving)
  Esc         quit

Legend (drawn on every panel): solid 18in-square robot = true position/
heading; grey outline square = BELIEVED position/heading (what the
suite's own pose estimate thinks); red line between them = the
pose-error vector this whole project is about. Gold = a fresh AprilTag/
IMU correction just fired. Red = currently stuck on a collision, still
trying. Orange wedge(s) = ToF cone(s). Blue wedge = camera FOV (only
drawn when fidelity narrows it below 360deg). Purple disc = lidar's
full scan. Dark/orange 18in square = the opponent robot (--opponent
static/moving), facing its own direction of travel when it's driving.
"""
import argparse
import copy
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import pygame

from nav.algorithms import astar
from nav.config import COST_INFLUENCE_RADIUS, COST_MAX_EXTRA
from nav.grid import Grid

from ftc.config import CELL_SIZE_IN, FTC_GRID_SIZE, ROBOT_RADIUS_CELLS
from ftc.field import build_grid, tag_sites_for
from ftc.sensors import SUITE_LABELS, SUITE_ORDER, SUITES, heading_deg
from ftc.suite_benchmark import DEVIATION_TYPE_ORDER, DEVIATION_TYPES, MAX_ATTEMPTS_PER_TRIAL, MIN_PATH_LEN
from ftc.trace import record_match
from nav.field_variance import generate_ground_truth

from pygame_app.ftc_viz import field_view

WHITE = (255, 255, 255)
PANEL_BG = (250, 250, 252)
LABEL_BG = (60, 60, 66)
LABEL_TEXT = (240, 240, 245)  # on LABEL_BG (dark) -- needs light text
HUD_TEXT = (40, 40, 46)       # on PANEL_BG (light) -- needs dark text
STATUS_BG = (30, 30, 34)
STATUS_TEXT = (225, 225, 230)
SUCCESS_TEXT = (110, 230, 140)
FAIL_TEXT = (230, 110, 110)
PENDING_TEXT = (200, 200, 205)

FIDELITY_ORDER = ["optimistic", "realistic", "pessimistic"]
OPPONENT_ORDER = ["none", "static", "moving"]
LABEL_H = 24
PAD = 6
# Cell size in pixels, keyed by column count -- a lookup rather than a
# computed fit, so the window stays a predictable, presentable size at
# every panel count. Larger than the first version of this file's --
# the robot is now drawn at true 18in (3-cell) scale, which needs more
# room per cell to read clearly than the old single-cell marker did.
CELL_BY_COLS = {1: 32, 2: 24, 3: 18, 4: 14, 5: 14}
STEP_SECONDS = 0.5  # Left/Right scrub increment while paused


def _hud_height(font):
    return font.get_linesize() * 3 + 8


def _layout_cols(n):
    if n <= 1:
        return 1
    if n <= 4:
        return 2
    return 3


def _pad_field_boundary(grid, radius):
    """Local fix, scoped to this visualizer only: ftc/field.py's own
    hard-inflation (build_grid's _hard_inflate) only grows obstacles
    AWAY from real game elements -- it never checks whether the FIELD'S
    OWN outer boundary leaves room for an 18in robot. A cell right at
    the edge (row/col 0 or size-1) can come back "free," and a robot
    centered there would need up to 1.5 cells of its body extending
    past the real perimeter wall -- physically impossible, and the
    direct cause of the robot ever being drawn as if part of it were
    cut off inside the wall.

    Not fixed in ftc/field.py itself -- every benchmark's already-
    published, regression-tested numbers depend on that module's exact
    current behavior, and this is cosmetic/physical-realism-only, not a
    finding that changes any of them. Fixed here instead, locally: a
    `radius`-thick border around the whole grid is marked obstacle
    before any suite ever plans against it, so the planner (and
    therefore every suite's actual executed position) never targets a
    cell an 18in robot couldn't really occupy. Applied BEFORE free_cells
    is computed below, so start/goal sampling never picks a boundary
    cell either.
    """
    size = grid.size
    for i in range(size):
        for d in range(radius):
            grid.cells[d][i] = Grid.OBSTACLE
            grid.cells[size - 1 - d][i] = Grid.OBSTACLE
            grid.cells[i][d] = Grid.OBSTACLE
            grid.cells[i][size - 1 - d] = Grid.OBSTACLE
    # The soft cost map (drawn nowhere in this visualizer, but consulted
    # by astar's step costs) was computed before this padding -- stale
    # near the new border otherwise. Recomputed with the exact same
    # defaults ftc/field.py's own build_grid already used.
    grid.compute_cost_map(influence_radius=COST_INFLUENCE_RADIUS, max_extra=COST_MAX_EXTRA)


def _wall_adjacent_cells(grid, radius):
    """Cells immediately inside the padded boundary (see
    _pad_field_boundary) -- the closest a robot can legally get to the
    real perimeter wall, and where a real FTC match actually begins:
    the game manual's legal starting positions are against the field
    perimeter, not out in the open field. Sorted (not left as a set)
    so sampling from it is seed-reproducible."""
    size = grid.size
    cells = set()
    for i in range(size):
        for r, c in ((radius, i), (size - 1 - radius, i), (i, radius), (i, size - 1 - radius)):
            if grid.is_valid(r, c) and grid.cells[r][c] == 0:
                cells.add((r, c))
    return sorted(cells)


def _solvable_scenario_wall_start(seed, grid, wall_cells, free_cells):
    """The same solvability search ftc/suite_benchmark.py's own
    _solvable_scenario runs, constrained so START is always a wall-
    adjacent cell (see _wall_adjacent_cells) -- GOAL can be anywhere
    else free. Local to this visualizer, not ftc/suite_benchmark.py:
    every headline-study trial samples both start and goal from the
    same free-cell pool, and changing that would touch the
    already-published, regression-tested sweep this project's other
    numbers depend on."""
    rng = random.Random(seed)
    for _ in range(MAX_ATTEMPTS_PER_TRIAL):
        start = rng.choice(wall_cells)
        goal = rng.choice(free_cells)
        if start == goal:
            continue
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= MIN_PATH_LEN:
            return start, goal
    raise RuntimeError(f"no solvable wall-start scenario for seed {seed} after {MAX_ATTEMPTS_PER_TRIAL} attempts")


OPPONENT_SECONDS_PER_CELL = 0.5  # ballpark -- see OpponentRobot's own docstring
# Collision-avoidance inflation radius for the opponent robot's footprint
# -- bigger than ROBOT_RADIUS_CELLS (the radius ftc/field.py inflates
# STATIC obstacles by). Two robots are each drawn at true 18in scale
# (half-width 1.5 cells); for their drawn squares to never overlap, their
# CENTERS need to stay >= 3.0 cells apart. A Chebyshev "keep clear of
# radius R" zone only guarantees centers end up >= R+1 cells apart, so
# R=2 is the smallest integer radius that actually delivers that -- R=1
# (the same radius a static obstacle uses, safe there only because
# field_view.eroded_obstacle_cells draws it SMALLER than its true
# inflated collision footprint) would still let two full-size drawn
# robots clip each other by up to a cell.
OPPONENT_SEPARATION_RADIUS_CELLS = 2 * ROBOT_RADIUS_CELLS


class OpponentRobot:
    """A second robot driving its OWN pre-planned autonomous path across
    the shared field -- an alliance partner or the opposing alliance's
    robot (drawn red -- see field_view.OPPONENT_COLOR, matching the red
    wall it starts against), not a single randomly-wandering cell (nav/
    obstacles.py's MovingObstacle -- the model `--opponent moving` used
    before this; that class is still used by ftc/opponent_benchmark.py's
    actual research sweep, which needs a generic "unplanned,
    unpredictable" proxy, not a literal second robot -- this is a
    visualization-only companion, not a second research subject).

    Satisfies the exact same tick(grid, now_ms)/.position contract
    MovingObstacle already established (ftc/match.py's moving_obstacles
    hook doesn't know or care which implementation it's driving), so
    every primary suite's own obstacle-sensing/collision-recovery logic
    already treats it as a real, moving hazard with zero changes to
    ftc/match.py beyond exposing `.heading_deg` (optional, read via
    getattr) for the renderer to draw it facing the right way. Marks a
    whole OPPONENT_SEPARATION_RADIUS_CELLS-radius footprint obstacle as
    it moves (tracking exactly which currently-obstacle cells it itself
    claimed, in `_owned`, so releasing an old footprint on the move
    never frees a cell that was a REAL static obstacle before this robot
    ever reached it) -- not just its single center cell -- which is what
    actually guarantees the primary robot's own drawn 18in square can
    never overlap this one: ftc/match.py only ever lets the primary
    robot's center rest on a cell this footprint doesn't cover.

    Advances along its own precomputed astar path at a fixed pace
    (OPPONENT_SECONDS_PER_CELL, a ballpark figure -- no attempt to run a
    second full ftc/match.py simulation, including its own drift/turn-
    cost/replanning, for a companion that exists to be watched, not
    measured) rather than reacting to anything the primary robot does,
    matching how an actual alliance partner or opposing robot's pre-
    programmed auto period actually behaves: it runs its own script, it
    does not know or care that your robot exists. `state_at(seconds)`
    is a pure function of elapsed match time (no mutation, safe to call
    from the renderer on a dedicated reference instance) -- what makes
    the two robots render as running SIMULTANEOUSLY, on one shared
    clock, rather than the opponent's visible path being however far
    THIS PARTICULAR suite's own match trace happened to get before its
    own match ended.
    """

    def __init__(self, path, seconds_per_cell=OPPONENT_SECONDS_PER_CELL):
        self.path = path
        self.seconds_per_cell = seconds_per_cell
        self.idx = 0
        self.cell = path[0]
        self._owned = set()

    @property
    def position(self):
        return self.cell

    @property
    def heading_deg(self):
        nxt = self.path[min(self.idx + 1, len(self.path) - 1)]
        return heading_deg(self.cell, nxt) if nxt != self.cell else heading_deg(self.path[max(self.idx - 1, 0)],
                                                                                  self.cell)

    def _footprint(self, grid, cell):
        r0, c0 = cell
        radius = OPPONENT_SEPARATION_RADIUS_CELLS
        return {(r0 + dr, c0 + dc) for dr in range(-radius, radius + 1) for dc in range(-radius, radius + 1)
                if grid.is_valid(r0 + dr, c0 + dc)}

    def _apply_footprint(self, grid):
        new_fp = self._footprint(grid, self.cell)
        for r, c in self._owned - new_fp:
            if grid.cells[r][c] == Grid.OBSTACLE:
                grid.cells[r][c] = Grid.FREE
        newly_owned = set()
        for r, c in new_fp:
            if grid.cells[r][c] == Grid.FREE:
                grid.cells[r][c] = Grid.OBSTACLE
                newly_owned.add((r, c))
            elif (r, c) in self._owned:
                newly_owned.add((r, c))
        self._owned = newly_owned

    def place(self, grid):
        self._apply_footprint(grid)

    def tick(self, grid, now_ms):
        target_idx = min(len(self.path) - 1, int(now_ms / 1000.0 / self.seconds_per_cell))
        if target_idx == self.idx:
            return False
        self.idx = target_idx
        self.cell = self.path[self.idx]
        self._apply_footprint(grid)
        return True

    def state_at(self, seconds):
        """(position, heading_deg) at `seconds` into the match, smoothly
        interpolated between path cells -- a pure lookup, no mutation,
        independent of any suite's own match length or ftc/match.py's
        tick loop entirely. See class docstring."""
        exact_idx = max(0.0, seconds) / self.seconds_per_cell
        idx0 = min(len(self.path) - 1, int(exact_idx))
        idx1 = min(len(self.path) - 1, idx0 + 1)
        frac = (exact_idx - idx0) if idx0 < len(self.path) - 1 else 0.0
        r0, c0 = self.path[idx0]
        r1, c1 = self.path[idx1]
        position = (r0 + (r1 - r0) * frac, c0 + (c1 - c0) * frac)
        if self.path[idx0] != self.path[idx1]:
            heading = heading_deg(self.path[idx0], self.path[idx1])
        else:
            heading = heading_deg(self.path[max(idx0 - 1, 0)], self.path[idx0])
        return position, heading


def _wall_side(cell, radius, grid_size):
    """Which of the field's 4 walls `cell` sits against -- "top"/
    "bottom"/"left"/"right" -- or None if it isn't actually a wall-
    adjacent cell at all. A true corner cell satisfies two of these at
    once; row-adjacency is checked first, an arbitrary but deterministic
    tie-break."""
    row, col = cell
    if row == radius:
        return "top"
    if row == grid_size - 1 - radius:
        return "bottom"
    if col == radius:
        return "left"
    if col == grid_size - 1 - radius:
        return "right"
    return None


OPPOSITE_WALL = {"top": "bottom", "bottom": "top", "left": "right", "right": "left"}


def _opponent_path(grid, wall_cells, free_cells, primary_start, seed):
    """The opponent's own start->goal route. Start is a wall-adjacent
    cell on the wall OPPOSITE the primary robot's own starting wall
    (its own alliance/opposing station -- an FTC field's Red Wall and
    Blue Wall are genuinely opposite sides, see field_view.py's own
    sourcing comment) -- the farthest such cell from the primary's own
    start, among that opposite wall's candidates, so the two robots'
    paths have real room to cross rather than starting on top of each
    other. Falls back to every wall cell if the opposite wall happens to
    have none free (a heavily-obstructed layout/seed combination).
    Goal is a solvable random free cell, found the same way
    _solvable_scenario_wall_start finds the primary robot's own
    scenario. Returns None if no route exists at all."""
    size = grid.size
    primary_side = _wall_side(primary_start, ROBOT_RADIUS_CELLS, size)
    opposite = OPPOSITE_WALL.get(primary_side)
    candidates = [c for c in wall_cells
                  if opposite is not None and _wall_side(c, ROBOT_RADIUS_CELLS, size) == opposite]
    if not candidates:
        candidates = wall_cells
    opponent_start = max(candidates, key=lambda c: (c[0] - primary_start[0]) ** 2 + (c[1] - primary_start[1]) ** 2)

    rng = random.Random(seed + 500_000)
    for _ in range(MAX_ATTEMPTS_PER_TRIAL):
        goal = rng.choice(free_cells)
        if goal == opponent_start:
            continue
        path, _, _ = astar(grid, opponent_start, goal)
        if path is not None and len(path) >= MIN_PATH_LEN:
            return path
    return None


def build_scenario(layout, deviation_type, level, seed, opponent):
    grid = build_grid(layout)
    _pad_field_boundary(grid, ROBOT_RADIUS_CELLS)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    wall_cells = _wall_adjacent_cells(grid, ROBOT_RADIUS_CELLS)
    tag_sites = tag_sites_for(layout)
    start, goal = _solvable_scenario_wall_start(seed, grid, wall_cells, free_cells)

    ground_truth, actual_start = generate_ground_truth(grid, start, goal, level, seed=seed,
                                                          **DEVIATION_TYPES[deviation_type])

    static_opponent_cell = None
    if opponent == "static":
        # A stationary robot -- one that's already finished its own
        # auto and is parked somewhere -- placed directly in ground
        # truth only (never in `grid`, the map suites plan against), the
        # same "unplanned" asymmetry nav/field_variance.py's own
        # _place_unplanned_blocker already uses: a suite that doesn't
        # sense obstacles has no way to know it's there ahead of time.
        # A full OPPONENT_SEPARATION_RADIUS_CELLS footprint (not just its
        # own single cell), same overlap-prevention reasoning as
        # OpponentRobot's own docstring -- it never moves again once
        # placed, so a one-off mark is enough, no ownership bookkeeping
        # needed.
        path, _, _ = astar(grid, start, goal)
        if path and len(path) >= 3:
            static_opponent_cell = random.Random(seed + 600_000).choice(path[1:-1])
            r0, c0 = static_opponent_cell
            radius = OPPONENT_SEPARATION_RADIUS_CELLS
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    r, c = r0 + dr, c0 + dc
                    if ground_truth.is_valid(r, c) and ground_truth.cells[r][c] == Grid.FREE:
                        ground_truth.cells[r][c] = Grid.OBSTACLE

    opponent_path = _opponent_path(grid, wall_cells, free_cells, start, seed) if opponent == "moving" else None
    opponent_reference = OpponentRobot(opponent_path) if opponent_path is not None else None

    return (grid, tag_sites, start, goal, ground_truth, actual_start, static_opponent_cell, opponent_path,
            opponent_reference)


def record_all(suite_names, grid, tag_sites, start, goal, ground_truth, actual_start, seed, fidelity,
                 opponent_path):
    """Records one MatchTrace per suite, all against the identical
    scenario. A non-None `opponent_path` gives EACH suite its own fresh
    ground_truth copy + a freshly-built OpponentRobot driving the exact
    same precomputed path -- deliberately not the shared-mutable-object
    pattern ftc/opponent_benchmark.py's own run_combo uses for its
    MovingObstacle (see that class's docstring), so every panel's own
    match sees the identical opponent trajectory instead of picking up
    wherever the previous suite's run left a shared object. This is
    ONLY for each suite's own sensing/collision interaction with the
    opponent -- the renderer never reads the opponent's position back
    out of these traces (see OpponentRobot.state_at and main()'s own
    dedicated reference instance) precisely because each suite's match
    ends at a different time, and the opponent has to keep running
    (and be drawn identically in every panel) regardless."""
    traces = []
    for name in suite_names:
        suite = SUITES[name]()
        suite_ground_truth = ground_truth
        moving_obstacles = ()
        if opponent_path is not None:
            suite_ground_truth = copy.deepcopy(ground_truth)
            robot = OpponentRobot(opponent_path)
            robot.place(suite_ground_truth)
            moving_obstacles = (robot,)
        trace = record_match(suite, grid, start, goal, suite_ground_truth, actual_start, tag_sites,
                               random.Random(seed), fidelity=fidelity, on_collision="replan",
                               moving_obstacles=moving_obstacles)
        traces.append(trace)
    return traces


def trail_up_to(trace, match_time_s):
    return [t["true_position"] for t in trace.ticks
            if t["event"] in ("start", "step") and t["elapsed_s"] <= match_time_s]


def draw_panel(screen, font, hud_font, x0, y0, cell_px, grid_size, obstacle_cells, start, goal, tag_sites,
                trace, match_time_s, fidelity, static_opponent_cell=None, opponent_reference=None):
    panel_w = grid_size * cell_px
    panel_h = LABEL_H + grid_size * cell_px + _hud_height(hud_font)
    pygame.draw.rect(screen, PANEL_BG, (x0, y0, panel_w, panel_h))
    pygame.draw.rect(screen, LABEL_BG, (x0, y0, panel_w, LABEL_H))
    name = SUITE_LABELS.get(trace.suite.name, trace.suite.name)
    screen.blit(font.render(f"{name}  (${trace.suite.cost_usd:.0f})", True, LABEL_TEXT), (x0 + 6, y0 + 4))

    gx0, gy0 = x0, y0 + LABEL_H
    clip_rect = pygame.Rect(gx0, gy0, panel_w, grid_size * cell_px)
    screen.set_clip(clip_rect)

    field_view.draw_field_skin(screen, gx0, gy0, cell_px, grid_size)
    field_view.draw_obstacles(screen, gx0, gy0, cell_px, obstacle_cells)
    field_view.draw_start_goal(screen, gx0, gy0, cell_px, start, goal)
    field_view.draw_tag_sites(screen, gx0, gy0, cell_px, tag_sites)

    snap = field_view.interp_snapshot(trace, match_time_s, grid_size=grid_size)
    field_view.draw_planned_path(screen, gx0, gy0, cell_px, snap["path"])
    field_view.draw_trail(screen, gx0, gy0, cell_px, trail_up_to(trace, match_time_s))

    visuals = field_view.suite_sensor_visuals(trace.suite)
    field_view.draw_sensor_visual(screen, gx0, gy0, cell_px, visuals, snap["true_position"],
                                    snap["heading_deg"], fidelity)
    if snap["newly_seen_believed"]:
        field_view.draw_newly_seen(screen, gx0, gy0, cell_px, snap["newly_seen_believed"])
    if opponent_reference is not None:
        # Read from the dedicated reference instance via the SHARED
        # match_time_s clock, not from this trace's own moving_obstacle_
        # positions -- this suite's own match may already have ended
        # (succeeded, collided, timed out) while the opponent's separate
        # timeline keeps going; drawing from state_at() is what makes
        # the two robots render as running simultaneously, identically,
        # in every panel, regardless of how long THIS suite's own match
        # actually lasted. See OpponentRobot's own docstring.
        opp_cell, opp_heading = opponent_reference.state_at(match_time_s)
        # Render-time-only safety net on top of the simulation-level
        # footprint avoidance -- see field_view.separate_if_overlapping's
        # own docstring for why both are needed.
        opp_cell = field_view.separate_if_overlapping(snap["true_position"], opp_cell)
        field_view.draw_opponent(screen, gx0, gy0, cell_px, opp_cell, opp_heading)
    if static_opponent_cell is not None:
        # Facing the field's own center -- an arbitrary but reasonable
        # "parked" pose (there's no real direction-of-travel to derive
        # one from, since it isn't moving), drawn as a full robot shape
        # rather than a plain square so it visually reads as the same
        # kind of thing the moving opponent is, just stationary.
        parked_heading = heading_deg(static_opponent_cell, (grid_size // 2, grid_size // 2))
        static_draw_cell = field_view.separate_if_overlapping(snap["true_position"], static_opponent_cell)
        field_view.draw_opponent(screen, gx0, gy0, cell_px, static_draw_cell, parked_heading)

    field_view.draw_robots(screen, gx0, gy0, cell_px, snap)
    if snap["collision_flash"]:
        field_view.draw_collision_mark(screen, gx0, gy0, cell_px, snap["attempted_position"])

    screen.set_clip(None)

    hud_y = gy0 + grid_size * cell_px + 2
    result = trace.result
    if snap["finished"]:
        outcome = "SUCCESS" if result.success else "OVER BUDGET" if result.over_budget else "STUCK"
        outcome_color = SUCCESS_TEXT if outcome == "SUCCESS" else FAIL_TEXT
    else:
        outcome = "STUCK, TRYING..." if snap["is_stuck"] else "en route..."
        outcome_color = FAIL_TEXT if snap["is_stuck"] else PENDING_TEXT
    err_in = _error_in(snap)
    line1 = f"t={snap['elapsed_s']:.1f}s/30s  err={err_in:.1f}in  hdg_err={snap['heading_error_deg']:.1f}deg"
    line2 = f"bumps={snap['collisions']}  replans={snap['replans']}"
    screen.blit(hud_font.render(line1, True, HUD_TEXT), (x0 + 4, hud_y))
    screen.blit(hud_font.render(line2, True, HUD_TEXT), (x0 + 4, hud_y + hud_font.get_linesize()))
    screen.blit(hud_font.render(outcome, True, outcome_color), (x0 + 4, hud_y + 2 * hud_font.get_linesize()))


def _error_in(snap):
    er, ec = snap["error"]
    return (er ** 2 + ec ** 2) ** 0.5 * CELL_SIZE_IN


STATUS_LINES = 4  # kept in sync with draw_status_bar's own line count below


def status_bar_height(font):
    return font.get_linesize() * STATUS_LINES + 14


def draw_status_bar(screen, font, window_w, window_h, status_h, layout, deviation_type, level, fidelity, seed,
                      opponent, playing, speed, match_time_s, max_time_s):
    """Split across STATUS_LINES short, fixed lines rather than one or
    two long ones -- a long single control-legend line was getting cut
    off at the window's own right edge once the window-fit sizing
    (main()'s own _fit_cell_px) started producing genuinely narrow
    windows on smaller screens; each line here is short enough to stay
    readable at any panel-count/window-width this file produces."""
    y0 = window_h - status_h
    pygame.draw.rect(screen, STATUS_BG, (0, y0, window_w, status_h))
    line1 = (f"layout={layout}  deviation={deviation_type}  variance_level={level:.1f}  fidelity={fidelity}  "
             f"opponent={opponent}  seed={seed}")
    line2 = f"{'PLAYING' if playing else 'PAUSED'}  t={match_time_s:.1f}s / {max_time_s:.1f}s  ({speed:.2f}x real time)"
    line3 = "[SPACE] play/pause  [<-/->] step  [Up/Down] speed  [Esc] quit"
    line4 = "[R] reroll  [N] deviation  [/] level  [F] fidelity  [O] opponent"
    for i, line in enumerate((line1, line2, line3, line4)):
        screen.blit(font.render(line, True, STATUS_TEXT), (10, y0 + 6 + i * font.get_linesize()))


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--suite", default=None, help="single suite name (shorthand for --suites NAME)")
    p.add_argument("--suites", default=None,
                     help="comma-separated suite names, or 'all' for every suite ftc/sensors.py defines "
                          "(default: the 5 headline suites, ftc.sensors.SUITE_ORDER)")
    p.add_argument("--layout", default="cluttered", choices=["sparse", "cluttered", "corridor"])
    p.add_argument("--deviation-type", default="start_drift", choices=DEVIATION_TYPE_ORDER)
    p.add_argument("--level", type=float, default=0.5,
                     help="variance_level in [0, 1] (default 0.5) -- higher means more field/pose deviation "
                          "and more frequent bumps/near-misses; ftc_fidelity_writeup.md's own numbers show "
                          "success rate falls further at the 'realistic'/'pessimistic' tiers than the "
                          "'optimistic' headline figures, so this default is deliberately moderate rather "
                          "than the headline sweep's own >=0.3 floor")
    p.add_argument("--fidelity", default="realistic", choices=FIDELITY_ORDER,
                     help="MODEL_FIDELITY tier -- 'optimistic' reproduces the published headline numbers "
                          "exactly (no heading error/camera-FOV gating); 'realistic'/'pessimistic' are more "
                          "visually interesting for a demo (default: realistic)")
    p.add_argument("--opponent", default="none", choices=OPPONENT_ORDER,
                     help="place a second robot (alliance partner or the opposing alliance) on the field: "
                          "'static' (parked, already done with its own auto) or 'moving' (driving its own "
                          "pre-planned auto path, identical trajectory replayed in every panel)")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier, 1.0 = real match time")
    return p.parse_args(argv)


def resolve_suite_names(args):
    if args.suite:
        return [args.suite]
    if not args.suites:
        return list(SUITE_ORDER)
    if args.suites == "all":
        return list(SUITES.keys())
    names = [s.strip() for s in args.suites.split(",")]
    unknown = [n for n in names if n not in SUITES]
    if unknown:
        raise SystemExit(f"unknown suite(s): {unknown} -- valid names: {sorted(SUITES.keys())}")
    return names


MIN_CELL_PX = 8
# Fraction of the actual screen a window is allowed to fill -- leaves
# room for the OS menu bar/dock/taskbar so the window doesn't get
# clipped at the bottom/top of the display, which is exactly what a
# fixed CELL_BY_COLS size (this file's first version) can't account
# for on a smaller or differently-scaled screen.
SCREEN_FIT_FRACTION = 0.88


def _fit_cell_px(cols, rows, hud_h, screen_w, screen_h, status_h):
    """Largest cell_px (capped by CELL_BY_COLS' own lookup, never
    scaled UP past it) that keeps the whole window within
    SCREEN_FIT_FRACTION of the actual detected screen size. Falls back
    to MIN_CELL_PX if even that can't fit (a very small or headless
    display) rather than producing a zero/negative size."""
    max_w = screen_w * SCREEN_FIT_FRACTION
    max_h = screen_h * SCREEN_FIT_FRACTION
    by_width = (max_w - (cols + 1) * PAD) / (cols * FTC_GRID_SIZE)
    by_height = (max_h - status_h - rows * (LABEL_H + hud_h) - (rows + 1) * PAD) / (rows * FTC_GRID_SIZE)
    fitted = int(min(by_width, by_height))
    lookup = CELL_BY_COLS.get(cols, 12)
    return max(MIN_CELL_PX, min(lookup, fitted))


def main(argv=None):
    args = parse_args(argv)
    suite_names = resolve_suite_names(args)

    pygame.init()
    pygame.display.set_caption("FTC sensor-suite comparison")
    font = pygame.font.SysFont(None, 18)
    hud_font = pygame.font.SysFont(None, 16)
    status_h = status_bar_height(font)

    cols = _layout_cols(len(suite_names))
    rows = (len(suite_names) + cols - 1) // cols
    display_info = pygame.display.Info()
    cell_px = _fit_cell_px(cols, rows, _hud_height(hud_font), display_info.current_w, display_info.current_h,
                             status_h)
    panel_w = FTC_GRID_SIZE * cell_px
    panel_h = LABEL_H + FTC_GRID_SIZE * cell_px + _hud_height(hud_font)
    window_w = cols * panel_w + (cols + 1) * PAD
    window_h = rows * panel_h + (rows + 1) * PAD + status_h
    screen = pygame.display.set_mode((window_w, window_h))
    clock = pygame.time.Clock()

    state = dict(layout=args.layout, deviation_type=args.deviation_type, level=args.level,
                  fidelity=args.fidelity, seed=args.seed, opponent=args.opponent)

    def rerecord():
        (grid, tag_sites, start, goal, ground_truth, actual_start, static_opp, opponent_path,
         opponent_reference) = build_scenario(state["layout"], state["deviation_type"], state["level"],
                                                 state["seed"], state["opponent"])
        traces = record_all(suite_names, grid, tag_sites, start, goal, ground_truth, actual_start,
                              state["seed"], state["fidelity"], opponent_path)
        # Computed once per scenario, not once per frame -- see field_view.
        # eroded_obstacle_cells's own docstring for why this (not
        # ground_truth.cells directly) is what actually gets drawn.
        obstacle_cells = field_view.eroded_obstacle_cells(ground_truth)
        return (grid, tag_sites, start, goal, ground_truth, actual_start, traces, obstacle_cells, static_opp,
                opponent_reference)

    (grid, tag_sites, start, goal, ground_truth, actual_start, traces, obstacle_cells, static_opponent_cell,
     opponent_reference) = rerecord()

    match_time_s = 0.0
    playing = True
    speed = args.speed

    def _max_time_s():
        # The opponent's own route can legitimately take longer than
        # every primary suite's own match -- its clock isn't capped by
        # any of them (see OpponentRobot's own docstring), so the
        # shared playback ceiling has to cover its full travel time too,
        # not just stop the instant the last primary suite finishes.
        primary_max = max(t.ticks[-1]["elapsed_s"] for t in traces)
        if opponent_reference is None:
            return primary_max
        opponent_total = (len(opponent_reference.path) - 1) * opponent_reference.seconds_per_cell
        return max(primary_max, opponent_total)

    running = True
    while running:
        max_time_s = _max_time_s()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    playing = not playing
                elif event.key == pygame.K_RIGHT:
                    match_time_s = min(max_time_s, match_time_s + STEP_SECONDS)
                elif event.key == pygame.K_LEFT:
                    match_time_s = max(0.0, match_time_s - STEP_SECONDS)
                elif event.key == pygame.K_UP:
                    speed = min(8.0, speed * 1.25)
                elif event.key == pygame.K_DOWN:
                    speed = max(0.1, speed / 1.25)
                elif event.key == pygame.K_r:
                    state["seed"] = random.randint(0, 10_000_000)
                    (grid, tag_sites, start, goal, ground_truth, actual_start, traces, obstacle_cells,
                     static_opponent_cell, opponent_reference) = rerecord()
                    match_time_s = 0.0
                elif event.key == pygame.K_n:
                    i = DEVIATION_TYPE_ORDER.index(state["deviation_type"])
                    state["deviation_type"] = DEVIATION_TYPE_ORDER[(i + 1) % len(DEVIATION_TYPE_ORDER)]
                    (grid, tag_sites, start, goal, ground_truth, actual_start, traces, obstacle_cells,
                     static_opponent_cell, opponent_reference) = rerecord()
                    match_time_s = 0.0
                elif event.key == pygame.K_LEFTBRACKET:
                    state["level"] = round(max(0.0, state["level"] - 0.1), 1)
                    (grid, tag_sites, start, goal, ground_truth, actual_start, traces, obstacle_cells,
                     static_opponent_cell, opponent_reference) = rerecord()
                    match_time_s = 0.0
                elif event.key == pygame.K_RIGHTBRACKET:
                    state["level"] = round(min(1.0, state["level"] + 0.1), 1)
                    (grid, tag_sites, start, goal, ground_truth, actual_start, traces, obstacle_cells,
                     static_opponent_cell, opponent_reference) = rerecord()
                    match_time_s = 0.0
                elif event.key == pygame.K_f:
                    i = FIDELITY_ORDER.index(state["fidelity"])
                    state["fidelity"] = FIDELITY_ORDER[(i + 1) % len(FIDELITY_ORDER)]
                    (grid, tag_sites, start, goal, ground_truth, actual_start, traces, obstacle_cells,
                     static_opponent_cell, opponent_reference) = rerecord()
                    match_time_s = 0.0
                elif event.key == pygame.K_o:
                    i = OPPONENT_ORDER.index(state["opponent"])
                    state["opponent"] = OPPONENT_ORDER[(i + 1) % len(OPPONENT_ORDER)]
                    (grid, tag_sites, start, goal, ground_truth, actual_start, traces, obstacle_cells,
                     static_opponent_cell, opponent_reference) = rerecord()
                    match_time_s = 0.0

        dt_s = clock.tick(60) / 1000.0
        if playing:
            match_time_s = min(max_time_s, match_time_s + dt_s * speed)

        screen.fill(WHITE)
        for i, trace in enumerate(traces):
            row, col = divmod(i, cols)
            x0 = PAD + col * (panel_w + PAD)
            y0 = PAD + row * (panel_h + PAD)
            draw_panel(screen, font, hud_font, x0, y0, cell_px, FTC_GRID_SIZE, obstacle_cells, start, goal,
                        tag_sites, trace, match_time_s, state["fidelity"], static_opponent_cell,
                        opponent_reference)
        draw_status_bar(screen, font, window_w, window_h, status_h, state["layout"], state["deviation_type"],
                          state["level"], state["fidelity"], state["seed"], state["opponent"], playing, speed,
                          match_time_s, max_time_s)
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
