"""
The RoadRunner-style demo: N sensor suites (ftc/sensors.py), replaying
the IDENTICAL seeded scenario in lockstep, side by side -- the same
"share the random scenario across everything being compared" rule
ftc/suite_benchmark.py itself already runs on, just watchable instead
of only ever summarized as a success-rate table. Every match is
recorded once, up front, via ftc/trace.py's record_match (built on
ftc/match.py's on_tick hook -- purely additive, see that module's own
docstring for why recording a trace can't perturb the simulation
itself); this file only ever steps an already-computed tick index
forward and draws whatever ftc/trace.MatchTrace it lands on
(pygame_app/ftc_viz/field_view.py owns the actual drawing).

Not built on pygame_app.scenario.ScenarioConfig / pygame_app.visualizer
.main() -- same reason pygame_app/scenarios/scenario_uncertainty.py
isn't: this steps N synchronized match traces against a shared field,
not a single search algorithm against a single displayed grid.

    python3 pygame_app/scenarios/scenario_ftc_suites.py
        # all 5 headline suites, side by side, the default demo scenario

    python3 pygame_app/scenarios/scenario_ftc_suites.py --suite apriltag
        # one suite, one big panel

    python3 pygame_app/scenarios/scenario_ftc_suites.py --suites apriltag,full_suite
        # "best suites compared" -- an arbitrary subset, side by side

    python3 pygame_app/scenarios/scenario_ftc_suites.py --suites all
        # every suite ftc/sensors.py defines, including the Priority-3/4
        # additions (imu, lidar, dual_camera_apriltag, ...) that never
        # ran in the headline sweep (ftc/sensors.py's SUITE_ORDER)

    python3 pygame_app/scenarios/scenario_ftc_suites.py --layout corridor \\
        --deviation-type obstacle_drift --level 0.9 --fidelity pessimistic --seed 7

Keys:
  SPACE       play / pause
  Left/Right  step one tick back / forward (while paused)
  Up/Down     playback speed faster / slower
  R           reroll -- new random seed, same layout/deviation/level/fidelity
  N           cycle deviation type (start_drift / obstacle_drift / unplanned_blocker)
  [ / ]       decrease / increase variance_level by 0.1
  F           cycle MODEL_FIDELITY tier (optimistic / realistic / pessimistic)
  Esc         quit

Legend (drawn on every panel): solid triangle = true position/heading;
grey outline triangle = BELIEVED position/heading (what the suite's own
pose estimate thinks); red line between them = the pose-error vector
this whole project is about. Gold = a fresh AprilTag/IMU correction just
fired this tick. Orange wedge(s) = ToF cone(s). Blue wedge(s) = camera
FOV (only drawn when fidelity narrows it below 360deg). Purple disc =
lidar's full scan. Red X = a collision.
"""
import argparse
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import pygame

from nav.algorithms import astar

from ftc.config import FTC_GRID_SIZE
from ftc.field import build_grid, tag_sites_for
from ftc.sensors import SUITE_LABELS, SUITE_ORDER, SUITES
from ftc.suite_benchmark import DEVIATION_TYPE_ORDER, DEVIATION_TYPES, _solvable_scenario
from ftc.trace import record_match
from nav.field_variance import generate_ground_truth

from pygame_app.ftc_viz import field_view

WHITE = (255, 255, 255)
PANEL_BG = (250, 250, 252)
LABEL_BG = (225, 225, 225)
LABEL_TEXT = (30, 30, 30)
STATUS_BG = (40, 40, 45)
STATUS_TEXT = (230, 230, 230)
SUCCESS_TEXT = (110, 230, 140)
FAIL_TEXT = (230, 110, 110)

FIDELITY_ORDER = ["optimistic", "realistic", "pessimistic"]
LABEL_H = 22
STATUS_H = 78
PAD = 6
# Cell size in pixels, keyed by column count -- a lookup rather than a
# computed fit, so the window stays a predictable, presentable size at
# every panel count from 1 (a big single-suite view) to "all suites"
# (a denser grid) instead of shrinking unpredictably.
CELL_BY_COLS = {1: 28, 2: 20, 3: 15, 4: 12, 5: 12}


def _hud_height(font):
    return font.get_linesize() * 3 + 8


def _layout_cols(n):
    if n <= 1:
        return 1
    if n <= 4:
        return 2
    return 3


def build_scenario(layout, deviation_type, level, seed):
    grid = build_grid(layout)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(layout)
    start, goal = _solvable_scenario(seed, grid, free_cells)
    scale_kwargs = DEVIATION_TYPES[deviation_type]
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, level, seed=seed, **scale_kwargs)
    return grid, tag_sites, start, goal, ground_truth, actual_start


def record_all(suite_names, grid, tag_sites, start, goal, ground_truth, actual_start, seed, fidelity):
    traces = []
    for name in suite_names:
        suite = SUITES[name]()
        trace = record_match(suite, grid, start, goal, ground_truth, actual_start, tag_sites,
                               random.Random(seed), fidelity=fidelity)
        traces.append(trace)
    return traces


def trail_up_to(trace, tick_idx):
    trail = []
    for t in trace.ticks[:tick_idx + 1]:
        if t["event"] in ("start", "step"):
            trail.append(t["true_position"])
    return trail


def draw_panel(screen, font, hud_font, x0, y0, cell_px, grid_size, ground_truth, start, goal, tag_sites,
                trace, tick_idx, fidelity):
    panel_w = grid_size * cell_px
    panel_h = LABEL_H + grid_size * cell_px + _hud_height(hud_font)
    pygame.draw.rect(screen, PANEL_BG, (x0, y0, panel_w, panel_h))
    pygame.draw.rect(screen, LABEL_BG, (x0, y0, panel_w, LABEL_H))
    label = f"{SUITE_LABELS[trace.suite.name]}  (${trace.suite.cost_usd:.0f})" \
        if trace.suite.name in SUITE_LABELS else f"{trace.suite.name}  (${trace.suite.cost_usd:.0f})"
    screen.blit(font.render(label, True, LABEL_TEXT), (x0 + 6, y0 + 3))

    gx0, gy0 = x0, y0 + LABEL_H
    clip_rect = pygame.Rect(gx0, gy0, panel_w, grid_size * cell_px)
    screen.set_clip(clip_rect)

    field_view.draw_grid(screen, gx0, gy0, cell_px, ground_truth)
    field_view.draw_start_goal(screen, gx0, gy0, cell_px, start, goal)
    field_view.draw_tag_sites(screen, gx0, gy0, cell_px, tag_sites)

    idx = min(tick_idx, len(trace.ticks) - 1)
    tick = trace.ticks[idx]
    field_view.draw_planned_path(screen, gx0, gy0, cell_px, tick.get("path"))
    field_view.draw_trail(screen, gx0, gy0, cell_px, trail_up_to(trace, idx))

    kind = field_view.sensor_kind(trace.suite)
    field_view.draw_sensor_visual(screen, gx0, gy0, cell_px, kind, tick["true_position"], tick["heading_deg"],
                                    trace.suite, fidelity)
    if tick.get("newly_seen_believed"):
        field_view.draw_newly_seen(screen, gx0, gy0, cell_px, tick["newly_seen_believed"])

    field_view.draw_robots(screen, gx0, gy0, cell_px, tick["true_position"], tick["heading_deg"], tick["error"],
                             tick["heading_error_deg"], tick.get("tag_corrected") or tick.get("event") == "collision")
    if tick["event"] == "collision":
        field_view.draw_collision_mark(screen, gx0, gy0, cell_px, tick["attempted_position"])

    screen.set_clip(None)

    hud_y = gy0 + grid_size * cell_px + 2
    result = trace.result
    outcome = ("SUCCESS" if result.success else "OVER BUDGET" if result.over_budget
               else "COLLIDED" if result.collisions else "...") if idx == len(trace.ticks) - 1 else "..."
    outcome_color = SUCCESS_TEXT if outcome == "SUCCESS" else FAIL_TEXT if outcome in ("OVER BUDGET", "COLLIDED") \
        else LABEL_TEXT
    line1 = f"t={tick['elapsed_s']:.1f}s/30s  err={_error_in(tick):.1f}in  hdg_err={tick['heading_error_deg']:.1f}deg"
    line2 = f"tick {idx}/{len(trace.ticks) - 1}  replans={tick['replans']}  collisions={tick['collisions']}"
    line3 = outcome
    screen.blit(hud_font.render(line1, True, LABEL_TEXT), (x0 + 4, hud_y))
    screen.blit(hud_font.render(line2, True, LABEL_TEXT), (x0 + 4, hud_y + hud_font.get_linesize()))
    screen.blit(hud_font.render(line3, True, outcome_color), (x0 + 4, hud_y + 2 * hud_font.get_linesize()))


def _error_in(tick):
    from ftc.config import CELL_SIZE_IN
    import math
    er, ec = tick["error"]
    return math.hypot(er, ec) * CELL_SIZE_IN


def draw_status_bar(screen, font, window_w, window_h, layout, deviation_type, level, fidelity, seed, playing,
                      interval_ms, tick_idx, max_tick):
    y0 = window_h - STATUS_H
    pygame.draw.rect(screen, STATUS_BG, (0, y0, window_w, STATUS_H))
    line1 = (f"layout={layout}  deviation={deviation_type}  variance_level={level:.1f}  "
             f"fidelity={fidelity}  seed={seed}")
    line2 = f"{'PLAYING' if playing else 'PAUSED'}  tick {tick_idx}/{max_tick}  ({1000 / interval_ms:.1f} ticks/s)"
    line3 = "[SPACE] play/pause  [<-/->] step  [Up/Down] speed  [R] reroll  [N] deviation  [/] level  [F] fidelity  [Esc] quit"
    screen.blit(font.render(line1, True, STATUS_TEXT), (10, y0 + 6))
    screen.blit(font.render(line2, True, STATUS_TEXT), (10, y0 + 6 + font.get_linesize()))
    screen.blit(font.render(line3, True, STATUS_TEXT), (10, y0 + 6 + 2 * font.get_linesize()))


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--suite", default=None, help="single suite name (shorthand for --suites NAME)")
    p.add_argument("--suites", default=None,
                     help="comma-separated suite names, or 'all' for every suite ftc/sensors.py defines "
                          "(default: the 5 headline suites, ftc.sensors.SUITE_ORDER)")
    p.add_argument("--layout", default="cluttered", choices=["sparse", "cluttered", "corridor"])
    p.add_argument("--deviation-type", default="start_drift", choices=DEVIATION_TYPE_ORDER)
    p.add_argument("--level", type=float, default=0.7)
    p.add_argument("--fidelity", default="realistic", choices=FIDELITY_ORDER,
                     help="MODEL_FIDELITY tier -- 'optimistic' reproduces the published headline numbers "
                          "exactly (no heading error/camera-FOV gating); 'realistic'/'pessimistic' are more "
                          "visually interesting for a demo (default: realistic)")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--fps", type=float, default=2.5, help="playback ticks per second (default 2.5)")
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


def main(argv=None):
    args = parse_args(argv)
    suite_names = resolve_suite_names(args)

    pygame.init()
    pygame.display.set_caption("FTC sensor-suite comparison")
    font = pygame.font.SysFont(None, 18)
    hud_font = pygame.font.SysFont(None, 16)

    cols = _layout_cols(len(suite_names))
    rows = (len(suite_names) + cols - 1) // cols
    cell_px = CELL_BY_COLS.get(cols, 12)
    panel_w = FTC_GRID_SIZE * cell_px
    panel_h = LABEL_H + FTC_GRID_SIZE * cell_px + _hud_height(hud_font)
    window_w = cols * panel_w + (cols + 1) * PAD
    window_h = rows * panel_h + (rows + 1) * PAD + STATUS_H
    screen = pygame.display.set_mode((window_w, window_h))
    clock = pygame.time.Clock()

    state = dict(layout=args.layout, deviation_type=args.deviation_type, level=args.level,
                  fidelity=args.fidelity, seed=args.seed)

    def rerecord():
        grid, tag_sites, start, goal, ground_truth, actual_start = build_scenario(
            state["layout"], state["deviation_type"], state["level"], state["seed"])
        traces = record_all(suite_names, grid, tag_sites, start, goal, ground_truth, actual_start,
                              state["seed"], state["fidelity"])
        return grid, tag_sites, start, goal, ground_truth, actual_start, traces

    grid, tag_sites, start, goal, ground_truth, actual_start, traces = rerecord()

    tick_idx = 0
    playing = True
    interval_ms = max(1, round(1000 / args.fps))
    next_step_time = pygame.time.get_ticks() + interval_ms

    running = True
    while running:
        max_tick = max(len(t.ticks) for t in traces) - 1
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    playing = not playing
                elif event.key == pygame.K_RIGHT:
                    tick_idx = min(max_tick, tick_idx + 1)
                elif event.key == pygame.K_LEFT:
                    tick_idx = max(0, tick_idx - 1)
                elif event.key == pygame.K_UP:
                    interval_ms = max(40, round(interval_ms / 1.25))
                elif event.key == pygame.K_DOWN:
                    interval_ms = min(2000, round(interval_ms * 1.25))
                elif event.key == pygame.K_r:
                    state["seed"] = random.randint(0, 10_000_000)
                    grid, tag_sites, start, goal, ground_truth, actual_start, traces = rerecord()
                    tick_idx = 0
                elif event.key == pygame.K_n:
                    i = DEVIATION_TYPE_ORDER.index(state["deviation_type"])
                    state["deviation_type"] = DEVIATION_TYPE_ORDER[(i + 1) % len(DEVIATION_TYPE_ORDER)]
                    grid, tag_sites, start, goal, ground_truth, actual_start, traces = rerecord()
                    tick_idx = 0
                elif event.key == pygame.K_LEFTBRACKET:
                    state["level"] = round(max(0.0, state["level"] - 0.1), 1)
                    grid, tag_sites, start, goal, ground_truth, actual_start, traces = rerecord()
                    tick_idx = 0
                elif event.key == pygame.K_RIGHTBRACKET:
                    state["level"] = round(min(1.0, state["level"] + 0.1), 1)
                    grid, tag_sites, start, goal, ground_truth, actual_start, traces = rerecord()
                    tick_idx = 0
                elif event.key == pygame.K_f:
                    i = FIDELITY_ORDER.index(state["fidelity"])
                    state["fidelity"] = FIDELITY_ORDER[(i + 1) % len(FIDELITY_ORDER)]
                    grid, tag_sites, start, goal, ground_truth, actual_start, traces = rerecord()
                    tick_idx = 0

        now = pygame.time.get_ticks()
        if playing and now >= next_step_time:
            if tick_idx < max_tick:
                tick_idx += 1
            next_step_time = now + interval_ms

        screen.fill(WHITE)
        for i, trace in enumerate(traces):
            row, col = divmod(i, cols)
            x0 = PAD + col * (panel_w + PAD)
            y0 = PAD + row * (panel_h + PAD)
            draw_panel(screen, font, hud_font, x0, y0, cell_px, FTC_GRID_SIZE, ground_truth, start, goal,
                        tag_sites, trace, tick_idx, state["fidelity"])
        draw_status_bar(screen, font, window_w, window_h, state["layout"], state["deviation_type"],
                          state["level"], state["fidelity"], state["seed"], playing, interval_ms, tick_idx, max_tick)
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
