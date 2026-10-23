"""
A dedicated visualizer for ftc/bundle.py: browse EVERY buildable
combination of 2 or more sensor suites, one at a time, each one run and
drawn live against the identical seeded scenario -- the animated
counterpart to ftc/optimizer_benchmark.py's static writeup. Separate
from pygame_app/scenarios/scenario_ftc_suites.py on purpose: that file
is the single-suite comparison tool (SUITE_ORDER's 7 headline suites,
or any explicit list of them, side by side); this one's whole subject
is COMBINATIONS, and giving it its own entry point keeps "compare fixed
suites" and "browse the bundle space" from fighting over one file's
keybindings, panel-count logic, and CLI surface.

*** HOW TO SEE EVERY BUNDLE: press Left/Right. *** That's the entire
mechanism -- Left/Right steps to the previous/next bundle in a list that
covers EVERY combination of 2+ suites ftc/bundle.py can build from
--candidates (6 suites by default: everything ftc/optimizer.py's
DEFAULT_COMPONENTS offers except dead_reckoning, which never changes a
bundle's behavior -- see DEFAULT_CANDIDATES's reasoning in this
project's own optimizer module), deduplicated by PHYSICAL HARDWARE the
same way ftc/bundle.py's enumerate_bundles always does (two suite names
that buy the same parts -- e.g. "apriltag+apriltag_imu" and
"apriltag_imu" alone -- are the same purchase and appear only once).
That's 19 distinct robots at the defaults (12 two-suite bundles, 6
three-suite, 1 four-suite); PageUp/PageDown jump 5 at a
time, Home/End jump to the first/last, and the status bar always shows
"Bundle i/19" plus the browsing keys, so there is no way to lose track
of where you are in the list or how to keep moving through it. Nothing
here needs typing a suite name -- every combination is already queued
up, in order, one keypress away.

Every panel for the CURRENTLY SELECTED bundle shows that bundle's own
components run ALONE, each its own panel, followed by the BUNDLE itself
(bold "BUNDLE:" label) as the last, rightmost panel -- so paging through
the list is literally watching "here are the ingredients, here's what
they add up to" for every combination in turn, at true 18in robot
scale, in real time, on the identical scenario every other panel that
frame is running. The panel COUNT changes with the selected bundle's
size (3 panels for a 2-suite bundle, up to 6 for the one 5-suite
bundle) -- the window resizes to fit each time you page to a
differently-sized combination, which is the one piece of runtime
behavior this file needs that the fixed-panel-count scenario_ftc_
suites.py doesn't.

    python3 pygame_app/scenarios/scenario_ftc_bundles.py
        # every 2+-suite combination of the 6 default candidates,
        # sorted cheapest-first, starting at the cheapest bundle

    python3 pygame_app/scenarios/scenario_ftc_bundles.py --sort parts
        # browse fewest-parts-first instead of cheapest-first

    python3 pygame_app/scenarios/scenario_ftc_bundles.py \\
        --candidates odometry_pods,apriltag,imu,dual_camera_apriltag --max-size 2
        # a smaller candidate pool, pairs only (6 combinations)

    python3 pygame_app/scenarios/scenario_ftc_bundles.py \\
        --start-bundle odometry_pods,apriltag
        # open already positioned on one specific combination

    python3 pygame_app/scenarios/scenario_ftc_bundles.py --opponent moving --fidelity pessimistic

Keys:
  Left / Right      *** browse to the previous / next bundle -- this is how you see them all ***
  PageUp / PageDown previous / next bundle, 5 at a time
  Home / End        jump to the first / last bundle in the list
  SPACE             play / pause
  , / .             step playback half a second back / forward (while paused)
  Up / Down         playback speed faster / slower
  R                 reroll -- new random seed, same layout/deviation/level/fidelity/opponent
  N                 cycle deviation type (start_drift / obstacle_drift / unplanned_blocker)
  [ / ]             decrease / increase variance_level by 0.1
  F                 cycle MODEL_FIDELITY tier (optimistic / realistic / pessimistic)
  O                 cycle opponent (none / static / moving)
  Esc               quit

Legend: same as scenario_ftc_suites.py (solid 18in square = true
position/heading, grey outline = believed, red line = pose error, gold
= a fresh AprilTag/IMU correction, orange wedge(s) = ToF cone(s), blue
wedge = camera FOV, red X = collision) -- one
addition: the rightmost, bundle panel's title is prefixed "BUNDLE:" so
it's never ambiguous which panel is the combination and which are its
own ingredients running alone.

WHAT "THE SAME TRIAL" MEANS HERE. Every panel on a page -- every
component running alone AND the bundle itself -- is handed the
identical scenario (one build_scenario() call: same grid, ground
truth, start/goal, tag sites) and the identical rng seed, exactly the
"shared scenarios, paired statistics" design ftc/optimizer.py's own
module docstring lays out. This isn't a visualizer-local convention
either: build_scenario is imported from scenario_ftc_suites.py and
called with the same default (layout, deviation_type, level, seed) that
file uses, so at the defaults a component panel here (e.g. the
"AprilTag" component of some bundle) is the byte-identical trial to
that suite's own panel over there -- see
pygame_app/scratch/scenario_ftc_bundles_test.py's
check_component_panels_match_individual_suite_results for the check
that actually enforces this rather than just asserting it in prose.

WHAT THIS TOOL DOES NOT TELL YOU. It plays out ONE seed at a time --
useful for building intuition about how a bundle actually behaves, not
for concluding "bundle X beats suite Y" from what you watch, which is
one trial's worth of luck. The statistically rigorous version of that
question -- every buildable combination, run for dozens of trials per
scenario profile, compared against its own best single component with
a paired bootstrap CI -- is what ftc/optimizer_benchmark.py runs; its
findings live in benchmark_results/ftc_optimizer_writeup.md (and are
summarized in this project's own README.md, "Sensor bundle optimizer"
section). Reroll (R) here to get a feel for the variance those trial
counts are averaging over, then go read that writeup for the verdict.
"""
import argparse
import copy
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import pygame

from ftc.bundle import enumerate_bundles
from ftc.config import FTC_GRID_SIZE, usd
from ftc.optimizer import DEFAULT_COMPONENTS
from ftc.sensors import SUITE_LABELS, SUITES, heading_deg
from ftc.suite_benchmark import DEVIATION_TYPE_ORDER
from ftc.trace import record_match

from pygame_app.ftc_viz import field_view
# Reused wholesale from the single-suite visualizer rather than
# reimplemented: scenario/opponent construction has nothing bundle-
# specific about it (a bundle is just another SensorSuite to
# ftc/match.py, see ftc/bundle.py's own docstring), and OpponentRobot's
# footprint bookkeeping in particular is exactly the kind of logic that
# should have one tested implementation, not two drifting copies.
from pygame_app.scenarios.scenario_ftc_suites import (
    FAIL_TEXT, FIDELITY_ORDER, HUD_TEXT, LABEL_BG, LABEL_H, LABEL_TEXT, OPPONENT_ORDER, PAD, PANEL_BG,
    PENDING_TEXT, STATUS_BG, STATUS_TEXT, SUCCESS_TEXT, WHITE, OpponentRobot, _error_in, _fit_cell_px,
    _hud_height, _layout_cols, build_scenario, trail_up_to,
)

# ftc/optimizer.py's DEFAULT_COMPONENTS minus dead_reckoning -- bundling
# dead reckoning changes nothing (every suite already drifts at least at
# its rate; ftc/bundle.py's BundleSuite takes the MIN drift_per_cell
# across components), so offering it as a combinable candidate would
# only produce bundles indistinguishable from ones without it -- exactly
# the kind of duplicate ftc/bundle.py's own part-signature dedup already
# strips back out, just without wasting a candidate slot getting there.
DEFAULT_CANDIDATES = [n for n in DEFAULT_COMPONENTS if n != "dead_reckoning"]

SORT_KEYS = {
    "cost": lambda b: (b.cost_usd, b.part_count, b.name),
    "parts": lambda b: (b.part_count, b.cost_usd, b.name),
    "name": lambda b: (b.name,),
}

STEP_SECONDS = 0.5
PAGE_JUMP = 5


def enumerate_all_bundles(candidates, min_size, max_size, sort_by):
    """Every distinct 2+-suite robot buildable from `candidates`
    (ftc/bundle.py's enumerate_bundles -- part-signature deduplicated,
    conflicting-parts combinations already excluded), sorted for
    browsing. This IS "all the possible combinations of 2 or more
    suites" from the candidate pool -- nothing here samples or caps the
    space; a smaller pool (--candidates) or a --max-size is the only way
    to see fewer than every combination."""
    bundles = enumerate_bundles(candidates, min_size=min_size, max_size=max_size)
    if not bundles:
        raise SystemExit(f"no buildable 2+-suite combinations from {candidates} at sizes "
                          f"[{min_size}, {max_size or len(candidates)}]")
    return sorted(bundles, key=SORT_KEYS[sort_by])


def _record_suite(suite, grid, tag_sites, start, goal, ground_truth, actual_start, seed, fidelity,
                    opponent_path):
    """One MatchTrace for an already-built suite -- a plain ftc/
    sensors.py suite OR an ftc/bundle.py BundleSuite, identically:
    record_match only ever needs the common SensorSuite interface both
    satisfy. Mirrors scenario_ftc_suites.py's own per-suite opponent-
    copying (see that file's record_all docstring for why each panel
    needs its own fresh ground_truth + OpponentRobot copy rather than a
    shared mutable one)."""
    suite_ground_truth = ground_truth
    moving_obstacles = ()
    if opponent_path is not None:
        suite_ground_truth = copy.deepcopy(ground_truth)
        robot = OpponentRobot(opponent_path)
        robot.place(suite_ground_truth)
        moving_obstacles = (robot,)
    return record_match(suite, grid, start, goal, suite_ground_truth, actual_start, tag_sites,
                          random.Random(seed), fidelity=fidelity, on_collision="replan",
                          moving_obstacles=moving_obstacles)


def panels_for_bundle(bundle):
    """(suite, title, is_bundle) for every panel the currently selected
    bundle needs: each of its own components, run alone, in the order
    ftc/bundle.py's BundleSuite stored them -- then the bundle itself,
    last, titled "BUNDLE: ..." so it's unmistakable which panel is the
    combination. Reuses the EXACT component suite objects the bundle
    itself holds (bundle.components), not fresh SUITES[name]() copies --
    guarantees a component panel reflects the identical configuration
    (mount headings, etc.) that went into the bundle, not just the same
    suite NAME."""
    panels = [(suite, SUITE_LABELS.get(suite.name, suite.name), False) for suite in bundle.components]
    panels.append((bundle, f"BUNDLE: {bundle.label()}", True))
    return panels


def draw_bundle_panel(screen, font, hud_font, x0, y0, cell_px, grid_size, obstacle_cells, start, goal,
                        tag_sites, trace, title, is_bundle, match_time_s, fidelity, cost_usd,
                        static_opponent_cell=None, opponent_reference=None):
    """scenario_ftc_suites.py's draw_panel, with the label driven by an
    explicit `title` (computed by panels_for_bundle above) instead of a
    plain SUITE_LABELS lookup on trace.suite.name -- a BundleSuite's own
    `.name` is a raw "+"-joined key ("odometry_pods+apriltag"), not
    something SUITE_LABELS has an entry for, and the bundle panel needs
    its own "BUNDLE:" prefix besides. The bundle panel's label bar is
    drawn in a distinct color so it reads as the combination at a glance
    even before the text is read."""
    panel_w = grid_size * cell_px
    panel_h = LABEL_H + grid_size * cell_px + _hud_height(hud_font)
    pygame.draw.rect(screen, PANEL_BG, (x0, y0, panel_w, panel_h))
    pygame.draw.rect(screen, BUNDLE_LABEL_BG if is_bundle else LABEL_BG, (x0, y0, panel_w, LABEL_H))
    # Same overflow the status bar's own lines had (see status_min_width's
    # docstring), but NOT fixed the same way here: a panel's own width is
    # set by the GRID (cell_px x grid_size) so the robot stays at true
    # 18in scale, which this file has no reason to widen just to fit
    # text -- and a bundle's title ("BUNDLE: Odometry pods + Distance
    # sensors + ...") can run well past even a wide panel. So this one
    # IS still truncated with an ellipsis, on this panel only; it is
    # never the only place the name appears -- status_min_width already
    # guarantees the SAME name renders in full, untruncated, on the
    # status bar's own bundle-label line below every frame, which is
    # what "each bundle's name can be seen" actually rests on.
    label_text = _truncate_to_width(font, f"{title}  (${usd(cost_usd)})", panel_w - 12)
    screen.blit(font.render(label_text, True, LABEL_TEXT), (x0 + 6, y0 + 4))

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
        opp_cell, opp_heading = opponent_reference.state_at(match_time_s)
        opp_cell = field_view.separate_if_overlapping(snap["true_position"], opp_cell)
        field_view.draw_opponent(screen, gx0, gy0, cell_px, opp_cell, opp_heading)
    if static_opponent_cell is not None:
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


BUNDLE_LABEL_BG = (70, 40, 120)  # distinct from LABEL_BG -- the bundle panel's title bar reads as different
# at a glance, not just by its "BUNDLE:" text prefix.

# 8 short lines rather than the 5 long ones a first draft packed this
# into -- this file's own panel counts (3-6, see panels_for_bundle)
# produce much narrower windows than scenario_ftc_suites.py's typical
# 5-panel layout ever does (a 2-suite bundle's 3-panel, 2-column window
# can be under 500px wide), and several of the 5-line version's lines
# ran wider than that on this project's own dev screen -- text quietly
# clipped at the window's right edge, exactly the trap scenario_ftc_
# suites.py's own status bar docstring already names and split short to
# avoid. Every line below is a FIXED format whose max rendered width is
# knowable in advance (bounded enum values, bounded digit counts) --
# STATUS_LINE_EXAMPLES holds a worst-case instance of each, used once at
# startup to floor the window's width so none of them can ever clip, at
# any screen size.
#
# The bundle LABEL line (2nd of 8, see draw_status_bar) is the one
# exception: a bundle's name has no fixed max length (a 5-suite
# combination's is well over 60 characters), so it can't be given a
# worst-case example here. It used to be handled by truncating it with
# an ellipsis at draw time -- which meant "make sure each bundle's name
# can be seen" wasn't actually guaranteed, just usually true. Fixed
# properly instead: status_min_width is handed the COMPLETE list of
# bundles this run will ever page through (already enumerated before
# the window is ever built, see main()), measures every one of their
# label lines, and floors the window at whichever is widest -- so no
# bundle in the current run can ever have its name cut off. _truncate_
# to_width below is kept only as a defensive fallback (it should never
# actually shorten anything now) and draw_status_bar still calls it, so
# a future change that breaks this guarantee degrades to truncation
# instead of clipped/overflowing text.
STATUS_LINES = 8
STATUS_LINE_EXAMPLES = [
    "BUNDLE 42/42 (sorted by cost)",
    None,  # the bundle-label line -- unbounded, handled by status_min_width's `bundles` arg instead
    "layout=cluttered  deviation=unplanned_blocker  variance_level=1.0",
    "fidelity=pessimistic  opponent=moving  seed=9999999",
    "PLAYING  t=12.3s / 30.0s  (8.00x real time)",
    "[<-/->] prev/next bundle   [PgUp/PgDn] jump 5   [Home/End] first/last bundle",
    "[SPACE] play/pause  [,/.] step  [Up/Down] speed",
    "[R] reroll  [N] deviation  [/] level  [F] fidelity  [O] opponent  [Esc] quit",
]


def status_bar_height(font):
    return font.get_linesize() * STATUS_LINES + 14


def bundle_status_label(bundle):
    """The exact text draw_status_bar's bundle-name line renders for
    `bundle` -- factored out so status_min_width measures the very
    string that will actually need to fit, rather than a guess that
    could silently drift from the real format string."""
    return f"{bundle.label()}  (${bundle.cost_usd:.2f}, {bundle.part_count} parts)"


def status_min_width(font, bundles, margin=20):
    """The narrowest a window can be while still fitting every FIXED
    status line AND every bundle's own full label line without clipping
    or truncating -- the floor `_build_window` applies on top of
    whatever the panel grid itself needs. `bundles` is the complete list
    this run will ever page through, so the widest possible label line
    is knowable up front rather than guessed at; that's what turns "each
    bundle's name can be seen" into an actual guarantee. Computed once
    at startup (font metrics don't change mid-run, neither does the
    bundle list) and passed in rather than recomputed per frame."""
    widths = [font.size(line)[0] for line in STATUS_LINE_EXAMPLES if line is not None]
    widths += [font.size(bundle_status_label(b))[0] for b in bundles]
    return margin + max(widths)


def _truncate_to_width(font, text, max_width):
    """`text`, shortened with a trailing "..." if it renders wider than
    `max_width`, else `text` unchanged. Binary search over character
    count rather than a fixed-percentage guess -- glyph widths aren't
    uniform (this file's own bundle labels mix narrow ("I") and wide
    ("W", "+") characters), so a length-based estimate would either
    under-truncate (still clipping) or over-truncate (trimming more
    than necessary) depending on which suites happen to be in the
    label."""
    if font.size(text)[0] <= max_width:
        return text
    ellipsis = "..."
    lo, hi = 0, len(text)
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if font.size(text[:mid] + ellipsis)[0] <= max_width:
            lo = mid
        else:
            hi = mid - 1
    return text[:lo] + ellipsis if lo > 0 else ellipsis


def draw_status_bar(screen, font, window_w, window_h, status_h, layout, deviation_type, level, fidelity, seed,
                      opponent, playing, speed, match_time_s, max_time_s, bundle_index, num_bundles, sort_by,
                      bundle_label):
    y0 = window_h - status_h
    pygame.draw.rect(screen, STATUS_BG, (0, y0, window_w, status_h))
    lines = [
        f"BUNDLE {bundle_index + 1}/{num_bundles} (sorted by {sort_by})",
        _truncate_to_width(font, bundle_label, window_w - 20),
        f"layout={layout}  deviation={deviation_type}  variance_level={level:.1f}",
        f"fidelity={fidelity}  opponent={opponent}  seed={seed}",
        f"{'PLAYING' if playing else 'PAUSED'}  t={match_time_s:.1f}s / {max_time_s:.1f}s  "
        f"({speed:.2f}x real time)",
        "[<-/->] prev/next bundle   [PgUp/PgDn] jump 5   [Home/End] first/last bundle",
        "[SPACE] play/pause  [,/.] step  [Up/Down] speed",
        "[R] reroll  [N] deviation  [/] level  [F] fidelity  [O] opponent  [Esc] quit",
    ]
    for i, line in enumerate(lines):
        screen.blit(font.render(line, True, STATUS_TEXT), (10, y0 + 6 + i * font.get_linesize()))


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--candidates", default=",".join(DEFAULT_CANDIDATES),
                     help=f"comma-separated ftc/sensors.py suite names to combine (default: "
                          f"{','.join(DEFAULT_CANDIDATES)} -- ftc/optimizer.py's DEFAULT_COMPONENTS minus "
                          "dead_reckoning, which never changes a bundle)")
    p.add_argument("--min-size", type=int, default=2, help="minimum suites per bundle (default 2 -- this "
                     "visualizer's whole subject is combinations, not single suites)")
    p.add_argument("--max-size", type=int, default=None,
                     help="maximum suites per bundle (default: no cap -- every size up to all candidates "
                          "at once)")
    p.add_argument("--sort", choices=sorted(SORT_KEYS), default="cost",
                     help="browsing order (default: cost, cheapest first)")
    p.add_argument("--start-index", type=int, default=0, help="which bundle (0-based, in the sorted list) "
                     "to open on")
    p.add_argument("--start-bundle", default=None,
                     help="comma-separated suite names identifying a specific bundle to open on instead of "
                          "--start-index -- must match one of the enumerated combinations' own component sets")
    p.add_argument("--layout", default="cluttered", choices=["sparse", "cluttered", "corridor"])
    p.add_argument("--deviation-type", default="start_drift", choices=DEVIATION_TYPE_ORDER)
    p.add_argument("--level", type=float, default=0.5)
    p.add_argument("--fidelity", default="realistic", choices=FIDELITY_ORDER)
    p.add_argument("--opponent", default="none", choices=OPPONENT_ORDER)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--speed", type=float, default=1.0)
    return p.parse_args(argv)


def resolve_candidates(args):
    names = [n.strip() for n in args.candidates.split(",") if n.strip()]
    unknown = [n for n in names if n not in SUITES]
    if unknown:
        raise SystemExit(f"unknown suite(s) {unknown} -- valid names: {sorted(SUITES.keys())}")
    return names


def resolve_start_index(args, bundles):
    if args.start_bundle is None:
        if not (0 <= args.start_index < len(bundles)):
            raise SystemExit(f"--start-index {args.start_index} out of range [0, {len(bundles) - 1}]")
        return args.start_index
    wanted = frozenset(n.strip() for n in args.start_bundle.split(",") if n.strip())
    for i, bundle in enumerate(bundles):
        if frozenset(bundle.component_names) == wanted:
            return i
    raise SystemExit(f"--start-bundle {sorted(wanted)} doesn't match any enumerated combination -- run with "
                      f"no --start-bundle and press Left/Right to browse the {len(bundles)} that exist, or "
                      f"check --candidates/--min-size/--max-size covers it")


def _build_window(total_panels, hud_h, status_h, screen_w, screen_h, min_window_w):
    """Sizes the window for `total_panels`, then WIDENS it if needed so
    the status bar's own fixed lines fit -- panel count here is small
    (3-6, see panels_for_bundle) and drives a 2- or 3-column grid, which
    can size the panel area narrower than the status text needs (a
    2-suite bundle's 3-panel window can come out under 500px on a
    modest screen; several status lines are 400px+). `min_window_w`
    (status_min_width(font), computed once at startup) is the floor;
    the panel grid stays left-anchored inside whatever's wider, so the
    only visible effect of the floor kicking in is a strip of empty
    background to the right of the panels, never clipped text."""
    cols = _layout_cols(total_panels)
    rows = (total_panels + cols - 1) // cols
    cell_px = _fit_cell_px(cols, rows, hud_h, screen_w, screen_h, status_h)
    panel_w = FTC_GRID_SIZE * cell_px
    panel_h = LABEL_H + FTC_GRID_SIZE * cell_px + hud_h
    window_w = max(cols * panel_w + (cols + 1) * PAD, min_window_w)
    window_h = rows * panel_h + (rows + 1) * PAD + status_h
    screen = pygame.display.set_mode((window_w, window_h))
    return screen, cols, cell_px, panel_w, panel_h, window_w, window_h


def main(argv=None):
    args = parse_args(argv)
    candidates = resolve_candidates(args)
    bundles = enumerate_all_bundles(candidates, args.min_size, args.max_size, args.sort)
    bundle_index = resolve_start_index(args, bundles)

    print(f"Loaded {len(bundles)} distinct 2+-suite combinations from {len(candidates)} candidates "
          f"({candidates}), sorted by {args.sort}.")
    print("Press Left/Right to step through EVERY combination one at a time (PageUp/PageDown to jump 5, "
          "Home/End for the first/last) -- the status bar always shows which one you're on.")

    pygame.init()
    pygame.display.set_caption("FTC sensor bundle browser")
    font = pygame.font.SysFont(None, 18)
    hud_font = pygame.font.SysFont(None, 16)
    status_h = status_bar_height(font)
    # Queried once, BEFORE the first set_mode -- pygame.display.Info()
    # can start reporting the CURRENT WINDOW's size instead of the real
    # desktop's once a mode has been set on some platforms/drivers, and
    # this file calls set_mode again on every bundle-size change (see
    # _build_window), so the true screen size has to be captured up
    # front and reused, never re-queried mid-run.
    display_info = pygame.display.Info()
    screen_w, screen_h = display_info.current_w, display_info.current_h
    hud_h = _hud_height(hud_font)
    # `bundles` (every combination this run will ever page through) is
    # already known at this point -- passed in so the width floor covers
    # the widest bundle NAME too, not just the fixed control-legend
    # text. See status_min_width's own docstring.
    min_window_w = status_min_width(font, bundles)

    state = dict(layout=args.layout, deviation_type=args.deviation_type, level=args.level,
                  fidelity=args.fidelity, seed=args.seed, opponent=args.opponent)

    current_panel_count = [None]  # boxed so the closure below can rebind it
    screen = cols = cell_px = panel_w = panel_h = window_w = window_h = None

    def rerecord():
        bundle = bundles[bundle_index]
        (grid, tag_sites, start, goal, ground_truth, actual_start, static_opp, opponent_path,
         opponent_reference) = build_scenario(state["layout"], state["deviation_type"], state["level"],
                                                 state["seed"], state["opponent"])
        panels = panels_for_bundle(bundle)
        traces = [_record_suite(suite, grid, tag_sites, start, goal, ground_truth, actual_start,
                                  state["seed"], state["fidelity"], opponent_path)
                  for suite, _, _ in panels]
        obstacle_cells = field_view.eroded_obstacle_cells(ground_truth)
        return (grid, tag_sites, start, goal, ground_truth, actual_start, panels, traces, obstacle_cells,
                static_opp, opponent_reference)

    def ensure_window(total_panels):
        nonlocal screen, cols, cell_px, panel_w, panel_h, window_w, window_h
        if current_panel_count[0] == total_panels:
            return
        current_panel_count[0] = total_panels
        screen, cols, cell_px, panel_w, panel_h, window_w, window_h = _build_window(
            total_panels, hud_h, status_h, screen_w, screen_h, min_window_w)

    ensure_window(len(panels_for_bundle(bundles[bundle_index])))
    (grid, tag_sites, start, goal, ground_truth, actual_start, panels, traces, obstacle_cells,
     static_opponent_cell, opponent_reference) = rerecord()

    match_time_s = 0.0
    playing = True
    speed = args.speed
    clock = pygame.time.Clock()

    def _max_time_s():
        primary_max = max(t.ticks[-1]["elapsed_s"] for t in traces)
        if opponent_reference is None:
            return primary_max
        opponent_total = (len(opponent_reference.path) - 1) * opponent_reference.seconds_per_cell
        return max(primary_max, opponent_total)

    def _goto(new_index):
        nonlocal bundle_index
        bundle_index = new_index % len(bundles)

    running = True
    while running:
        max_time_s = _max_time_s()
        need_rerecord = False
        need_window_check = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    playing = not playing
                elif event.key == pygame.K_COMMA:
                    match_time_s = max(0.0, match_time_s - STEP_SECONDS)
                elif event.key == pygame.K_PERIOD:
                    match_time_s = min(max_time_s, match_time_s + STEP_SECONDS)
                elif event.key == pygame.K_UP:
                    speed = min(8.0, speed * 1.25)
                elif event.key == pygame.K_DOWN:
                    speed = max(0.1, speed / 1.25)
                elif event.key == pygame.K_RIGHT:
                    _goto(bundle_index + 1)
                    need_rerecord = need_window_check = True
                elif event.key == pygame.K_LEFT:
                    _goto(bundle_index - 1)
                    need_rerecord = need_window_check = True
                elif event.key == pygame.K_PAGEDOWN:
                    _goto(bundle_index + PAGE_JUMP)
                    need_rerecord = need_window_check = True
                elif event.key == pygame.K_PAGEUP:
                    _goto(bundle_index - PAGE_JUMP)
                    need_rerecord = need_window_check = True
                elif event.key == pygame.K_HOME:
                    _goto(0)
                    need_rerecord = need_window_check = True
                elif event.key == pygame.K_END:
                    _goto(len(bundles) - 1)
                    need_rerecord = need_window_check = True
                elif event.key == pygame.K_r:
                    state["seed"] = random.randint(0, 10_000_000)
                    need_rerecord = True
                elif event.key == pygame.K_n:
                    i = DEVIATION_TYPE_ORDER.index(state["deviation_type"])
                    state["deviation_type"] = DEVIATION_TYPE_ORDER[(i + 1) % len(DEVIATION_TYPE_ORDER)]
                    need_rerecord = True
                elif event.key == pygame.K_LEFTBRACKET:
                    state["level"] = round(max(0.0, state["level"] - 0.1), 1)
                    need_rerecord = True
                elif event.key == pygame.K_RIGHTBRACKET:
                    state["level"] = round(min(1.0, state["level"] + 0.1), 1)
                    need_rerecord = True
                elif event.key == pygame.K_f:
                    i = FIDELITY_ORDER.index(state["fidelity"])
                    state["fidelity"] = FIDELITY_ORDER[(i + 1) % len(FIDELITY_ORDER)]
                    need_rerecord = True
                elif event.key == pygame.K_o:
                    i = OPPONENT_ORDER.index(state["opponent"])
                    state["opponent"] = OPPONENT_ORDER[(i + 1) % len(OPPONENT_ORDER)]
                    need_rerecord = True

        if need_window_check:
            ensure_window(len(panels_for_bundle(bundles[bundle_index])))
        if need_rerecord:
            (grid, tag_sites, start, goal, ground_truth, actual_start, panels, traces, obstacle_cells,
             static_opponent_cell, opponent_reference) = rerecord()
            match_time_s = 0.0

        dt_s = clock.tick(60) / 1000.0
        if playing:
            match_time_s = min(max_time_s, match_time_s + dt_s * speed)

        screen.fill(WHITE)
        for i, (trace, (suite, title, is_bundle)) in enumerate(zip(traces, panels)):
            row, col = divmod(i, cols)
            x0 = PAD + col * (panel_w + PAD)
            y0 = PAD + row * (panel_h + PAD)
            draw_bundle_panel(screen, font, hud_font, x0, y0, cell_px, FTC_GRID_SIZE, obstacle_cells, start,
                                goal, tag_sites, trace, title, is_bundle, match_time_s, state["fidelity"],
                                suite.cost_usd, static_opponent_cell, opponent_reference)
        current_bundle = bundles[bundle_index]
        draw_status_bar(screen, font, window_w, window_h, status_h, state["layout"], state["deviation_type"],
                          state["level"], state["fidelity"], state["seed"], state["opponent"], playing, speed,
                          match_time_s, max_time_s, bundle_index, len(bundles), args.sort,
                          bundle_status_label(current_bundle))
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
