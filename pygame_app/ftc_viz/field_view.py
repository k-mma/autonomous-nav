"""
Drawing primitives for one suite's animated field panel -- the
RoadRunner/MeepMeep-style "robot moving across the field" view
pygame_app/scenarios/scenario_ftc_suites.py arranges N of side by side.
Everything here is a pure function of an already-recorded
ftc.trace.MatchTrace and a point in REAL match time (seconds, not a
tick index -- see interp_snapshot); nothing simulates anything (that's
ftc/match.py's job) or holds playback state (that's the scenario
file's job).

Two robots are drawn every frame, not one, and both are drawn as an
actual 18in x 18in square (ftc/config.py's ROBOT_SIZE_IN, exactly 3x3
grid cells) instead of an abstract marker -- a solid one at the
ground-truth TRUE position/heading, and a lighter dashed outline at the
BELIEVED position/heading (true position minus the accumulated pose-
error vector, true heading minus the accumulated heading-error scalar)
-- connected by a thin line whenever they've diverged enough to notice.
That gap is the entire point of this project's pose-error model
(ftc/match.py's own docstring), and it's the single detail every suite
comparison in this codebase already reduces to a number; drawing it at
true robot scale, in real time, is what makes AprilTag's periodic
snap-back, or dead reckoning's steady drift, visible instead of implied.

The field skin below (perimeter, alliance-color corner triangles,
driver-station marks, tile seams every 24in) is deliberately generic --
evocative of "this is unmistakably an FTC field," not a recreation of
any one season's specific game elements. ftc/field.py's own module
docstring explains why the obstacle LAYOUT stays season-agnostic
(hardcoding one season's game would date this project the moment the
season rotates); the same reasoning applies to the visual skin.
"""
import math

import pygame

from ftc.config import (
    APRILTAG_RANGE_CELLS, CELL_SIZE_IN, DISTANCE_SENSOR_HALF_ANGLE_DEG, DISTANCE_SENSOR_RANGE_CELLS,
    FIDELITY_TIERS, FTC_GRID_SIZE, ROBOT_SIZE_IN,
)
from ftc.bundle import CompositeObstacleSensor
from ftc.field import eroded_obstacle_cells, in_to_cell  # noqa: F401 -- re-exported, see below
from ftc.sensors import ConeSensor, angular_diff

# --- Field skin -------------------------------------------------------------
# VERIFIED against real FTC field hardware, not guessed: the 36 floor
# tiles are solid GRAY EVA foam mats, 24in x 24in each (AndyMark's own
# "FIRST Tech Challenge Field Soft Tiles" listing) -- not white, not a
# checkerboard. The perimeter wall stands 11.5-12.375in tall, and the
# field's own coordinate-system convention (ftc-docs' Field Coordinate
# System page) names a "Red Wall" and a "Blue Wall" as two OPPOSITE
# perimeter sides (not diagonal corners), with the other two sides
# being plain, unmarked audience walls. This skin draws exactly that:
# a gray tile floor with a heavier seam every 24in tile boundary, one
# full wall edge colored red, the opposite edge blue, top/bottom left
# plain -- rather than the first version's invented white tiles and
# diagonal alliance-color corner triangles, neither of which are a real
# FTC field's appearance.
TILE_BG = (182, 184, 187)         # foam-mat gray
TILE_BG_ALT = (176, 178, 181)     # subtle per-tile shading, reads as distinct mats
TILE_SEAM = (120, 122, 126)       # every 24in tile boundary -- the heavier line
CELL_LINE = (163, 165, 168)       # every 6in model cell -- faint, not a real seam
PERIMETER_COLOR = (25, 25, 28)
OBSTACLE_FILL = (58, 62, 70)
OBSTACLE_BORDER = (28, 31, 36)
ALLIANCE_BLUE = (28, 95, 210)
ALLIANCE_RED = (205, 35, 40)
WALL_THICKNESS_FRAC = 0.05  # fraction of one grid cell -- a thin accent band, not a corner block

START_COLOR = (34, 197, 94)
GOAL_COLOR = (220, 60, 60)
TRAIL_COLOR = (240, 175, 40)
BELIEVED_ROBOT_COLOR = (150, 150, 158)
ERROR_VECTOR_COLOR = (220, 60, 60)
TAG_RING = (255, 255, 255)
TAG_FILL = (25, 25, 30)
CONE_COLOR = (255, 150, 20, 55)
CAMERA_COLOR = (40, 170, 240, 55)
NEWLY_SEEN_COLOR = (255, 90, 90)
COLLISION_COLOR = (230, 30, 30)
TAG_FLASH_COLOR = (255, 210, 20)
# The opponent -- an alliance partner or the opposing alliance's robot
# -- is drawn the same red as ALLIANCE_RED/the red wall it starts
# against, mirroring the primary robot's own blue against the blue
# wall (draw_robots' (20, 120, 220)).
OPPONENT_COLOR = ALLIANCE_RED
OPPONENT_STRIPE = (255, 210, 90)

FLASH_WINDOW_S = 0.35
STUCK_WINDOW_S = 0.6


def suite_sensor_visuals(suite):
    """Every active sensor visual `suite` has, as a list of ("cone",
    mount_headings_deg) / ("camera", mount_headings_deg) tuples -- ONE
    per physical sensor TYPE the robot carries, not one per suite it was
    built from. A ftc/bundle.py BundleSuite's obstacle sensor is a
    CompositeObstacleSensor (a union of its components' sensors, e.g.
    two differently-mounted ToF cone layouts together); this walks its
    `.sensors` rather than treating the whole composite as one opaque
    kind, so a bundle draws every sensor it actually has instead of only
    the first one found. A suite that both senses obstacles AND fixes
    pose (ftc/sensors.py's FullSuite, or any pose+obstacle bundle) draws
    both -- the single-suite `sensor_kind` this replaces picked only one
    of the two even for FullSuite, a pre-existing gap this
    generalization also closes (cosmetic only, see this module's own
    docstring on why the field skin/visuals are allowed to differ from
    what's simulated: nothing here feeds back into ftc/match.py).

    Two suites in a bundle that both carry a camera don't produce two
    camera entries: ftc/bundle.py's BundleSuite already unions
    camera_mount_headings_deg onto ONE detection pipeline (see its own
    docstring for why -- a second independent pipeline would consume
    extra rng draws and break the byte-exact single-suite reproduction
    ftc/scratch/bundle_test.py checks), so `suite.fixes_pose` here is
    already exactly one robot, one camera visual, at every mount it
    actually has."""
    visuals = []
    if suite.senses_obstacles:
        sensor = suite.make_obstacle_sensor()
        sub_sensors = sensor.sensors if isinstance(sensor, CompositeObstacleSensor) else [sensor]
        for sub in sub_sensors:
            if isinstance(sub, ConeSensor):
                visuals.append(("cone", sub.mount_headings_deg))
    if suite.fixes_pose:
        visuals.append(("camera", getattr(suite, "camera_mount_headings_deg", [0.0])))
    return visuals


def interp_snapshot(trace, match_time_s, grid_size=None):
    """Real-time playback state for `trace` at `match_time_s` seconds
    into the match -- linearly interpolated between whichever two
    recorded ticks bracket that moment (position and pose-error vector
    componentwise, heading via the shortest angular path so it never
    spins the long way around at the 359/0 wrap), so a suite whose
    steps take 0.3s each and one whose steps take 1.2s each both play
    back at the SAME real-world pace, matching MAX_DRIVE_SPEED_MPS/
    MAX_ACCEL_MPS2/TURN_TIME_PER_90DEG_S rather than one tick per
    animation frame regardless of how long that tick actually took.

    A robot that's stuck (ftc/match.py's on_collision="replan" stall-
    retry loop -- consecutive "collision" events share the same
    true_position with increasing elapsed_s) is drawn holding perfectly
    still at its last confirmed-valid cell, not wiggling or recoloring
    -- the goal is for it to read as still on its route, just taking a
    beat here, exactly like every other moment of the run, not as a
    dramatized "something is broken" event. `is_stuck` is still
    reported for the HUD text readout; nothing about the drawn
    position itself changes because of it.

    The TRUE robot's position is drawn ONLY at cells ftc/match.py's own
    collision check has already validated (it never accepts a move
    onto an obstacle or off the grid -- see that module's own collision
    block), so interpolating between two such validated cells is
    already safe; `grid_size` (defaults to ftc.config.FTC_GRID_SIZE) is
    still clamped against defensively, since a real robot physically
    cannot occupy a cell off the field or inside a wall regardless of
    what any single number here computes to."""
    ticks = trace.ticks
    if match_time_s <= ticks[0]["elapsed_s"]:
        lo = hi = ticks[0]
        frac = 0.0
    elif match_time_s >= ticks[-1]["elapsed_s"]:
        lo = hi = ticks[-1]
        frac = 0.0
    else:
        lo, hi = ticks[0], ticks[-1]
        for i in range(len(ticks) - 1):
            if ticks[i]["elapsed_s"] <= match_time_s <= ticks[i + 1]["elapsed_s"]:
                lo, hi = ticks[i], ticks[i + 1]
                break
        span = hi["elapsed_s"] - lo["elapsed_s"]
        frac = 0.0 if span <= 1e-9 else (match_time_s - lo["elapsed_s"]) / span

    tr0, tc0 = lo["true_position"]
    tr1, tc1 = hi["true_position"]
    true_position = (tr0 + (tr1 - tr0) * frac, tc0 + (tc1 - tc0) * frac)

    size = grid_size if grid_size is not None else FTC_GRID_SIZE
    true_position = (min(max(true_position[0], 0.0), size - 1.0),
                       min(max(true_position[1], 0.0), size - 1.0))

    er0, ec0 = lo["error"]
    er1, ec1 = hi["error"]
    error = (er0 + (er1 - er0) * frac, ec0 + (ec1 - ec0) * frac)

    h0, h1 = lo["heading_deg"], hi["heading_deg"]
    heading_deg_now = h0 + angular_diff(h1, h0) * frac
    he0, he1 = lo["heading_error_deg"], hi["heading_error_deg"]
    heading_error_deg = he0 + (he1 - he0) * frac

    recency = match_time_s - lo["elapsed_s"]
    recent_enough = recency < FLASH_WINDOW_S
    is_stuck = lo["event"] == "collision" and recency < STUCK_WINDOW_S

    return dict(
        true_position=true_position, error=error, heading_deg=heading_deg_now,
        heading_error_deg=heading_error_deg, path=lo.get("path"),
        collisions=lo["collisions"], replans=lo["replans"], elapsed_s=lo["elapsed_s"],
        moving_obstacle_positions=lo.get("moving_obstacle_positions", []),
        newly_seen_believed=(lo.get("newly_seen_believed") or set()) if recent_enough else set(),
        tag_flash=bool(lo.get("tag_corrected")) and recent_enough,
        collision_flash=lo["event"] == "collision" and recent_enough,
        attempted_position=lo.get("attempted_position"),
        is_stuck=is_stuck,
        finished=match_time_s >= ticks[-1]["elapsed_s"],
    )


def cell_to_px(row, col, x0, y0, cell_px):
    return x0 + col * cell_px, y0 + row * cell_px


def _alpha_surface(size):
    return pygame.Surface(size, pygame.SRCALPHA)


def draw_field_skin(screen, x0, y0, cell_px, grid_size):
    """Gray foam-tile floor, a heavier seam every 24in tile boundary, a
    red wall along one full edge and a blue wall along the OPPOSITE
    edge (ftc-docs' Field Coordinate System page names these as two
    opposing perimeter walls, not diagonal corners -- see module
    docstring for the sourcing)."""
    w = h = grid_size * cell_px
    tile_cells = max(1, round(24.0 / CELL_SIZE_IN))

    pygame.draw.rect(screen, TILE_BG, (x0, y0, w, h))
    for trow in range(0, grid_size, tile_cells):
        for tcol in range(0, grid_size, tile_cells):
            if (trow // tile_cells + tcol // tile_cells) % 2 == 0:
                continue
            tw = min(tile_cells, grid_size - tcol) * cell_px
            th = min(tile_cells, grid_size - trow) * cell_px
            pygame.draw.rect(screen, TILE_BG_ALT, (x0 + tcol * cell_px, y0 + trow * cell_px, tw, th))

    for row in range(grid_size + 1):
        color = TILE_SEAM if row % tile_cells == 0 else CELL_LINE
        width = 2 if row % tile_cells == 0 and cell_px >= 10 else 1
        y = y0 + row * cell_px
        pygame.draw.line(screen, color, (x0, y), (x0 + w, y), width)
    for col in range(grid_size + 1):
        color = TILE_SEAM if col % tile_cells == 0 else CELL_LINE
        width = 2 if col % tile_cells == 0 and cell_px >= 10 else 1
        x = x0 + col * cell_px
        pygame.draw.line(screen, color, (x, y0), (x, y0 + h), width)

    pygame.draw.rect(screen, PERIMETER_COLOR, (x0, y0, w, h), max(2, cell_px // 5))
    wall_px = max(3, int(cell_px * WALL_THICKNESS_FRAC * 6))
    pygame.draw.rect(screen, ALLIANCE_BLUE, (x0, y0, wall_px, h))       # left = blue wall
    pygame.draw.rect(screen, ALLIANCE_RED, (x0 + w - wall_px, y0, wall_px, h))  # right = red wall (opposite side)


# eroded_obstacle_cells moved to ftc/field.py (pure grid geometry, no
# pygame dependency, and ftc/match.py's footprint_overlaps_cells
# collision check now needs the identical function this module already
# used for drawing -- see that function's own docstring for why "what's
# drawn as overlapping" and "what's counted as a collision" being the
# same computation, not two independently-maintained approximations of
# it, is the point). Imported above and left callable as
# `field_view.eroded_obstacle_cells(...)` so nothing downstream
# (scenario_ftc_suites.py, scenario_ftc_bundles.py) has to change.


def draw_obstacles(screen, x0, y0, cell_px, obstacle_cells):
    """`obstacle_cells` -- pass eroded_obstacle_cells(ground_truth)'s
    result (computed once per scenario, not every frame; see
    scenario_ftc_suites.py), not ground_truth.cells directly."""
    inset = max(1, cell_px // 10)
    for row, col in obstacle_cells:
        rect = (x0 + col * cell_px + inset, y0 + row * cell_px + inset,
                 cell_px - 2 * inset, cell_px - 2 * inset)
        pygame.draw.rect(screen, OBSTACLE_FILL, rect, border_radius=max(1, cell_px // 8))
        pygame.draw.rect(screen, OBSTACLE_BORDER, rect, max(1, cell_px // 12), border_radius=max(1, cell_px // 8))


def draw_start_goal(screen, x0, y0, cell_px, start, goal):
    sx, sy = cell_to_px(*start, x0, y0, cell_px)
    gx, gy = cell_to_px(*goal, x0, y0, cell_px)
    pygame.draw.rect(screen, START_COLOR, (sx, sy, cell_px, cell_px), max(2, cell_px // 6))
    pygame.draw.rect(screen, GOAL_COLOR, (gx, gy, cell_px, cell_px), max(2, cell_px // 6))
    # a small flag-like triangle on the goal cell reads better at a
    # glance than a bare colored outline once several panels are small
    fx, fy = gx + cell_px * 0.5, gy + cell_px * 0.5
    r = cell_px * 0.28
    pygame.draw.polygon(screen, GOAL_COLOR, [(fx, fy - r), (fx + r, fy), (fx, fy + r), (fx - r, fy)])


def draw_tag_sites(screen, x0, y0, cell_px, tag_sites):
    for tag in tag_sites:
        row, col = in_to_cell(tag.x_in, tag.y_in)
        cx, cy = x0 + col * cell_px + cell_px // 2, y0 + row * cell_px + cell_px // 2
        r = max(4, int(cell_px * 0.4))
        pygame.draw.rect(screen, TAG_FILL, (cx - r, cy - r, 2 * r, 2 * r), border_radius=max(1, r // 4))
        pygame.draw.rect(screen, TAG_RING, (cx - r, cy - r, 2 * r, 2 * r), max(1, r // 5), border_radius=max(1, r // 4))
        # heading_deg's convention (ftc/sensors.py) is atan2(d_row,
        # d_col): 0deg=+col(screen-right), 90deg=+row(screen-down) -- so
        # the on-screen direction vector is (cos, sin) applied directly
        # to (screen-x, screen-y), matching every other heading_deg
        # consumer in this project.
        rad = math.radians(tag.heading_deg)
        dx, dy = math.cos(rad), math.sin(rad)
        tip = (cx + dx * cell_px * 0.9, cy + dy * cell_px * 0.9)
        pygame.draw.line(screen, TAG_FILL, (cx, cy), tip, max(1, cell_px // 6))


def _wedge_points(cx, cy, heading_deg_now, half_angle_deg, range_px, steps=10):
    points = [(cx, cy)]
    for i in range(steps + 1):
        a = math.radians(heading_deg_now - half_angle_deg + (2 * half_angle_deg) * i / steps)
        points.append((cx + math.cos(a) * range_px, cy + math.sin(a) * range_px))
    return points


def draw_sensor_visual(screen, x0, y0, cell_px, visuals, true_position, heading_deg_now, fidelity):
    """Draws every entry in `visuals` (suite_sensor_visuals(suite)'s
    output) on its own full-screen-sized transparent overlay blitted at
    (0, 0) -- a cone's real range can extend well past this panel's own
    grid when the robot is near an edge, and the caller already clips
    drawing to this panel's rect. A bundle with several active sensor
    types draws several overlays in one call, each already color-coded
    by kind (CONE_COLOR/CAMERA_COLOR), so e.g. odometry pods + AprilTag
    + rear camera reads at a glance as "two blue wedges," not one
    merged, ambiguous shape."""
    cx = x0 + true_position[1] * cell_px + cell_px / 2
    cy = y0 + true_position[0] * cell_px + cell_px / 2
    screen_size = screen.get_size()

    for kind, mount_headings in visuals:
        if kind == "cone":
            range_px = DISTANCE_SENSOR_RANGE_CELLS * cell_px
            overlay = _alpha_surface(screen_size)
            for mount in mount_headings:
                pts = _wedge_points(cx, cy, heading_deg_now + mount, DISTANCE_SENSOR_HALF_ANGLE_DEG, range_px)
                pygame.draw.polygon(overlay, CONE_COLOR, pts)
            screen.blit(overlay, (0, 0))
        elif kind == "camera":
            tier = FIDELITY_TIERS[fidelity]
            fov = tier["camera_fov_deg"]
            if fov >= 360.0:
                continue
            range_px = APRILTAG_RANGE_CELLS * cell_px
            overlay = _alpha_surface(screen_size)
            for mount in mount_headings:
                pts = _wedge_points(cx, cy, heading_deg_now + mount, fov / 2, range_px)
                pygame.draw.polygon(overlay, CAMERA_COLOR, pts)
            screen.blit(overlay, (0, 0))


def draw_newly_seen(screen, x0, y0, cell_px, cells):
    for row, col in cells:
        rect = (x0 + col * cell_px, y0 + row * cell_px, cell_px, cell_px)
        pygame.draw.rect(screen, NEWLY_SEEN_COLOR, rect, max(1, cell_px // 6))


def draw_trail(screen, x0, y0, cell_px, trail):
    if len(trail) < 2:
        return
    points = [(x0 + c * cell_px + cell_px / 2, y0 + r * cell_px + cell_px / 2) for r, c in trail]
    pygame.draw.lines(screen, TRAIL_COLOR, False, points, max(1, cell_px // 8))


PLANNED_PATH_COLOR = (60, 70, 225)
PLANNED_PATH_OUTLINE = (255, 255, 255)


def draw_planned_path(screen, x0, y0, cell_px, path):
    """The suite's CURRENT route to the goal -- drawn as a bold, high-
    contrast line (a light halo underneath a saturated blue, not a
    thin muted one) specifically so it stays readable in real time
    while the match is running, over a busy gray tile floor, at
    whatever panel size is currently on screen."""
    if not path or len(path) < 2:
        return
    points = [(x0 + c * cell_px + cell_px / 2, y0 + r * cell_px + cell_px / 2) for r, c in path]
    pygame.draw.lines(screen, PLANNED_PATH_OUTLINE, False, points, max(3, cell_px // 4))
    pygame.draw.lines(screen, PLANNED_PATH_COLOR, False, points, max(2, cell_px // 6))


def _robot_corners(cx, cy, heading_deg_now, cell_px):
    """The 4 screen-space corners of an actual ROBOT_SIZE_IN x
    ROBOT_SIZE_IN square (exactly 3x3 grid cells at this project's 6in
    cells) centered at (cx, cy), rotated to heading_deg_now -- drawn as
    a real polygon rather than pygame.transform.rotate() specifically
    to avoid that function's rotation-direction sign convention (ccw in
    its own reasoning, which reads as CW on a y-down screen) -- this
    reuses the same directly-verified (cos, sin) = (screen-dx, screen-dy)
    mapping every other heading_deg consumer in this module already
    uses (draw_tag_sites, the sensor-cone wedges)."""
    half = (ROBOT_SIZE_IN / CELL_SIZE_IN / 2.0) * cell_px
    rad = math.radians(heading_deg_now)
    fdx, fdy = math.cos(rad), math.sin(rad)
    rdx, rdy = -fdy, fdx
    return [
        (cx + half * fdx - half * rdx, cy + half * fdy - half * rdy),  # front-left
        (cx + half * fdx + half * rdx, cy + half * fdy + half * rdy),  # front-right
        (cx - half * fdx + half * rdx, cy - half * fdy + half * rdy),  # back-right
        (cx - half * fdx - half * rdx, cy - half * fdy - half * rdy),  # back-left
    ], (fdx, fdy)


def _draw_robot_shape(screen, cx, cy, heading_deg_now, cell_px, fill_color, outline_only, wheel_marks=True):
    corners, (fdx, fdy) = _robot_corners(cx, cy, heading_deg_now, cell_px)
    if outline_only:
        pygame.draw.polygon(screen, fill_color, corners, max(1, cell_px // 10))
    else:
        pygame.draw.polygon(screen, fill_color, corners)
        pygame.draw.polygon(screen, (255, 255, 255), corners, max(1, cell_px // 14))
        # a small forward-pointing notch (like a headlight) so heading
        # reads at a glance even without comparing against the trail
        half = (ROBOT_SIZE_IN / CELL_SIZE_IN / 2.0) * cell_px
        nose = (cx + fdx * half * 1.15, cy + fdy * half * 1.15)
        rdx, rdy = -fdy, fdx
        base_l = (cx + fdx * half * 0.7 - rdx * half * 0.3, cy + fdy * half * 0.7 - rdy * half * 0.3)
        base_r = (cx + fdx * half * 0.7 + rdx * half * 0.3, cy + fdy * half * 0.7 + rdy * half * 0.3)
        pygame.draw.polygon(screen, (255, 255, 255), [nose, base_l, base_r])
        if wheel_marks and cell_px >= 10:
            rdx, rdy = -fdy, fdx
            half = (ROBOT_SIZE_IN / CELL_SIZE_IN / 2.0) * cell_px
            wheel_w, wheel_l = max(2, cell_px // 6), max(3, cell_px // 3)
            for fs in (0.62, -0.62):
                for rs in (0.62, -0.62):
                    wx = cx + fdx * half * fs + rdx * half * rs
                    wy = cy + fdy * half * fs + rdy * half * rs
                    pygame.draw.circle(screen, (20, 20, 20), (int(wx), int(wy)), wheel_w)


def draw_robots(screen, x0, y0, cell_px, snapshot, tag_flash_override=False):
    true_position = snapshot["true_position"]
    heading_deg_now = snapshot["heading_deg"]
    error = snapshot["error"]
    heading_error_deg = snapshot["heading_error_deg"]

    tcx = x0 + true_position[1] * cell_px + cell_px / 2
    tcy = y0 + true_position[0] * cell_px + cell_px / 2
    believed_row = true_position[0] - error[0]
    believed_col = true_position[1] - error[1]
    bcx = x0 + believed_col * cell_px + cell_px / 2
    bcy = y0 + believed_row * cell_px + cell_px / 2

    offset_px = math.hypot(tcx - bcx, tcy - bcy)
    if offset_px > cell_px * 0.1:
        pygame.draw.line(screen, ERROR_VECTOR_COLOR, (tcx, tcy), (bcx, bcy), max(1, cell_px // 14))

    believed_heading = heading_deg_now - heading_error_deg
    _draw_robot_shape(screen, bcx, bcy, believed_heading, cell_px, BELIEVED_ROBOT_COLOR, outline_only=True)

    # Deliberately NOT recolored while stuck (see interp_snapshot's own
    # docstring) -- a robot holding position mid-route should read as
    # "still on its run," the same blue it always is, not as something
    # visibly broken. The HUD's text readout (draw_panel) is where
    # "stuck" actually gets communicated.
    color = TAG_FLASH_COLOR if (snapshot.get("tag_flash") or tag_flash_override) else (20, 120, 220)
    _draw_robot_shape(screen, tcx, tcy, heading_deg_now, cell_px, color, outline_only=False)


# Sum of both robots' half-widths (ROBOT_SIZE_IN/CELL_SIZE_IN/2 each) --
# the axis-aligned-square threshold below which two 18in robots would
# draw as overlapping. A small margin over the exact 3.0 sum since
# neither robot is generally axis-aligned (both can be rotated to any
# heading) -- not a rigorous rotated-rectangle separating-axis test,
# just enough slack that a typical relative rotation still clears.
MIN_ROBOT_SEPARATION_CELLS = (ROBOT_SIZE_IN / CELL_SIZE_IN) + 0.2


def separate_if_overlapping(primary_position, opponent_position, min_separation=MIN_ROBOT_SEPARATION_CELLS):
    """Render-time-only safety net. ftc/match.py's own collision-
    avoidance (OpponentRobot's inflated footprint, see that class's
    docstring) keeps the PRIMARY robot's plan clear of the opponent's
    position at each SIMULATED TICK -- but the two robots' ticks land at
    different simulated moments (the primary's own step timing vs. the
    opponent's fixed per-cell pace), and neither replans continuously
    against the other's position in between. A primary robot sitting
    still for a stretch (waiting on its own next tick) and an opponent
    gliding past during that stretch can still graze mid-interpolation
    even though neither ever violated the grid-level check at the
    moments that check actually ran.

    Nudges the OPPONENT's drawn position directly away from the
    primary's, along the line between them, until the squares just
    clear -- never touches any suite's own true_position, elapsed_s, or
    outcome, only what gets drawn this one frame."""
    pr, pc = primary_position
    orow, ocol = opponent_position
    dr, dc = orow - pr, ocol - pc
    dist = math.hypot(dr, dc)
    if dist >= min_separation:
        return opponent_position
    if dist < 1e-6:
        # Degenerate (near-exact same point) -- push along the robot's
        # own believed-vs-true error direction if there is one, else an
        # arbitrary fixed direction, rather than dividing by ~0.
        dr, dc = 1.0, 0.0
        dist = 1.0
    scale = min_separation / dist
    return (pr + dr * scale, pc + dc * scale)


def draw_opponent(screen, x0, y0, cell_px, cell, heading_deg_now=None):
    """A second robot on the field -- an alliance partner or the
    opposing alliance's robot, drawn at true 18in scale with a facing
    direction (the same _draw_robot_shape every suite's own robot uses,
    just recolored) whenever `heading_deg_now` is known. `heading_deg_
    now=None` (a plain nav.obstacles.MovingObstacle, which has no
    orientation concept -- --opponent's own "static" mode still uses
    this, a stationary robot that's already finished its own auto)
    falls back to a plain undirected square."""
    row, col = cell
    cx, cy = x0 + col * cell_px + cell_px / 2, y0 + row * cell_px + cell_px / 2
    if heading_deg_now is not None:
        _draw_robot_shape(screen, cx, cy, heading_deg_now, cell_px, OPPONENT_COLOR, outline_only=False,
                            wheel_marks=False)
        return
    r = cell_px * 0.42
    rect = (cx - r, cy - r, 2 * r, 2 * r)
    pygame.draw.rect(screen, OPPONENT_COLOR, rect, border_radius=max(1, cell_px // 6))
    pygame.draw.rect(screen, OPPONENT_STRIPE, rect, max(1, cell_px // 8), border_radius=max(1, cell_px // 6))


def draw_collision_mark(screen, x0, y0, cell_px, attempted_position):
    if attempted_position is None:
        return
    row, col = attempted_position
    cx, cy = x0 + col * cell_px + cell_px / 2, y0 + row * cell_px + cell_px / 2
    r = max(4, cell_px // 2 - 2)
    pygame.draw.line(screen, COLLISION_COLOR, (cx - r, cy - r), (cx + r, cy + r), max(2, cell_px // 6))
    pygame.draw.line(screen, COLLISION_COLOR, (cx - r, cy + r), (cx + r, cy - r), max(2, cell_px // 6))
