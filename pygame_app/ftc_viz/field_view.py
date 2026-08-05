"""
Drawing primitives for one suite's animated field panel -- the
RoadRunner-style "robot moving across the field" view pygame_app/
scenarios/scenario_ftc_suites.py arranges N of side by side. Everything
here is a pure function of an already-recorded ftc.trace.MatchTrace and
a tick index; nothing simulates anything (that's ftc/match.py's job) or
holds playback state (that's the scenario file's job) -- this module
only ever answers "given this snapshot, what does it look like."

Two robots are drawn every frame, not one: a solid triangle at the
ground-truth TRUE position/heading, and a lighter dashed outline at the
BELIEVED position/heading (true position minus the accumulated pose-
error vector, true heading minus the accumulated heading-error scalar)
-- connected by a thin line whenever they've diverged enough to notice.
That gap is the entire point of this project's pose-error model
(ftc/match.py's own docstring), and it's the single detail every suite
comparison in this codebase already reduces to a number; drawing it
directly is what makes AprilTag's periodic snap-back, or dead
reckoning's steady drift, visible instead of implied.
"""
import math

import pygame

from nav.sensor import LidarSensor

from ftc.config import (
    APRILTAG_RANGE_CELLS, DISTANCE_SENSOR_HALF_ANGLE_DEG, DISTANCE_SENSOR_MOUNT_HEADINGS_DEG,
    DISTANCE_SENSOR_RANGE_CELLS, FIDELITY_TIERS,
)
from ftc.field import in_to_cell
from ftc.sensors import ConeSensor

WHITE = (255, 255, 255)
BLACK = (30, 30, 30)
GRID_LINE = (200, 200, 200)
START_COLOR = (34, 197, 94)
GOAL_COLOR = (220, 60, 60)
TRAIL_COLOR = (240, 190, 60)
TRUE_ROBOT_COLOR = (20, 120, 220)
BELIEVED_ROBOT_COLOR = (140, 140, 150)
ERROR_VECTOR_COLOR = (220, 60, 60)
TAG_COLOR = (120, 60, 200)
CONE_COLOR = (255, 140, 0, 60)
CAMERA_COLOR = (30, 160, 230, 60)
LIDAR_COLOR = (140, 60, 220, 35)
NEWLY_SEEN_COLOR = (255, 90, 90)
COLLISION_COLOR = (220, 30, 30)
TAG_FLASH_COLOR = (255, 215, 0)


def sensor_kind(suite):
    """Which sensor visualization a suite gets: "cone" (ConeSensor --
    DistanceSensorSuite/FullSuite/the coverage-sweep variants), "lidar"
    (a full 360-degree disc scan -- LidarSuite), "camera" (a suite whose
    tag_correction gates on the robot's own camera FOV -- every
    AprilTag-family suite), or "none" (dead reckoning, odometry pods,
    ImuSuite -- nothing exteroceptive to draw)."""
    if suite.senses_obstacles:
        sensor = suite.make_obstacle_sensor()
        if isinstance(sensor, ConeSensor):
            return "cone"
        if isinstance(sensor, LidarSensor):
            return "lidar"
    if suite.fixes_pose:
        return "camera"
    return "none"


def cell_to_px(row, col, x0, y0, cell_px):
    return x0 + col * cell_px, y0 + row * cell_px


def _alpha_surface(size):
    surf = pygame.Surface(size, pygame.SRCALPHA)
    return surf


def draw_grid(screen, x0, y0, cell_px, ground_truth):
    size = ground_truth.size
    for row in range(size):
        for col in range(size):
            rect = (x0 + col * cell_px, y0 + row * cell_px, cell_px, cell_px)
            color = BLACK if ground_truth.cells[row][col] == 1 else WHITE
            pygame.draw.rect(screen, color, rect)
    if cell_px >= 8:
        for row in range(size + 1):
            y = y0 + row * cell_px
            pygame.draw.line(screen, GRID_LINE, (x0, y), (x0 + size * cell_px, y))
        for col in range(size + 1):
            x = x0 + col * cell_px
            pygame.draw.line(screen, GRID_LINE, (x, y0), (x, y0 + size * cell_px))


def draw_start_goal(screen, x0, y0, cell_px, start, goal):
    sx, sy = cell_to_px(*start, x0, y0, cell_px)
    gx, gy = cell_to_px(*goal, x0, y0, cell_px)
    pygame.draw.rect(screen, START_COLOR, (sx, sy, cell_px, cell_px), 0 if cell_px < 10 else 2)
    pygame.draw.rect(screen, GOAL_COLOR, (gx, gy, cell_px, cell_px), 0 if cell_px < 10 else 2)


def draw_tag_sites(screen, x0, y0, cell_px, tag_sites):
    for tag in tag_sites:
        row, col = in_to_cell(tag.x_in, tag.y_in)
        cx, cy = x0 + col * cell_px + cell_px // 2, y0 + row * cell_px + cell_px // 2
        pygame.draw.circle(screen, TAG_COLOR, (cx, cy), max(3, cell_px // 3))
        # heading_deg's own convention (ftc/sensors.py) is atan2(d_row,
        # d_col): 0deg=+col(screen-right), 90deg=+row(screen-down) -- so
        # the on-screen direction vector is (cos, sin) applied directly
        # to (screen-x, screen-y), matching every other heading_deg
        # consumer in this project rather than a screen-native
        # atan2(dy, dx) that would silently draw every tag facing the
        # wrong way relative to ftc/field.py's own DEFAULT_TAG_SITES.
        rad = math.radians(tag.heading_deg)
        dx, dy = math.cos(rad), math.sin(rad)
        tip = (cx + dx * cell_px * 1.3, cy + dy * cell_px * 1.3)
        pygame.draw.line(screen, TAG_COLOR, (cx, cy), tip, max(1, cell_px // 8))


def _wedge_points(cx, cy, heading_deg_now, half_angle_deg, range_px, steps=10):
    points = [(cx, cy)]
    for i in range(steps + 1):
        a = math.radians(heading_deg_now - half_angle_deg + (2 * half_angle_deg) * i / steps)
        points.append((cx + math.cos(a) * range_px, cy + math.sin(a) * range_px))
    return points


def draw_sensor_visual(screen, x0, y0, cell_px, kind, true_position, heading_deg_now, suite, fidelity):
    """Drawn on a full-screen-sized transparent overlay blitted at
    (0, 0), not a panel-sized one -- a cone's real range (up to
    APRILTAG_RANGE_CELLS/DISTANCE_SENSOR_RANGE_CELLS cells) can extend
    well past this panel's own 24x24 grid when the robot is near an
    edge, and the caller already clips drawing to this panel's rect
    (see scenario_ftc_suites.py), so there's no need to separately
    guess a panel-local surface size that might clip a real cone short."""
    cx = x0 + true_position[1] * cell_px + cell_px // 2
    cy = y0 + true_position[0] * cell_px + cell_px // 2
    screen_size = screen.get_size()

    if kind == "cone":
        mount_headings = getattr(suite, "mount_headings_deg", DISTANCE_SENSOR_MOUNT_HEADINGS_DEG)
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
            return  # omnidirectional -- nothing meaningful to draw as a cone
        mount_headings = getattr(suite, "camera_mount_headings_deg", [0.0])
        range_px = APRILTAG_RANGE_CELLS * cell_px
        overlay = _alpha_surface(screen_size)
        for mount in mount_headings:
            pts = _wedge_points(cx, cy, heading_deg_now + mount, fov / 2, range_px)
            pygame.draw.polygon(overlay, CAMERA_COLOR, pts)
        screen.blit(overlay, (0, 0))
    elif kind == "lidar":
        radius_px = 22 * cell_px  # comfortably covers this project's whole 24x24 grid from any cell
        overlay = _alpha_surface(screen_size)
        pygame.draw.circle(overlay, LIDAR_COLOR, (cx, cy), radius_px)
        screen.blit(overlay, (0, 0))


def draw_newly_seen(screen, x0, y0, cell_px, cells):
    for row, col in cells:
        rect = (x0 + col * cell_px, y0 + row * cell_px, cell_px, cell_px)
        pygame.draw.rect(screen, NEWLY_SEEN_COLOR, rect, max(1, cell_px // 6))


def draw_trail(screen, x0, y0, cell_px, trail):
    if len(trail) < 2:
        return
    points = [(x0 + c * cell_px + cell_px // 2, y0 + r * cell_px + cell_px // 2) for r, c in trail]
    pygame.draw.lines(screen, TRAIL_COLOR, False, points, max(1, cell_px // 8))


def draw_planned_path(screen, x0, y0, cell_px, path):
    if not path or len(path) < 2:
        return
    points = [(x0 + c * cell_px + cell_px // 2, y0 + r * cell_px + cell_px // 2) for r, c in path]
    pygame.draw.lines(screen, (150, 150, 220), False, points, max(1, cell_px // 10))


def _triangle_points(cx, cy, heading_deg_now, size):
    rad = math.radians(heading_deg_now)
    fx, fy = math.cos(rad), math.sin(rad)
    bx, by = -fy, fx  # perpendicular, for the two back corners
    tip = (cx + fx * size, cy + fy * size)
    left = (cx - fx * size * 0.6 + bx * size * 0.7, cy - fy * size * 0.6 + by * size * 0.7)
    right = (cx - fx * size * 0.6 - bx * size * 0.7, cy - fy * size * 0.6 - by * size * 0.7)
    return [tip, left, right]


def draw_robots(screen, x0, y0, cell_px, true_position, heading_deg_now, error, heading_error_deg, tag_flash):
    tcx = x0 + true_position[1] * cell_px + cell_px // 2
    tcy = y0 + true_position[0] * cell_px + cell_px // 2

    believed_row = true_position[0] - error[0]
    believed_col = true_position[1] - error[1]
    bcx = x0 + believed_col * cell_px + cell_px // 2
    bcy = y0 + believed_row * cell_px + cell_px // 2

    offset_px = math.hypot(tcx - bcx, tcy - bcy)
    if offset_px > cell_px * 0.15:
        pygame.draw.line(screen, ERROR_VECTOR_COLOR, (tcx, tcy), (bcx, bcy), max(1, cell_px // 12))

    size = max(4, int(cell_px * 0.55))
    believed_heading = heading_deg_now - heading_error_deg
    pygame.draw.polygon(screen, BELIEVED_ROBOT_COLOR, _triangle_points(bcx, bcy, believed_heading, size), 2)

    color = TAG_FLASH_COLOR if tag_flash else TRUE_ROBOT_COLOR
    pygame.draw.polygon(screen, color, _triangle_points(tcx, tcy, heading_deg_now, size))


def draw_collision_mark(screen, x0, y0, cell_px, attempted_position):
    row, col = attempted_position
    cx, cy = x0 + col * cell_px + cell_px // 2, y0 + row * cell_px + cell_px // 2
    r = max(4, cell_px // 2 - 2)
    pygame.draw.line(screen, COLLISION_COLOR, (cx - r, cy - r), (cx + r, cy + r), max(2, cell_px // 6))
    pygame.draw.line(screen, COLLISION_COLOR, (cx - r, cy + r), (cx + r, cy - r), max(2, cell_px // 6))
