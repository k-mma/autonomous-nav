"""
Five sensor suites an FTC team could actually buy, each a consumer of
the ground-truth grid producing observations that feed nav.sensor.
KnownGrid (reactive replanning) for the suites that sense obstacles.
Reuses nav/sensor.py's LidarSensor as a base where it fits (ConeSensor
below borrows its disc-scan style, restricted to an angular mask) and
never modifies its existing behavior -- every existing LidarSensor
caller keeps working unchanged.

The key modeling point this whole module exists to represent: suites 2
and 4 (OdometryPodSuite, AprilTagSuite) fix POSE error -- how far the
robot's belief about its own location has drifted from where it really
is. Suite 3 (DistanceSensorSuite) fixes OBSTACLE error -- how well the
robot's map of the world matches reality. Which one actually matters in
a given match depends on which kind of deviation dominates, which is
exactly what ftc/suite_benchmark.py's deviation-type sweep (Phase 2's
independent-axis nav/field_variance.py scaling) is built to answer.

Pose error is tracked as a continuous (row, col) vector `error` such
that true_position = believed_position + error -- the same offset
mechanic nav/policies.py's OpenLoopPolicy already uses for a single
fixed start-drift offset, generalized here into a per-tick quantity
that dead-reckoning drift grows and AprilTag detections shrink. See
ftc/match.py for the loop that actually drives this each tick.
"""
import math

from nav.grid import Grid

from ftc.field import in_to_cell
from ftc.config import (
    DISTANCE_SENSOR_COST_USD, DISTANCE_SENSOR_COUNT, DISTANCE_SENSOR_HALF_ANGLE_DEG,
    DISTANCE_SENSOR_MOUNT_HEADINGS_DEG, DISTANCE_SENSOR_RANGE_CELLS,
    ODOMETRY_POD_COST_USD, APRILTAG_COST_USD, APRILTAG_RANGE_CELLS, APRILTAG_FOV_DEG,
    APRILTAG_CORRECTION_FACTOR, DEAD_RECKONING_DRIFT_PER_CELL, ODOMETRY_DRIFT_PER_CELL,
)


def heading_deg(from_cell, to_cell):
    """Direction from `from_cell` to `to_cell`, in the same
    atan2(d_row, d_col) convention ftc/field.py's TagSite.heading_deg
    uses -- 0 is +col, 90 is +row, so every heading in this module means
    the same thing regardless of which piece of code produced it."""
    dr, dc = to_cell[0] - from_cell[0], to_cell[1] - from_cell[1]
    return math.degrees(math.atan2(dr, dc))


def angular_diff(a, b):
    """Smallest signed difference a - b, wrapped into [-180, 180]."""
    d = (a - b + 180) % 360 - 180
    return d


def line_of_sight(grid, cell_a, cell_b):
    """True if no obstacle cell sits strictly between `cell_a` and
    `cell_b` -- a coarse (sampled, not true Bresenham) check, matching
    the level of fidelity nav/sensor.py's own LidarSensor already
    accepts (a plain disc scan with no occlusion at all); this is only
    used for the AprilTag range/FOV check below, which needs *some*
    notion of "behind a wall" but doesn't need to be exact."""
    r0, c0 = cell_a
    r1, c1 = cell_b
    dist = math.hypot(r1 - r0, c1 - c0)
    steps = max(int(dist * 2), 1)
    for i in range(1, steps):
        t = i / steps
        r = round(r0 + (r1 - r0) * t)
        c = round(c0 + (c1 - c0) * t)
        if (r, c) in ((r0, c0), (r1, c1)):
            continue
        if grid.is_valid(r, c) and grid.cells[r][c] == Grid.OBSTACLE:
            return False
    return True


class ConeSensor:
    """Like nav.sensor.LidarSensor's disc scan, but restricted to one or
    more narrow angular cones instead of the full 360-degree radius --
    modeling fixed-mount ToF range sensors that only see obstacles
    directly in front of wherever they're pointed. No occlusion/ray-
    casting (same simplification LidarSensor itself makes): every
    obstacle within range and inside a cone is detected, walls between
    the sensor and it notwithstanding.

    `mount_headings_deg` are relative to the robot's current heading of
    travel (0 = straight ahead); `sense` takes the robot's live heading
    each call rather than a heading fixed at construction, since the
    robot's direction of travel changes throughout a match.
    """

    def __init__(self, mount_headings_deg, half_angle_deg, range_cells):
        self.mount_headings_deg = mount_headings_deg
        self.half_angle_deg = half_angle_deg
        self.range_cells = range_cells
        self.known_obstacles = set()

    def _in_cone(self, angle_to_cell, heading_deg_now):
        for rel in self.mount_headings_deg:
            absolute = heading_deg_now + rel
            if abs(angular_diff(angle_to_cell, absolute)) <= self.half_angle_deg:
                return True
        return False

    def sense(self, grid, position, heading_deg_now):
        """Scan for real obstacles within range and inside any mount's
        cone, from `position` (ground truth). Returns newly-seen cells
        as (row, col) tuples in the SAME frame as `position` -- callers
        that need them shifted into a believed frame (ftc/match.py, to
        correct for pose error) do that shift themselves."""
        row, col = position
        newly_seen = set()
        for dr in range(-self.range_cells, self.range_cells + 1):
            for dc in range(-self.range_cells, self.range_cells + 1):
                if dr == 0 and dc == 0:
                    continue
                if math.hypot(dr, dc) > self.range_cells:
                    continue
                r, c = row + dr, col + dc
                if not grid.is_valid(r, c) or grid.cells[r][c] != Grid.OBSTACLE:
                    continue
                if not self._in_cone(math.degrees(math.atan2(dr, dc)), heading_deg_now):
                    continue
                cell = (r, c)
                if cell not in self.known_obstacles:
                    self.known_obstacles.add(cell)
                    newly_seen.add(cell)
        return newly_seen


class SensorSuite:
    """Common interface every suite below implements. `senses_obstacles`
    and `fixes_pose` are the two independent axes ftc/match.py's loop
    branches on; a suite can be neither, either, or both."""
    name = "base"
    cost_usd = 0.0
    integration_notes = ""
    senses_obstacles = False
    fixes_pose = False
    # Stddev of pose error (cells) injected per cell of real travel --
    # every suite drifts at some rate; only `fixes_pose` suites ever
    # correct it back down.
    drift_per_cell = DEAD_RECKONING_DRIFT_PER_CELL

    def make_obstacle_sensor(self):
        return None

    def tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites, rng):
        """None if no tag is currently visible, else a float in [0, 1]:
        the fraction of accumulated pose error this detection removes
        (APRILTAG_CORRECTION_FACTOR, plus a little per-detection
        jitter so repeated corrections don't all land identically)."""
        return None


class DeadReckoningSuite(SensorSuite):
    """No exteroception at all -- plans once against the assumed map and
    executes blindly, exactly like nav/policies.py's OpenLoopPolicy, but
    with pose error that keeps accumulating for the whole match instead
    of being a single fixed start-of-match offset. This is what a
    typical FTC team's autonomous looks like with nothing beyond built-
    in motor encoders: cheapest possible suite, and the one most exposed
    to both obstacle drift (never sensed) and pose drift (never
    corrected)."""
    name = "dead_reckoning"
    cost_usd = 0.0
    integration_notes = "Built-in motor encoders only -- no extra hardware, no extra wiring or code beyond what every FTC robot already has."
    senses_obstacles = False
    fixes_pose = False
    drift_per_cell = DEAD_RECKONING_DRIFT_PER_CELL


class OdometryPodSuite(SensorSuite):
    """Dead-wheel odometry pods cut pose drift roughly 5x versus encoder-
    only dead reckoning (see ftc/config.py's ODOMETRY_DRIFT_PER_CELL),
    since unpowered/spring-loaded dead wheels don't slip the way driven
    wheels do -- but there's still no exteroception, no absolute
    correction, and no obstacle sensing: the reduced drift rate is the
    entire benefit."""
    name = "odometry_pods"
    cost_usd = ODOMETRY_POD_COST_USD
    integration_notes = "Dead-wheel pods + a pose-tracking library (e.g. RoadRunner/PedroPathing-style localization) -- moderate wiring, no vision pipeline."
    senses_obstacles = False
    fixes_pose = False
    drift_per_cell = ODOMETRY_DRIFT_PER_CELL


class DistanceSensorSuite(SensorSuite):
    """2-4 narrow ToF cones at fixed mounts (DISTANCE_SENSOR_COUNT), each
    seeing obstacles only where it's pointed, ~2m range -- fixes
    OBSTACLE error (lets the robot react to a field that doesn't match
    the assumed layout) but does nothing for POSE error: it still drifts
    at the plain dead-reckoning rate, same as DeadReckoningSuite."""
    name = "distance_sensors"
    cost_usd = DISTANCE_SENSOR_COST_USD * DISTANCE_SENSOR_COUNT
    integration_notes = "N REV 2M distance sensors on fixed mounts + I2C wiring/multiplexing -- straightforward hardware, simple reactive-replan logic."
    senses_obstacles = True
    fixes_pose = False
    drift_per_cell = DEAD_RECKONING_DRIFT_PER_CELL

    def make_obstacle_sensor(self):
        return ConeSensor(DISTANCE_SENSOR_MOUNT_HEADINGS_DEG, DISTANCE_SENSOR_HALF_ANGLE_DEG,
                            DISTANCE_SENSOR_RANGE_CELLS)


class AprilTagSuite(SensorSuite):
    """Periodic absolute pose correction from AprilTag detections -- but
    only when a tag is within FOV cone + range + line of sight (see
    tag_correction below); fixes POSE, not obstacles: it never senses
    the environment, so it plans exactly as blindly as DeadReckoningSuite
    against the assumed map, just from a periodically-corrected position
    estimate instead of a monotonically drifting one."""
    name = "apriltag"
    cost_usd = APRILTAG_COST_USD
    integration_notes = "A dedicated webcam (or the Control Hub's built-in one) + AprilTag detection pipeline (FTC's SDK ships one) -- camera mounting/aiming matters more than the code."
    senses_obstacles = False
    fixes_pose = True
    drift_per_cell = DEAD_RECKONING_DRIFT_PER_CELL

    def tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites, rng):
        for tag in tag_sites:
            tag_cell = in_to_cell(tag.x_in, tag.y_in)
            dist = math.hypot(tag_cell[0] - true_position[0], tag_cell[1] - true_position[1])
            if dist > APRILTAG_RANGE_CELLS:
                continue
            angle_tag_to_robot = heading_deg(tag_cell, true_position)
            if abs(angular_diff(angle_tag_to_robot, tag.heading_deg)) > APRILTAG_FOV_DEG / 2:
                continue
            if not line_of_sight(true_grid, tag_cell, true_position):
                continue
            return min(1.0, max(0.0, APRILTAG_CORRECTION_FACTOR + rng.gauss(0, 0.05)))
        return None


class FullSuite(SensorSuite):
    """Distance sensors + AprilTag + odometry pods together: fixes both
    OBSTACLE error (reactive replanning off the cone sensors) and POSE
    error (odometry's lower base drift rate, periodically re-anchored by
    AprilTag), at the sum of all three suites' cost. The point of
    including it isn't "the expensive suite should obviously win" --
    ftc_suite_writeup.md is where that assumption actually gets
    checked."""
    name = "full_suite"
    cost_usd = DistanceSensorSuite.cost_usd + APRILTAG_COST_USD + ODOMETRY_POD_COST_USD
    integration_notes = "Everything above combined -- the most wiring, the most code paths to integrate and debug, the highest chance something in the pipeline breaks mid-season."
    senses_obstacles = True
    fixes_pose = True
    drift_per_cell = ODOMETRY_DRIFT_PER_CELL

    def make_obstacle_sensor(self):
        return ConeSensor(DISTANCE_SENSOR_MOUNT_HEADINGS_DEG, DISTANCE_SENSOR_HALF_ANGLE_DEG,
                            DISTANCE_SENSOR_RANGE_CELLS)

    def tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites, rng):
        return AprilTagSuite.tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites, rng)


SUITES = {
    "dead_reckoning": DeadReckoningSuite,
    "odometry_pods": OdometryPodSuite,
    "distance_sensors": DistanceSensorSuite,
    "apriltag": AprilTagSuite,
    "full_suite": FullSuite,
}
SUITE_ORDER = ["dead_reckoning", "odometry_pods", "distance_sensors", "apriltag", "full_suite"]
SUITE_LABELS = {
    "dead_reckoning": "Dead reckoning",
    "odometry_pods": "Odometry pods",
    "distance_sensors": "Distance sensors",
    "apriltag": "AprilTag",
    "full_suite": "Full suite",
}
