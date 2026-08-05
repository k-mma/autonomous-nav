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
from nav.sensor import LidarSensor

import ftc.config as config_module
from ftc.field import in_to_cell
from ftc.config import (
    DISTANCE_SENSOR_COST_USD, DISTANCE_SENSOR_COUNT, DISTANCE_SENSOR_HALF_ANGLE_DEG,
    DISTANCE_SENSOR_MOUNT_HEADINGS_DEG, DISTANCE_SENSOR_MOUNT_HEADINGS_BY_COUNT,
    DISTANCE_SENSOR_RANGE_CELLS,
    ODOMETRY_POD_COST_USD, APRILTAG_COST_USD, APRILTAG_RANGE_CELLS, APRILTAG_FOV_DEG,
    APRILTAG_CORRECTION_FACTOR_MAX, APRILTAG_RANGE_DEGRADATION, APRILTAG_ANGLE_DEGRADATION,
    DEAD_RECKONING_DRIFT_PER_CELL, ODOMETRY_DRIFT_PER_CELL,
    CAMERA_MOUNT_HEADINGS_DEG, DUAL_CAMERA_APRILTAG_COST_USD,
    IMU_COST_USD, IMU_HEADING_CORRECTION_FACTOR,
    LIDAR_COST_USD, LIDAR_RANGE_CELLS,
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


def _tag_in_camera_fov(true_position, tag_cell, heading_deg_now, camera_mount_headings_deg, camera_fov_deg):
    """True if `tag_cell` falls inside the robot's own camera FOV
    cone(s) right now, given the robot's TRUE heading -- the gate
    Priority 1(a) found missing entirely: AprilTagSuite.tag_correction
    accepted heading_deg_now but never used it, so a robot detected tags
    regardless of which way its camera actually faced. Separate from
    the tag's own FOV check (is the robot standing somewhere the tag
    can be read from) -- this is "is the robot's camera actually
    pointed at the tag." At camera_fov_deg >= 360 (the optimistic tier)
    this is always True -- an omnidirectional camera, which is exactly
    the assumption ftc/config.py's MODEL_FIDELITY docstring says the
    optimistic tier exists to reproduce unchanged."""
    if camera_fov_deg >= 360.0:
        return True
    angle_robot_to_tag = heading_deg(true_position, tag_cell)
    half = camera_fov_deg / 2.0
    return any(
        abs(angular_diff(angle_robot_to_tag, heading_deg_now + mount)) <= half
        for mount in camera_mount_headings_deg
    )


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
    branches on; a suite can be neither, either, or both. `fixes_heading`
    is a third, independent axis (Priority 4's ImuSuite) -- an IMU
    corrects HEADING error only, continuously, with no need for anything
    to be "in view" the way a tag detection is, and never triggers a
    replan (see ftc/match.py's module docstring)."""
    name = "base"
    cost_usd = 0.0
    integration_notes = ""
    senses_obstacles = False
    fixes_pose = False
    fixes_heading = False
    # Stddev of pose error (cells) injected per cell of real travel --
    # every suite drifts at some rate; only `fixes_pose` suites ever
    # correct it back down.
    drift_per_cell = DEAD_RECKONING_DRIFT_PER_CELL
    # Camera mount heading(s) relative to the robot's own heading, used
    # only by suites that override tag_correction below -- a suite
    # hardware choice (DualCameraAprilTagSuite mounts two), not a
    # fidelity-tier one (ftc/config.py's CAMERA_FOV_DEG_BY_TIER is the
    # tier-level piece: how WIDE each of these mounts can see).
    camera_mount_headings_deg = CAMERA_MOUNT_HEADINGS_DEG

    def make_obstacle_sensor(self):
        return None

    def tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites, rng, fidelity=None):
        """None if no tag is currently visible, else a float in [0, 1]:
        the fraction of accumulated pose error this detection removes
        -- degrades with range and viewing obliquity (see
        AprilTagSuite.tag_correction and ftc/config.py's APRILTAG_*
        constants), plus a little per-detection jitter so repeated
        corrections don't all land identically. `fidelity` (one of
        ftc.config.FIDELITY_TIERS' keys, defaulting to
        ftc.config.MODEL_FIDELITY when None) additionally gates
        visibility on the robot's own camera FOV and can drop an
        otherwise-valid detection -- see AprilTagSuite.tag_correction."""
        return None

    def heading_correction(self, rng):
        """Fraction of accumulated HEADING error this tick's correction
        removes -- only ever called when fixes_heading is True (see
        ImuSuite). 0.0 (no-op) for every suite that doesn't override
        it."""
        return 0.0


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
    # Class attribute, same status as cost_usd/drift_per_cell above (see
    # ftc/robustness.py's docstring for why that matters): the headline
    # 3-sensor layout by default; ftc/coverage_benchmark.py's
    # make_distance_sensor_suite builds a differently-covered variant by
    # overriding this and cost_usd on the INSTANCE, never mutating the
    # class or ftc.config.
    mount_headings_deg = DISTANCE_SENSOR_MOUNT_HEADINGS_DEG

    def make_obstacle_sensor(self):
        return ConeSensor(self.mount_headings_deg, DISTANCE_SENSOR_HALF_ANGLE_DEG,
                            DISTANCE_SENSOR_RANGE_CELLS)


def make_distance_sensor_suite(count):
    """A DistanceSensorSuite variant with `count` ToF sensors at the
    documented mount-heading layout for that count (ftc/config.py's
    DISTANCE_SENSOR_MOUNT_HEADINGS_BY_COUNT) -- ftc/coverage_benchmark.py's
    Priority 3 sweep over {3, 4, 6, 8}. INSTANCE overrides only (cost_usd
    and mount_headings_deg are both class attributes baked at import
    time, exactly the trap ftc/robustness.py's docstring warns about --
    see ftc/scratch/coverage_test.py's check_suite_override_takes_effect,
    which is written to FAIL if this used a class-level or ftc.config
    mutation instead)."""
    if count not in DISTANCE_SENSOR_MOUNT_HEADINGS_BY_COUNT:
        raise ValueError(f"no documented mount-heading layout for count={count}")
    suite = DistanceSensorSuite()
    suite.name = f"distance_sensors_{count}"
    suite.cost_usd = DISTANCE_SENSOR_COST_USD * count
    suite.mount_headings_deg = DISTANCE_SENSOR_MOUNT_HEADINGS_BY_COUNT[count]
    return suite


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

    def tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites, rng, fidelity=None):
        """Correction quality isn't uniform across the detection
        envelope -- it degrades linearly with range (fraction of
        APRILTAG_RANGE_CELLS used) and, more steeply, with viewing
        obliquity (fraction of the FOV half-angle used), both
        independent of whether the tag clears the range/FOV/LOS gates
        at all. `incidence` below is the exact same angle the FOV gate
        just checked -- 0deg is dead-on, APRILTAG_FOV_DEG/2 is the edge
        of the usable cone -- so a detection right at the edge of
        either envelope is technically "in view" but contributes only
        a fraction of APRILTAG_CORRECTION_FACTOR_MAX, not the same
        correction a close, head-on detection would.

        `fidelity` (defaulting to ftc.config.MODEL_FIDELITY, read fresh
        here rather than baked in at import time -- unlike APRILTAG_
        CORRECTION_FACTOR_MAX etc. below, which ARE module globals baked
        at import, see ftc/robustness.py's docstring) adds two gates the
        old model didn't have: the robot's own camera FOV (Priority 1a
        -- _tag_in_camera_fov, gated on heading_deg_now, which used to be
        accepted and silently ignored) and a flat per-detection dropout
        rate. Both are no-ops at the optimistic tier (360deg camera,
        0.0 dropout) so this reproduces the pre-fidelity-tier behavior
        exactly -- see ftc/scratch/fidelity_test.py."""
        fidelity = fidelity or config_module.MODEL_FIDELITY
        tier = config_module.FIDELITY_TIERS[fidelity]
        for tag in tag_sites:
            tag_cell = in_to_cell(tag.x_in, tag.y_in)
            dist = math.hypot(tag_cell[0] - true_position[0], tag_cell[1] - true_position[1])
            if dist > APRILTAG_RANGE_CELLS:
                continue
            angle_tag_to_robot = heading_deg(tag_cell, true_position)
            incidence = abs(angular_diff(angle_tag_to_robot, tag.heading_deg))
            if incidence > APRILTAG_FOV_DEG / 2:
                continue
            if not _tag_in_camera_fov(true_position, tag_cell, heading_deg_now,
                                        self.camera_mount_headings_deg, tier["camera_fov_deg"]):
                continue
            if not line_of_sight(true_grid, tag_cell, true_position):
                continue
            dropout = tier["apriltag_detection_dropout_rate"]
            if dropout > 0 and rng.random() < dropout:
                continue
            range_factor = 1.0 - APRILTAG_RANGE_DEGRADATION * (dist / APRILTAG_RANGE_CELLS)
            angle_factor = 1.0 - APRILTAG_ANGLE_DEGRADATION * (incidence / (APRILTAG_FOV_DEG / 2))
            correction = APRILTAG_CORRECTION_FACTOR_MAX * range_factor * angle_factor
            return min(1.0, max(0.0, correction + rng.gauss(0, 0.05)))
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

    def tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites, rng, fidelity=None):
        return AprilTagSuite.tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites, rng, fidelity)


class ImuSuite(SensorSuite):
    """Every REV Control Hub already ships an integrated IMU -- this
    suite corrects HEADING error only (an IMU has no absolute position
    reference at all, so translation drift is untouched, same rate as
    DeadReckoningSuite) continuously, every tick, with no need for a tag
    -- or anything else -- to be in view, unlike AprilTag's per-
    detection correction. An IMU fix never triggers a replan either
    (ftc/match.py only replans on a POSITION correction or a newly-
    sensed obstacle; a heading-only fix doesn't change which grid cell
    the robot believes it's at, so there's nothing for a fresh astar
    call to find that the old path wouldn't). Hardware cost is
    genuinely $0 -- the question this suite exists to make honest is
    "is the free hardware worth the integration code," which a
    $-per-percentage-point metric can't express at cost_usd == 0 (see
    ftc/*_benchmark.py's explicit handling of that case rather than a
    silent divide-by-zero)."""
    name = "imu"
    cost_usd = IMU_COST_USD
    integration_notes = "The Control Hub's built-in IMU + a fusion loop reading it every tick -- $0 hardware, real firmware/integration effort."
    senses_obstacles = False
    fixes_pose = False
    fixes_heading = True
    drift_per_cell = DEAD_RECKONING_DRIFT_PER_CELL
    heading_correction_factor = IMU_HEADING_CORRECTION_FACTOR

    def heading_correction(self, rng):
        return self.heading_correction_factor


class AprilTagImuSuite(SensorSuite):
    """AprilTag's absolute position correction plus an IMU's continuous
    heading correction -- an IMU-augmented variant of AprilTagSuite. The
    two hardware upgrades correct different error components (position
    vs. heading) and stack cleanly rather than competing, at
    AprilTagSuite's cost alone, since the IMU adds $0."""
    name = "apriltag_imu"
    cost_usd = APRILTAG_COST_USD + IMU_COST_USD
    integration_notes = "AprilTagSuite's webcam pipeline plus the Control Hub's built-in IMU fusion loop -- the IMU adds $0 to AprilTagSuite's existing integration cost."
    senses_obstacles = False
    fixes_pose = True
    fixes_heading = True
    drift_per_cell = DEAD_RECKONING_DRIFT_PER_CELL
    heading_correction_factor = IMU_HEADING_CORRECTION_FACTOR

    def tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites, rng, fidelity=None):
        return AprilTagSuite.tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites, rng, fidelity)

    def heading_correction(self, rng):
        return self.heading_correction_factor


class DualCameraAprilTagSuite(SensorSuite):
    """Two cameras (front + rear, each priced like AprilTagSuite's own
    -- see ftc/config.py's DUAL_CAMERA_APRILTAG_COST_USD) instead of
    AprilTagSuite's one. Under Priority 1's camera-FOV gating this
    roughly doubles the
    robot's angular tag coverage (two camera_fov_deg-wide cones on
    opposite sides of the robot instead of one) -- under the OLD
    omnidirectional-camera model a second camera would have done
    literally nothing (an omnidirectional camera already sees
    everything the first one did), which is exactly the point: this
    suite is a clean demonstration of why the Priority 1 fidelity fix
    mattered, not just "a more expensive AprilTag.\""""
    name = "dual_camera_apriltag"
    cost_usd = DUAL_CAMERA_APRILTAG_COST_USD
    integration_notes = "A second webcam (rear-facing) on the same AprilTag detection pipeline -- double the camera hardware and mounting, same code path run twice per tick."
    senses_obstacles = False
    fixes_pose = True
    drift_per_cell = DEAD_RECKONING_DRIFT_PER_CELL
    camera_mount_headings_deg = [0.0, 180.0]

    def tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites, rng, fidelity=None):
        return AprilTagSuite.tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites, rng, fidelity)


class _OmniLidarSensor(LidarSensor):
    """Adapts nav.sensor.LidarSensor's 2-arg sense(grid, position) to the
    3-arg (grid, position, heading_deg_now) signature every obstacle
    sensor gets called through in ftc/match.py -- a full 360-degree disc
    scan has no heading dependence at all, so this just ignores the
    extra argument rather than requiring nav/ (which must stay FTC-free,
    see README.md's "nav/ vs ftc/" section) to grow an FTC-specific call
    signature."""
    def sense(self, grid, position, heading_deg_now):
        return super().sense(grid, position)


class LidarSuite(SensorSuite):
    """An RPLidar-A1-class 2D scanner -- the direct head-to-head
    Priority 3's coverage sweep exists to run: DistanceSensorSuite's 3
    narrow ToF cones cover only ~75deg of 360deg for a REV-sensor-based
    price (see ftc/config.py's DISTANCE_SENSOR_COST_USD); this is a
    full 360-degree disc scan (nav/sensor.py's LidarSensor, already
    exactly this sensing model -- see module docstring) for a
    comparable price (ftc/config.py's LIDAR_COST_USD)."""
    name = "lidar"
    cost_usd = LIDAR_COST_USD
    integration_notes = ("A single 2D lidar scanner + mount -- one sensor instead of N, no I2C "
                          "multiplexing to wire up. CHECK THE CURRENT SEASON'S FTC GAME MANUAL's "
                          "laser/rules section before treating this as a legal component for a real "
                          "robot -- not asserted here, only priced and simulated.")
    senses_obstacles = True
    fixes_pose = False
    drift_per_cell = DEAD_RECKONING_DRIFT_PER_CELL

    def make_obstacle_sensor(self):
        return _OmniLidarSensor(LIDAR_RANGE_CELLS)


SUITES = {
    "dead_reckoning": DeadReckoningSuite,
    "odometry_pods": OdometryPodSuite,
    "distance_sensors": DistanceSensorSuite,
    "apriltag": AprilTagSuite,
    "full_suite": FullSuite,
    # Not part of SUITE_ORDER / the headline sweep -- Priority 3/4
    # additions, exercised by their own studies (ftc/coverage_
    # benchmark.py, ftc/newsuites_benchmark.py) so the published
    # headline numbers above stay untouched by their presence here.
    "imu": ImuSuite,
    "apriltag_imu": AprilTagImuSuite,
    "dual_camera_apriltag": DualCameraAprilTagSuite,
    "lidar": LidarSuite,
}
SUITE_ORDER = ["dead_reckoning", "odometry_pods", "distance_sensors", "apriltag", "full_suite"]
SUITE_LABELS = {
    "dead_reckoning": "Dead reckoning",
    "odometry_pods": "Odometry pods",
    "distance_sensors": "Distance sensors",
    "apriltag": "AprilTag",
    "full_suite": "Full suite",
    "imu": "IMU",
    "apriltag_imu": "AprilTag + IMU",
    "dual_camera_apriltag": "Dual-camera AprilTag",
    "lidar": "Lidar",
}
