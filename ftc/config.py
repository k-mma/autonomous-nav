"""
FTC domain constants -- the numbers that turn nav/'s domain-neutral grid
and belief-planning machinery into a model of one specific robot on one
specific field, instead of an abstract cell grid. See README.md's
"nav/ vs ftc/" section for why these identifiers are allowed to say
"FTC" and "AprilTag" while nothing under nav/ is.

Every constant below is a real-world estimate with its source noted --
several (turn time, planning overhead, drift rates) don't have a single
official spec and are documented as ballpark engineering estimates
instead. That's an explicit, honest simplification: the point of this
testbed is to be calibrated *closely enough* to real FTC deviation to
make the sensing-investment comparison meaningful, not to be a validated
hardware model.
"""

# Field -- 12ft x 12ft (144in x 144in) square is the FTC standard field
# size across recent seasons (Into The Deep 2024-25, Centerstage 2023-24,
# etc.); see the FTC game manual's field-setup section for the current
# season.
FIELD_SIZE_IN = 144.0
# 6in cells: fine enough to resolve typical FTC game-element footprints
# (scoring zones, poles, submersible walls) without the grid getting
# large enough to slow the sweep down.
CELL_SIZE_IN = 6.0
FTC_GRID_SIZE = round(FIELD_SIZE_IN / CELL_SIZE_IN)  # 24

# Robot -- 18in x 18in x 18in is the FTC game manual's standard
# starting-configuration size limit (most teams build to the limit for
# reach/mechanism room). 18in / 6in cells = 3 cells wide: the whole
# reason ftc/field.py can't plan this robot as a point, unlike every
# grid in nav/.
ROBOT_SIZE_IN = 18.0
ROBOT_FOOTPRINT_CELLS = round(ROBOT_SIZE_IN / CELL_SIZE_IN)  # 3
# Cells from the robot's center to its edge -- the Minkowski-sum hard-
# inflation radius ftc/field.py grows every obstacle by so a point-robot
# plan on the inflated grid is exactly equivalent to a real-footprint
# plan on the original one (see ftc/field.py's build_grid).
ROBOT_RADIUS_CELLS = ROBOT_FOOTPRINT_CELLS // 2  # 1

# Autonomous period length, per the FTC game manual.
AUTONOMOUS_PERIOD_S = 30.0

# Top drive speed -- ballpark for a geared 4-motor FTC drivetrain (e.g.
# goBILDA 5203-series motors around 300-450 RPM output, 4in mecanum/
# traction wheels): commonly cited team drivetrain calculators put this
# around 4.5-5.5 ft/s, so 1.5 m/s (~4.9 ft/s) sits in the middle of that
# range.
MAX_DRIVE_SPEED_MPS = 1.5
# In-place rotation rate for a tank/mecanum FTC drivetrain is commonly
# reported in the 200-250 deg/s range at typical gearing -- 90 degrees
# in roughly 0.36-0.45s. Used as a flat per-90-degree-turn cost rather
# than modeling acceleration.
TURN_TIME_PER_90DEG_S = 0.4
# Straight-line acceleration -- no FTC-official spec exists (this varies
# by gearing, wheel choice, and robot mass), so this is a ballpark
# engineering estimate sized the same way TURN_TIME_PER_90DEG_S already
# is: 0-to-MAX_DRIVE_SPEED_MPS in roughly half a second (1.5 / 3.0) for
# a geared 4-motor mecanum/traction drivetrain, the same rough
# acceleration-time-scale as this project's turn-rate estimate rather
# than a separately-justified number. Used for a per-step trapezoidal
# (or, when a step is too short to reach cruise speed, triangular)
# velocity profile in ftc/match.py instead of assuming a robot reaches
# MAX_DRIVE_SPEED_MPS instantaneously.
MAX_ACCEL_MPS2 = 3.0
# Per-replan control-loop overhead on FTC-legal onboard compute (a REV
# Control Hub): sensor read + odometry fusion + the search itself, not
# just the raw grid search (which is sub-millisecond in Python and would
# understate what a real re-plan actually costs mid-match).
PLANNING_OVERHEAD_S = 0.05

# Sensor suite costs -- ballpark 2024-25 street prices from common FTC
# vendor sources (REV Robotics, goBILDA); meant to give a defensible
# relative ordering across suites; not a procurement quote.
ODOMETRY_POD_COST_USD = 100.0  # dead-wheel odometry pod set + mounting
DISTANCE_SENSOR_COST_USD = 30.0  # per REV 2M distance sensor (2m ToF)
DISTANCE_SENSOR_COUNT = 3  # narrow ToF cones at fixed mounts (spec: 2-4)
APRILTAG_COST_USD = 40.0  # marginal cost of a dedicated webcam (~$30-40)

# Distance sensor (ToF) geometry -- REV's 2M sensor's headline range is
# ~2m; VL53L0X-class ToF sensors have a narrow beam, commonly quoted
# around 25 degrees full width (half-angle ~12.5deg) -- "sees obstacles
# only where pointed," not a lidar disc.
DISTANCE_SENSOR_RANGE_CELLS = round((2.0 / 0.0254) / CELL_SIZE_IN)  # ~13 cells
DISTANCE_SENSOR_HALF_ANGLE_DEG = 12.5
# Mount headings for DISTANCE_SENSOR_COUNT sensors, relative to the
# robot's heading of travel -- front, left, right covers the direction
# of motion plus both flanks, which is the usual FTC mounting pattern
# for exactly 3 range sensors.
DISTANCE_SENSOR_MOUNT_HEADINGS_DEG = [0.0, -90.0, 90.0]

# Pose drift -- stddev of pose error (in cells) injected per cell of
# real travel, i.e. a random walk driven by distance traveled rather
# than elapsed time, matching wheel-encoder slip's real cause (it
# accumulates with wheel rotations, not with the clock).
#
# DEAD_RECKONING: encoder-only dead reckoning. Chosen so that over a
# full-field ~24-cell traverse, 1-sigma accumulated error lands around
# several cells (a foot-plus) by the end of a long path -- consistent
# with commonly reported FTC dead-reckoning-only drift.
DEAD_RECKONING_DRIFT_PER_CELL = 0.15
# ODOMETRY: dedicated dead-wheel odometry pods are unpowered/spring-
# loaded and don't slip the way driven wheels do -- roughly 5x better
# than encoder-only dead reckoning in commonly reported team experience.
ODOMETRY_DRIFT_PER_CELL = 0.03

# AprilTag absolute pose correction.
# Reliable detection range for a webcam/Limelight-class FTC vision setup
# at typical field lighting, ~6ft.
APRILTAG_RANGE_CELLS = round(72.0 / CELL_SIZE_IN)  # 12 cells
# Combined camera-FOV / tag-readability usable detection cone (webcam
# FOV is commonly ~60-78deg; tag readability angle is the tighter
# constraint at this range).
APRILTAG_FOV_DEG = 60.0
#
# Correction quality degrades with range and with viewing obliquity
# (incidence angle off the tag's surface normal) -- real fiducial pose
# estimation isn't uniformly good everywhere inside the nominal
# range/FOV envelope: a tag's projected pixel area shrinks with
# distance (fewer pixels -> noisier corner localization -> noisier
# pose), and shrinks again under foreshortening as the viewing angle
# gets more oblique (a tag viewed edge-on has far less usable corner
# geometry than one viewed head-on), both independent of whether the
# tag is technically still "detected" at all. These are documented
# engineering estimates (same status as every other constant in this
# file), not measurements of a real detector's error curve -- ftc/
# calibration.py has no odometry-camera-specific calibration input for
# this yet, only for drift-per-cell.
#
# Correction factor at the ideal case (close range, dead-on viewing
# angle) -- still not a full reset, since even a great detection has
# residual vision-pipeline error.
APRILTAG_CORRECTION_FACTOR_MAX = 0.90
# Fraction of APRILTAG_CORRECTION_FACTOR_MAX lost going from 0 range to
# APRILTAG_RANGE_CELLS (linear falloff).
APRILTAG_RANGE_DEGRADATION = 0.5
# Fraction of APRILTAG_CORRECTION_FACTOR_MAX lost going from dead-on
# (0deg incidence) to the edge of the usable FOV (APRILTAG_FOV_DEG/2
# incidence) -- steeper than the range falloff, since foreshortening
# degrades corner localization faster than distance alone does.
APRILTAG_ANGLE_DEGRADATION = 0.6

# Opponent-robot repositioning cadence for the "moving_blocker" deviation
# type (ftc/opponent_benchmark.py's nav/obstacles.py MovingObstacle
# integration) -- how often, in simulated match milliseconds, an
# unpredictable opponent/alliance robot's field position meaningfully
# changes. No FTC-specific spec exists for this (it's a property of the
# *other* team's driving, not this robot), so it reuses nav/obstacles.py's
# own MovingObstacle default (already tuned, in the pygame visualizer, for
# a cadence that reads as "visibly moving" without being chaotic) rather
# than inventing a separate, equally-unjustified number.
OPPONENT_REPOSITION_PERIOD_MS = 700

# === Model fidelity tiers ===============================================
# Two unmodeled optimisms flattered every suite in this project until
# now: (a) AprilTagSuite.tag_correction accepted heading_deg_now and
# never used it -- the robot detected tags regardless of which way its
# camera actually faced; (b) ftc/match.py tracked pose error as a
# (row, col) translation vector only, with no heading error anywhere --
# the robot always knew exactly which way it pointed, even though small
# angular error compounding into large lateral error over distance is
# the dominant real dead-reckoning failure mode. MODEL_FIDELITY picks
# which tier's assumptions ftc/sensors.py and ftc/match.py use.
# "optimistic" is the default specifically so every existing caller that
# doesn't explicitly ask for a tier keeps reproducing the published
# headline numbers (56/45/35/21/19% success rates, 40.0 vs. 16.2
# pp/$100) trial-for-trial -- see ftc/scratch/fidelity_test.py's
# check_optimistic_tier_reproduces_headline_exactly, the regression
# guarantee for those figures.
MODEL_FIDELITY = "optimistic"

# Camera field of view (full angle, degrees) -- how wide a cone the
# robot's own onboard camera can see, centered on wherever the robot's
# TRUE heading currently points (ftc/sensors.py's _tag_in_camera_fov).
# This is a separate gate from APRILTAG_FOV_DEG above, which is the
# TAG's own readable cone (is the robot standing somewhere the tag can
# be read from); a detection now needs to clear both -- the tag has to
# be readable from where the robot is standing, AND the robot's camera
# has to actually be pointed at it.
CAMERA_FOV_DEG_BY_TIER = {
    "optimistic": 360.0,  # omnidirectional -- reproduces pre-fidelity-tier behavior exactly
    # A real Control-Hub-webcam FOV is commonly ~60-78deg (the same
    # ballpark APRILTAG_FOV_DEG already cites) -- 70 sits in the middle.
    "realistic": 70.0,
    # A narrower and/or worse-aimed camera than the realistic estimate --
    # ballpark engineering estimate, not sourced to a specific product.
    "pessimistic": 50.0,
}
# Camera mount heading(s), relative to the robot's own heading (0 =
# straight ahead) -- a single forward-facing webcam is the common FTC
# choice. A suite that mounts more than one camera (DualCameraAprilTag
# Suite) overrides this on the suite class itself, since that's a suite
# hardware choice, not a fidelity-tier one.
CAMERA_MOUNT_HEADINGS_DEG = [0.0]

# Heading (yaw) drift -- stddev of heading error, in degrees, injected
# per cell of real travel -- the rotational analogue of DEAD_RECKONING_
# DRIFT_PER_CELL/ODOMETRY_DRIFT_PER_CELL above. Ballpark: gyro-free yaw
# integration from wheel odometry alone commonly drifts on the order of
# single-digit degrees over a multi-foot run; scaled per grid cell the
# same way the existing translation drift rates are, so both error
# sources compound consistently with each other one cell at a time.
# 0.0 at the optimistic tier -- no heading error at all, the second of
# the two unmodeled optimisms this fidelity system exists to bound.
HEADING_DRIFT_DEG_PER_CELL_BY_TIER = {
    "optimistic": 0.0,
    "realistic": 0.6,
    "pessimistic": 1.5,
}

# Fraction of accumulated HEADING error a single successful AprilTag
# detection removes -- the rotational analogue of APRILTAG_CORRECTION_
# FACTOR_MAX, but applied as a flat fraction whenever a detection fires
# (not further degraded by range/angle the way the position correction
# is) -- a documented simplification, not a claim that heading and
# position correction quality degrade identically across the detection
# envelope; a real AprilTag pose solve returns full 6-DOF (position +
# orientation) from one detection, so correcting both from the same
# event is the physically correct shape, even if this factor's exact
# value is a ballpark. 0.0 at the optimistic tier is moot -- heading
# error is already always 0 there.
APRILTAG_HEADING_CORRECTION_FACTOR_BY_TIER = {
    "optimistic": 0.0,
    "realistic": 0.85,
    "pessimistic": 0.7,
}

# Fraction of otherwise-geometrically-valid AprilTag detections (in
# range, in the tag's own FOV, in the robot's camera FOV, unoccluded)
# that simply fail to resolve a pose this tick anyway -- motion blur, a
# corner partly out of frame, a bad exposure. Ballpark. 0.0 at both
# optimistic and realistic -- "realistic" means a real (non-
# omnidirectional) camera and real heading error, but every
# geometrically valid detection still works; "pessimistic" goes one
# step further and admits detections can still just fail.
APRILTAG_DETECTION_DROPOUT_RATE_BY_TIER = {
    "optimistic": 0.0,
    "realistic": 0.0,
    "pessimistic": 0.15,
}

FIDELITY_TIERS = {
    tier: dict(
        camera_fov_deg=CAMERA_FOV_DEG_BY_TIER[tier],
        heading_drift_deg_per_cell=HEADING_DRIFT_DEG_PER_CELL_BY_TIER[tier],
        apriltag_heading_correction_factor=APRILTAG_HEADING_CORRECTION_FACTOR_BY_TIER[tier],
        apriltag_detection_dropout_rate=APRILTAG_DETECTION_DROPOUT_RATE_BY_TIER[tier],
    )
    for tier in ("optimistic", "realistic", "pessimistic")
}

# === Drivetrain (tank vs. mecanum) ======================================
# Orthogonal to sensor suite -- see ftc/drivetrain.py. goBILDA-class
# wheel-set street prices, ballpark 2024-25 (same sourcing caveat as the
# sensor costs above: a defensible relative ordering, not a procurement
# quote).
TANK_WHEEL_COST_USD = 80.0      # 4x traction wheel set + mounting hardware
MECANUM_WHEEL_COST_USD = 200.0  # 4x mecanum wheel set + mounting hardware

# A mecanum robot can translate in any direction without turning, but
# only at full speed while driving "forward" relative to whatever
# heading it's holding -- strafing (moving sideways/diagonally relative
# to that held heading) is slower, since the wheels' rollers are doing
# more of the sideways work than the motors are doing forward work.
# Ballpark engineering estimate (no FTC-official spec for any specific
# wheel), not a measured figure.
MECANUM_STRAFE_SPEED_FACTOR = 0.8
# Strafing also drifts more than driving straight -- mecanum rollers
# scrub sideways against the floor in a way a traction wheel driving
# straight doesn't, so encoder-based position estimates degrade faster
# during a strafe. > 1 by construction; ballpark, not measured.
MECANUM_STRAFE_DRIFT_MULTIPLIER = 1.6

# === Sensor coverage sweep ===============================================
# DISTANCE_SENSOR_COUNT/DISTANCE_SENSOR_MOUNT_HEADINGS_DEG above stay the
# headline study's fixed 3-sensor layout; this is the swept range used
# by ftc/coverage_benchmark.py to ask "can you buy your way out of the
# blind-spot problem the headline study found?" Mount-heading rationale
# documented per count -- even coverage vs. front-weighted is itself a
# real design choice, not a detail:
#  3 (existing): front/left/right -- front-weighted, prioritizes the
#    direction of travel over the rear, the usual FTC 3-sensor pattern.
#  4: front/left/right/back -- the natural even extension of 3, first
#    count that covers all four cardinal directions.
#  6: even 60deg spacing around the full circle -- once there are enough
#    sensors to bother, spreading them evenly stops costing much extra
#    over a front-weighted layout and closes the remaining blind arcs
#    evenly instead of picking a side to leave open.
#  8: even 45deg spacing -- matches the grid's own 8-directional move
#    set, so every cardinal + diagonal direction of travel has a sensor
#    pointed straight down it.
DISTANCE_SENSOR_COUNTS_SWEPT = [3, 4, 6, 8]
DISTANCE_SENSOR_MOUNT_HEADINGS_BY_COUNT = {
    3: [0.0, -90.0, 90.0],
    4: [0.0, -90.0, 90.0, 180.0],
    6: [i * 60.0 for i in range(6)],
    8: [i * 45.0 for i in range(8)],
}

# RPLidar-A1-class 2D scanner, ballpark 2024-25 street price -- a full
# 360-degree disc scan, which nav/sensor.py's LidarSensor already models
# exactly (see ftc/sensors.py's LidarSuite). NOTE: FTC's laser-class-
# device rules must be checked against the CURRENT season's game manual
# before treating this as a real, legal recommendation for a team to
# buy -- this repo prices and simulates the sensing model, it does not
# assert legality.
LIDAR_COST_USD = 100.0
# An RPLidar A1's spec range (commonly ~12m) comfortably exceeds this
# field's own diagonal (144in x 144in = ~5.2m) -- set to safely exceed
# the 24x24 grid's own ~34-cell diagonal so the sensor model reads as
# "sees the whole field," not artificially range-limited below its real
# spec.
LIDAR_RANGE_CELLS = FTC_GRID_SIZE * 2

# === New suites enabled by the fidelity-tier model ======================
# Every REV Control Hub already ships an integrated IMU -- this suite's
# hardware cost is genuinely $0; the only real cost is the integration
# effort of reading and fusing it, which this project's dollar-based
# cost model has no way to price (see ftc/sensors.py's ImuSuite and
# ftc/*_benchmark.py's explicit handling of cost_usd == 0.0 -- pp/$100
# is undefined there, not infinite, and has to be reported as such
# rather than divided-by-zero).
IMU_COST_USD = 0.0
# Fraction of accumulated HEADING error one IMU fusion cycle removes,
# applied every tick (no need for a tag, or anything else, to be in
# view) -- separate from AprilTag's own per-detection heading
# correction above. Ballpark: a well-fused IMU heading estimate is
# usually tighter than a single vision-based pose solve (no corner-
# localization noise), so this sits above the realistic-tier AprilTag
# heading-correction factor.
IMU_HEADING_CORRECTION_FACTOR = 0.9

# Two cameras (front + rear) instead of AprilTag's one, ~$40 each --
# ftc/sensors.py's DualCameraAprilTagSuite.
DUAL_CAMERA_APRILTAG_COST_USD = APRILTAG_COST_USD * 2

# === Drivetrain speed / gearing (optional, Priority 5) ==================
# MAX_DRIVE_SPEED_MPS/MAX_ACCEL_MPS2 above are constants, but ftc/
# budget_benchmark.py found the 30s budget genuinely starts binding
# around 15-20s under the trapezoidal kinematics model -- which makes a
# faster-motor purchase an actually testable question for the first
# time (a robot that never runs out of time has nothing to gain from
# more speed). goBILDA/REV both sell higher-speed, lower-torque gearing
# options for the same 5203-series-class motor at a real price premium;
# GEARING_OPTIONS below is a small, explicit menu of them, keyed by
# name, each overriding max_speed_mps/max_accel_mps2 and adding a
# real tradeoff a faster-is-just-better model would hide: more speed
# means more wheel slip, so `slip_factor` scales `drift_per_cell`
# UP along with speed, rather than being a free win. "stock" reproduces
# MAX_DRIVE_SPEED_MPS/MAX_ACCEL_MPS2/no-slip-penalty exactly -- ftc/
# match.py's `gearing=None` default resolves to "stock", so every
# existing caller is unaffected (see ftc/scratch/gearing_test.py's
# regression check).
#
# Speed/accel multipliers and slip factors are ballpark engineering
# estimates (no FTC-official spec for any specific gearing swap, same
# status as MAX_ACCEL_MPS2 itself); prices are the same ballpark
# 2024-25 street-price sourcing as every other cost in this file, for a
# 4-motor gearing swap (not a full drivetrain replacement -- wheels/
# mounts are assumed unchanged).
GEARING_OPTIONS = {
    "stock": dict(max_speed_mps=MAX_DRIVE_SPEED_MPS, max_accel_mps2=MAX_ACCEL_MPS2,
                   cost_usd=0.0, slip_factor=1.0),
    # A higher-speed-rated motor option (e.g. goBILDA 6000-series-class
    # "Yellow Jacket" swapped to a faster ratio) -- roughly 1.4x top
    # speed, a more modest 1.15x accel bump (higher speed, not
    # proportionally higher torque), and a real wheel-slip cost.
    "fast": dict(max_speed_mps=MAX_DRIVE_SPEED_MPS * 1.4, max_accel_mps2=MAX_ACCEL_MPS2 * 1.15,
                  cost_usd=60.0, slip_factor=1.35),
    # A further, more aggressive gearing swap -- diminishing accel
    # returns (torque drops faster than speed rises at this end of a
    # typical DC gearmotor's curve) and a correspondingly larger slip
    # penalty.
    "faster": dict(max_speed_mps=MAX_DRIVE_SPEED_MPS * 1.8, max_accel_mps2=MAX_ACCEL_MPS2 * 1.25,
                    cost_usd=120.0, slip_factor=1.8),
}
GEARING_ORDER = ["stock", "fast", "faster"]
GEARING_LABELS = {"stock": "Stock gearing", "fast": "Fast gearing", "faster": "Faster gearing"}

# math.radians(APRILTAG_FOV_DEG) etc. computed on demand where needed;
# nothing below this line is a tunable, just a derived convenience.
INCHES_PER_METER = 1.0 / 0.0254
