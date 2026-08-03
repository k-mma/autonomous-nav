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
# Fraction of accumulated pose error removed by one successful tag
# detection -- not a full reset, since vision-based pose estimation has
# its own residual error.
APRILTAG_CORRECTION_FACTOR = 0.85

# math.radians(APRILTAG_FOV_DEG) etc. computed on demand where needed;
# nothing below this line is a tunable, just a derived convenience.
INCHES_PER_METER = 1.0 / 0.0254
