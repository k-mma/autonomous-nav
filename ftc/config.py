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

# Field -- VERIFIED against the FTC Competition Manual's ARENA section
# and ftc-docs' Playing Field Resources: the playing field is 12ft x
# 12ft (3.66m x 3.66m), built from thirty-six (36) foam tiles of
# approximately 24in x 24in (610mm x 610mm) each, 6x6. 6 x 24in =
# 144in, so the nominal figure below is exact by construction.
#
# One caveat the manual states explicitly and this model does NOT
# represent: the field wall's actual INSIDE dimension varies by which
# manufacturer's perimeter and tiles an event uses, and that variation
# shifts game-element locations and the gap between the outermost tiles
# and the wall. The manual gives no single authoritative inside
# dimension for that reason. 144in is the nominal design figure, not a
# measured one -- a real surveyed field will differ by roughly a tile
# tolerance.
FIELD_SIZE_IN = 144.0
# 6in cells: fine enough to resolve typical FTC game-element footprints
# (scoring zones, poles, submersible walls) without the grid getting
# large enough to slow the sweep down. Also divides the 24in field tile
# exactly 4 ways, so a cell boundary never falls partway across a tile.
CELL_SIZE_IN = 6.0
FTC_GRID_SIZE = round(FIELD_SIZE_IN / CELL_SIZE_IN)  # 24

# Robot -- VERIFIED against the 2025-26 DECODE Competition Manual's
# ROBOT Construction Rules (section 12): the starting configuration must
# fit within an 18in x 18in x 18in cube (most teams build to the limit
# for reach/mechanism room). Notably for this model, DECODE also caps
# HORIZONTAL expansion at 18in x 18in for the whole match (vertical
# expansion is allowed up to 38in), so an 18in footprint is correct for
# the entire autonomous period, not just its first instant -- which is
# what makes a single fixed footprint-inflation radius (below) a fair
# model rather than a start-of-match-only one.
# 18in / 6in cells = 3 cells wide: the whole reason ftc/field.py can't
# plan this robot as a point, unlike every grid in nav/.
ROBOT_SIZE_IN = 18.0
ROBOT_FOOTPRINT_CELLS = round(ROBOT_SIZE_IN / CELL_SIZE_IN)  # 3
# Cells from the robot's center to its edge -- the Minkowski-sum hard-
# inflation radius ftc/field.py grows every obstacle by so a point-robot
# plan on the inflated grid is exactly equivalent to a real-footprint
# plan on the original one (see ftc/field.py's build_grid).
ROBOT_RADIUS_CELLS = ROBOT_FOOTPRINT_CELLS // 2  # 1

# Autonomous period length -- VERIFIED: the FTC Competition Manual
# defines a Match as a 30-second Autonomous Period, an 8-second
# transition, then a 2-minute Driver-Controlled Period. This project
# only models the first of those three.
AUTONOMOUS_PERIOD_S = 30.0

# Top drive speed -- now DERIVED from real published hardware specs
# rather than "commonly cited team drivetrain calculators." The
# reference drivetrain is the most common FTC configuration: four
# goBILDA 5203-series Yellow Jacket motors at the 19.2:1 ratio
# (published no-load output 312 RPM) driving 96mm goBILDA wheels
# (mecanum set or Hogback traction, both 96mm).
#
#   312 RPM / 60 = 5.2 rev/s
#   96mm wheel circumference = pi * 0.096m = 0.3016m
#   5.2 * 0.3016 = 1.568 m/s no-load (~5.1 ft/s)
#
# 1.5 m/s is that figure rounded slightly down, since a loaded
# drivetrain never reaches its no-load speed. The other common FTC
# drivetrain ratio, 13.7:1 (435 RPM), works out to 2.19 m/s by the same
# arithmetic -- see GEARING_OPTIONS at the bottom of this file, which
# models exactly that swap as a purchasable upgrade.
MAX_DRIVE_SPEED_MPS = 1.5
# In-place rotation rate -- also derived rather than asserted. A robot
# spinning in place drives its wheels at some tangential speed v about
# its own center; for a wheel at radius r from that center, the angular
# rate is v/r. At the 18in-cube size limit the drive wheels sit roughly
# 9in (0.23m) out from center on each side, so at the 1.5 m/s above:
#
#   1.5 / 0.23 = 6.5 rad/s = 374 deg/s -> 90 degrees in ~0.24s
#
# That is the frictionless upper bound. 0.4s (225 deg/s) is used
# instead, deliberately conservative: a real in-place turn scrubs all
# four wheels sideways across foam tiles, never reaches the theoretical
# rate, and has to accelerate and stop within the turn. No FTC-official
# spec exists for turn rate (it depends on drivetrain geometry and
# wheel choice), so this remains an engineering estimate -- but one
# bracketed by a derived bound rather than a bare guess.
TURN_TIME_PER_90DEG_S = 0.4
# Straight-line acceleration -- no FTC-official spec exists (it depends
# on gearing, wheel choice, robot mass, and floor traction), so this
# stays an engineering estimate. It is, however, bounded on both sides
# rather than picked freely:
#  - Motor-torque ceiling: four 5203 motors at 19.2:1 produce 338 oz-in
#    (2.39 N-m) each; across a 48mm wheel radius that is ~50N per motor,
#    ~199N total, which on a ~15kg (33lb) competition robot would be
#    ~13 m/s^2 -- far above what's usable.
#  - Traction ceiling: the real limit is friction, not torque. Even at
#    a generous coefficient of friction of ~1.0 against foam tiles, the
#    robot cannot accelerate faster than ~9.8 m/s^2 without the wheels
#    simply slipping.
# 3.0 m/s^2 (0-to-1.5 m/s in half a second) sits well under both, which
# is the honest place for it: an FTC drivetrain accelerating hard is
# traction-limited and slip-limited, not torque-limited. Used for the
# per-step trapezoidal (or, when a step is too short to reach cruise
# speed, triangular) velocity profile in ftc/match.py instead of
# assuming the robot reaches MAX_DRIVE_SPEED_MPS instantaneously.
MAX_ACCEL_MPS2 = 3.0
# Per-replan control-loop overhead on FTC-legal onboard compute (a REV
# Control Hub): sensor read + odometry fusion + the search itself, not
# just the raw grid search (which is sub-millisecond in Python and would
# understate what a real re-plan actually costs mid-match).
PLANNING_OVERHEAD_S = 0.05

# === Collision recovery (optional -- ftc/match.py's on_collision="replan") ==
# The default collision behavior everywhere in this project (ftc/match.py's
# on_collision="halt", every existing benchmark's own call) treats a
# collision as the end of the match -- which is what every published
# number in this repo already measures, and stays unchanged. "replan" is
# an opt-in alternative, used by the pygame_app/ftc_viz/ visualizer by
# default (a real FTC autonomous program commonly has stall-detection/
# retry logic of exactly this shape -- encoders stop advancing, back off,
# replan, try again -- rather than a robot that simply powers through a
# wall or gives up entirely on first contact): a bump is treated as a
# generic stall (detectable via motor encoder feedback, which every FTC
# robot has, independent of which sensor suite it's running), the
# attempted cell is marked blocked in the robot's own belief, and it
# replans around it and keeps going -- up to MAX_STALL_RETRIES consecutive
# failed attempts before genuinely giving up (the same terminal outcome
# "halt" always produces, just reached after trying rather than
# immediately). See ftc/scratch/collision_recovery_test.py.
#
# Recovery time charged per stall -- back off, re-read encoders/sensors,
# replan (PLANNING_OVERHEAD_S is charged separately, on top of this, by
# the replan itself) -- no FTC-official spec, ballpark engineering
# estimate sized the same rough way TURN_TIME_PER_90DEG_S is.
COLLISION_RECOVERY_S = 0.3
# How many consecutive failed attempts at a stall before giving up --
# ballpark: enough that a suite with a real way to route around the
# obstacle (one that senses obstacles, or gets a pose correction that
# shifts its plan) almost always succeeds on its very next attempt, but
# not so many that a suite with no way to perceive anything new (it
# would replan the identical failing path every time, deterministically)
# burns a large, unrealistic amount of simulated match time repeating
# the same failed move.
MAX_STALL_RETRIES = 3

# Sensor suite costs -- VERIFIED against current vendor listings
# (REV Robotics, goBILDA, Logitech retail), not ballpark estimates.
# Each is a real listed price for a real part a team would actually
# buy; still not a procurement quote (no tax, shipping, spares, or the
# motors/hubs every suite shares).

# goBILDA Odometry Pack -- two odometry pods plus one Pinpoint odometry
# computer, listed at $279.99 as a bundle (Swingarm and 4-Bar packs are
# the same price). Pods are $99.99 EACH separately and the Pinpoint V2
# computer is $79.99, so the pack is the cheapest real path to working
# dead-wheel odometry, which is why it's priced here rather than a
# single pod.
#
# CORRECTION: this was previously $100.0, described as a "pod set +
# mounting." That was wrong by nearly 3x -- $100 is roughly ONE pod,
# not a working two-pod setup with the computer needed to fuse them.
# Fixing it materially changes this project's reliability-per-dollar
# headline; see README.md's "Threats to validity" and
# benchmark_results/ftc_suite_writeup.md.
ODOMETRY_POD_COST_USD = 279.99
# REV 2m Distance Sensor (REV-31-1505), listed at $31.50.
DISTANCE_SENSOR_COST_USD = 31.50
DISTANCE_SENSOR_COUNT = 3  # narrow ToF cones at fixed mounts (spec: 2-4)
# Marginal cost of a dedicated webcam for AprilTag detection. FTC's own
# VisionPortal documentation lists four cameras that ship with built-in
# lens calibrations usable for AprilTag POSE estimation (without a
# calibration, a camera can still see a tag but can't reliably solve
# its pose): Logitech C270 (~$25), Logitech C310, Microsoft LifeCam
# HD-3000, and Logitech C920 (~$126). The C270 is the one FTC's docs
# call the workhorse webcam for the program, so its price is used here;
# the real range a team might spend is roughly $25-$126.
#
# CORRECTION: previously $40.0 ("~$30-40"), which matched no listed
# calibration-supported camera.
APRILTAG_COST_USD = 25.0

# Distance sensor (ToF) geometry -- VERIFIED against REV's own
# REV-31-1505 datasheet/product page: measurement range 5cm-200cm
# (hence the 2.0m below) and field of view 25 degrees, i.e. a 12.5
# degree half-angle. Both figures are manufacturer spec, not estimates
# -- which matters, because the ~75-degrees-of-360 coverage figure
# behind this project's headline negative result is computed directly
# from DISTANCE_SENSOR_COUNT x 2 x this half-angle. "Sees obstacles
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
# Detection range -- VERIFIED upward. FIRST's own AprilTag
# documentation reports that with a well-calibrated lens the SDK's
# returned range lands within about an inch of the true measured
# distance on 5in tags at TEN FEET. 120in is used accordingly.
#
# CORRECTION: this was previously 72in (6ft), described as the
# "reliable detection range" -- that understated the officially
# documented figure by nearly half, and it understated it in the
# direction that made this project's own best-value winner look WORSE
# than it should have. Fixing it raises AprilTag's success rate.
APRILTAG_RANGE_CELLS = round(120.0 / CELL_SIZE_IN)  # 20 cells
# The TAG's own readable cone -- how far off the tag's surface normal
# the robot can stand and still resolve the tag's geometry. This used
# to be documented as a COMBINED camera-FOV/tag-readability figure,
# which stopped being accurate once MODEL_FIDELITY's CAMERA_FOV_DEG_BY_
# TIER (below) split the camera's own aiming cone out into its own,
# separately-gated check: the two are now independent tests in
# ftc/sensors.py's tag_correction, so this constant has to mean the tag
# half of that pair alone, not both at once.
# 60 degrees remains the right magnitude (a fiducial viewed much more
# obliquely than ~30 degrees off-normal loses the corner geometry a
# pose solve needs), and is documented as an engineering estimate: FTC
# publishes no per-tag readability-angle spec.
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
# headline numbers (benchmark_results/ftc_suite_writeup.md -- deliberately
# not restated here as a specific figure, since a hardcoded snapshot of
# them already went stale once, when the sensor costs/AprilTag range
# below were corrected against real vendor/FTC-doc sources) trial-for-
# trial -- see ftc/scratch/fidelity_test.py's
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
# VERIFIED against FIRST's VisionPortal webcam documentation, which
# lists the cameras shipping with built-in lens calibrations usable for
# AprilTag pose estimation, with published fields of view:
#   Logitech C270 .............. 60 deg  (FTC docs call this the
#                                         "workhorse webcam" for FTC)
#   Logitech C310 .............. 60 deg
#   Microsoft LifeCam HD-3000 .. 68.5 deg
#   Logitech C920 .............. 78 deg
# So the real span across supported hardware is 60-78 degrees, and the
# tiers below are anchored to actual products rather than picked freely.
CAMERA_FOV_DEG_BY_TIER = {
    "optimistic": 360.0,  # omnidirectional -- reproduces pre-fidelity-tier behavior exactly
    # The C270/C310 figure -- the low end of the supported range, and
    # the camera FTC's own docs single out as the program's most common
    # choice. Deliberately not the 78deg C920 number: picking the
    # widest supported camera for the "realistic" tier would flatter
    # every AprilTag-based suite on a hardware assumption most teams
    # don't actually make.
    "realistic": 60.0,
    # Below every supported camera's nominal FOV -- not a product spec,
    # and explicitly a ballpark engineering estimate. It stands in for
    # the gap between a camera's NOMINAL field of view and its usable
    # one: a tag right at the edge of frame is geometrically "in view"
    # but is exactly where lens distortion is worst and where a partly
    # cut-off tag fails to resolve at all.
    "pessimistic": 45.0,
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
# Orthogonal to sensor suite -- see ftc/drivetrain.py. VERIFIED against
# goBILDA's current listings, matched at the same 96mm wheel diameter
# the MAX_DRIVE_SPEED_MPS derivation above assumes, so the two costs
# describe drivetrains that actually drive at the modeled speed.
#
# CORRECTION: these were previously $80 and $200, both ballpark. Real
# listed prices are lower, and the GAP between them (the premium
# ftc/drivetrain_benchmark.py asks whether mecanum repays) is $130, not
# the $120 previously modeled.
TANK_WHEEL_COST_USD = 39.96      # 4x goBILDA Hogback Traction Wheel, 96mm, $9.99 each
MECANUM_WHEEL_COST_USD = 169.99  # goBILDA Mecanum Wheel Set, 96mm, set of 4

# A mecanum robot can translate in any direction without turning, but
# not at full speed in every direction. Game Manual 0's drivetrain
# reference states the mechanism directly: a mecanum drivetrain drives
# faster forward/backward than in any other direction, both because of
# friction and because the rollers sit at a 45-degree angle to the
# wheel's axis of rotation, so it structurally "can't strafe as fast as
# [it] can drive."
#
# The DIRECTION of this effect is therefore sourced, but the MAGNITUDE
# is not: neither GM0 nor any vendor publishes a strafe-speed
# percentage, since it depends on wheel quality, roller durometer,
# weight distribution, and floor surface. 0.8 remains an explicit
# ballpark engineering estimate for that reason -- ftc/robustness.py's
# style of tipping-point sweep is the right way to bound it, not a
# more confident-sounding number.
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

# Slamtec RPLIDAR A1 -- VERIFIED at $99.95 (Adafruit's listed price for
# the A1M8 360-degree laser range scanner); other resellers list the
# same unit anywhere from ~$99 to ~$220, so this is the low, most
# commonly cited end of a real spread. A full 360-degree disc scan,
# which nav/sensor.py's LidarSensor already models exactly (see
# ftc/sensors.py's LidarSuite).
#
# NOTE: FTC's laser-class-device rules must be checked against the
# CURRENT season's game manual before treating this as a real, legal
# recommendation for a team to buy -- this repo prices and simulates
# the sensing model, it does not assert legality. (The A1's own spec
# sheet lists it as a Class 1 laser product, the same class as the REV
# 2m Distance Sensor's 940nm emitter above, but "same laser class as a
# part teams already use" is an argument to check the manual with, not
# a substitute for checking it.)
LIDAR_COST_USD = 100.0
# The A1's published scan radius is 12m, which comfortably exceeds this
# field's own corner-to-corner diagonal (144in x 144in = 203in = 5.2m)
# -- so range genuinely is not the binding constraint for this sensor
# on this field. Set to safely exceed the 24x24 grid's own ~34-cell
# diagonal so the sensor model reads as "sees the whole field," which
# is what the real hardware would do here.
LIDAR_RANGE_CELLS = FTC_GRID_SIZE * 2

# === New suites enabled by the fidelity-tier model ======================
# VERIFIED: every REV Control Hub ships with an integrated IMU -- a
# Bosch BNO055 on units built before September 2022, and a Bosch
# BHI260AP on units built after, both exposed through the FTC SDK's
# Universal IMU Interface (SDK 8.1+). So this suite's hardware cost is
# genuinely $0 for any team that already has a Control Hub, which is
# every team; the only real cost is the integration effort of reading
# and fusing it, which this project's dollar-based cost model has no
# way to price (see ftc/sensors.py's ImuSuite and ftc/*_benchmark.py's
# explicit handling of cost_usd == 0.0 -- pp/$100 is undefined there,
# not infinite, and has to be reported as such rather than
# divided-by-zero).
IMU_COST_USD = 0.0
# Fraction of accumulated HEADING error one IMU fusion cycle removes,
# applied every tick (no need for a tag, or anything else, to be in
# view) -- separate from AprilTag's own per-detection heading
# correction above. Ballpark: a well-fused IMU heading estimate is
# usually tighter than a single vision-based pose solve (no corner-
# localization noise), so this sits above the realistic-tier AprilTag
# heading-correction factor.
IMU_HEADING_CORRECTION_FACTOR = 0.9

# Two cameras (front + rear) instead of AprilTag's one -- ftc/sensors.py's
# DualCameraAprilTagSuite. Derived from APRILTAG_COST_USD rather than
# hardcoded, so it tracks that constant's now-verified $25 C270 price
# automatically (it previously read "~$40 each," inheriting the same
# unsourced figure corrected above).
DUAL_CAMERA_APRILTAG_COST_USD = APRILTAG_COST_USD * 2

# === Drivetrain speed / gearing (optional, Priority 5) ==================
# MAX_DRIVE_SPEED_MPS/MAX_ACCEL_MPS2 above are constants, but ftc/
# budget_benchmark.py found the 30s budget genuinely starts binding
# around 15-20s under the trapezoidal kinematics model -- which makes a
# faster-motor purchase an actually testable question for the first
# time (a robot that never runs out of time has nothing to gain from
# more speed). goBILDA/REV both sell higher-speed, lower-torque gearing
# options for the same 5203-series-class motor at a real price premium;
# GEARING_OPTIONS below is now built directly from goBILDA's PUBLISHED
# 5203-series spec table rather than invented multipliers, keyed by
# name, each overriding max_speed_mps/max_accel_mps2 and adding the
# tradeoff a faster-is-just-better model would hide.
#
#   Ratio     No-load RPM   Torque (oz-in)   Price
#   19.2:1        312            338         $54.99   <- "stock"
#   13.7:1        435            260         $54.99   <- "fast"
#    5.2:1       1150            109         $54.99   <- "faster"
#
# Two corrections this table forced, both of which had the previous
# model pointing the wrong way:
#
# 1. ACCELERATION FALLS as gearing gets faster; it does not rise. The
#    previous version had accel multipliers of 1.15x and 1.25x
#    alongside its speed increases, i.e. it modeled a faster gearing
#    option as accelerating harder TOO. Real gearmotors trade exactly
#    the other way -- 435 RPM comes with 260 oz-in against 312 RPM's
#    338 oz-in -- so accel now scales by the published TORQUE ratio,
#    which is below 1 for every faster option.
# 2. COST is per-motor and ratio-independent. Every 5203 variant lists
#    at the same $54.99 regardless of ratio, so a drivetrain gearing
#    swap costs 4 x $54.99 = $219.96 whichever ratio is chosen -- not
#    the $60/$120 "more speed costs more" schedule previously modeled.
#    What a team buys with the higher price is nothing; the choice is
#    free at the till and paid for entirely in torque.
#
# `slip_factor` (scales `drift_per_cell` up, so more speed means more
# wheel slip rather than a free win) is the one number here still NOT
# sourced -- no vendor publishes slip-vs-gearing data. It stays an
# explicit ballpark engineering estimate, scaled with top speed.
#
# "stock" reproduces MAX_DRIVE_SPEED_MPS/MAX_ACCEL_MPS2/no-slip-penalty
# exactly -- ftc/match.py's `gearing=None` default resolves to "stock",
# so every existing caller is unaffected (see ftc/scratch/
# gearing_test.py's regression check).
_MOTOR_COST_USD = 54.99          # goBILDA 5203 Yellow Jacket, any ratio
_DRIVETRAIN_MOTOR_COUNT = 4
_GEARING_SWAP_COST_USD = _MOTOR_COST_USD * _DRIVETRAIN_MOTOR_COUNT  # $219.96
GEARING_OPTIONS = {
    # 19.2:1, 312 RPM, 338 oz-in -- the reference drivetrain every other
    # constant in this file is derived against.
    "stock": dict(max_speed_mps=MAX_DRIVE_SPEED_MPS, max_accel_mps2=MAX_ACCEL_MPS2,
                   cost_usd=0.0, slip_factor=1.0),
    # 13.7:1, 435 RPM, 260 oz-in -- the other genuinely common FTC
    # drivetrain ratio. 435/312 = 1.394x speed, 260/338 = 0.769x torque
    # and therefore 0.769x acceleration.
    "fast": dict(max_speed_mps=MAX_DRIVE_SPEED_MPS * (435 / 312),
                  max_accel_mps2=MAX_ACCEL_MPS2 * (260 / 338),
                  cost_usd=_GEARING_SWAP_COST_USD, slip_factor=1.35),
    # 5.2:1, 1150 RPM, 109 oz-in. This is a real, purchasable 5203
    # variant, but it is NOT a sensible drivetrain choice -- 1150 RPM on
    # a 96mm wheel is ~5.8 m/s (19 ft/s) across a 12ft field, at less
    # than a third of stock's torque. It's included as a deliberate
    # far-end anchor for the sweep (does more speed EVER pay off?), not
    # as a configuration this project recommends.
    "faster": dict(max_speed_mps=MAX_DRIVE_SPEED_MPS * (1150 / 312),
                    max_accel_mps2=MAX_ACCEL_MPS2 * (109 / 338),
                    cost_usd=_GEARING_SWAP_COST_USD, slip_factor=1.8),
}
GEARING_ORDER = ["stock", "fast", "faster"]
GEARING_LABELS = {"stock": "Stock gearing", "fast": "Fast gearing", "faster": "Faster gearing"}

# math.radians(APRILTAG_FOV_DEG) etc. computed on demand where needed;
# nothing below this line is a tunable, just a derived convenience.
INCHES_PER_METER = 1.0 / 0.0254
