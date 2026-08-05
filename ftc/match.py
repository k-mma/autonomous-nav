"""
A 30-second FTC autonomous-period budget model: drives a sensor suite
(ftc/sensors.py) from an actual (possibly drifted) start to a goal on a
ground-truth grid, charging simulated elapsed time for every cell
driven, every turn taken, and every replan call -- so a policy that
senses and replans a lot pays for it in the one currency that actually
matters in a real match, not just in "did it succeed." A run that
reaches the goal after the 30-second budget is a FAILURE, not a
success: this is what makes replanning cost a real tradeoff instead of
a footnote, unlike nav/uncertainty_benchmark.py's harness (which has no
time budget at all -- every closed-loop policy there gets to replan for
free, forever).

Pose error is tracked as (row, col, heading_deg): a continuous
translation vector `error` such that true_position = believed_position
+ error (unchanged from before Priority 1), plus a continuous scalar
`heading_error` (degrees) representing how wrong the robot's own
heading BELIEF is. This generalizes nav/policies.py's OpenLoopPolicy
offset mechanic (a single fixed value, set once from field_variance's
start drift) into something that grows every tick a suite has no way to
correct it (dead reckoning) and shrinks when a suite does (AprilTag,
IMU). Obstacle sensing and planning both happen entirely in the robot's
own *believed* frame -- exactly what a real robot does, since it only
ever has its own (possibly wrong) idea of where it is and which way it
points -- and the resulting motion command gets translated into the
true frame by BOTH error terms before being checked against ground
truth: `error` offsets it, and `heading_error` ROTATES it (the commanded
step is computed in the believed frame and executed in the true frame,
so a wrong heading belief doesn't just misreport a number, it steers
the robot somewhere it didn't intend to go -- see
ftc/scratch/fidelity_test.py's check_heading_error_rotates_execution).

Which fidelity tier's assumptions apply (`fidelity`, defaulting to
ftc.config.MODEL_FIDELITY, resolved fresh on every call the same way
AUTONOMOUS_PERIOD_S already is below -- NOT baked in at import time)
governs camera FOV gating, heading drift rate, AprilTag's heading-
correction strength, and AprilTag detection dropout; see ftc/config.py's
MODEL_FIDELITY docstring. At the "optimistic" default, heading_error
never leaves 0.0 and every new code path in this module is either
skipped outright or evaluates to an exact no-op, so this reproduces
every pre-Priority-1 caller's results trial-for-trial (see
ftc/scratch/fidelity_test.py's check_optimistic_tier_reproduces_
headline_exactly, the regression guarantee for the published headline
numbers).

`drivetrain` (defaulting to None -- ftc/drivetrain.py's TANK/MECANUM,
Priority 2) is a second, independent axis: None reproduces the exact
pre-Priority-2 behavior (a flat per-90-degree turn cost on every
direction change, chassis heading = true direction of travel) for
every existing caller; an explicit Drivetrain instance switches to a
drivetrain-aware model of turn cost, chassis heading policy, and (for a
holonomic drivetrain) a strafe speed/drift penalty -- see
ftc/drivetrain.py's module docstring for the heading-policy choice
(mecanum holds a fixed heading facing the nearest AprilTag wall; tank
always faces its direction of travel, identical to the None default).

Drive time per step follows a trapezoidal (accelerate, cruise,
decelerate) velocity profile bounded by MAX_ACCEL_MPS2, not
distance/MAX_DRIVE_SPEED_MPS's implicit assumption of instantaneous
acceleration. Each step is charged independently, starting and ending
at rest -- the same assumption the existing flat per-90-degree turn
cost already makes (a full stop at every direction change) -- rather
than carrying velocity across consecutive collinear steps, which would
need restructuring how elapsed_s is accrued across the whole loop, a
larger change than this deliberately-scoped addition. One consequence
worth knowing before reading any elapsed_s number this produces: a
single 6in-cell (or diagonal) step is almost always too short to reach
MAX_DRIVE_SPEED_MPS at all under MAX_ACCEL_MPS2 (see
`_trapezoidal_drive_time_s`'s docstring for the actual distance
threshold), so nearly every step in practice uses the triangular
(never-reaches-cruise) branch of the profile, not the trapezoidal one --
drive time is strictly >= the old naive distance/speed figure, never
less.
"""
import copy
import math
import time
from dataclasses import dataclass

from nav.algorithms import astar
from nav.grid import Grid
from nav.sensor import KnownGrid, blocks_remaining_path

import ftc.config as config_module
from ftc.config import (
    AUTONOMOUS_PERIOD_S, CELL_SIZE_IN, COLLISION_RECOVERY_S, INCHES_PER_METER, MAX_ACCEL_MPS2,
    MAX_DRIVE_SPEED_MPS, MAX_STALL_RETRIES, PLANNING_OVERHEAD_S, TURN_TIME_PER_90DEG_S,
)
from ftc.field import in_to_cell
from ftc.sensors import angular_diff, heading_deg

MAX_TICKS = 500


def _trapezoidal_drive_time_s(distance_m, max_speed_mps=MAX_DRIVE_SPEED_MPS, max_accel_mps2=MAX_ACCEL_MPS2):
    """Time to cover `distance_m` starting and ending at rest, under a
    trapezoidal (reaches cruise speed) or triangular (too short to)
    velocity profile bounded by `max_accel_mps2`.

    The distance needed to accelerate from 0 to max_speed_mps is
    `max_speed_mps**2 / (2 * max_accel_mps2)` -- at this project's
    defaults (1.5 m/s, 3.0 m/s^2), that's 0.375m, well over a single
    6in cell (0.1524m) or even a diagonal step (0.2155m). So a lone
    step almost always falls in the triangular branch below, peaking at
    some speed under max_speed_mps and never actually cruising -- the
    trapezoidal branch exists for correctness (and for any future
    caller with a longer single leg) but rarely fires at this grid
    scale.
    """
    if distance_m <= 0:
        return 0.0
    accel_distance_m = max_speed_mps ** 2 / (2 * max_accel_mps2)
    if distance_m >= 2 * accel_distance_m:
        cruise_distance_m = distance_m - 2 * accel_distance_m
        return 2 * (max_speed_mps / max_accel_mps2) + cruise_distance_m / max_speed_mps
    return 2 * math.sqrt(distance_m / max_accel_mps2)


def _rotate(vec, deg):
    """Rotate a (row, col) vector by `deg` degrees, in the same
    atan2(d_row, d_col) convention every heading in this project uses.
    Exact identity at deg == 0.0 (short-circuited rather than computed
    via cos(0)/sin(0)) -- the guarantee Priority 1's optimistic-tier
    regression depends on: heading_error stays exactly 0.0 at that tier,
    so every rotation this module performs has to be a true no-op, not
    just numerically close to one."""
    if deg == 0.0:
        return vec
    rad = math.radians(deg)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    return (vec[0] * cos_a - vec[1] * sin_a, vec[0] * sin_a + vec[1] * cos_a)


@dataclass
class MatchResult:
    success: bool
    elapsed_s: float
    over_budget: bool
    collisions: int
    replans: int
    planning_time_s: float
    final_pose_error_in: float
    steps: int
    final_heading_error_deg: float = 0.0


def run_match(suite, assumed_grid, start, goal, ground_truth, actual_start, tag_sites, rng,
              moving_obstacles=(), fidelity=None, drivetrain=None, gearing=None, on_tick=None,
              on_collision="halt"):
    """Drive `suite` from `actual_start` (ground truth) to `goal`,
    planning against `assumed_grid`'s layout (the suite's only source of
    obstacle knowledge unless it senses otherwise) until it succeeds,
    collides, gets stuck, or exhausts AUTONOMOUS_PERIOD_S. See module
    docstring for the believed-frame planning / true-frame execution
    split, and for `fidelity`/`drivetrain`/`gearing`'s defaults
    reproducing every pre-Priority-1/2/5 caller's behavior exactly.

    `moving_obstacles` (optional, empty by default so every existing
    caller is unaffected) is a sequence of nav.obstacles.MovingObstacle
    instances -- see ftc/opponent_benchmark.py, which places one to model
    an opponent robot that actually moves, instead of the static single
    obstacle nav/field_variance.py's unplanned_blocker deviation type
    drops once and leaves in place. Each is ticked once per loop
    iteration with `now_ms = elapsed_s * 1000` -- SIMULATED match time,
    not wall-clock time, since MovingObstacle.tick's period_ms/now_ms
    timing has to stay reproducible independent of how fast this process
    actually executes.

    `gearing` (defaulting to "stock" via ftc.config.GEARING_OPTIONS,
    optional Priority 5) overrides MAX_DRIVE_SPEED_MPS/MAX_ACCEL_MPS2
    with a named option's own values and multiplies translation drift
    by that option's `slip_factor` -- a faster gearing swap trades drive
    time for more wheel slip, not a free win. "stock" is exactly
    MAX_DRIVE_SPEED_MPS/MAX_ACCEL_MPS2/no slip penalty, so the default
    is a byte-for-byte no-op.

    `on_tick` (optional, None by default) is a read-only side channel
    for animation/visualization (see ftc/trace.py, pygame_app/ftc_viz/)
    -- if given, it's called with an already-computed snapshot dict at
    a handful of points each tick (see _emit below): "start" once
    before the loop, "step" after each successful move, "collision" for
    an attempted-but-rejected move, and "end" once after the loop, with
    the final success/over_budget outcome. It is purely a dead end --
    nothing on_tick does or returns is ever read back by run_match, so
    it cannot perturb `rng`, `path`, `error`, or anything else this
    function's own determinism depends on. That's what keeps it safe to
    add without touching the byte-for-byte optimistic-tier regression
    guarantee ftc/scratch/fidelity_test.py enforces: on_tick=None (every
    caller before this addition, and every existing test) skips every
    call site outright.

    `on_collision` ("halt", the default -- every existing caller/
    benchmark, unaffected) ends the match the instant a move would
    collide, exactly as this function has always behaved. "replan" (ftc/
    config.py's COLLISION_RECOVERY_S/MAX_STALL_RETRIES; used by pygame_
    app/ftc_viz/'s visualizer) instead treats it as a generic stall --
    detectable via motor encoder feedback, which every FTC robot has,
    independent of sensor suite -- marks the attempted cell blocked in
    the robot's own belief, charges COLLISION_RECOVERY_S, and forces a
    replan around it rather than ending the match outright. It keeps
    trying (this is what actually gives a sensing or pose-correcting
    suite a real chance to route around what it just bumped) until
    MAX_STALL_RETRIES consecutive attempts fail, at which point it gives
    up -- the same terminal outcome "halt" always produces, just reached
    after trying rather than immediately. See ftc/scratch/
    collision_recovery_test.py."""
    fidelity = fidelity or config_module.MODEL_FIDELITY
    tier = config_module.FIDELITY_TIERS[fidelity]
    gearing_config = config_module.GEARING_OPTIONS[gearing or "stock"]

    obstacle_sensor = suite.make_obstacle_sensor()
    known_obstacles_believed = set()

    true_position = actual_start
    error = (float(actual_start[0] - start[0]), float(actual_start[1] - start[1]))
    heading_error = 0.0

    # Mecanum holds a fixed heading for the whole match -- the natural
    # choice for a robot investing in an AprilTag-reading camera is to
    # aim it at the nearest tag wall and never turn away from it (see
    # ftc/drivetrain.py's module docstring). Computed once, from the
    # true starting position, since a real robot would pick its held
    # heading before the match starts, not re-derive it mid-run.
    fixed_heading_deg = None
    if drivetrain is not None and drivetrain.holonomic and tag_sites:
        def _tag_dist(t):
            tc = in_to_cell(t.x_in, t.y_in)
            return math.hypot(tc[0] - actual_start[0], tc[1] - actual_start[1])
        nearest_tag = min(tag_sites, key=_tag_dist)
        fixed_heading_deg = heading_deg(actual_start, in_to_cell(nearest_tag.x_in, nearest_tag.y_in))

    if drivetrain is not None and drivetrain.holonomic and fixed_heading_deg is not None:
        heading = fixed_heading_deg
    else:
        heading = heading_deg(actual_start, goal) if actual_start != goal else 0.0

    elapsed_s = 0.0
    replans = 0
    collisions = 0
    planning_time_s = 0.0
    steps = 0
    over_budget = False
    path = None
    idx = 0
    planned_once = False
    stalled = False
    # Consecutive collisions since the last SUCCESSFUL step (any
    # collision counts, not just a repeat of the identical target cell
    # -- a robot that tries several different nearby cells in a row,
    # each blocked, is exactly as stuck as one retrying the same cell,
    # and capping only same-target repeats let a stuck episode rack up
    # many real attempts, each against a different target, before ever
    # tripping this cap). Reset to 0 on every successful step, so this
    # bounds each STUCK EPISODE independently, not the whole match.
    stall_streak = 0
    # A copy of assumed_grid, mutated in place to add stall-blocked
    # cells on top of it -- built lazily (only if on_collision="replan"
    # ever actually stalls) since deep-copying a Grid is real, if small,
    # work. Only used for suites that DON'T sense obstacles: a sensing
    # suite already plans against a KnownGrid built from
    # known_obstacles_believed below, so a stall-blocked cell is added
    # there instead, in the same believed-frame set it already uses.
    stalled_grid = None

    tick_counter = 0

    def _emit(event, **extra):
        """Read-only snapshot for `on_tick` -- see run_match's own
        docstring for the safety argument. Reads the CURRENT value of
        every enclosing-scope variable at call time (ordinary Python
        closure lookup, not a copy taken when _emit was defined), so a
        call site further down the loop always sees this tick's
        already-updated state."""
        nonlocal tick_counter
        if on_tick is None:
            return
        snapshot = dict(
            tick=tick_counter, event=event, elapsed_s=elapsed_s,
            true_position=true_position, heading_deg=heading,
            error=error, heading_error_deg=heading_error,
            path=list(path) if path else None,
            collisions=collisions, replans=replans,
            # (position, heading_deg_or_None) per moving obstacle --
            # heading is None for a plain nav.obstacles.MovingObstacle
            # (no orientation concept at all), and a real value for
            # anything that has one (e.g. pygame_app/scenarios/
            # scenario_ftc_suites.py's own OpponentRobot, which drives a
            # real path and therefore has a facing direction worth
            # drawing). getattr with a default keeps this working for
            # ANY object satisfying the existing tick()/.position
            # contract, orientation or not.
            moving_obstacle_positions=[(o.position, getattr(o, "heading_deg", None)) for o in moving_obstacles],
        )
        snapshot.update(extra)
        on_tick(snapshot)
        tick_counter += 1

    _emit("start")

    for _ in range(MAX_TICKS):
        if true_position == goal:
            break

        for obstacle in moving_obstacles:
            obstacle.tick(ground_truth, elapsed_s * 1000.0)

        newly_seen_believed = set()
        if suite.senses_obstacles:
            newly_seen_true = obstacle_sensor.sense(ground_truth, true_position, heading)
            err_r, err_c = round(error[0]), round(error[1])
            for r, c in newly_seen_true:
                cell = (r - err_r, c - err_c)
                # A large accumulated pose error can shift a real,
                # in-bounds detection to a believed-frame cell outside
                # the grid -- KnownGrid doesn't bounds-check its
                # known_obstacles (every existing caller only ever feeds
                # it cells LidarSensor.sense() already validated), so
                # this suite has to drop out-of-bounds ones itself
                # rather than pass them through.
                if not assumed_grid.is_valid(*cell):
                    continue
                newly_seen_believed.add(cell)
                known_obstacles_believed.add(cell)

        tag_corrected = False
        if suite.fixes_pose:
            frac = suite.tag_correction(ground_truth, true_position, heading, tag_sites, rng, fidelity=fidelity)
            if frac is not None:
                error = (error[0] * (1 - frac), error[1] * (1 - frac))
                tag_corrected = True
                heading_factor = tier["apriltag_heading_correction_factor"]
                if heading_factor > 0:
                    heading_error *= (1 - heading_factor)

        if suite.fixes_heading:
            hfrac = suite.heading_correction(rng)
            if hfrac:
                heading_error *= (1 - hfrac)

        replan_needed = not planned_once or tag_corrected or stalled
        stalled = False
        if suite.senses_obstacles and not replan_needed:
            current_believed = (round(true_position[0] - error[0]), round(true_position[1] - error[1]))
            remaining = path[idx:] if path is not None else []
            if path is None or blocks_remaining_path(newly_seen_believed, current_believed, remaining):
                replan_needed = True

        if replan_needed:
            current_believed = (round(true_position[0] - error[0]), round(true_position[1] - error[1]))
            source_grid = (
                KnownGrid(known_obstacles_believed, diagonal=assumed_grid.diagonal, size=assumed_grid.size)
                if suite.senses_obstacles else (stalled_grid if stalled_grid is not None else assumed_grid)
            )
            t0 = time.perf_counter()
            new_path, _, _ = astar(source_grid, current_believed, goal)
            planning_time_s += time.perf_counter() - t0
            if planned_once:
                replans += 1
                elapsed_s += PLANNING_OVERHEAD_S
            planned_once = True
            path = new_path
            idx = 0

        if path is None or idx >= len(path) - 1:
            break

        prev_believed = path[idx]
        idx += 1
        next_believed = path[idx]
        step_vec_believed = (next_believed[0] - prev_believed[0], next_believed[1] - prev_believed[1])

        # The commanded step is computed in the believed frame; heading
        # error rotates the realized motion vector when it's executed
        # in the true frame (Priority 1b) -- a no-op (exact (0.0, 0.0)
        # delta) whenever heading_error is exactly 0.0, which it always
        # is at the optimistic tier.
        if heading_error != 0.0:
            step_vec_true = _rotate(step_vec_believed, heading_error)
            heading_delta = (step_vec_true[0] - step_vec_believed[0], step_vec_true[1] - step_vec_believed[1])
        else:
            heading_delta = (0.0, 0.0)

        next_true_continuous = (next_believed[0] + error[0] + heading_delta[0],
                                 next_believed[1] + error[1] + heading_delta[1])
        next_true = (round(next_true_continuous[0]), round(next_true_continuous[1]))

        if not ground_truth.is_valid(*next_true) or ground_truth.is_obstacle(*next_true):
            collisions += 1
            _emit("collision", attempted_position=next_true, newly_seen_believed=set(newly_seen_believed))
            if on_collision != "replan":
                break

            # Stall recovery (see run_match's own docstring). Caps this
            # STUCK EPISODE at MAX_STALL_RETRIES total failed attempts,
            # not just repeats of the identical target -- a robot that
            # tries several different nearby cells in a row, each
            # blocked, is exactly as stuck as one retrying the same
            # cell.
            stall_streak += 1
            if stall_streak >= MAX_STALL_RETRIES:
                break

            # A stall is detectable via motor encoder feedback alone --
            # a generic capability every FTC robot has, independent of
            # sensor suite -- so the blocked cell is learned regardless
            # of suite.senses_obstacles, unlike newly_seen_believed
            # above (which only ever updates for suites with a real
            # obstacle sensor). Same believed-frame approximation
            # nav/sensor.py's KnownGrid and this project's own
            # WRITEUPS.md already document for sensed obstacles: computed
            # from the CURRENT error at the moment of the stall, which
            # can go stale if pose error drifts a lot afterward -- an
            # existing, accepted limitation, not a new one.
            believed_blocked = (round(next_true[0] - error[0]), round(next_true[1] - error[1]))
            if suite.senses_obstacles:
                if assumed_grid.is_valid(*believed_blocked):
                    known_obstacles_believed.add(believed_blocked)
            else:
                if stalled_grid is None:
                    stalled_grid = copy.deepcopy(assumed_grid)
                if stalled_grid.is_valid(*believed_blocked):
                    stalled_grid.cells[believed_blocked[0]][believed_blocked[1]] = Grid.OBSTACLE

            elapsed_s += COLLISION_RECOVERY_S
            if elapsed_s > AUTONOMOUS_PERIOD_S:
                over_budget = True
                break
            stalled = True
            continue

        step_dist = math.hypot(next_true[0] - true_position[0], next_true[1] - true_position[1])
        speed_factor, drift_mult = 1.0, 1.0
        if step_dist > 1e-9:
            # Direction of the TRUE realized motion (after error/heading-
            # error have already been applied above) -- used uniformly
            # for both the legacy path and every Drivetrain, which is
            # exactly what makes an explicit TANK instance byte-for-byte
            # identical to the drivetrain=None default (ftc/drivetrain.py's
            # own docstring promises this; see ftc/scratch/
            # drivetrain_test.py's check_tank_matches_legacy_default):
            # TANK.robot_heading_deg returns travel_heading unchanged and
            # TANK.speed_and_drift_factor is always (1.0, 1.0), so the
            # drivetrain-aware branch below reduces to exactly the same
            # arithmetic the legacy `if drivetrain is None` branch does.
            travel_heading = heading_deg(true_position, next_true)
            if drivetrain is None:
                turn_deg = abs(angular_diff(travel_heading, heading))
                elapsed_s += (turn_deg / 90.0) * TURN_TIME_PER_90DEG_S
                heading = travel_heading
            else:
                new_heading = drivetrain.robot_heading_deg(heading, travel_heading, fixed_heading_deg)
                elapsed_s += drivetrain.turn_cost_s(heading, new_heading)
                speed_factor, drift_mult = drivetrain.speed_and_drift_factor(new_heading, travel_heading)
                heading = new_heading

        step_meters = (step_dist * CELL_SIZE_IN) / INCHES_PER_METER
        elapsed_s += _trapezoidal_drive_time_s(
            step_meters,
            max_speed_mps=gearing_config["max_speed_mps"] * speed_factor,
            max_accel_mps2=gearing_config["max_accel_mps2"],
        )

        if elapsed_s > AUTONOMOUS_PERIOD_S:
            over_budget = True
            break

        true_position = next_true
        stall_streak = 0
        if step_dist > 1e-9:
            # Wheel slip: a faster/harder-geared drivetrain drifts more
            # per cell of real travel, not just arrives sooner --
            # gearing_config["slip_factor"] is 1.0 at "stock" (a no-op),
            # > 1.0 for every faster option (ftc/config.py's
            # GEARING_OPTIONS).
            sigma = suite.drift_per_cell * step_dist * drift_mult * gearing_config["slip_factor"]
            error = (error[0] + rng.gauss(0, sigma), error[1] + rng.gauss(0, sigma))
            heading_drift = tier["heading_drift_deg_per_cell"]
            if heading_drift > 0:
                heading_error += rng.gauss(0, heading_drift * step_dist)
        steps += 1
        _emit("step", replanned=replan_needed, tag_corrected=tag_corrected,
              newly_seen_believed=set(newly_seen_believed))

    success = (true_position == goal) and not over_budget
    _emit("end", success=success, over_budget=over_budget)
    return MatchResult(
        success=success,
        elapsed_s=round(elapsed_s, 4),
        over_budget=over_budget,
        collisions=collisions,
        replans=replans,
        planning_time_s=round(planning_time_s, 6),
        final_pose_error_in=round(math.hypot(*error) * CELL_SIZE_IN, 3),
        steps=steps,
        final_heading_error_deg=round(heading_error, 3),
    )
