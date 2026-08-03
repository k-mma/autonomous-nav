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

Pose error is tracked as a single continuous vector `error` such that
true_position = believed_position + error. This generalizes nav/
policies.py's OpenLoopPolicy offset mechanic (a single fixed value, set
once from field_variance's start drift) into something that grows every
tick a suite has no way to correct it (dead reckoning) and shrinks when
a suite does (AprilTag). Obstacle sensing and planning both happen
entirely in the robot's *believed* frame -- exactly what a real robot
does, since it only ever has its own (possibly wrong) idea of where it
is -- and the resulting motion command gets translated into the true
frame by the current `error` before being checked against ground truth.
"""
import math
import time
from dataclasses import dataclass

from nav.algorithms import astar
from nav.sensor import KnownGrid, blocks_remaining_path

from ftc.config import (
    AUTONOMOUS_PERIOD_S, CELL_SIZE_IN, INCHES_PER_METER, MAX_DRIVE_SPEED_MPS,
    PLANNING_OVERHEAD_S, TURN_TIME_PER_90DEG_S,
)
from ftc.sensors import angular_diff, heading_deg

MAX_TICKS = 500


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


def run_match(suite, assumed_grid, start, goal, ground_truth, actual_start, tag_sites, rng):
    """Drive `suite` from `actual_start` (ground truth) to `goal`,
    planning against `assumed_grid`'s layout (the suite's only source of
    obstacle knowledge unless it senses otherwise) until it succeeds,
    collides, gets stuck, or exhausts AUTONOMOUS_PERIOD_S. See module
    docstring for the believed-frame planning / true-frame execution
    split."""
    obstacle_sensor = suite.make_obstacle_sensor()
    known_obstacles_believed = set()

    true_position = actual_start
    error = (float(actual_start[0] - start[0]), float(actual_start[1] - start[1]))
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

    for _ in range(MAX_TICKS):
        if true_position == goal:
            break

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
            frac = suite.tag_correction(ground_truth, true_position, heading, tag_sites, rng)
            if frac is not None:
                error = (error[0] * (1 - frac), error[1] * (1 - frac))
                tag_corrected = True

        replan_needed = not planned_once or tag_corrected
        if suite.senses_obstacles and not replan_needed:
            current_believed = (round(true_position[0] - error[0]), round(true_position[1] - error[1]))
            remaining = path[idx:] if path is not None else []
            if path is None or blocks_remaining_path(newly_seen_believed, current_believed, remaining):
                replan_needed = True

        if replan_needed:
            current_believed = (round(true_position[0] - error[0]), round(true_position[1] - error[1]))
            source_grid = (
                KnownGrid(known_obstacles_believed, diagonal=assumed_grid.diagonal, size=assumed_grid.size)
                if suite.senses_obstacles else assumed_grid
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

        idx += 1
        next_believed = path[idx]
        next_true = (round(next_believed[0] + error[0]), round(next_believed[1] + error[1]))

        if not ground_truth.is_valid(*next_true) or ground_truth.is_obstacle(*next_true):
            collisions += 1
            break

        step_dist = math.hypot(next_true[0] - true_position[0], next_true[1] - true_position[1])
        if step_dist > 1e-9:
            new_heading = heading_deg(true_position, next_true)
            turn_deg = abs(angular_diff(new_heading, heading))
            elapsed_s += (turn_deg / 90.0) * TURN_TIME_PER_90DEG_S
            heading = new_heading

        step_meters = (step_dist * CELL_SIZE_IN) / INCHES_PER_METER
        elapsed_s += step_meters / MAX_DRIVE_SPEED_MPS

        if elapsed_s > AUTONOMOUS_PERIOD_S:
            over_budget = True
            break

        true_position = next_true
        if step_dist > 1e-9:
            sigma = suite.drift_per_cell * step_dist
            error = (error[0] + rng.gauss(0, sigma), error[1] + rng.gauss(0, sigma))
        steps += 1

    success = (true_position == goal) and not over_budget
    return MatchResult(
        success=success,
        elapsed_s=round(elapsed_s, 4),
        over_budget=over_budget,
        collisions=collisions,
        replans=replans,
        planning_time_s=round(planning_time_s, 6),
        final_pose_error_in=round(math.hypot(*error) * CELL_SIZE_IN, 3),
        steps=steps,
    )
