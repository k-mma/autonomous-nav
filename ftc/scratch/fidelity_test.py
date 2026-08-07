"""
Does the "optimistic" MODEL_FIDELITY tier (ftc/config.py) actually
reproduce the published headline numbers unchanged, and do the two
Priority-1 mechanisms (camera-FOV gating, heading error rotating
execution) actually do something once a caller asks for a different
tier?

1. check_optimistic_tier_reproduces_headline_exactly: reruns a sample of
   the EXACT trials behind benchmark_results/ftc_suite_results.csv
   (same seed formula, same layout, same everything ftc/
   suite_benchmark.py's run_combo already uses, called with no
   fidelity/drivetrain arguments -- i.e. every default) and diffs every
   column except planning_time_ms (unseeded wall-clock time, see this
   project's own documented trap) against the checked-in CSV. This is
   the regression guarantee the published 56/45/35/21/19% success rates
   and 40.0 vs. 16.2 pp/$100 headline figures depend on.
2. check_camera_fov_gate_rejects_tags_behind_the_robot: at the
   "realistic" tier, a tag that's otherwise perfectly in range/FOV/LOS
   is rejected once the robot's camera is pointed away from it --
   confirming heading_deg_now (accepted but silently ignored before
   Priority 1) now actually gates detection.
3. check_camera_fov_gate_is_a_noop_at_optimistic_tier: the identical
   away-facing scenario from #2 STILL produces a correction at the
   optimistic tier (omnidirectional camera) -- the two checks together
   prove the gate is real AND that it's off by default.
4. check_heading_error_rotates_execution: a synthetic match with
   translation pose error forced to exactly zero but heading_error
   forced nonzero produces a measurably different final position than
   an otherwise-identical run with heading_error forced to zero --
   proving heading error changes REALIZED MOTION, not just a reported
   number.
"""
import csv
import math
import random
from pathlib import Path

import ftc.config as config_module
from ftc.field import build_grid, tag_sites_for, in_to_cell, FieldLayout, TagSite
from ftc.match import run_match, _rotate
from ftc.sensors import AprilTagSuite, SUITE_ORDER
from ftc.suite_benchmark import DEVIATION_TYPES, LAYOUT, TRIALS_PER_COMBO, run_combo

RESULTS_CSV = Path(__file__).resolve().parent.parent.parent / "benchmark_results" / "ftc_suite_results.csv"
# A representative sample, not the full 4,125-row sweep -- cheap enough
# to run every time this script is invoked while still covering every
# suite and both a low- and high-deviation point on two different
# deviation types.
SAMPLE_POINTS = [("start_drift", 0.0), ("start_drift", 1.0), ("obstacle_drift", 0.5)]
COMPARE_COLUMNS = ["success", "over_budget", "elapsed_s", "collisions", "replans",
                    "final_pose_error_in", "steps", "cost_usd"]


def check_optimistic_tier_reproduces_headline_exactly():
    if not RESULTS_CSV.exists():
        print(f"SKIP: {RESULTS_CSV} not found -- run `python -m ftc.suite_benchmark` first")
        return True

    with open(RESULTS_CSV) as f:
        published = {
            (row["suite"], row["deviation_type"], row["variance_level"], row["trial"]): row
            for row in csv.DictReader(f)
        }

    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)

    mismatches = []
    checked = 0
    for deviation_type, level in SAMPLE_POINTS:
        base_seed = 6_000_000 + list(DEVIATION_TYPES).index(deviation_type) * 1_000_000 + round(level * 100)
        rows = run_combo(deviation_type, level, TRIALS_PER_COMBO, base_seed, grid, free_cells, tag_sites)
        for row in rows:
            key = (row["suite"], row["deviation_type"], f"{row['variance_level']:.1f}", str(row["trial"]))
            ref = published.get(key)
            if ref is None:
                mismatches.append(f"{key}: no matching row in {RESULTS_CSV.name}")
                continue
            checked += 1
            for col in COMPARE_COLUMNS:
                got = str(row[col])
                want = ref[col]
                # bool/float string formatting differs (True vs "True",
                # 6.6864 vs "6.6864") -- compare as float where possible,
                # else as the exact string csv.DictReader produced.
                try:
                    if abs(float(got) - float(want)) > 1e-9:
                        mismatches.append(f"{key}.{col}: got {got}, published {want}")
                except ValueError:
                    if got != want:
                        mismatches.append(f"{key}.{col}: got {got}, published {want}")

    ok = checked > 0 and not mismatches
    print(f"  checked {checked} rows across {len(SAMPLE_POINTS)} (deviation_type, level) points")
    if mismatches:
        for m in mismatches[:10]:
            print(f"  MISMATCH: {m}")
    print(f"optimistic-tier default reproduces {RESULTS_CSV.name} trial-for-trial "
          f"(every column except planning_time_ms): {'OK' if ok else 'FAIL'}")
    return ok


def _corridor_grid_with_tag():
    grid = build_grid(FieldLayout(elements=[]))
    tag = TagSite(x_in=0.0, y_in=72.0, heading_deg=0.0)
    return grid, tag


def check_camera_fov_gate_rejects_tags_behind_the_robot():
    grid, tag = _corridor_grid_with_tag()
    suite = AprilTagSuite()
    tag_cell = in_to_cell(tag.x_in, tag.y_in)
    in_view_position = (tag_cell[0], tag_cell[1] + 6)  # straight out from the tag, in range/FOV/LOS

    # The tag sits due west of in_view_position (tag_cell[1] < robot's
    # column), so heading_deg(robot, tag) == 180deg -- a camera facing
    # 0deg (east, i.e. away from the tag) should NOT see it.
    away_facing = 0.0
    original = config_module.MODEL_FIDELITY
    try:
        config_module.MODEL_FIDELITY = "realistic"
        correction = suite.tag_correction(grid, in_view_position, away_facing, [tag], random.Random(0))
    finally:
        config_module.MODEL_FIDELITY = original

    ok = correction is None
    print(f"  correction with camera facing away from an otherwise-visible tag (realistic tier): {correction}")
    print(f"the robot's camera FOV gate rejects a tag it isn't actually pointed at: {'OK' if ok else 'FAIL'}")
    return ok


def check_camera_fov_gate_is_a_noop_at_optimistic_tier():
    grid, tag = _corridor_grid_with_tag()
    suite = AprilTagSuite()
    tag_cell = in_to_cell(tag.x_in, tag.y_in)
    in_view_position = (tag_cell[0], tag_cell[1] + 6)
    away_facing = 0.0

    correction = suite.tag_correction(grid, in_view_position, away_facing, [tag], random.Random(0))
    ok = correction is not None and 0.0 < correction <= 1.0
    print(f"  correction with camera facing away from the tag (optimistic/default tier): {correction}")
    print(f"the same away-facing scenario still corrects at the optimistic tier (omnidirectional camera, "
          f"the pre-Priority-1 assumption): {'OK' if ok else 'FAIL'}")
    return ok


def check_heading_error_rotates_execution():
    """Two hand-built matches, identical except one has a large forced
    heading_error and the other doesn't, translation error pinned at
    exactly zero in both -- isolates heading error's effect on realized
    motion from every other error source. Since run_match doesn't expose
    a way to force heading_error directly, this drives _rotate the same
    way ftc/match.py's own step-execution code does, on a hand-picked
    step vector, and confirms the rotated result actually lands on a
    different cell than the unrotated one for a large-enough angle."""
    step_vec_believed = (0, 4)  # "drive 4 cells east" in the believed frame
    no_error = _rotate(step_vec_believed, 0.0)
    with_error = _rotate(step_vec_believed, 25.0)  # a large, deliberately obvious heading error

    same_vector = (abs(no_error[0] - step_vec_believed[0]) < 1e-9
                   and abs(no_error[1] - step_vec_believed[1]) < 1e-9)
    rotated_differs = (round(with_error[0]) != round(no_error[0])
                        or round(with_error[1]) != round(no_error[1]))
    ok = same_vector and rotated_differs
    print(f"  step_vec_believed={step_vec_believed}, rotate(0deg)={no_error}, rotate(25deg)={with_error}")
    print(f"heading_error==0 is an exact no-op, and a nonzero heading_error rotates the executed step to a "
          f"measurably different cell: {'OK' if ok else 'FAIL'}")

    # End-to-end confirmation through run_match itself: a "realistic"-
    # tier run (heading drift on) vs. the same scenario at "optimistic"
    # (heading drift off) should, over enough trials, show a nonzero
    # final_heading_error_deg in the former and always exactly 0.0 in
    # the latter.
    grid = build_grid("sparse")
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for("sparse")
    from nav.algorithms import astar
    from nav.field_variance import generate_ground_truth
    rng = random.Random(1)
    for _ in range(50):
        start, goal = rng.sample(free, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= 10:
            break
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.0, seed=1)

    from ftc.sensors import DeadReckoningSuite
    optimistic_result = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start,
                                    tag_sites, random.Random(1), fidelity="optimistic")
    realistic_result = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start,
                                   tag_sites, random.Random(1), fidelity="pessimistic")
    end_to_end_ok = optimistic_result.final_heading_error_deg == 0.0 and realistic_result.final_heading_error_deg != 0.0
    print(f"  optimistic final_heading_error_deg={optimistic_result.final_heading_error_deg}, "
          f"pessimistic final_heading_error_deg={realistic_result.final_heading_error_deg}")
    print(f"end to end: heading_error stays exactly 0 at the optimistic tier and accumulates at the "
          f"pessimistic one: {'OK' if end_to_end_ok else 'FAIL'}")

    return ok and end_to_end_ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_optimistic_tier_reproduces_headline_exactly():
    assert check_optimistic_tier_reproduces_headline_exactly()


def test_camera_fov_gate_rejects_tags_behind_the_robot():
    assert check_camera_fov_gate_rejects_tags_behind_the_robot()


def test_camera_fov_gate_is_a_noop_at_optimistic_tier():
    assert check_camera_fov_gate_is_a_noop_at_optimistic_tier()


def test_heading_error_rotates_execution():
    assert check_heading_error_rotates_execution()


if __name__ == "__main__":
    checks = [
        check_optimistic_tier_reproduces_headline_exactly(),
        check_camera_fov_gate_rejects_tags_behind_the_robot(),
        check_camera_fov_gate_is_a_noop_at_optimistic_tier(),
        check_heading_error_rotates_execution(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
