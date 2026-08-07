"""
Does ftc/robustness.py's parameter-override mechanism actually override
anything? This is the check the module's own docstring demands before
trusting a single number out of it: SensorSuite.drift_per_cell and the
AprilTag correction constants are NOT live references to ftc.config's
module-level names (they're baked in at class-definition/import time),
so a naive `ftc.config.SOME_CONSTANT = x` monkeypatch silently does
nothing -- a robustness sweep built on that mistake would run cleanly,
produce plausible-looking numbers, and measure nothing at all.

1. Patching ftc.config.DEAD_RECKONING_DRIFT_PER_CELL after import does
   NOT change a freshly-constructed DeadReckoningSuite's drift_per_cell
   -- confirms the broken approach really is broken, not a
   documentation scare story.
2. Setting the INSTANCE attribute (what ftc/robustness.py's
   _make_suite actually does) DOES change it.
3. Patching ftc.sensors.APRILTAG_CORRECTION_FACTOR_MAX (a module
   global, not ftc.config's) changes what AprilTagSuite.tag_correction
   actually returns for an identical, otherwise-unchanged detection.
4. End to end: run_sweep_point with a drastically exaggerated drift
   override produces a measurably different dead_reckoning success
   rate than the unmodified baseline -- the override has to survive
   all the way through ftc.match.run_match, not just be readable back
   off the suite object immediately after being set.
"""
import random

import ftc.config as config_module
import ftc.sensors as sensors_module
from ftc.field import build_grid, tag_sites_for, in_to_cell, FieldLayout, TagSite
from ftc.sensors import DeadReckoningSuite, AprilTagSuite
import ftc.robustness as robustness_module

LAYOUT = "sparse"


def check_config_patch_does_nothing():
    original = config_module.DEAD_RECKONING_DRIFT_PER_CELL
    baseline = DeadReckoningSuite().drift_per_cell
    try:
        config_module.DEAD_RECKONING_DRIFT_PER_CELL = 999.0
        after_patch = DeadReckoningSuite().drift_per_cell
    finally:
        config_module.DEAD_RECKONING_DRIFT_PER_CELL = original
    ok = after_patch == baseline
    print(f"  baseline drift_per_cell={baseline}, after patching ftc.config={after_patch}")
    print(f"patching ftc.config does NOT change a new suite's drift_per_cell (confirms the class-attribute "
          f"trap is real): {'OK' if ok else 'FAIL'}")
    return ok


def check_instance_override_works():
    suite = DeadReckoningSuite()
    suite.drift_per_cell = 999.0
    ok = suite.drift_per_cell == 999.0
    print(f"setting the instance attribute directly overrides drift_per_cell: {'OK' if ok else 'FAIL'}")
    return ok


def check_apriltag_sensors_module_override_works():
    grid = build_grid(FieldLayout(elements=[]))
    tag = TagSite(x_in=0.0, y_in=72.0, heading_deg=0.0)
    tag_cell = in_to_cell(tag.x_in, tag.y_in)
    in_view_position = (tag_cell[0], tag_cell[1] + 6)
    suite = AprilTagSuite()
    rng = random.Random(0)

    original = sensors_module.APRILTAG_CORRECTION_FACTOR_MAX
    try:
        baseline_correction = suite.tag_correction(grid, in_view_position, 0.0, [tag], random.Random(0))
        sensors_module.APRILTAG_CORRECTION_FACTOR_MAX = original * 0.1
        reduced_correction = suite.tag_correction(grid, in_view_position, 0.0, [tag], random.Random(0))
    finally:
        sensors_module.APRILTAG_CORRECTION_FACTOR_MAX = original

    ok = baseline_correction is not None and reduced_correction is not None and reduced_correction < baseline_correction
    print(f"  baseline correction={baseline_correction:.4f}, after patching ftc.sensors to 0.1x={reduced_correction:.4f}")
    print(f"patching ftc.sensors.APRILTAG_CORRECTION_FACTOR_MAX changes AprilTagSuite.tag_correction's output: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_run_sweep_point_override_survives_end_to_end():
    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)

    baseline_rows = robustness_module.run_sweep_point(grid, free_cells, tag_sites, drift_overrides=None)
    baseline_rate, _, _ = robustness_module.overall_rate(baseline_rows, "dead_reckoning")

    huge_drift = {"dead_reckoning": 50.0, "distance_sensors": 50.0, "apriltag": 50.0}
    overridden_rows = robustness_module.run_sweep_point(grid, free_cells, tag_sites, drift_overrides=huge_drift)
    overridden_rate, _, _ = robustness_module.overall_rate(overridden_rows, "dead_reckoning")

    ok = overridden_rate < baseline_rate
    print(f"  dead_reckoning success rate: baseline={baseline_rate:.2f}, with 50.0 drift override={overridden_rate:.2f}")
    print(f"run_sweep_point's drift_overrides measurably changes dead_reckoning's outcome end to end: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_config_patch_does_nothing():
    assert check_config_patch_does_nothing()


def test_instance_override_works():
    assert check_instance_override_works()


def test_apriltag_sensors_module_override_works():
    assert check_apriltag_sensors_module_override_works()


def test_run_sweep_point_override_survives_end_to_end():
    assert check_run_sweep_point_override_survives_end_to_end()


if __name__ == "__main__":
    checks = [
        check_config_patch_does_nothing(),
        check_instance_override_works(),
        check_apriltag_sensors_module_override_works(),
        check_run_sweep_point_override_survives_end_to_end(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
