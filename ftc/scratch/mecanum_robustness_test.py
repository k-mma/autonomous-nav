"""
Does ftc/mecanum_robustness.py's parameter-override mechanism actually
override anything? Same discipline ftc/scratch/robustness_test.py
already applies to ftc/robustness.py's own overrides: `Drivetrain.
speed_and_drift_factor` reads MECANUM_STRAFE_SPEED_FACTOR/MECANUM_
STRAFE_DRIFT_MULTIPLIER as names bound into ftc.drivetrain's OWN
module namespace at import time (`from ftc.config import ...`), so a
naive `ftc.config.SOME_CONSTANT = x` patch would run cleanly, produce
plausible-looking numbers, and measure nothing at all.

1. check_config_patch_does_nothing: patching ftc.config.MECANUM_
   STRAFE_SPEED_FACTOR after import does NOT change what a fresh
   Drivetrain.speed_and_drift_factor call returns for a pure-strafe
   step -- confirms the trap is real for this constant too.
2. check_drivetrain_module_patch_works: patching ftc.drivetrain.
   MECANUM_STRAFE_SPEED_FACTOR (what ftc/mecanum_robustness.py's
   run_sweep_point actually does) DOES change it.
3. check_drift_multiplier_config_patch_does_nothing /
4. check_drift_multiplier_drivetrain_module_patch_works: the identical
   pair of checks for MECANUM_STRAFE_DRIFT_MULTIPLIER.
5. check_tank_immune_to_strafe_patches: patching BOTH constants on
   ftc.drivetrain leaves TANK.speed_and_drift_factor at (1.0, 1.0) --
   the assumption run_all relies on to compute the tank baseline once
   and reuse it across every swept point, rather than recomputing it
   per point for no reason.
6. check_end_to_end_override_survives_through_run_match: run_sweep_
   point with speed_factor=1.0/drift_multiplier=1.0 (the "both at
   physical limit" best case) produces a measurably higher match_travel
   success rate than the unmodified baseline on the same scenario
   sequence -- the override has to survive all the way through ftc.
   match.run_match, not just be readable back off the module object
   immediately after being set.
7. check_overrides_restored_after_call: run_sweep_point restores
   ftc.drivetrain's module-level constants to their original values
   after returning (even via the `finally` on an exception path) --
   a leaked override would silently corrupt every later call in the
   same process, including ftc/drivetrain_benchmark.py's own numbers
   if run in the same session.
"""
import ftc.config as config_module
import ftc.drivetrain as drivetrain_module
from ftc.drivetrain import MECANUM, MECANUM_MATCH_TRAVEL, TANK
from ftc.field import build_grid, tag_sites_for
import ftc.mecanum_robustness as mecanum_robustness_module

LAYOUT = "corridor"


def _pure_strafe_factors(drivetrain):
    # chassis heading 0, travel heading 90 -> a full 90-degree offset,
    # i.e. a pure strafe -- the case MECANUM_STRAFE_SPEED_FACTOR/_DRIFT_
    # MULTIPLIER apply at full strength (strafe_frac == 1.0).
    return drivetrain.speed_and_drift_factor(chassis_heading_deg=0.0, step_heading_deg=90.0)


def check_config_patch_does_nothing():
    original = config_module.MECANUM_STRAFE_SPEED_FACTOR
    baseline = _pure_strafe_factors(MECANUM)
    try:
        config_module.MECANUM_STRAFE_SPEED_FACTOR = 1.0
        after_patch = _pure_strafe_factors(MECANUM)
    finally:
        config_module.MECANUM_STRAFE_SPEED_FACTOR = original
    ok = after_patch == baseline
    print(f"  baseline speed/drift factors={baseline}, after patching ftc.config={after_patch}")
    print(f"patching ftc.config.MECANUM_STRAFE_SPEED_FACTOR does NOT change speed_and_drift_factor's output "
          f"(confirms the trap is real): {'OK' if ok else 'FAIL'}")
    return ok


def check_drivetrain_module_patch_works():
    original = drivetrain_module.MECANUM_STRAFE_SPEED_FACTOR
    baseline = _pure_strafe_factors(MECANUM)
    try:
        drivetrain_module.MECANUM_STRAFE_SPEED_FACTOR = 1.0
        after_patch = _pure_strafe_factors(MECANUM)
    finally:
        drivetrain_module.MECANUM_STRAFE_SPEED_FACTOR = original
    ok = after_patch != baseline and after_patch == (1.0, baseline[1])
    print(f"  baseline speed/drift factors={baseline}, after patching ftc.drivetrain={after_patch}")
    print(f"patching ftc.drivetrain.MECANUM_STRAFE_SPEED_FACTOR DOES change speed_and_drift_factor's output "
          f"(the mechanism ftc/mecanum_robustness.py actually uses): {'OK' if ok else 'FAIL'}")
    return ok


def check_drift_multiplier_config_patch_does_nothing():
    original = config_module.MECANUM_STRAFE_DRIFT_MULTIPLIER
    baseline = _pure_strafe_factors(MECANUM)
    try:
        config_module.MECANUM_STRAFE_DRIFT_MULTIPLIER = 1.0
        after_patch = _pure_strafe_factors(MECANUM)
    finally:
        config_module.MECANUM_STRAFE_DRIFT_MULTIPLIER = original
    ok = after_patch == baseline
    print(f"patching ftc.config.MECANUM_STRAFE_DRIFT_MULTIPLIER does NOT change speed_and_drift_factor's "
          f"output: {'OK' if ok else 'FAIL'}")
    return ok


def check_drift_multiplier_drivetrain_module_patch_works():
    original = drivetrain_module.MECANUM_STRAFE_DRIFT_MULTIPLIER
    baseline = _pure_strafe_factors(MECANUM)
    try:
        drivetrain_module.MECANUM_STRAFE_DRIFT_MULTIPLIER = 1.0
        after_patch = _pure_strafe_factors(MECANUM)
    finally:
        drivetrain_module.MECANUM_STRAFE_DRIFT_MULTIPLIER = original
    ok = after_patch != baseline and after_patch == (baseline[0], 1.0)
    print(f"patching ftc.drivetrain.MECANUM_STRAFE_DRIFT_MULTIPLIER DOES change speed_and_drift_factor's "
          f"output: {'OK' if ok else 'FAIL'}")
    return ok


def check_tank_immune_to_strafe_patches():
    original_speed = drivetrain_module.MECANUM_STRAFE_SPEED_FACTOR
    original_drift = drivetrain_module.MECANUM_STRAFE_DRIFT_MULTIPLIER
    try:
        drivetrain_module.MECANUM_STRAFE_SPEED_FACTOR = 0.1
        drivetrain_module.MECANUM_STRAFE_DRIFT_MULTIPLIER = 9.0
        tank_factors = _pure_strafe_factors(TANK)
    finally:
        drivetrain_module.MECANUM_STRAFE_SPEED_FACTOR = original_speed
        drivetrain_module.MECANUM_STRAFE_DRIFT_MULTIPLIER = original_drift
    ok = tank_factors == (1.0, 1.0)
    print(f"TANK.speed_and_drift_factor stays (1.0, 1.0) under extreme strafe-constant patches: {tank_factors} "
          f"-- {'OK' if ok else 'FAIL'}")
    return ok


def check_end_to_end_override_survives_through_run_match():
    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)

    baseline_outcomes = mecanum_robustness_module.run_sweep_point(
        grid, free_cells, tag_sites, "mecanum_match_travel")
    best_case_outcomes = mecanum_robustness_module.run_sweep_point(
        grid, free_cells, tag_sites, "mecanum_match_travel", speed_factor=1.0, drift_multiplier=1.0)

    baseline_rate = sum(baseline_outcomes) / len(baseline_outcomes)
    best_case_rate = sum(best_case_outcomes) / len(best_case_outcomes)
    ok = best_case_rate > baseline_rate
    print(f"match_travel success rate: baseline={baseline_rate:.2%}, best-case (zero strafe penalty)="
          f"{best_case_rate:.2%} -- {'OK' if ok else 'FAIL'}")
    return ok


def check_overrides_restored_after_call():
    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)
    original_speed = drivetrain_module.MECANUM_STRAFE_SPEED_FACTOR
    original_drift = drivetrain_module.MECANUM_STRAFE_DRIFT_MULTIPLIER

    mecanum_robustness_module.run_sweep_point(grid, free_cells, tag_sites, "mecanum_match_travel",
                                                speed_factor=0.3, drift_multiplier=5.0)

    ok = (drivetrain_module.MECANUM_STRAFE_SPEED_FACTOR == original_speed
          and drivetrain_module.MECANUM_STRAFE_DRIFT_MULTIPLIER == original_drift)
    print(f"ftc.drivetrain's module constants restored after run_sweep_point returns: "
          f"speed_factor={drivetrain_module.MECANUM_STRAFE_SPEED_FACTOR} (orig {original_speed}), "
          f"drift_multiplier={drivetrain_module.MECANUM_STRAFE_DRIFT_MULTIPLIER} (orig {original_drift}) -- "
          f"{'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------


def test_config_patch_does_nothing():
    assert check_config_patch_does_nothing()


def test_drivetrain_module_patch_works():
    assert check_drivetrain_module_patch_works()


def test_drift_multiplier_config_patch_does_nothing():
    assert check_drift_multiplier_config_patch_does_nothing()


def test_drift_multiplier_drivetrain_module_patch_works():
    assert check_drift_multiplier_drivetrain_module_patch_works()


def test_tank_immune_to_strafe_patches():
    assert check_tank_immune_to_strafe_patches()


def test_end_to_end_override_survives_through_run_match():
    assert check_end_to_end_override_survives_through_run_match()


def test_overrides_restored_after_call():
    assert check_overrides_restored_after_call()


if __name__ == "__main__":
    checks = [
        check_config_patch_does_nothing(),
        check_drivetrain_module_patch_works(),
        check_drift_multiplier_config_patch_does_nothing(),
        check_drift_multiplier_drivetrain_module_patch_works(),
        check_tank_immune_to_strafe_patches(),
        check_end_to_end_override_survives_through_run_match(),
        check_overrides_restored_after_call(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
