"""
Standalone proof for the one new piece of behavior ftc/
drivetrain_suite_benchmark.py needs before it can exist: ftc/
suite_benchmark.py's run_combo/run_sweep gained an optional
`drivetrain` parameter (a ftc.drivetrain.Drivetrain instance, or None)
threaded straight into run_match. Written before wiring it into the new
benchmark module, per this project's own ftc/scratch/ convention (see
e.g. ftc/scratch/fidelity_test.py, ftc/scratch/gearing_test.py for the
same "prove the new knob is a no-op by default, and a real effect when
used" shape).

1. check_default_drivetrain_reproduces_existing_behavior: run_combo
   called with no `drivetrain` argument at all (every existing caller --
   ftc/suite_benchmark.py's own __main__, ftc/layout_benchmark.py, ftc/
   fidelity_benchmark.py, ftc/budget_benchmark.py, ftc/
   opponent_benchmark.py, ftc/scripted_auto_benchmark.py) must keep
   producing byte-for-byte identical results to before this parameter
   existed. Checked two ways: (a) run_combo(drivetrain=None) is
   identical to run_combo() with the argument omitted entirely, and (b)
   both are identical to run_combo(drivetrain=DRIVETRAINS["tank"]) --
   confirming "no drivetrain" and "explicit tank" are the same model,
   exactly as ftc/drivetrain.py's own module docstring documents.
2. check_mecanum_drivetrain_changes_results: the identical scenario,
   suite-for-suite, run under DRIVETRAINS["mecanum"] instead, produces
   at least one measurably different row -- confirming the parameter
   actually reaches run_match rather than being silently ignored.
3. check_run_sweep_threads_drivetrain_through_parallel_workers: the
   same drivetrain parameter survives ProcessPoolExecutor -- run_sweep
   at max_workers=1 (serial) and max_workers=2 (parallel), both with an
   explicit mecanum drivetrain, produce byte-for-byte identical rows
   (mirroring ftc/scratch/parallel_determinism_test.py's own technique
   for the fidelity parameter).
"""
import random

from ftc.drivetrain import DRIVETRAINS
from ftc.field import build_grid, tag_sites_for
from ftc.suite_benchmark import DEVIATION_TYPE_ORDER, LAYOUT, run_combo, run_sweep

COMPARE_COLUMNS = ["suite", "deviation_type", "variance_level", "trial", "success", "over_budget",
                    "elapsed_s", "collisions", "replans", "final_pose_error_in", "steps", "cost_usd"]


def _grid_and_context():
    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)
    return grid, free_cells, tag_sites


def _rows_equal(a, b):
    return [{k: r[k] for k in COMPARE_COLUMNS} for r in a] == [{k: r[k] for k in COMPARE_COLUMNS} for r in b]


def check_default_drivetrain_reproduces_existing_behavior():
    grid, free_cells, tag_sites = _grid_and_context()
    rows_omitted = run_combo("start_drift", 0.5, 5, 8_800_000, grid, free_cells, tag_sites)
    rows_none = run_combo("start_drift", 0.5, 5, 8_800_000, grid, free_cells, tag_sites, drivetrain=None)
    rows_tank = run_combo("start_drift", 0.5, 5, 8_800_000, grid, free_cells, tag_sites,
                           drivetrain=DRIVETRAINS["tank"])

    omitted_vs_none = _rows_equal(rows_omitted, rows_none)
    none_vs_tank = _rows_equal(rows_none, rows_tank)
    ok = omitted_vs_none and none_vs_tank
    print(f"  drivetrain omitted == drivetrain=None: {omitted_vs_none}; "
          f"drivetrain=None == drivetrain=tank: {none_vs_tank}")
    print(f"No drivetrain, drivetrain=None, and drivetrain=tank are the identical model: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_mecanum_drivetrain_changes_results():
    grid, free_cells, tag_sites = _grid_and_context()
    rows_tank = run_combo("obstacle_drift", 0.7, 8, 8_900_000, grid, free_cells, tag_sites,
                           drivetrain=DRIVETRAINS["tank"])
    rows_mecanum = run_combo("obstacle_drift", 0.7, 8, 8_900_000, grid, free_cells, tag_sites,
                              drivetrain=DRIVETRAINS["mecanum"])

    identical = _rows_equal(rows_tank, rows_mecanum)
    differing_rows = sum(
        1 for t, m in zip(rows_tank, rows_mecanum)
        if any(t[k] != m[k] for k in ("success", "elapsed_s", "final_pose_error_in", "steps"))
    )
    ok = (not identical) and differing_rows > 0
    print(f"  {differing_rows}/{len(rows_tank)} rows differ between tank and mecanum")
    print(f"Passing a mecanum drivetrain measurably changes match outcomes: {'OK' if ok else 'FAIL'}")
    return ok


def check_run_sweep_threads_drivetrain_through_parallel_workers():
    grid, free_cells, tag_sites = _grid_and_context()
    levels = [0.3, 0.7]
    serial = run_sweep(DEVIATION_TYPE_ORDER, levels, 4, grid, free_cells, tag_sites,
                        drivetrain=DRIVETRAINS["mecanum"], max_workers=1)
    parallel = run_sweep(DEVIATION_TYPE_ORDER, levels, 4, grid, free_cells, tag_sites,
                          drivetrain=DRIVETRAINS["mecanum"], max_workers=2)

    ok = _rows_equal(serial, parallel)
    print(f"  {len(serial)} rows serial, {len(parallel)} rows parallel, identical={ok}")
    print(f"drivetrain survives ProcessPoolExecutor unchanged: {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_default_drivetrain_reproduces_existing_behavior():
    assert check_default_drivetrain_reproduces_existing_behavior()


def test_mecanum_drivetrain_changes_results():
    assert check_mecanum_drivetrain_changes_results()


def test_run_sweep_threads_drivetrain_through_parallel_workers():
    assert check_run_sweep_threads_drivetrain_through_parallel_workers()


if __name__ == "__main__":
    checks = [
        check_default_drivetrain_reproduces_existing_behavior(),
        check_mecanum_drivetrain_changes_results(),
        check_run_sweep_threads_drivetrain_through_parallel_workers(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
