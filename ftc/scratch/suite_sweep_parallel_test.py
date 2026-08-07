"""
Does parallelizing ftc/suite_benchmark.py's run_sweep across processes
change a single result? It must not -- run_combo already derives every
trial's seed from base_seed(deviation_type, level) + trial_index, so no
state is shared between (deviation_type, level) combos, and farming
combos out to separate processes should be invisible in the output.
This is the check that actually proves that, by diffing real CSVs,
rather than just asserting it in a docstring.

Every column except planning_time_ms is compared. That column is real
wall-clock time (how long astar()/run_match() actually took to execute
on this machine, this run), not a seeded quantity -- it was never
reproducible run-to-run even before this change, parallel or not, and
ftc/scratch/fidelity_test.py's own COMPARE_COLUMNS already excludes it
for the same reason. Excluding it here isn't a workaround specific to
parallelism; it's this repo's existing, documented convention for what
"identical" means for a row that contains a timing measurement.

1. check_serial_and_parallel_produce_identical_rows: writes a reduced
   sweep's CSV under max_workers=1 (serial, in-process, no
   ProcessPoolExecutor involved at all) and again under the default
   parallel path, and diffs every non-timing column of every row.
2. check_worker_count_does_not_change_output: reruns the same reduced
   sweep at two different worker counts and confirms the rows still
   match -- catches a bug that happens to be invisible in the
   1-vs-parallel comparison above but shows up only when the number of
   workers itself changes (e.g. a race that needs 2+ workers racing).
"""
import csv
import tempfile
from pathlib import Path

from ftc.field import build_grid, tag_sites_for
from ftc.suite_benchmark import run_sweep, write_csv

# Deliberately small -- this test's job is to prove determinism, not to
# be a real sweep. 2 deviation types x 3 levels x 3 trials x 5 suites =
# 90 rows, fast enough to run 4x (once per check below) every time
# pytest runs.
LAYOUT = "cluttered"
DEVIATION_TYPES = ["start_drift", "obstacle_drift"]
LEVELS = [0.0, 0.5, 1.0]
TRIALS = 3
COMPARE_COLUMNS = ["suite", "deviation_type", "variance_level", "trial", "seed", "success", "over_budget",
                   "elapsed_s", "collisions", "replans", "final_pose_error_in", "steps", "cost_usd"]


def _sweep_rows_as_csv(max_workers, path):
    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)
    rows = run_sweep(DEVIATION_TYPES, LEVELS, TRIALS, grid, free_cells, tag_sites, max_workers=max_workers)
    write_csv(rows, path)
    with open(path) as f:
        return [tuple(row[col] for col in COMPARE_COLUMNS) for row in csv.DictReader(f)]


def check_serial_and_parallel_produce_identical_rows():
    with tempfile.TemporaryDirectory() as tmp:
        serial_rows = _sweep_rows_as_csv(1, Path(tmp) / "serial.csv")
        parallel_rows = _sweep_rows_as_csv(None, Path(tmp) / "parallel.csv")  # ProcessPoolExecutor's own default

        ok = serial_rows == parallel_rows
        print(f"  serial: {len(serial_rows)} rows, parallel: {len(parallel_rows)} rows")
        print(f"serial (max_workers=1) and parallel (default worker count) sweeps produce identical rows "
              f"(every column but planning_time_ms): {'OK' if ok else 'FAIL'}")
        return ok


def check_worker_count_does_not_change_output():
    with tempfile.TemporaryDirectory() as tmp:
        two_rows = _sweep_rows_as_csv(2, Path(tmp) / "two_workers.csv")
        three_rows = _sweep_rows_as_csv(3, Path(tmp) / "three_workers.csv")

        ok = two_rows == three_rows
        print(f"  2-worker: {len(two_rows)} rows, 3-worker: {len(three_rows)} rows")
        print(f"2-worker and 3-worker sweeps produce identical rows: {'OK' if ok else 'FAIL'}")
        return ok


# --- pytest entry points --------------------------------------------------


def test_serial_and_parallel_produce_identical_rows():
    assert check_serial_and_parallel_produce_identical_rows()


def test_worker_count_does_not_change_output():
    assert check_worker_count_does_not_change_output()


if __name__ == "__main__":
    checks = [
        check_serial_and_parallel_produce_identical_rows(),
        check_worker_count_does_not_change_output(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
