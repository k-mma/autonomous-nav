"""
Same question as ftc/scratch/parallel_determinism_test.py, for
nav/uncertainty_benchmark.py's run_sweep instead of
ftc/suite_benchmark.py's: does parallelizing the headline
open-loop/reactive/belief sweep across processes change a single
result? It must not -- run_variance_level already derives every trial's
seed from base_seed(level) + trial_index, so no state is shared between
levels.

planning_time_s is excluded from the comparison for the same reason
ftc/scratch/fidelity_test.py's COMPARE_COLUMNS excludes
planning_time_ms: it's real wall-clock time, never reproducible
run-to-run regardless of parallelism.
"""
import csv
import tempfile
from pathlib import Path

from nav.uncertainty_benchmark import run_sweep, write_csv

# Small and cheap -- 3 levels x 3 trials x 3 policies = 27 rows, run 4x
# (once per check below) every time pytest runs.
LEVELS = [0.0, 0.5, 1.0]
TRIALS = 3
COMPARE_COLUMNS = ["policy", "variance_level", "trial", "seed", "success", "path_cost", "steps", "replans",
                   "collisions"]


def _sweep_rows(max_workers, path):
    rows = run_sweep(LEVELS, TRIALS, max_workers=max_workers)
    write_csv(rows, path)
    with open(path) as f:
        return [tuple(row[col] for col in COMPARE_COLUMNS) for row in csv.DictReader(f)]


def check_serial_and_parallel_produce_identical_rows():
    with tempfile.TemporaryDirectory() as tmp:
        serial_rows = _sweep_rows(1, Path(tmp) / "serial.csv")
        parallel_rows = _sweep_rows(None, Path(tmp) / "parallel.csv")

        ok = serial_rows == parallel_rows
        print(f"  serial: {len(serial_rows)} rows, parallel: {len(parallel_rows)} rows")
        print(f"serial (max_workers=1) and parallel (default worker count) sweeps produce identical rows "
              f"(every column but planning_time_s): {'OK' if ok else 'FAIL'}")
        return ok


def check_worker_count_does_not_change_output():
    with tempfile.TemporaryDirectory() as tmp:
        two_rows = _sweep_rows(2, Path(tmp) / "two_workers.csv")
        three_rows = _sweep_rows(3, Path(tmp) / "three_workers.csv")

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
