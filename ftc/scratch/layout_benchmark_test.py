"""
Does ftc/layout_benchmark.py actually generalize ftc/suite_benchmark.py's
sweep across layouts, without silently drifting from what the headline
'cluttered' run itself measures?

1. LAYOUT_ORDER covers exactly the three layouts ftc/field.py ships,
   cluttered listed second (matching ftc/suite_benchmark.py's choice of
   it as the headline layout).
2. run_layout tags every row with the layout it came from.
3. The seed-formula consistency claim in ftc_layout_writeup.md's own
   "Consistency check" section is true, not just asserted in prose:
   run_layout("cluttered") and a direct call to ftc/suite_benchmark.py's
   own run_combo with the identical base_seed formula produce the exact
   same success/collision outcomes, trial for trial.
4. value_ranking excludes the free dead_reckoning baseline from its
   result dict (it's the denominator, not a candidate) and always
   returns a best suite drawn from the suites it did rank.

Uses a drastically reduced sweep (1 variance_level, 3 trials/point,
patched directly onto ftc.layout_benchmark's module globals -- these
are looked up fresh on every run_layout() call, unlike ftc/sensors.py's
class-attribute trap that ftc/robustness.py's docstring warns about) so
this runs in well under a second instead of ftc_layout_benchmark.py's
real ~55s three-layout sweep.
"""
import ftc.layout_benchmark as layout_benchmark_module
from ftc.field import LAYOUTS, build_grid, tag_sites_for
from ftc.layout_benchmark import LAYOUT_LABELS, LAYOUT_ORDER, run_layout, value_ranking
from ftc.suite_benchmark import run_combo


def check_layout_order():
    ok = LAYOUT_ORDER == ["sparse", "cluttered", "corridor"] and set(LAYOUT_ORDER) == set(LAYOUTS)
    ok = ok and set(LAYOUT_LABELS) == set(LAYOUT_ORDER)
    print(f"  LAYOUT_ORDER={LAYOUT_ORDER}")
    print(f"LAYOUT_ORDER matches ftc/field.py's layouts, cluttered listed second, every layout labeled: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_run_layout_tags_rows():
    original_levels, original_trials = layout_benchmark_module.VARIANCE_LEVELS, layout_benchmark_module.TRIALS_PER_COMBO
    try:
        layout_benchmark_module.VARIANCE_LEVELS = [0.5]
        layout_benchmark_module.TRIALS_PER_COMBO = 3
        rows = run_layout("sparse")
    finally:
        layout_benchmark_module.VARIANCE_LEVELS = original_levels
        layout_benchmark_module.TRIALS_PER_COMBO = original_trials
    ok = len(rows) > 0 and all(r["layout"] == "sparse" for r in rows)
    print(f"  {len(rows)} rows, all tagged layout='sparse': {'OK' if ok else 'FAIL'}")
    print(f"run_layout adds a layout column to every row: {'OK' if ok else 'FAIL'}")
    return ok


def check_cluttered_seed_formula_matches_suite_benchmark():
    original_levels, original_trials = layout_benchmark_module.VARIANCE_LEVELS, layout_benchmark_module.TRIALS_PER_COMBO
    reduced_levels, reduced_trials = [0.3, 0.7], 3
    try:
        layout_benchmark_module.VARIANCE_LEVELS = reduced_levels
        layout_benchmark_module.TRIALS_PER_COMBO = reduced_trials
        via_layout_benchmark = run_layout("cluttered")
    finally:
        layout_benchmark_module.VARIANCE_LEVELS = original_levels
        layout_benchmark_module.TRIALS_PER_COMBO = original_trials

    grid = build_grid("cluttered")
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for("cluttered")
    via_suite_benchmark = []
    for deviation_type in ["start_drift", "obstacle_drift", "unplanned_blocker"]:
        for level in reduced_levels:
            base_seed = 6_000_000 + ["start_drift", "obstacle_drift", "unplanned_blocker"].index(deviation_type) \
                * 1_000_000 + round(level * 100)
            via_suite_benchmark.extend(
                run_combo(deviation_type, level, reduced_trials, base_seed, grid, free_cells, tag_sites))

    key = lambda r: (r["suite"], r["deviation_type"], r["variance_level"], r["trial"])
    a = {key(r): (r["success"], r["collisions"], r["final_pose_error_in"]) for r in via_layout_benchmark}
    b = {key(r): (r["success"], r["collisions"], r["final_pose_error_in"]) for r in via_suite_benchmark}
    ok = a == b
    print(f"  {len(a)} rows compared, identical outcomes: {'OK' if ok else 'FAIL'}")
    print("run_layout('cluttered') reproduces ftc/suite_benchmark.py's own run_combo trial-for-trial "
          f"(same seed formula): {'OK' if ok else 'FAIL'}")
    return ok


def check_value_ranking_excludes_baseline():
    original_levels, original_trials = layout_benchmark_module.VARIANCE_LEVELS, layout_benchmark_module.TRIALS_PER_COMBO
    try:
        layout_benchmark_module.VARIANCE_LEVELS = [0.5]
        layout_benchmark_module.TRIALS_PER_COMBO = 5
        rows = run_layout("corridor")
    finally:
        layout_benchmark_module.VARIANCE_LEVELS = original_levels
        layout_benchmark_module.TRIALS_PER_COMBO = original_trials
    results, baseline_rate, best = value_ranking(rows, "corridor")
    ok = "dead_reckoning" not in results and best in results and 0.0 <= baseline_rate <= 1.0
    print(f"  best={best}, baseline_rate={baseline_rate:.2f}, ranked suites={list(results)}")
    print(f"value_ranking excludes dead_reckoning and returns a valid best suite: {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_layout_order():
    assert check_layout_order()


def test_run_layout_tags_rows():
    assert check_run_layout_tags_rows()


def test_cluttered_seed_formula_matches_suite_benchmark():
    assert check_cluttered_seed_formula_matches_suite_benchmark()


def test_value_ranking_excludes_baseline():
    assert check_value_ranking_excludes_baseline()


if __name__ == "__main__":
    checks = [
        check_layout_order(),
        check_run_layout_tags_rows(),
        check_cluttered_seed_formula_matches_suite_benchmark(),
        check_value_ranking_excludes_baseline(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
