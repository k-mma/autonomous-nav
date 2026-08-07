"""
Does ftc/budget_benchmark.py's budget-patching mechanism actually change
match outcomes, and is its "the example budgets in the ask do bind"
claim (current state, post-kinematics -- see ftc/match.py's
_trapezoidal_drive_time_s) true rather than assumed?

1. ftc.match.AUTONOMOUS_PERIOD_S is a plain module global that run_match
   looks up fresh on every call (unlike ftc/sensors.py's class-attribute
   trap) -- patching it changes a suite's success rate end to end,
   exactly the pattern ftc/scratch/match_test.py already proved for a
   single suite/scenario, checked here through
   ftc/budget_benchmark.py's own run_budget wrapper instead.
2. run_budget tags every row with the budget_s it ran under.
3. summarize's ranking and over_budget_rate math is sane: at an
   effectively infinite budget nothing is ever over_budget, and at a
   budget of 0.01s (smaller than a single cell's drive time, even under
   the old naive timing model let alone the current trapezoidal one)
   nothing can ever succeed.
4. A reduced probe (~600 matches across all 3 deviation types, high
   variance_level) finds a real elapsed_s distribution with a multi-
   second median, and a meaningful fraction of matches already over
   budget at 7s -- consistent with (not contradicting) the full 4,125-
   match-per-point sweep's own finding that budgets in the prompt's
   example range now bind for real, not just as a rare tail event.
"""
import statistics

import ftc.match as match_module
from ftc.budget_benchmark import run_budget, summarize
from ftc.field import build_grid, tag_sites_for
from ftc.sensors import SUITE_ORDER
from ftc.suite_benchmark import DEVIATION_TYPES, _solvable_scenario
from nav.field_variance import generate_ground_truth

LAYOUT = "cluttered"


def check_budget_patch_changes_outcome_end_to_end():
    """At an effectively infinite budget, nothing is ever over_budget. At
    0.01s (smaller than a single cell's drive time at MAX_DRIVE_SPEED_MPS),
    nothing can ever *succeed* -- but not every trial is recorded as
    over_budget specifically, since a trial that collides or gets stuck
    with no path exits ftc/match.py's run_match loop before the budget
    check ever runs (see ftc/match.py's collision/no-path `break`s, both
    ahead of the `elapsed_s > AUTONOMOUS_PERIOD_S` check). So the sane
    invariant at 0.01s is "success is always False," not "over_budget is
    always True.\""""
    original_budget = match_module.AUTONOMOUS_PERIOD_S
    try:
        huge_rows = run_budget(1000.0)
    finally:
        match_module.AUTONOMOUS_PERIOD_S = original_budget
    tiny_rows = run_budget(0.01)

    huge_over = sum(r["over_budget"] for r in huge_rows)
    tiny_success = sum(r["success"] for r in tiny_rows)
    tiny_over = sum(r["over_budget"] for r in tiny_rows)
    ok = huge_over == 0 and tiny_success == 0 and tiny_over > 0.8 * len(tiny_rows)
    print(f"  1000s budget: over_budget={huge_over}/{len(huge_rows)}")
    print(f"  0.01s budget: success={tiny_success}/{len(tiny_rows)}, over_budget={tiny_over}/{len(tiny_rows)}")
    print(f"budget patching changes outcomes end to end (never over_budget at 1000s, never succeeds at 0.01s, "
          f"mostly over_budget specifically): {'OK' if ok else 'FAIL'}")
    return ok


def check_run_budget_tags_rows():
    rows = run_budget(30.0)
    ok = len(rows) > 0 and all(r["budget_s"] == 30.0 for r in rows)
    print(f"  {len(rows)} rows, all tagged budget_s=30.0: {'OK' if ok else 'FAIL'}")
    print(f"run_budget adds a budget_s column to every row: {'OK' if ok else 'FAIL'}")
    return ok


def check_summarize_sane_at_extremes():
    huge_results, huge_ranking = summarize(run_budget(1000.0), 1000.0)
    tiny_results, tiny_ranking = summarize(run_budget(0.01), 0.01)
    ok = (all(huge_results[s]["over_budget_rate"] == 0.0 for s in SUITE_ORDER)
          and all(tiny_results[s]["over_budget_rate"] > 0.8 for s in SUITE_ORDER)
          and all(tiny_results[s]["rate"] == 0.0 for s in SUITE_ORDER))
    print(f"  huge-budget over_budget_rates={[huge_results[s]['over_budget_rate'] for s in SUITE_ORDER]}")
    print(f"  tiny-budget over_budget_rates={[tiny_results[s]['over_budget_rate'] for s in SUITE_ORDER]}")
    print(f"summarize's over_budget_rate/success rate are sane at both extremes: {'OK' if ok else 'FAIL'}")
    return ok


def check_example_budgets_now_bind():
    """Reduced probe (not the full 25-trial/11-level sweep) across all 3
    deviation types, at the high-variance levels where a run is most
    likely to take a while -- if the budget were going to bind anywhere
    in the prompt's suggested [7, 30] range, this is where it would show
    up first."""
    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)

    original_budget = match_module.AUTONOMOUS_PERIOD_S
    elapsed = []
    over_budget_at_7 = 0
    n = 0
    deviation_types = list(DEVIATION_TYPES)
    levels = [0.5, 0.7, 0.9, 1.0]
    try:
        for dt_idx, deviation_type in enumerate(deviation_types):
            scale_kwargs = DEVIATION_TYPES[deviation_type]
            for level_idx, level in enumerate(levels):
                for t in range(10):
                    # Index-based, not Python's built-in hash() on a str
                    # tuple -- hash() is randomized per-process for str
                    # objects (nav/stats.py's own module docstring warns
                    # about exactly this), which would make this probe's
                    # seeds, and therefore its scenarios, different on
                    # every run.
                    trial_seed = 8_900_000 + dt_idx * 10_000 + level_idx * 100 + t
                    start, goal = _solvable_scenario(trial_seed, grid, free_cells)
                    ground_truth, actual_start = generate_ground_truth(
                        grid, start, goal, level, seed=trial_seed, **scale_kwargs)
                    match_module.AUTONOMOUS_PERIOD_S = 1000.0
                    from ftc.sensors import SUITES
                    import random
                    for suite_name in SUITE_ORDER:
                        result = match_module.run_match(SUITES[suite_name](), grid, start, goal, ground_truth,
                                                          actual_start, tag_sites, random.Random(trial_seed))
                        elapsed.append(result.elapsed_s)
                    match_module.AUTONOMOUS_PERIOD_S = 7.0
                    for suite_name in SUITE_ORDER:
                        result = match_module.run_match(SUITES[suite_name](), grid, start, goal, ground_truth,
                                                          actual_start, tag_sites, random.Random(trial_seed))
                        over_budget_at_7 += result.over_budget
                        n += 1
    finally:
        match_module.AUTONOMOUS_PERIOD_S = original_budget

    elapsed.sort()
    median = elapsed[len(elapsed) // 2]
    p90 = elapsed[int(len(elapsed) * 0.9)]
    # Loose bounds around this probe's actual observed values (median
    # ~4.3s, p90 ~10.7s, over_budget_at_7 ~24% at the time this was
    # written) -- wide enough to tolerate this probe's own small-sample
    # noise across reruns, tight enough to catch a real regression back
    # toward the old (pre-kinematics) sub-2-second median.
    ok = 2.0 < median < 8.0 and 6.0 < p90 < 16.0 and over_budget_at_7 > 0.05 * n
    print(f"  elapsed_s: median={median:.2f}s p90={p90:.2f}s (n={len(elapsed)})")
    print(f"  over_budget at 7s budget: {over_budget_at_7}/{n}")
    print(f"elapsed_s distribution has shifted to a multi-second median under the trapezoidal drive-time "
          f"model, and 7s now binds for real (not a rare tail): {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_budget_patch_changes_outcome_end_to_end():
    assert check_budget_patch_changes_outcome_end_to_end()


def test_run_budget_tags_rows():
    assert check_run_budget_tags_rows()


def test_summarize_sane_at_extremes():
    assert check_summarize_sane_at_extremes()


def test_example_budgets_now_bind():
    assert check_example_budgets_now_bind()


if __name__ == "__main__":
    checks = [
        check_budget_patch_changes_outcome_end_to_end(),
        check_run_budget_tags_rows(),
        check_summarize_sane_at_extremes(),
        check_example_budgets_now_bind(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
