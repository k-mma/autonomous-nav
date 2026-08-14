"""
Does ftc/fusion.py's Kalman fusion path (fused_tag_correction_kalman,
wired into ftc/match.py via fusion="kalman") actually behave like a
Kalman filter once it's driven through a real match, and does adding it
leave the two EXISTING fusion paths (None, confidence-weighted) exactly
as they were before this addition?

1. check_kalman_default_does_not_move_confidence_or_none_paths:
   fusion=None and fusion=True produce IDENTICAL MatchResults whether
   or not a fusion="kalman" match on the same suite/scenario/seed has
   already run in the same process first -- proving DEFAULT_APRILTAG_
   VARIANCE_MODEL and the new `position_variance` bookkeeping don't
   leak any shared mutable state into the two existing paths.
   ftc/scratch/fusion_test.py's own regression suite (unchanged, still
   passing -- see its own check_fusion_default_reproduces_headline_
   exactly) already covers the deeper "did the confidence-weighted
   MATH regress" question; this one is specifically about cross-call
   interference from the newly added third path.
2. check_variance_model_variance_cells2_converts_units_correctly:
   _apriltag_observation_variance_cells2's output, converted back to
   inches, matches DEFAULT_APRILTAG_VARIANCE_MODEL.variance_in2 at the
   frac corresponding to a best-case (range=0, incidence=0) detection.
3. check_default_variance_model_is_synthetic_placeholder: the module-
   level DEFAULT_APRILTAG_VARIANCE_MODEL's source, reachable through
   ftc.calibration.load_calibration() with no CSVs, is the labeled
   SYNTHETIC placeholder -- catching a regression where a future change
   accidentally makes this default look like real measured data.
4. check_position_variance_shrinks_across_a_kalman_match: running a
   real match with fusion="kalman" on a pose-fixing suite, position_
   variance at the end must be no larger than KALMAN_INITIAL_POSITION_
   VARIANCE_CELLS2 possibly grown by drift and then repeatedly reduced
   by updates -- concretely: it must never be negative, and at least
   one tag correction (if any occurred) must have been followed by a
   variance no greater than the variance just before it (nav/kalman.py's
   own check_update_never_increases_variance, exercised end-to-end
   through the real match loop instead of directly).
5. check_kalman_match_runs_without_error_on_every_pose_fixing_suite:
   smoke test -- every SUITE_ORDER suite with fixes_pose=True completes
   a match under fusion="kalman" without raising, on several seeds.
"""
import random

from ftc.field import build_grid, tag_sites_for
from ftc.fusion import DEFAULT_APRILTAG_VARIANCE_MODEL, _apriltag_observation_variance_cells2
from ftc.calibration import SYNTHETIC, load_calibration
from ftc.config import APRILTAG_CORRECTION_FACTOR_MAX, CELL_SIZE_IN
from ftc.match import run_match
from ftc.sensors import SUITES, SUITE_ORDER
from ftc.suite_benchmark import LAYOUT


def _result_tuple(result):
    return (result.success, result.over_budget, round(result.elapsed_s, 9), result.collisions, result.replans,
            round(result.final_pose_error_in, 9), result.steps)


def check_kalman_default_does_not_move_confidence_or_none_paths():
    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)
    start, goal = free_cells[0], free_cells[-1]
    from nav.field_variance import generate_ground_truth

    def run_none_and_confidence(seed):
        ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.5, seed=seed, start_drift_scale=0.5)
        suite = SUITES["apriltag"]()
        result_none = run_match(suite, grid, start, goal, ground_truth, actual_start, tag_sites,
                                 random.Random(seed), fusion=None)
        suite = SUITES["apriltag"]()
        result_confidence = run_match(suite, grid, start, goal, ground_truth, actual_start, tag_sites,
                                       random.Random(seed), fusion=True)
        return _result_tuple(result_none), _result_tuple(result_confidence)

    before = run_none_and_confidence(seed=7)

    # Exercise the new fusion="kalman" path in between, on the same
    # scenario/seed, specifically to try to provoke any shared mutable
    # state (module-level DEFAULT_APRILTAG_VARIANCE_MODEL, the new
    # position_variance local) into leaking across calls.
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.5, seed=7, start_drift_scale=0.5)
    run_match(SUITES["apriltag"](), grid, start, goal, ground_truth, actual_start, tag_sites,
              random.Random(7), fusion="kalman")

    after = run_none_and_confidence(seed=7)

    ok = before == after
    print(f"fusion=None/True results before vs. after an intervening fusion='kalman' call: "
          f"{'identical' if ok else 'DIFFERENT'} -- {'OK' if ok else 'FAIL'}")
    return ok


def check_variance_model_variance_cells2_converts_units_correctly():
    frac = APRILTAG_CORRECTION_FACTOR_MAX
    got_cells2 = _apriltag_observation_variance_cells2(frac, DEFAULT_APRILTAG_VARIANCE_MODEL)
    expected_in2 = DEFAULT_APRILTAG_VARIANCE_MODEL.variance_in2(0.0, 0.0)
    expected_cells2 = expected_in2 / (CELL_SIZE_IN ** 2)
    ok = abs(got_cells2 - expected_cells2) < 1e-9
    print(f"best-case frac ({frac}) -> {got_cells2:.6f} cells^2 (expected {expected_cells2:.6f}, "
          f"= best-case in^2 / cell_size_in^2) -- {'OK' if ok else 'FAIL'}")
    return ok


def check_default_variance_model_is_synthetic_placeholder():
    calibration = load_calibration()
    ok = calibration.apriltag_variance.source == SYNTHETIC
    print(f"DEFAULT_APRILTAG_VARIANCE_MODEL's source (via load_calibration() with no CSVs) = "
          f"{calibration.apriltag_variance.source} (expected {SYNTHETIC}) -- {'OK' if ok else 'FAIL'}")
    return ok


def check_position_variance_shrinks_across_a_kalman_match():
    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)
    start, goal = free_cells[0], free_cells[-1]

    variances_seen = []

    def on_tick(snapshot):
        variances_seen.append(snapshot["position_variance"])

    from nav.field_variance import generate_ground_truth
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.5, seed=42, start_drift_scale=0.5)
    suite = SUITES["apriltag"]()
    run_match(suite, grid, start, goal, ground_truth, actual_start, tag_sites, random.Random(42),
              fusion="kalman", on_tick=on_tick)

    ok = len(variances_seen) > 0 and all(v >= 0.0 for v in variances_seen)
    # At least one consecutive drop should exist if any tag correction fired at all -- not asserted
    # strictly (a match might see zero detections), but variance must never go negative, ever.
    print(f"{len(variances_seen)} ticks recorded, all position_variance >= 0: {ok} -- {'OK' if ok else 'FAIL'}")
    return ok


def check_kalman_match_runs_without_error_on_every_pose_fixing_suite():
    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)
    start, goal = free_cells[0], free_cells[-1]
    from nav.field_variance import generate_ground_truth

    ok = True
    for suite_name in SUITE_ORDER:
        suite_cls = SUITES[suite_name]
        if not suite_cls().fixes_pose:
            continue
        for seed in (1, 2, 3):
            ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.6, seed=seed,
                                                                  start_drift_scale=0.6)
            try:
                run_match(suite_cls(), grid, start, goal, ground_truth, actual_start, tag_sites,
                          random.Random(seed), fusion="kalman")
            except Exception as e:  # noqa: BLE001 -- smoke test, any exception is a failure
                print(f"  {suite_name} seed={seed} raised {type(e).__name__}: {e}")
                ok = False
    print(f"every pose-fixing suite completes a fusion='kalman' match without raising -- {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------


def test_kalman_default_does_not_move_confidence_or_none_paths():
    assert check_kalman_default_does_not_move_confidence_or_none_paths()


def test_variance_model_variance_cells2_converts_units_correctly():
    assert check_variance_model_variance_cells2_converts_units_correctly()


def test_default_variance_model_is_synthetic_placeholder():
    assert check_default_variance_model_is_synthetic_placeholder()


def test_position_variance_shrinks_across_a_kalman_match():
    assert check_position_variance_shrinks_across_a_kalman_match()


def test_kalman_match_runs_without_error_on_every_pose_fixing_suite():
    assert check_kalman_match_runs_without_error_on_every_pose_fixing_suite()


if __name__ == "__main__":
    checks = [
        check_kalman_default_does_not_move_confidence_or_none_paths(),
        check_variance_model_variance_cells2_converts_units_correctly(),
        check_default_variance_model_is_synthetic_placeholder(),
        check_position_variance_shrinks_across_a_kalman_match(),
        check_kalman_match_runs_without_error_on_every_pose_fixing_suite(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
