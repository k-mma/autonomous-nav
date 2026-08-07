"""
Does run_match's fusion=None default actually reproduce every existing
caller's behavior exactly, and does ftc/fusion.py's confidence-weighted
fusion do the two things it's supposed to once enabled -- partially
damp a small systematic bias (case 1), and down-weight an outright bad
detection rather than trust it at face value (case 2)?

1. check_fusion_default_reproduces_headline_exactly: the same pattern
   ftc/scratch/fidelity_test.py's check_optimistic_tier_reproduces_
   headline_exactly uses -- reruns a sample of the exact trials behind
   benchmark_results/ftc_suite_results.csv via ftc.suite_benchmark.
   run_combo (which never passes `fusion`, so every call takes the
   default) and diffs every column except planning_time_ms against the
   checked-in CSV. This is the regression guarantee that adding fusion
   support didn't move a single published number.
2. check_fusion_none_matches_omitting_it: run_match called with
   fusion=None explicitly vs. fusion omitted entirely produce identical
   MatchResults on an AprilTag+odometry bundle -- confirms None really
   is the default, not just documented as one.
3. check_bad_detection_gets_downweighted: a seed chosen (by brute-force
   search, not luck) to land in fused_tag_correction's bad-detection
   branch on its first draw produces a fused result measurably CLOSER
   to odometry's prior than an undamped (distrust_factor=1.0) blend of
   the same two estimates would have been -- proving the distrust cut
   actually pulls the result back toward the trusted source, not just
   that the code path is reachable.
4. check_systematic_bias_is_partially_damped: averaged over many seeds
   forced into the ORDINARY (non-bad) branch, the fused result's
   deviation from the bias-free correction is smaller than the raw
   bias itself -- the bias is damped, not fully absorbed (which the old
   plain frac-based model would do, having no concept of bias at all)
   and not fully rejected either.
"""
import csv
import random
from pathlib import Path

from nav.estimation import PositionEstimate, fuse

import ftc.config as config_module
from ftc.config import APRILTAG_BAD_DETECTION_PROBABILITY, APRILTAG_SYSTEMATIC_BIAS_CELLS
from ftc.field import build_grid, tag_sites_for
from ftc.fusion import fused_tag_correction
from ftc.match import run_match
from ftc.suite_benchmark import DEVIATION_TYPES, LAYOUT, TRIALS_PER_COMBO, run_combo

RESULTS_CSV = Path(__file__).resolve().parent.parent.parent / "benchmark_results" / "ftc_suite_results.csv"
SAMPLE_POINTS = [("start_drift", 0.0), ("start_drift", 1.0), ("obstacle_drift", 0.5)]
COMPARE_COLUMNS = ["success", "over_budget", "elapsed_s", "collisions", "replans",
                   "final_pose_error_in", "steps", "cost_usd"]


def check_fusion_default_reproduces_headline_exactly():
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
    print(f"fusion=None default (never passed by run_combo) reproduces {RESULTS_CSV.name} trial-for-trial: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def _bundle_scenario():
    from ftc.bundle import make_bundle
    from nav.algorithms import astar
    from nav.field_variance import generate_ground_truth
    grid = build_grid("sparse")
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for("sparse")
    rng = random.Random(7)
    start = goal = None
    for _ in range(50):
        start, goal = rng.sample(free, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= 10:
            break
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.6, seed=7, start_drift_scale=1.0)
    return make_bundle("apriltag", "odometry_pods"), grid, start, goal, ground_truth, actual_start, tag_sites


def check_fusion_none_matches_omitting_it():
    """Compares every MatchResult field except planning_time_s -- real
    wall-clock time, never reproducible run-to-run regardless of this
    (or any other) change, the same trap ftc/scratch/fidelity_test.py's
    COMPARE_COLUMNS already documents."""
    bundle, grid, start, goal, ground_truth, actual_start, tag_sites = _bundle_scenario()
    explicit = run_match(bundle.fresh(), grid, start, goal, ground_truth, actual_start, tag_sites,
                          random.Random(3), fusion=None)
    omitted = run_match(bundle.fresh(), grid, start, goal, ground_truth, actual_start, tag_sites,
                         random.Random(3))

    fields = [f for f in explicit.__dataclass_fields__ if f != "planning_time_s"]
    mismatches = [f for f in fields if getattr(explicit, f) != getattr(omitted, f)]
    ok = not mismatches
    print(f"  fusion=None: {explicit}")
    print(f"  fusion omitted: {omitted}")
    if mismatches:
        print(f"  MISMATCHES (excluding planning_time_s): {mismatches}")
    print(f"fusion=None explicit matches fusion omitted entirely: {'OK' if ok else 'FAIL'}")
    return ok


def _find_seed_for_branch(want_bad, start_seed=0, limit=200):
    """Brute-force the first integer seed whose FIRST rng.random() draw
    lands on the wanted side of APRILTAG_BAD_DETECTION_PROBABILITY --
    fused_tag_correction's own bad-detection roll is exactly that first
    draw, so this reproduces its branch choice without depending on
    fused_tag_correction's internals staying at this exact call order
    forever (if it changes, this search just takes longer or fails
    loudly, not silently)."""
    for seed in range(start_seed, start_seed + limit):
        rng = random.Random(seed)
        is_bad = rng.random() < APRILTAG_BAD_DETECTION_PROBABILITY
        if is_bad == want_bad:
            return seed
    raise RuntimeError(f"no seed in [{start_seed}, {start_seed + limit}) lands on want_bad={want_bad}")


def check_bad_detection_gets_downweighted():
    seed = _find_seed_for_branch(want_bad=True)
    error = (2.0, -1.0)
    frac = 0.6

    fused_value = fused_tag_correction(error, frac, random.Random(seed))

    # What an UNDAMPED blend (distrust_factor=1.0 -- the bad reading
    # trusted at its nominal confidence, no outlier handling at all)
    # would have produced on the exact same draw, for comparison.
    rng_replay = random.Random(seed)
    is_bad = rng_replay.random() < APRILTAG_BAD_DETECTION_PROBABILITY
    assert is_bad, "seed search found a seed that isn't actually the bad branch"
    bad_value = (error[0] + rng_replay.gauss(0, config_module.APRILTAG_BAD_DETECTION_SIGMA_CELLS),
                 error[1] + rng_replay.gauss(0, config_module.APRILTAG_BAD_DETECTION_SIGMA_CELLS))
    undamped = fuse(PositionEstimate(error, config_module.ODOMETRY_FUSION_CONFIDENCE),
                     PositionEstimate(bad_value, config_module.APRILTAG_FUSION_CONFIDENCE * frac),
                     disagreement_threshold=config_module.FUSION_DISAGREEMENT_THRESHOLD_CELLS,
                     distrust_factor=1.0).value

    def dist(a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    fused_dist_from_prior = dist(fused_value, error)
    undamped_dist_from_prior = dist(undamped, error)
    ok = fused_dist_from_prior < undamped_dist_from_prior
    print(f"  seed={seed}, bad reading={bad_value}, prior={error}")
    print(f"  fused (distrust active): {fused_value}, distance from prior: {fused_dist_from_prior:.3f}")
    print(f"  undamped (distrust_factor=1.0): {undamped}, distance from prior: {undamped_dist_from_prior:.3f}")
    print(f"a bad detection's confidence cut pulls the fused result closer to the trusted prior than an "
          f"undamped blend of the same reading would: {'OK' if ok else 'FAIL'}")
    return ok


def check_systematic_bias_is_partially_damped():
    """Isolates the bias's own marginal effect on the fused result --
    NOT its distance from plain_corrected, which also reflects how far
    odometry's raw prior sits from the tag's implied correction (a
    real, legitimate effect whenever frac is large, unrelated to bias).
    Comparing fused-with-bias against fused-without-bias on the
    identical (error, frac, seed) holds that effect fixed and isolates
    just what the constant bias itself contributed."""
    error = (1.5, 0.5)
    frac = 0.7
    plain_corrected = (error[0] * (1 - frac), error[1] * (1 - frac))
    full_bias_magnitude = APRILTAG_SYSTEMATIC_BIAS_CELLS * (2 ** 0.5)

    deviations = []
    n_good = 0
    seed = 0
    while n_good < 30:
        rng = random.Random(seed)
        is_bad = rng.random() < APRILTAG_BAD_DETECTION_PROBABILITY
        seed += 1
        if is_bad:
            continue
        n_good += 1

        with_bias = fused_tag_correction(error, frac, random.Random(seed - 1))
        no_bias = fuse(PositionEstimate(error, config_module.ODOMETRY_FUSION_CONFIDENCE),
                        PositionEstimate(plain_corrected, config_module.APRILTAG_FUSION_CONFIDENCE * frac),
                        disagreement_threshold=config_module.FUSION_DISAGREEMENT_THRESHOLD_CELLS,
                        distrust_factor=config_module.FUSION_DISTRUST_FACTOR).value

        deviation = ((with_bias[0] - no_bias[0]) ** 2 + (with_bias[1] - no_bias[1]) ** 2) ** 0.5
        deviations.append(deviation)

    mean_deviation = sum(deviations) / len(deviations)
    # A full, unweighted pass-through of the bias (the old model's
    # behavior for anything it COULD represent, i.e. never -- it has no
    # bias concept at all) would move the result by exactly
    # full_bias_magnitude. Damped means strictly less than that, and
    # strictly more than 0 (the bias isn't rejected outright either).
    ok = 0 < mean_deviation < full_bias_magnitude
    print(f"  mean bias-attributable deviation over {n_good} ordinary detections: {mean_deviation:.4f} cells")
    print(f"  full unweighted bias magnitude: {full_bias_magnitude:.4f} cells")
    print(f"the systematic bias's own contribution is damped (nonzero, less than full pass-through): "
          f"{'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------


def test_fusion_default_reproduces_headline_exactly():
    assert check_fusion_default_reproduces_headline_exactly()


def test_fusion_none_matches_omitting_it():
    assert check_fusion_none_matches_omitting_it()


def test_bad_detection_gets_downweighted():
    assert check_bad_detection_gets_downweighted()


def test_systematic_bias_is_partially_damped():
    assert check_systematic_bias_is_partially_damped()


if __name__ == "__main__":
    checks = [
        check_fusion_default_reproduces_headline_exactly(),
        check_fusion_none_matches_omitting_it(),
        check_bad_detection_gets_downweighted(),
        check_systematic_bias_is_partially_damped(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
