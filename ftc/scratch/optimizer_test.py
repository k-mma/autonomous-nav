"""
Does ftc/optimizer.py's search and statistics do what the writeup claims
they do?

The dangerous failure here isn't a crash -- it's a plausible-looking
recommendation built on a comparison that was never valid. So these
checks target the machinery the conclusions rest on, not the plumbing:

1. check_candidates_share_identical_scenarios: every candidate really
   does run the same scenarios, which is the precondition for every
   paired statistic in the module. If this fails, "significant" means
   nothing anywhere else.
2. check_paired_bootstrap_beats_independent_cis: on a constructed case
   where B beats A on the same trials, the paired test detects it while
   overlapping independent CIs would not -- the reason for adding
   nav/stats.py's bootstrap_paired_diff_ci at all.
3. check_paired_bootstrap_rejects_a_wash: the same machinery says NO
   when the two candidates are equivalent -- a test that only ever
   confirms is not a test.
4. check_no_synergy_when_bundling_a_no_op: bundling a suite with one
   that adds no hardware it doesn't already have reports zero gain, not
   a spurious one.
5. check_pareto_frontier_is_actually_undominated: nothing on the
   frontier is beaten on both axes, and nothing off it is missing.
6. check_evaluation_cache_is_by_hardware: two differently-named bundles
   that buy the same parts are evaluated once and get identical results.
7. check_budget_and_objective_are_respected: the budget pick never
   exceeds the budget, and worst_case ranking genuinely optimizes the
   worst profile rather than the mean.
"""
import random

from nav.stats import bootstrap_ci, bootstrap_paired_diff_ci

from ftc.bundle import make_bundle
from ftc.optimizer import (
    BundleOptimizer, ScenarioProfile, best_under_budget, build_scenarios, exhaustive_search,
    pareto_frontier, rank, report_synergy,
)

# Two cheap profiles -- these checks are about the search and the
# statistics, not about producing publication-grade rates.
TEST_PROFILES = [
    ScenarioProfile("pose", "Pose drift", variance_level=0.5, start_drift_scale=1.0),
    ScenarioProfile("map", "Map error", variance_level=0.5, obstacle_drift_scale=1.0),
]
TEST_COMPONENTS = ["dead_reckoning", "odometry_pods", "apriltag", "distance_sensors"]


def _optimizer(trials=8):
    return BundleOptimizer(profiles=TEST_PROFILES, trials_per_profile=trials, base_seed=31_000_000)


def check_candidates_share_identical_scenarios():
    a = build_scenarios(TEST_PROFILES[0], 8, 31_000_000)
    b = build_scenarios(TEST_PROFILES[0], 8, 31_000_000)
    same_object = a is b  # cached, so no candidate can perturb another's scenarios
    same_content = all(x.seed == y.seed and x.start == y.start and x.goal == y.goal
                       and x.actual_start == y.actual_start for x, y in zip(a, b))
    different_profile = build_scenarios(TEST_PROFILES[1], 8, 31_000_000)
    # Different profiles must NOT be the same scenarios -- if the cache
    # key ignored the profile, every profile would silently measure the
    # same thing.
    distinct = any(x.ground_truth.cells != y.ground_truth.cells for x, y in zip(a, different_profile))
    ok = same_object and same_content and distinct
    print(f"  cached={same_object}, identical={same_content}, profiles differ from each other={distinct}")
    print(f"Every candidate is scored on identical scenarios per profile: {'OK' if ok else 'FAIL'}")
    return ok


def check_paired_bootstrap_beats_independent_cis():
    # 40 paired trials. Scenario difficulty dominates: both fail the
    # same 20 hard ones. B additionally wins 6 of the 20 that A loses --
    # a consistent, real 15-point edge with heavy shared variance.
    a = [1] * 14 + [0] * 6 + [0] * 20
    b = [1] * 14 + [1] * 6 + [0] * 20
    lo, hi, p = bootstrap_paired_diff_ci(a, b, seed=1)
    paired_detects = lo > 0 and p < 0.05

    a_lo, a_hi = bootstrap_ci(sum(a), len(a), seed=2)
    b_lo, b_hi = bootstrap_ci(sum(b), len(b), seed=3)
    independent_overlap = not (b_lo > a_hi or a_lo > b_hi)

    ok = paired_detects and independent_overlap
    print(f"  A={sum(a)}/{len(a)} CI [{a_lo:.0%}, {a_hi:.0%}]; B={sum(b)}/{len(b)} CI [{b_lo:.0%}, {b_hi:.0%}]"
          f" -> independent CIs overlap: {independent_overlap}")
    print(f"  paired diff CI [{lo:+.1%}, {hi:+.1%}], p={p:.4f} -> detects the difference: {paired_detects}")
    print(f"The paired test finds a real effect that overlapping independent CIs would have missed: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_paired_bootstrap_rejects_a_wash():
    rng = random.Random(7)
    trials = [rng.random() < 0.5 for _ in range(40)]
    a = [int(t) for t in trials]
    b = list(a)                     # literally the same outcomes
    lo, hi, p = bootstrap_paired_diff_ci(a, b, seed=4)
    identical_ok = lo == 0.0 and hi == 0.0 and p == 1.0

    # And a coin-flip difference in both directions is not called
    # significant either.
    c = [int(rng.random() < 0.5) for _ in range(40)]
    d = [int(rng.random() < 0.5) for _ in range(40)]
    lo2, hi2, p2 = bootstrap_paired_diff_ci(c, d, seed=5)
    noise_ok = not (lo2 > 0 and p2 < 0.05)
    ok = identical_ok and noise_ok
    print(f"  identical vectors -> CI [{lo:+.1%}, {hi:+.1%}], p={p:.3f}")
    print(f"  two coin flips    -> CI [{lo2:+.1%}, {hi2:+.1%}], p={p2:.3f}")
    print(f"The paired test says NO when there's nothing there: {'OK' if ok else 'FAIL'}")
    return ok


def check_no_synergy_when_bundling_a_no_op():
    optimizer = _optimizer()
    # apriltag + apriltag_imu is the SAME hardware as apriltag_imu (one
    # camera, one free IMU), so it cannot possibly gain anything.
    results = optimizer.evaluate_all(["apriltag", "imu", "apriltag_imu"])
    bundle = optimizer.evaluate(make_bundle("apriltag", "imu"))
    report = report_synergy(bundle, results + [bundle])
    same_hardware = bundle.parts == optimizer.evaluate("apriltag_imu").parts
    no_gain = report is None or report.delta <= 0 or not report.significant
    ok = same_hardware and no_gain
    if report is not None:
        print(f"  {report.verdict()}")
    print(f"  apriltag+imu parts == apriltag_imu parts: {same_hardware}")
    print(f"Bundling hardware a candidate already has reports no synergy: {'OK' if ok else 'FAIL'}")
    return ok


def _dominates(other, r):
    """Standard Pareto dominance for (minimize cost, maximize rate): at
    least as good on both axes, strictly better on at least one --
    matching pareto_frontier's own docstring ("no other result is at
    least as cheap AND at least as good, with at least one of those
    strict"). Cheaper at an EQUAL rate counts (dead_reckoning at $0 vs.
    apriltag at $25, same weighted_success_rate, is a real dominance --
    there's no reason to ever buy the pricier one); a formula that only
    ever checked for a strictly higher rate would miss exactly that
    case and misjudge an equal-rate/cheaper-cost candidate as
    wrongly-excluded, since the two components are randomized draws
    from the same free-cell pool and ties across them are expected, not
    a rare edge case."""
    if other is r:
        return False
    cost_ok = other.cost_usd <= r.cost_usd
    rate_ok = other.weighted_success_rate >= r.weighted_success_rate
    strict = other.cost_usd < r.cost_usd or other.weighted_success_rate > r.weighted_success_rate
    return cost_ok and rate_ok and strict


def check_pareto_frontier_is_actually_undominated():
    optimizer = _optimizer()
    results = exhaustive_search(optimizer, TEST_COMPONENTS, min_size=1, max_size=2)
    frontier = pareto_frontier(results)

    undominated = all(not any(_dominates(other, r) for other in results) for r in frontier)
    # Nothing off the frontier should itself be undominated.
    off = [r for r in results if r not in frontier]
    complete = all(any(_dominates(other, r) for other in results) for r in off)
    ok = undominated and complete
    print(f"  {len(frontier)} of {len(results)} robots on the frontier; undominated={undominated}, "
          f"complete={complete}")
    print(f"The Pareto frontier contains exactly the undominated robots: {'OK' if ok else 'FAIL'}")
    return ok


def check_evaluation_cache_is_by_hardware():
    optimizer = _optimizer()
    first = optimizer.evaluate(make_bundle("apriltag", "imu"))
    evaluated_after_first = len(optimizer._cache)
    # A different NAME for the same hardware -- must not cost a second
    # evaluation, and must return the same numbers.
    second = optimizer.evaluate(make_bundle("imu", "apriltag"))
    third = optimizer.evaluate("apriltag_imu")
    no_extra_work = len(optimizer._cache) == evaluated_after_first
    same_numbers = (first.outcome_vector() == second.outcome_vector() == third.outcome_vector())
    ok = no_extra_work and same_numbers
    print(f"  cache size stayed {len(optimizer._cache)} across 3 names for one robot; "
          f"identical outcomes={same_numbers}")
    print(f"Robots are cached by hardware, not by name: {'OK' if ok else 'FAIL'}")
    return ok


def check_budget_and_objective_are_respected():
    optimizer = _optimizer()
    results = exhaustive_search(optimizer, TEST_COMPONENTS, min_size=1, max_size=2)

    budget_ok = True
    for budget in (0.0, 50.0, 150.0, 1000.0):
        pick = best_under_budget(results, budget)
        if pick is not None and pick.cost_usd > budget:
            budget_ok = False
            print(f"  BUDGET VIOLATION at ${budget}: picked {pick.parts_label} at ${pick.cost_usd}")
    # $0 must still return something -- dead reckoning is free, so
    # "nothing affordable" would be wrong.
    free_pick = best_under_budget(results, 0.0)
    free_ok = free_pick is not None and free_pick.cost_usd == 0.0

    by_mean = rank(results, objective="weighted")[0]
    by_worst = rank(results, objective="worst_case")[0]
    # The worst_case winner must be at least as good as the mean winner
    # ON THE WORST-CASE METRIC -- that's what optimizing it means.
    objective_ok = by_worst.worst_profile_rate >= by_mean.worst_profile_rate
    ok = budget_ok and free_ok and objective_ok
    print(f"  best by mean: {by_mean.parts_label} (worst {by_mean.worst_profile_rate:.0%}); "
          f"best by worst-case: {by_worst.parts_label} (worst {by_worst.worst_profile_rate:.0%})")
    print(f"Budget caps hold and each objective optimizes its own metric: {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_candidates_share_identical_scenarios():
    assert check_candidates_share_identical_scenarios()


def test_paired_bootstrap_beats_independent_cis():
    assert check_paired_bootstrap_beats_independent_cis()


def test_paired_bootstrap_rejects_a_wash():
    assert check_paired_bootstrap_rejects_a_wash()


def test_no_synergy_when_bundling_a_no_op():
    assert check_no_synergy_when_bundling_a_no_op()


def test_pareto_frontier_is_actually_undominated():
    assert check_pareto_frontier_is_actually_undominated()


def test_evaluation_cache_is_by_hardware():
    assert check_evaluation_cache_is_by_hardware()


def test_budget_and_objective_are_respected():
    assert check_budget_and_objective_are_respected()


if __name__ == "__main__":
    checks = [
        check_candidates_share_identical_scenarios(),
        check_paired_bootstrap_beats_independent_cis(),
        check_paired_bootstrap_rejects_a_wash(),
        check_no_synergy_when_bundling_a_no_op(),
        check_pareto_frontier_is_actually_undominated(),
        check_evaluation_cache_is_by_hardware(),
        check_budget_and_objective_are_respected(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
