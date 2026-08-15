"""
A small decision CLI: given a sensor suite (or none, for a full
ranking) and a field variance -- measured via ftc/calibration.py's real
CSVs, or its clearly-labeled synthetic placeholder if you haven't
collected real measurements yet -- predict that suite's success rate
(with a bootstrap CI, nav/stats.py), expected time usage against the
30-second match budget, and dollar cost. This is the artifact another
team could actually use: not a chart to interpret, a question to type
in and an answer to read back out.

Every run prints ftc/calibration.py's Calibration.describe() first, so
the recommendation is never presented without also saying plainly
whether it's grounded in real measurements or the synthetic
placeholder -- see ftc/calibration.py's module docstring for why that
distinction has to stay visible at every layer, not just inside
calibration.py itself.

    python3 -m ftc.recommend                                   # rank all 7 suites
    python3 -m ftc.recommend --suite distance_sensors           # + how it compares to the best option
    python3 -m ftc.recommend --elements my_field.csv --odometry my_robot.csv
    python3 -m ftc.recommend --layout corridor --blocker-probability 0.3
"""
import argparse
import random
from dataclasses import dataclass

from nav.field_variance import generate_ground_truth
from nav.stats import bootstrap_ci

from ftc.calibration import load_calibration
from ftc.config import AUTONOMOUS_PERIOD_S
from ftc.field import build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import SUITES, SUITE_ORDER, SUITE_LABELS

DEFAULT_TRIALS = 60
MAX_ATTEMPTS_PER_TRIAL = 50
MIN_PATH_LEN = 4
# Not something ftc/calibration.py can fit from field-setup measurements
# (an opponent robot's presence is a match/strategy variable, not a
# property of your field or your robot) -- a flat, documented default
# rather than a fitted number. Override with --blocker-probability if
# you have a better estimate for your division/alliance patterns.
DEFAULT_BLOCKER_PROBABILITY = 0.15


@dataclass
class Prediction:
    suite: str
    success_rate: float
    ci_lo: float
    ci_hi: float
    avg_elapsed_s: float
    over_budget_rate: float
    cost_usd: float
    n_trials: int


def _solvable_scenario(trial_seed, grid, free_cells):
    from nav.algorithms import astar
    rng = random.Random(trial_seed)
    for _ in range(MAX_ATTEMPTS_PER_TRIAL):
        start, goal = rng.sample(free_cells, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= MIN_PATH_LEN:
            return start, goal
    raise RuntimeError(f"no solvable scenario for seed {trial_seed} after {MAX_ATTEMPTS_PER_TRIAL} attempts")


def predict_suite(suite_name, grid, free_cells, tag_sites, calibration, blocker_probability,
                    num_trials=DEFAULT_TRIALS, base_seed=8_000_000):
    """Runs `num_trials` live matches for `suite_name` against ground
    truth generated at the calibrated deviation levels (all three axes
    active at once, at their calibrated magnitudes -- unlike ftc/
    suite_benchmark.py's ablation sweep, this is meant to represent one
    real field/robot's actual combined deviation, not an isolated
    axis). Returns a Prediction with a 95% bootstrap CI on success
    rate."""
    successes = 0
    elapsed_total = 0.0
    over_budget = 0
    suite_cls = SUITES[suite_name]

    for t in range(num_trials):
        trial_seed = base_seed + t
        start, goal = _solvable_scenario(trial_seed, grid, free_cells)
        ground_truth, actual_start = generate_ground_truth(
            grid, start, goal, 1.0, seed=trial_seed,
            start_drift_scale=calibration.start_drift.value,
            obstacle_drift_scale=calibration.obstacle_drift.value,
            blocker_scale=blocker_probability,
        )
        result = run_match(suite_cls(), grid, start, goal, ground_truth, actual_start, tag_sites,
                             random.Random(trial_seed))
        successes += int(result.success)
        elapsed_total += result.elapsed_s
        over_budget += int(result.over_budget)

    ci_lo, ci_hi = bootstrap_ci(successes, num_trials, seed=9_000_000 + SUITE_ORDER.index(suite_name))
    return Prediction(
        suite=suite_name,
        success_rate=successes / num_trials,
        ci_lo=ci_lo,
        ci_hi=ci_hi,
        avg_elapsed_s=elapsed_total / num_trials,
        over_budget_rate=over_budget / num_trials,
        cost_usd=suite_cls.cost_usd,
        n_trials=num_trials,
    )


def format_prediction(p):
    budget_pct = p.avg_elapsed_s / AUTONOMOUS_PERIOD_S * 100
    return (
        f"{SUITE_LABELS[p.suite]:<18} ${p.cost_usd:>5.0f}   "
        f"success {p.success_rate:>4.0%} (95% CI {p.ci_lo:.0%}-{p.ci_hi:.0%})   "
        f"avg time {p.avg_elapsed_s:5.2f}s / {AUTONOMOUS_PERIOD_S:.0f}s ({budget_pct:4.0f}% of budget)   "
        f"over-budget {p.over_budget_rate:>4.0%}   n={p.n_trials}"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--suite", choices=SUITE_ORDER, default=None,
                         help="Report on this suite specifically (still shows the full ranking for comparison). "
                              "Omit to just get the ranking.")
    parser.add_argument("--layout", default="cluttered", choices=["sparse", "cluttered", "corridor"])
    parser.add_argument("--elements", default=None, help="CSV of nominal vs. actual field-element positions")
    parser.add_argument("--odometry", default=None, help="CSV of measured dead-reckoning drift over real runs")
    parser.add_argument("--blocker-probability", type=float, default=DEFAULT_BLOCKER_PROBABILITY,
                         help=f"Not fitted from measurements -- a match/strategy assumption, "
                              f"default {DEFAULT_BLOCKER_PROBABILITY}.")
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS)
    args = parser.parse_args()

    calibration = load_calibration(args.elements, args.odometry)
    print("=== Calibration ===")
    print(calibration.describe())
    if not calibration.obstacle_drift.is_measured() or not calibration.pose_drift_rate.is_measured():
        print("\n*** WARNING: recommendation below is partly or fully based on the SYNTHETIC placeholder ***")
        print("*** dataset above, not real field/robot measurements. Treat it as illustrative only,   ***")
        print("*** and pass --elements/--odometry with real data before trusting these numbers.        ***")
    print(f"\nblocker_probability = {args.blocker_probability} (not fitted -- see --help)")

    grid = build_grid(args.layout)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(args.layout)

    print(f"\n=== Predicted performance on the '{args.layout}' layout ({args.trials} trials/suite) ===")
    predictions = {
        suite_name: predict_suite(suite_name, grid, free_cells, tag_sites, calibration,
                                    args.blocker_probability, num_trials=args.trials)
        for suite_name in SUITE_ORDER
    }
    for suite_name in SUITE_ORDER:
        print(format_prediction(predictions[suite_name]))

    best_success = max(predictions.values(), key=lambda p: p.success_rate)
    baseline_rate = predictions["dead_reckoning"].success_rate
    def value_per_100(p):
        # success_rate is a fraction (e.g. 0.35); *100 converts
        # "fraction of success rate per $100" into percentage POINTS
        # per $100, matching the "pp/$100" unit actually printed below
        # (see ftc/suite_benchmark.py's write_writeup for the same fix).
        return (p.success_rate - baseline_rate) / (p.cost_usd / 100) * 100 if p.cost_usd > 0 else float("-inf")
    best_value = max((p for p in predictions.values() if p.suite != "dead_reckoning"), key=value_per_100)

    print(f"\n=== Recommendation ===")
    print(f"Best raw success rate: {SUITE_LABELS[best_success.suite]} ({best_success.success_rate:.0%})")
    print(f"Best value (success-rate gain per $100 over dead reckoning): "
          f"{SUITE_LABELS[best_value.suite]} ({value_per_100(best_value):+.1f}pp/$100)")

    if args.suite:
        chosen = predictions[args.suite]
        print(f"\n=== Your suite: {SUITE_LABELS[args.suite]} ===")
        print(format_prediction(chosen))
        if chosen.suite != best_success.suite:
            gap = best_success.success_rate - chosen.success_rate
            print(f"{SUITE_LABELS[best_success.suite]} predicts {gap:+.0%} higher success rate on this field/"
                  f"robot profile, at ${best_success.cost_usd - chosen.cost_usd:+.0f} more.")
        else:
            print("This is also the best raw performer among all 7 suites on this field/robot profile.")


if __name__ == "__main__":
    main()
