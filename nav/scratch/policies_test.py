"""
Do the three planning policies (nav/policies.py) behave the way
nav/uncertainty_benchmark.py's headline finding depends on?

1. OpenLoopPolicy never replans -- across every trial in a sweep,
   `policy.replans == 0` always (it has no mechanism to replan at all).
2. ReactivePolicy's and BeliefPolicy's success rate is monotonically
   non-increasing as variance_level increases, within a tolerance for
   trial-to-trial noise at this trial count -- a policy with real
   closed-loop feedback should get worse (or at worst stay flat) as the
   assumed map becomes less trustworthy, never *better*.
3. BeliefPolicy's success rate is never worse than OpenLoopPolicy's
   (again within a noise tolerance) at any variance_level in the swept
   range -- sensing and replanning against a probabilistic map should
   never be a net loss versus not looking at the world at all.
"""
import random

import pytest

from nav.field_variance import generate_ground_truth
from nav.policies import BeliefPolicy, OpenLoopPolicy, ReactivePolicy
from nav.uncertainty_benchmark import _solvable_scenario, execute_trial

VARIANCE_LEVELS = [0.0, 0.25, 0.5, 0.75, 1.0]
TRIALS_PER_LEVEL = 25
# Allowed upward blip in success rate between consecutive levels (or vs.
# open-loop) before it's treated as a real violation rather than sampling
# noise at TRIALS_PER_LEVEL trials.
NOISE_TOLERANCE = 0.20

POLICY_CLASSES = {"open_loop": OpenLoopPolicy, "reactive": ReactivePolicy, "belief": BeliefPolicy}


def run_sweep():
    """{policy_name: [success_rate at each VARIANCE_LEVELS entry]},
    plus an assertion that OpenLoopPolicy never records a replan."""
    success_rates = {name: [] for name in POLICY_CLASSES}
    replan_violations = 0

    for level in VARIANCE_LEVELS:
        successes = {name: 0 for name in POLICY_CLASSES}
        for t in range(TRIALS_PER_LEVEL):
            seed = 5_000_000 + round(level * 1000) + t
            grid, start, goal, path = _solvable_scenario(seed)
            ground_truth, actual_start = generate_ground_truth(grid, start, goal, level, seed=seed)
            max_steps = min(4 * len(path), len(path) + 60)

            for name, cls in POLICY_CLASSES.items():
                policy = cls(grid, start, goal, rng=random.Random(seed))
                outcome = execute_trial(policy, ground_truth, actual_start, goal, max_steps)
                if name == "open_loop" and policy.replans != 0:
                    replan_violations += 1
                    print(f"  OpenLoopPolicy replanned ({policy.replans}x) at "
                          f"variance_level={level}, seed={seed}")
                if outcome.success:
                    successes[name] += 1

        for name in POLICY_CLASSES:
            success_rates[name].append(successes[name] / TRIALS_PER_LEVEL)

    print(f"OpenLoopPolicy replan_count == 0 in every trial: "
          f"{'OK' if replan_violations == 0 else f'FAIL ({replan_violations} violations)'}")
    return success_rates, replan_violations == 0


def check_monotonic_non_increasing(name, rates):
    ok = True
    for i in range(1, len(rates)):
        if rates[i] > rates[i - 1] + NOISE_TOLERANCE:
            ok = False
            print(f"  {name}: success rate rose from {rates[i - 1]:.0%} at "
                  f"variance_level={VARIANCE_LEVELS[i - 1]} to {rates[i]:.0%} at "
                  f"variance_level={VARIANCE_LEVELS[i]} -- more than the "
                  f"{NOISE_TOLERANCE:.0%} noise tolerance")
    print(f"{name}: success rate roughly non-increasing across the sweep: {'OK' if ok else 'FAIL'} "
          f"({', '.join(f'{r:.0%}' for r in rates)})")
    return ok


def check_belief_beats_open_loop(open_loop_rates, belief_rates):
    ok = True
    for level, ol, b in zip(VARIANCE_LEVELS, open_loop_rates, belief_rates):
        if b < ol - NOISE_TOLERANCE:
            ok = False
            print(f"  variance_level={level}: belief ({b:.0%}) worse than open_loop ({ol:.0%}) "
                  f"by more than the {NOISE_TOLERANCE:.0%} noise tolerance")
    print(f"BeliefPolicy never meaningfully worse than OpenLoopPolicy: {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# run_sweep() is the expensive part (5 levels x 25 trials x 3 policies) --
# a module-scoped fixture runs it once and the three checks below all read
# from that single result, instead of each re-running the sweep.

@pytest.fixture(scope="module")
def sweep():
    return run_sweep()


def test_open_loop_never_replans(sweep):
    _, replans_ok = sweep
    assert replans_ok


def test_reactive_success_rate_roughly_non_increasing(sweep):
    success_rates, _ = sweep
    assert check_monotonic_non_increasing("reactive", success_rates["reactive"])


def test_belief_success_rate_roughly_non_increasing(sweep):
    success_rates, _ = sweep
    assert check_monotonic_non_increasing("belief", success_rates["belief"])


def test_belief_never_meaningfully_worse_than_open_loop(sweep):
    success_rates, _ = sweep
    assert check_belief_beats_open_loop(success_rates["open_loop"], success_rates["belief"])


if __name__ == "__main__":
    print(f"Running {len(VARIANCE_LEVELS)} variance_levels x {TRIALS_PER_LEVEL} trials x 3 policies...\n")
    success_rates, replans_ok = run_sweep()

    reactive_ok = check_monotonic_non_increasing("reactive", success_rates["reactive"])
    belief_ok = check_monotonic_non_increasing("belief", success_rates["belief"])
    belief_vs_open_ok = check_belief_beats_open_loop(success_rates["open_loop"], success_rates["belief"])

    all_ok = replans_ok and reactive_ok and belief_ok and belief_vs_open_ok
    print("\nALL PASS" if all_ok else "\nSOME CHECKS FAILED")
