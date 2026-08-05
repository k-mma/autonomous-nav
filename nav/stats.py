"""
Bootstrap confidence intervals for a success rate -- pure stdlib (no
numpy/scipy in this project's venv; see requirements.txt), and safer
than a normal/Wilson approximation at the trial counts this project's
benchmarks actually use (20/point): a Gaussian assumption is a poor fit
for a rate near 0 or 1, exactly where several of nav/uncertainty_
benchmark.py's and ftc/suite_benchmark.py's data points sit. Resampling
the real trial outcomes sidesteps that instead of assuming a shape.
"""
import random

DEFAULT_BOOTSTRAP_ITERS = 2000
DEFAULT_CONFIDENCE = 0.95


def bootstrap_ci(successes, n, iters=DEFAULT_BOOTSTRAP_ITERS, confidence=DEFAULT_CONFIDENCE, seed=None):
    """95% CI (percentile method) on a success rate from `successes`
    successes out of `n` binary trials: resample n trials with
    replacement `iters` times and take the [2.5, 97.5] percentiles of
    the resampled rate. `seed` makes a given call reproducible (a plain
    `random.Random()` default would make two runs over the identical
    input data report very slightly different CIs, which is confusing
    when nothing about the underlying trials changed).

    Returns (lower, upper). n == 0 returns (0.0, 0.0).
    """
    if n == 0:
        return 0.0, 0.0
    rng = random.Random(seed)
    trials = [1] * successes + [0] * (n - successes)
    boot_rates = []
    for _ in range(iters):
        resample_successes = sum(trials[rng.randrange(n)] for _ in range(n))
        boot_rates.append(resample_successes / n)
    boot_rates.sort()
    alpha = 1 - confidence
    lo_idx = max(int((alpha / 2) * iters), 0)
    hi_idx = min(int((1 - alpha / 2) * iters), iters - 1)
    return boot_rates[lo_idx], boot_rates[hi_idx]


def bootstrap_paired_diff_ci(outcomes_a, outcomes_b, iters=DEFAULT_BOOTSTRAP_ITERS,
                              confidence=DEFAULT_CONFIDENCE, seed=None):
    """CI on mean(b) - mean(a) when the two outcome lists are PAIRED --
    trial i of both ran against the identical scenario, which is how
    every benchmark in this project is already structured (ftc/
    suite_benchmark.py and nav/uncertainty_benchmark.py both share one
    random scenario across every policy/suite being compared).

    Resamples trial INDICES, not each list independently: a resample
    takes trial i's outcome from BOTH lists or from neither, which is
    what preserves the pairing. That matters because the scenario is
    usually the dominant source of variance -- two options compared on
    the same hard scenarios can differ reliably by 8 points while each
    one's own success rate has a 20-point CI, and comparing their
    independent CIs for overlap (what bootstrap_ci above supports)
    would call that a wash. This is the strictly stronger comparison
    whenever the pairing is real; using it on unpaired data would be
    wrong.

    Returns (lo, hi, p_one_sided) where p_one_sided is the fraction of
    resamples in which b did NOT beat a -- a bootstrap p-value for the
    one-sided claim "b is better than a." Raises on length mismatch
    rather than silently truncating, since a mismatch means the pairing
    the whole method depends on has already been broken somewhere
    upstream.
    """
    if len(outcomes_a) != len(outcomes_b):
        raise ValueError(f"paired comparison needs equal-length outcomes, got {len(outcomes_a)} and "
                         f"{len(outcomes_b)}")
    n = len(outcomes_a)
    if n == 0:
        return 0.0, 0.0, 1.0
    rng = random.Random(seed)
    deltas = [float(b) - float(a) for a, b in zip(outcomes_a, outcomes_b)]
    boot_diffs = []
    not_better = 0
    for _ in range(iters):
        total = 0.0
        for _ in range(n):
            total += deltas[rng.randrange(n)]
        mean_diff = total / n
        boot_diffs.append(mean_diff)
        if mean_diff <= 0:
            not_better += 1
    boot_diffs.sort()
    alpha = 1 - confidence
    lo_idx = max(int((alpha / 2) * iters), 0)
    hi_idx = min(int((1 - alpha / 2) * iters), iters - 1)
    return boot_diffs[lo_idx], boot_diffs[hi_idx], not_better / iters
