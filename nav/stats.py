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
