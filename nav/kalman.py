"""
A minimal 1-D-per-axis Kalman estimator over a 2D (row, col) position --
the domain-neutral primitive nav/estimation.py's fuse() was deliberately
NOT built as, and this module is why that was the right call at the
time (see nav/estimation.py's module docstring and README.md's
"Threats to validity"): fuse() combines two estimates using a plain
`confidence` weight, honestly labeled as an engineering estimate, not a
statistical property. A Kalman filter's whole mechanism instead depends
on comparing the VARIANCE of the prior against the variance of the new
observation -- so this module only becomes meaningful once a caller can
supply a real variance, not an invented one. ftc/calibration.py is
where those variances get fit from real (or clearly-labeled synthetic
placeholder) measurement data; ftc/fusion.py is where this module gets
wired into ftc/match.py's pose-error mechanic as an opt-in alternative
to fuse(), alongside it, not in place of it -- see that module's
docstring for why fuse() itself was left untouched rather than
retrofitted with variance-awareness.

Isotropic simplification, matching nav/estimation.py's PositionEstimate
(one scalar `confidence` for both axes, not a full 2x2 covariance
matrix): `variance` here is one scalar shared by both row and col. This
project has never modeled row/col error as having different scales or
being correlated with each other, so a single number is honest about
what's actually tracked, not a shortcut hiding a richer reality this
project doesn't otherwise represent. A caller with a real reason to
track row/col separately (e.g. a sensor whose error is much worse along
one axis) would need a genuine 2x2 covariance Kalman filter, which this
module deliberately does not implement.

Two operations, matching the standard predict/update split:

- `predict(state, process_variance)`: grows `state`'s uncertainty by
  `process_variance` (cells^2) to account for one step of motion whose
  outcome hasn't been observed yet. The MEAN is left untouched here on
  purpose -- this project's callers already realize the actual motion
  as an explicit random draw applied to their own tracked position
  (e.g. ftc/match.py's per-step drift injection: `error = error +
  gauss(0, sigma)`) and pass the already-moved value back in as
  `state.value` on the next call. This function's only job is to grow
  the FILTER's own belief about its uncertainty to match that
  motion -- which is the actual content of a Kalman predict step once
  the motion model itself is left in the domain that owns it, exactly
  the same nav/ vs ftc/ split this project already draws everywhere
  else (see README.md section 3.1).
- `update(prior, observation_value, observation_variance)`: the
  standard scalar Kalman update -- gain K = P / (P + R), posterior mean
  = prior + K * (observation - prior), posterior variance = (1 - K) * P.
  Blending is proportional to CERTAINTY the way fuse()'s confidence
  weighting is, but K falls straight out of the two variances instead
  of being handed in as a separate, disconnected number.
- `gated_update(...)`: the Kalman-proper version of the "innovation
  gating" nav/estimation.py's fuse() already approximates with a FIXED
  disagreement threshold (documented there as a simplification, not an
  oversight). Real innovation gating compares the observation's
  distance from the prior against `gate_sigma` standard deviations of
  the estimate the two would jointly produce -- so the SAME absolute
  distance is unsurprising against a wide-open prior and highly
  surprising against a tight one, which a fixed-cells threshold cannot
  express. `gate_multiplier` inflates (never rejects outright) a
  reading that fails the gate, the same "shrink trust, don't zero it"
  choice fuse()'s `distrust_factor` already makes, for the same reason
  given there: a reading that disagrees once could still be right.
"""
import math
from dataclasses import dataclass


@dataclass
class KalmanEstimate:
    """One tracked belief about a 2D quantity: `value` is a (row, col)
    2-tuple, `variance` is a single non-negative scalar in squared units
    of whatever `value` is expressed in (this project always uses grid
    cells, so cells^2) -- see module docstring for why one scalar
    covers both axes. Unlike nav/estimation.py's PositionEstimate,
    `variance` genuinely has to mean "squared uncertainty" for the math
    below to be honest -- passing in a made-up number defeats the
    entire point of using this module instead of fuse() (see
    ftc/calibration.py, whose job is supplying a real one)."""
    value: tuple
    variance: float


def predict(state, process_variance):
    """Advance `state` through one step of unobserved motion whose
    variance is `process_variance` (cells^2) -- see module docstring
    for why the mean is untouched. Negative process_variance would
    SHRINK uncertainty from motion alone, which has no physical
    meaning for this project's callers, so it's clamped to >= 0
    rather than silently accepted."""
    return KalmanEstimate(state.value, state.variance + max(process_variance, 0.0))


def update(prior, observation_value, observation_variance):
    """Fuse `prior` with a new `observation_value` whose own variance is
    `observation_variance` (cells^2). Returns a new KalmanEstimate;
    never mutates `prior`. Falls back to returning `prior` unchanged
    when prior.variance + observation_variance <= 0 -- the only way
    that happens is both are already exactly 0 (a state with no
    uncertainty at all, fed an equally certain observation), the same
    degenerate case nav/estimation.py's fuse() handles by returning the
    unmodified prior when total confidence is 0."""
    total = prior.variance + observation_variance
    if total <= 0:
        return prior
    k = prior.variance / total
    fused_value = (
        prior.value[0] + k * (observation_value[0] - prior.value[0]),
        prior.value[1] + k * (observation_value[1] - prior.value[1]),
    )
    fused_variance = (1.0 - k) * prior.variance
    return KalmanEstimate(fused_value, fused_variance)


def innovation(prior, observation_value):
    """Euclidean distance between `prior.value` and a candidate
    observation -- the quantity gated_update's gate check (and, in a
    real Kalman filter, the "innovation" the update step is actually
    responding to) is computed from."""
    return math.hypot(observation_value[0] - prior.value[0], observation_value[1] - prior.value[1])


def gated_update(prior, observation_value, observation_variance, gate_sigma, distrust_factor):
    """update(), but first checking whether `observation_value` is
    plausible given the two estimates' COMBINED uncertainty rather than
    a flat distance -- see module docstring. `gate_sigma` is how many
    standard deviations of sqrt(prior.variance + observation_variance)
    away counts as implausible (3.0 is the standard "3-sigma" outlier
    convention in filtering generally -- not a number fit to this
    project's own data, see ftc/config.py's KALMAN_GATE_SIGMA). A
    reading that fails the gate has its OWN variance inflated by
    dividing by `distrust_factor` (in (0, 1], so dividing always
    inflates) before the ordinary update runs -- inflating R lowers the
    Kalman gain K, which is the variance-aware equivalent of fuse()'s
    confidence cut, applied to the correct half of the math instead of
    to a re-derived confidence number."""
    total_sigma = math.sqrt(max(prior.variance + observation_variance, 0.0))
    gate = gate_sigma * total_sigma
    surprised = innovation(prior, observation_value) > gate if total_sigma > 0 else False
    effective_variance = observation_variance / distrust_factor if surprised else observation_variance
    return update(prior, observation_value, effective_variance), surprised
