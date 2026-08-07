"""
Confidence-weighted fusion of two 2D position estimates that might
disagree -- the domain-neutral primitive ftc/fusion.py wires into
ftc/match.py's pose-error mechanic for one specific pairing (AprilTag
vs. odometry). Deliberately small and generic: this file knows nothing
about sensors, suites, or FTC (see README.md's "nav/ vs ftc/" section)
-- just two labeled 2D vectors and how to combine them.

Two behaviors, both driven by `confidence` alone:

1. Ordinary weighted average. A source reporting confidence=0.8 pulls
   the fused estimate 4x as hard as one reporting confidence=0.2. This
   is what lets a SMALL, persistent disagreement (e.g. a biased but
   still-working sensor) get partially corrected rather than either
   fully trusted or fully ignored.
2. Outlier down-weighting. When the two estimates disagree by more than
   `disagreement_threshold` -- a genuinely different story about the
   same quantity, not just noise around the same one -- `observation`'s
   confidence is cut by `distrust_factor` before the average runs. This
   is a simplified version of what Kalman-style filters call innovation
   gating (rejecting/down-weighting a measurement whose residual from
   the predicted state is implausibly large); the simplification is a
   FIXED threshold rather than one scaled by the prior's own
   uncertainty, which is fine at this project's bounded position
   scale (see ftc/fusion.py) but wouldn't generalize as-is to a
   quantity whose scale varies a lot from call to call.
"""
from dataclasses import dataclass


@dataclass
class PositionEstimate:
    """One source's belief about a 2D quantity, plus how much to trust
    it. `value` is any (row, col)-shaped 2-tuple; `confidence` is a
    plain weight, not a probability -- it only has to be consistent
    between the two estimates passed to fuse() together, not normalized
    against anything else."""
    value: tuple
    confidence: float


def disagreement(a, b):
    """Euclidean distance between two estimates' values, in whatever
    units `value` is expressed in."""
    return ((a.value[0] - b.value[0]) ** 2 + (a.value[1] - b.value[1]) ** 2) ** 0.5


def fuse(prior, observation, disagreement_threshold, distrust_factor):
    """Combine `prior` (the existing belief) with `observation` (a new,
    possibly-disagreeing reading of the same quantity) into one fused
    PositionEstimate.

    `observation`'s confidence is scaled by `distrust_factor` (expected
    in [0, 1]) before blending whenever the two estimates disagree by
    more than `disagreement_threshold` -- see module docstring. The
    fused confidence is the larger of the two (post-distrust)
    confidences: fusing never makes the caller MORE certain than its
    most-trusted input, only combines where the estimate itself should
    land.
    """
    obs_confidence = observation.confidence
    if disagreement(prior, observation) > disagreement_threshold:
        obs_confidence *= distrust_factor

    total = prior.confidence + obs_confidence
    if total <= 0:
        return prior

    w_prior, w_obs = prior.confidence / total, obs_confidence / total
    fused_value = (
        prior.value[0] * w_prior + observation.value[0] * w_obs,
        prior.value[1] * w_prior + observation.value[1] * w_obs,
    )
    return PositionEstimate(fused_value, max(prior.confidence, obs_confidence))
