"""
Confidence-weighted fusion of AprilTag's per-detection position
correction against odometry's continuously-tracked position estimate --
the "how loose is the optimizer's upper bound" question README.md's
"Threats to validity" names: ftc/bundle.py merges a bundle's sensors
optimistically (a tag detection is applied at face value), never
modeling that a reading might actively disagree with what odometry
already believes. This is the wiring that plugs nav/estimation.py's
domain-neutral fuse() into ftc/match.py's pose-error mechanic for
exactly that one pairing -- see ftc/config.py's "Sensor fusion" section
for every constant used here and why each value was chosen.

Two disagreement shapes, matching the two things AprilTagSuite.
tag_correction's plain `error * (1 - frac)` model (frac in [0, 1])
cannot express on its own -- it can only shrink error toward exactly
zero, never introduce a correction that's actively wrong:

1. A small SYSTEMATIC bias (APRILTAG_SYSTEMATIC_BIAS_CELLS) -- an
   uncorrected camera/mount calibration error nudging every detection
   the same direction. Fusing against odometry's own tracked position
   (the prior) partially damps this rather than either fully absorbing
   it (today's model) or fully rejecting it.
2. An outright BAD detection (APRILTAG_BAD_DETECTION_PROBABILITY,
   APRILTAG_BAD_DETECTION_SIGMA_CELLS) -- a misidentified, occluded, or
   reflection-corrupted tag reports a position with no relationship to
   the truth. nav/estimation.py's disagreement check is what catches
   this: a reading far from the current estimate gets its confidence
   cut by FUSION_DISTRUST_FACTOR before being blended in, rather than
   applied unconditionally the way every detection is today.
"""
from nav.estimation import PositionEstimate, fuse

from ftc.config import (
    APRILTAG_BAD_DETECTION_PROBABILITY, APRILTAG_BAD_DETECTION_SIGMA_CELLS, APRILTAG_FUSION_CONFIDENCE,
    APRILTAG_SYSTEMATIC_BIAS_CELLS, FUSION_DISAGREEMENT_THRESHOLD_CELLS, FUSION_DISTRUST_FACTOR,
    ODOMETRY_FUSION_CONFIDENCE,
)


def fused_tag_correction(error, frac, rng):
    """Replaces ftc/match.py's plain `error = error * (1 - frac)` for
    one tag-detection event. `frac` is already computed by
    suite.tag_correction before this is called -- every range/FOV/
    line-of-sight/dropout gate that decides whether a detection happens
    AT ALL stays exactly as it was; this only changes what happens to
    `error` once a detection has occurred. Returns the fused error
    vector.

    `error` doubles as odometry's own continuously-tracked position
    estimate -- the same quantity ftc/match.py already drifts every
    step at the suite's drift_per_cell, which for an AprilTag+odometry
    bundle IS odometry's rate (ftc/bundle.py: "the best localization
    hardware on the robot sets its drift rate"). No second position
    tracker is added; this re-reads a value ftc/match.py already
    carries and treats it as one of the two sources being fused.
    """
    plain_corrected = (error[0] * (1 - frac), error[1] * (1 - frac))

    if rng.random() < APRILTAG_BAD_DETECTION_PROBABILITY:
        # Case 2: deliberately unrelated to the true correction, not a
        # slightly-worse version of a good one -- see module docstring.
        observation_value = (error[0] + rng.gauss(0, APRILTAG_BAD_DETECTION_SIGMA_CELLS),
                              error[1] + rng.gauss(0, APRILTAG_BAD_DETECTION_SIGMA_CELLS))
    else:
        # Case 1: an ordinary detection plus the constant bias the
        # plain frac-based model has no way to represent.
        observation_value = (plain_corrected[0] + APRILTAG_SYSTEMATIC_BIAS_CELLS,
                              plain_corrected[1] + APRILTAG_SYSTEMATIC_BIAS_CELLS)

    prior = PositionEstimate(error, ODOMETRY_FUSION_CONFIDENCE)
    # frac already carries tag_correction's own range/angle degradation
    # -- reused directly rather than a second confidence formula.
    observation = PositionEstimate(observation_value, APRILTAG_FUSION_CONFIDENCE * frac)

    fused = fuse(prior, observation, FUSION_DISAGREEMENT_THRESHOLD_CELLS, FUSION_DISTRUST_FACTOR)
    return fused.value
