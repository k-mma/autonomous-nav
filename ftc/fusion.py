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

A SECOND fusion path lives here too: fused_tag_correction_kalman, using
nav/kalman.py instead of nav/estimation.py's fuse(). Both paths share
the exact same generative observation model (_apriltag_observation_
value, below) -- an ordinary detection plus a constant bias, or (rarer)
an outright bad one -- so a paired comparison between "confidence" and
"kalman" fusion (ftc/fusion_benchmark.py) is comparing two ways of
COMBINING the identical simulated readings, not two different noise
models. fuse() itself was deliberately left untouched rather than
retrofitted to accept a variance (see nav/kalman.py's module docstring
for why): its `confidence` contract is honestly a plain weight, and
mixing that with a real variance in the same function would blur the
one distinction this whole addition exists to keep clear.
"""
from nav.estimation import PositionEstimate, fuse
from nav.kalman import KalmanEstimate, gated_update

from ftc.calibration import load_calibration
from ftc.config import (
    APRILTAG_BAD_DETECTION_PROBABILITY, APRILTAG_BAD_DETECTION_SIGMA_CELLS, APRILTAG_CORRECTION_FACTOR_MAX,
    APRILTAG_FUSION_CONFIDENCE, APRILTAG_SYSTEMATIC_BIAS_CELLS, CELL_SIZE_IN, FUSION_DISAGREEMENT_THRESHOLD_CELLS,
    FUSION_DISTRUST_FACTOR, KALMAN_GATE_SIGMA, ODOMETRY_FUSION_CONFIDENCE,
)

# The Kalman path's default AprilTag variance model, built ONCE at
# import time from ftc/calibration.py's labeled SYNTHETIC placeholder
# scatter -- this is NOT a claim about real hardware, exactly like
# every other unmeasured constant in ftc/config.py, just expressed as a
# fitted model instead of a single number. A caller with real detection
# scatter should fit their own ApriltagVarianceModel via ftc.calibration.
# load_calibration(apriltag_csv_path=...).apriltag_variance.model and
# pass it explicitly to fused_tag_correction_kalman -- this default only
# exists so the Kalman path is runnable out of the box, same as every
# other calibrated quantity in this project.
DEFAULT_APRILTAG_VARIANCE_MODEL = load_calibration().apriltag_variance.model

# Below this fraction of APRILTAG_CORRECTION_FACTOR_MAX, _observation_
# variance_cells2's range/incidence-degradation scaling (below) is
# floored rather than left to blow up toward infinity as frac -> 0 --
# a numerical safety floor, not a claim that a detection this degraded
# is meaningfully more or less trustworthy than the floor value itself.
_MIN_FRAC_RATIO = 0.05


def _apriltag_observation_value(error, frac, rng):
    """The single shared generative model for what an AprilTag
    detection event actually reports, used by BOTH fusion paths below
    (fused_tag_correction and fused_tag_correction_kalman) so a paired
    comparison between them is comparing fusion MATH, not two different
    noise models -- see module docstring. `error` doubles as odometry's
    own continuously-tracked position estimate -- the same quantity
    ftc/match.py already drifts every step at the suite's drift_per_
    cell, which for an AprilTag+odometry bundle IS odometry's rate
    (ftc/bundle.py: "the best localization hardware on the robot sets
    its drift rate"). No second position tracker is added; this re-reads
    a value ftc/match.py already carries and treats it as one of the
    two sources being fused."""
    plain_corrected = (error[0] * (1 - frac), error[1] * (1 - frac))

    if rng.random() < APRILTAG_BAD_DETECTION_PROBABILITY:
        # Case 2: deliberately unrelated to the true correction, not a
        # slightly-worse version of a good one -- see module docstring.
        return (error[0] + rng.gauss(0, APRILTAG_BAD_DETECTION_SIGMA_CELLS),
                error[1] + rng.gauss(0, APRILTAG_BAD_DETECTION_SIGMA_CELLS))
    # Case 1: an ordinary detection plus the constant bias the plain
    # frac-based model has no way to represent.
    return (plain_corrected[0] + APRILTAG_SYSTEMATIC_BIAS_CELLS,
            plain_corrected[1] + APRILTAG_SYSTEMATIC_BIAS_CELLS)


def fused_tag_correction(error, frac, rng):
    """Replaces ftc/match.py's plain `error = error * (1 - frac)` for
    one tag-detection event with nav/estimation.py's confidence-
    weighted fuse(). `frac` is already computed by suite.tag_correction
    before this is called -- every range/FOV/line-of-sight/dropout gate
    that decides whether a detection happens AT ALL stays exactly as it
    was; this only changes what happens to `error` once a detection has
    occurred. Returns the fused error vector."""
    observation_value = _apriltag_observation_value(error, frac, rng)

    prior = PositionEstimate(error, ODOMETRY_FUSION_CONFIDENCE)
    # frac already carries tag_correction's own range/angle degradation
    # -- reused directly rather than a second confidence formula.
    observation = PositionEstimate(observation_value, APRILTAG_FUSION_CONFIDENCE * frac)

    fused = fuse(prior, observation, FUSION_DISAGREEMENT_THRESHOLD_CELLS, FUSION_DISTRUST_FACTOR)
    return fused.value


def _apriltag_observation_variance_cells2(frac, variance_model):
    """Converts `frac` (AprilTagSuite.tag_correction's already-computed
    range/angle-degraded correction fraction) into an observation
    variance in cells^2, using `variance_model`'s fitted best-case
    (range=0, incidence=0) variance as the reference point and scaling
    it up as frac degrades from its own achievable maximum
    (APRILTAG_CORRECTION_FACTOR_MAX).

    This is a real, documented approximation: tag_correction collapses
    a detection's raw (range, incidence) geometry into the single
    scalar `frac` before this function ever sees it, and plumbing the
    raw geometry through to here instead would mean changing tag_
    correction's public signature (and therefore FullSuite/
    DualCameraAprilTagSuite/every other override -- see ftc/sensors.py).
    Reusing `frac` keeps this a small, additive change, at the cost of
    treating two geometrically different detections that happen to
    produce the same frac as equally uncertain, even though the fitted
    model would tell them apart given the raw range/incidence. Stated
    plainly rather than hidden -- see benchmark_results/
    ftc_fusion_kalman_writeup.md's own limitations section.
    """
    best_case_variance_in2 = variance_model.variance_in2(0.0, 0.0)
    ratio = max(frac / APRILTAG_CORRECTION_FACTOR_MAX, _MIN_FRAC_RATIO)
    variance_in2 = best_case_variance_in2 / (ratio ** 2)
    return variance_in2 / (CELL_SIZE_IN ** 2)


def fused_tag_correction_kalman(error, position_variance, frac, rng, apriltag_variance_model=None):
    """The Kalman-path alternative to fused_tag_correction: the exact
    same observation model (_apriltag_observation_value), fused with
    nav/kalman.py's gated_update instead of nav/estimation.py's fuse().

    `error`/`position_variance` are ftc/match.py's own running belief
    (mean, variance) -- this function does not own any state itself,
    matching how fused_tag_correction already re-reads `error` rather
    than tracking a second copy. `position_variance` is grown at every
    drift-injection step by ftc/match.py calling nav.kalman.predict
    directly (see ftc/match.py) -- this function only performs the
    UPDATE half of predict/update, the half that happens at a tag-
    detection event.

    Returns (new_error, new_position_variance). `apriltag_variance_model`
    defaults to DEFAULT_APRILTAG_VARIANCE_MODEL (this project's labeled
    synthetic placeholder, see module-level constant above) when not
    given explicitly.
    """
    apriltag_variance_model = apriltag_variance_model or DEFAULT_APRILTAG_VARIANCE_MODEL
    observation_value = _apriltag_observation_value(error, frac, rng)
    observation_variance_cells2 = _apriltag_observation_variance_cells2(frac, apriltag_variance_model)

    state = KalmanEstimate(error, position_variance)
    fused, _surprised = gated_update(state, observation_value, observation_variance_cells2,
                                       KALMAN_GATE_SIGMA, FUSION_DISTRUST_FACTOR)
    return fused.value, fused.variance
