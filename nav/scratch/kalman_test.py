"""
Does nav/kalman.py actually behave like a Kalman filter -- gain
weighting proportional to relative certainty (not a fixed split),
variance shrinking on update and growing on predict, a perfectly
certain prior refusing to move, gating that scales with the estimates'
own uncertainty instead of a flat distance, and -- the cross-check
tying this module back to the one it sits alongside -- degenerating to
nav/estimation.py's plain confidence-weighted fuse() in the specific
case where that comparison is actually apples-to-apples (equal
variances behave like equal confidences)?
"""
import math

from nav.estimation import PositionEstimate, fuse
from nav.kalman import KalmanEstimate, gated_update, predict, update

TOLERANCE = 1e-9


def check_equal_variance_is_a_50_50_blend():
    prior = KalmanEstimate((0.0, 0.0), 1.0)
    fused = update(prior, (10.0, 0.0), 1.0)
    ok = abs(fused.value[0] - 5.0) < TOLERANCE and abs(fused.variance - 0.5) < TOLERANCE
    print(f"equal variances (1.0, 1.0) -> value={fused.value}, variance={fused.variance:.4f} "
          f"(expected midpoint, variance 0.5) -- {'OK' if ok else 'FAIL'}")
    return ok


def check_more_certain_source_dominates():
    """A prior ten times more certain than the observation should pull
    the fused value most of the way toward itself, not halfway."""
    prior = KalmanEstimate((0.0, 0.0), 0.1)
    fused = update(prior, (10.0, 0.0), 1.0)
    # K = 0.1 / 1.1, fused = 0 + K*10 ~= 0.909
    expected = 0.1 / 1.1 * 10.0
    ok = abs(fused.value[0] - expected) < 1e-6 and fused.value[0] < 5.0
    print(f"low-variance prior (0.1) vs. high-variance observation (1.0) -> "
          f"value={fused.value[0]:.4f} (expected {expected:.4f}, should stay well below the midpoint) "
          f"-- {'OK' if ok else 'FAIL'}")
    return ok


def check_zero_variance_prior_refuses_to_move():
    """A perfectly certain prior (variance 0) should be completely
    unmoved by any observation, however confident."""
    prior = KalmanEstimate((3.0, 4.0), 0.0)
    fused = update(prior, (100.0, -100.0), 0.001)
    ok = fused.value == prior.value and fused.variance == 0.0
    print(f"zero-variance prior vs. wildly different observation -> value={fused.value} "
          f"(expected unchanged {prior.value}) -- {'OK' if ok else 'FAIL'}")
    return ok


def check_update_never_increases_variance():
    """Fusing in a second, independent observation should never leave
    the estimate LESS certain than the prior alone was -- that would
    mean the update step is throwing information away."""
    ok = True
    for prior_var in (0.1, 1.0, 5.0, 50.0):
        for obs_var in (0.1, 1.0, 5.0, 50.0):
            prior = KalmanEstimate((0.0, 0.0), prior_var)
            fused = update(prior, (1.0, 1.0), obs_var)
            if fused.variance > prior_var + TOLERANCE:
                ok = False
    print(f"update() never increases variance above the prior's own, across a grid of (prior, obs) "
          f"variance pairs -- {'OK' if ok else 'FAIL'}")
    return ok


def check_predict_grows_variance_and_leaves_mean_alone():
    state = KalmanEstimate((5.0, -2.0), 1.0)
    advanced = predict(state, 0.25)
    ok = advanced.value == state.value and abs(advanced.variance - 1.25) < TOLERANCE
    print(f"predict(variance=1.0, process_variance=0.25) -> value={advanced.value} (unchanged), "
          f"variance={advanced.variance:.4f} (expected 1.25) -- {'OK' if ok else 'FAIL'}")
    return ok


def check_predict_clamps_negative_process_variance():
    state = KalmanEstimate((0.0, 0.0), 2.0)
    advanced = predict(state, -5.0)
    ok = advanced.variance == 2.0
    print(f"predict() with a negative process_variance -> variance={advanced.variance:.4f} "
          f"(expected clamped to unchanged 2.0, not increased) -- {'OK' if ok else 'FAIL'}")
    return ok


def check_gate_scales_with_uncertainty_not_a_fixed_distance():
    """The exact same absolute distance (5 cells) should be accepted
    against a wide-open prior and rejected against a tight one -- the
    whole point of gating on sigma instead of a flat threshold (see
    ftc/config.py's FUSION_DISAGREEMENT_THRESHOLD_CELLS, the FIXED
    version nav/estimation.py's fuse() uses instead)."""
    far_observation = (5.0, 0.0)
    tight_prior = KalmanEstimate((0.0, 0.0), 0.01)
    _, tight_surprised = gated_update(tight_prior, far_observation, 0.01, gate_sigma=3.0, distrust_factor=0.2)
    wide_prior = KalmanEstimate((0.0, 0.0), 100.0)
    _, wide_surprised = gated_update(wide_prior, far_observation, 100.0, gate_sigma=3.0, distrust_factor=0.2)
    ok = tight_surprised and not wide_surprised
    print(f"identical 5-cell disagreement: tight prior (var=0.01) surprised={tight_surprised} "
          f"(expect True), wide prior (var=100) surprised={wide_surprised} (expect False) "
          f"-- {'OK' if ok else 'FAIL'}")
    return ok


def check_gated_update_inflates_rather_than_rejects():
    """A gated, surprising observation should still move the estimate
    (variance inflation lowers its weight, doesn't zero it) -- the same
    "shrink trust, don't zero it" choice fuse()'s distrust_factor makes,
    for the same documented reason."""
    prior = KalmanEstimate((0.0, 0.0), 1.0)
    plain, surprised = gated_update(prior, (20.0, 0.0), 1.0, gate_sigma=3.0, distrust_factor=0.2)
    ungated = update(prior, (20.0, 0.0), 1.0)
    ok = surprised and 0.0 < plain.value[0] < ungated.value[0]
    print(f"gated (surprised={surprised}) fused value={plain.value[0]:.3f} vs. ungated "
          f"{ungated.value[0]:.3f} -- gated should move less but not stay at 0 -- {'OK' if ok else 'FAIL'}")
    return ok


def check_degenerates_to_confidence_weighted_fuse_at_equal_uncertainty():
    """nav/kalman.py's update() and nav/estimation.py's fuse() are two
    genuinely different tools (see both modules' docstrings) -- but
    they should still agree in the one case where the comparison is
    fair: fuse() weights by `confidence` (higher = more trusted),
    update() weights by variance (lower = more trusted), so confidence
    a:b should match variance (1/a):(1/b). At a == b (equal confidence,
    i.e. equal variance too) both reduce to the same 50/50 blend --
    checked here as a sanity cross-check between the two modules, not a
    claim that they're interchangeable in general (see
    check_more_certain_source_dominates above for a case where they'd
    disagree if fed the same raw numbers as if they meant the same
    thing)."""
    prior_estimate = PositionEstimate((0.0, 0.0), 1.0)
    obs_estimate = PositionEstimate((10.0, 0.0), 1.0)
    fuse_result = fuse(prior_estimate, obs_estimate, disagreement_threshold=1e9, distrust_factor=1.0)

    kalman_prior = KalmanEstimate((0.0, 0.0), 1.0)
    kalman_result = update(kalman_prior, (10.0, 0.0), 1.0)

    ok = abs(fuse_result.value[0] - kalman_result.value[0]) < TOLERANCE
    print(f"equal-weight fuse() value={fuse_result.value[0]:.4f} vs. equal-variance update() "
          f"value={kalman_result.value[0]:.4f} -- {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------


def test_equal_variance_is_a_50_50_blend():
    assert check_equal_variance_is_a_50_50_blend()


def test_more_certain_source_dominates():
    assert check_more_certain_source_dominates()


def test_zero_variance_prior_refuses_to_move():
    assert check_zero_variance_prior_refuses_to_move()


def test_update_never_increases_variance():
    assert check_update_never_increases_variance()


def test_predict_grows_variance_and_leaves_mean_alone():
    assert check_predict_grows_variance_and_leaves_mean_alone()


def test_predict_clamps_negative_process_variance():
    assert check_predict_clamps_negative_process_variance()


def test_gate_scales_with_uncertainty_not_a_fixed_distance():
    assert check_gate_scales_with_uncertainty_not_a_fixed_distance()


def test_gated_update_inflates_rather_than_rejects():
    assert check_gated_update_inflates_rather_than_rejects()


def test_degenerates_to_confidence_weighted_fuse_at_equal_uncertainty():
    assert check_degenerates_to_confidence_weighted_fuse_at_equal_uncertainty()


if __name__ == "__main__":
    checks = [
        check_equal_variance_is_a_50_50_blend(),
        check_more_certain_source_dominates(),
        check_zero_variance_prior_refuses_to_move(),
        check_update_never_increases_variance(),
        check_predict_grows_variance_and_leaves_mean_alone(),
        check_predict_clamps_negative_process_variance(),
        check_gate_scales_with_uncertainty_not_a_fixed_distance(),
        check_gated_update_inflates_rather_than_rejects(),
        check_degenerates_to_confidence_weighted_fuse_at_equal_uncertainty(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
