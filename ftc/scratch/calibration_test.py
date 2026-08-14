"""
Does ftc/calibration.py's new AprilTag-variance and process-variance
fitting actually work, in isolation, before either is wired into
nav/kalman.py via ftc/fusion.py?

1. check_solver_recovers_exact_linear_relationship: feed
   fit_apriltag_measurement_variance a synthetic dataset built from a
   KNOWN exact linear relationship (no noise) -- the fitted
   intercept/range_coef/incidence_coef should recover those exact
   numbers, which is the only way to prove _solve_3x3's hand-written
   Gaussian elimination is actually solving the normal equations
   correctly rather than just returning *something*.
2. check_too_few_detections_returns_null_model: fewer than 3 detections
   is under-determined for 3 parameters -- must return model=None, not
   a bogus fit, and must still report the real n_samples/source.
3. check_variance_floors_at_minimum: a model whose fitted line predicts
   a negative or near-zero variance at some (range, incidence) must
   still return a small positive floor, never zero or negative --
   nav/kalman.py's update() would divide by (near-)zero R otherwise.
4. check_process_variance_is_drift_rate_squared: fit_process_variance_
   per_cell's output must equal the input pose_drift_rate's value
   squared, exactly, and preserve its source/n_samples.
5. check_synthetic_apriltag_measurements_parses: the shipped placeholder
   CSV text round-trips through the same parser real files use, with
   the right row count and column values.
6. check_load_calibration_apriltag_source_is_honest: load_calibration
   reports MEASURED when a real (temp-file) apriltag CSV is supplied,
   and SYNTHETIC_PLACEHOLDER when it isn't -- the same honesty contract
   every other calibrated field in this module already has to keep.
"""
import csv
import tempfile
from pathlib import Path

from ftc.calibration import (
    MEASURED, SYNTHETIC, TagDetectionMeasurement,
    fit_apriltag_measurement_variance, fit_pose_drift_rate, fit_process_variance_per_cell,
    load_calibration, synthetic_apriltag_measurements,
)

TOLERANCE = 1e-6


def _measurement(i, r, incidence, error):
    return TagDetectionMeasurement(detection_id=f"d{i}", range_in=r, incidence_deg=incidence,
                                     position_error_in=error)


def check_solver_recovers_exact_linear_relationship():
    """variance = 2.0 + 3.0*range_in + 5.0*incidence_deg, exactly, with
    position_error_in set to sqrt(variance) so squaring it in the fit
    recovers that exact relationship with zero noise."""
    true_intercept, true_range_coef, true_incidence_coef = 2.0, 3.0, 5.0
    points = [(0.0, 0.0), (10.0, 0.0), (0.0, 10.0), (20.0, 5.0), (5.0, 20.0), (15.0, 15.0)]
    measurements = []
    for i, (r, incidence) in enumerate(points):
        variance = true_intercept + true_range_coef * r + true_incidence_coef * incidence
        measurements.append(_measurement(i, r, incidence, variance ** 0.5))

    result = fit_apriltag_measurement_variance(measurements, SYNTHETIC)
    model = result.model
    ok = (
        model is not None
        and abs(model.intercept - true_intercept) < 1e-4
        and abs(model.range_coef - true_range_coef) < 1e-4
        and abs(model.incidence_coef - true_incidence_coef) < 1e-4
    )
    print(f"exact linear fit recovers intercept={model.intercept:.4f} (expected {true_intercept}), "
          f"range_coef={model.range_coef:.4f} (expected {true_range_coef}), "
          f"incidence_coef={model.incidence_coef:.4f} (expected {true_incidence_coef}) -- "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_too_few_detections_returns_null_model():
    measurements = [_measurement(0, 10.0, 5.0, 1.0), _measurement(1, 20.0, 10.0, 2.0)]
    result = fit_apriltag_measurement_variance(measurements, MEASURED)
    ok = result.model is None and result.n_samples == 2 and result.source == MEASURED
    print(f"2 detections (need >=3) -> model={result.model}, n_samples={result.n_samples}, "
          f"source={result.source} -- {'OK' if ok else 'FAIL'}")
    return ok


def check_variance_floors_at_minimum():
    # A dataset whose best-fit line predicts a strongly negative
    # variance at (0, 0), same shape the real synthetic placeholder
    # dataset in ftc/calibration.py already produces.
    measurements = [
        _measurement(0, 100.0, 25.0, 4.0),
        _measurement(1, 110.0, 28.0, 4.3),
        _measurement(2, 120.0, 30.0, 4.6),
    ]
    result = fit_apriltag_measurement_variance(measurements, SYNTHETIC)
    v = result.model.variance_in2(0.0, 0.0)
    ok = v > 0.0
    print(f"variance at (0,0) extrapolated from a far-only dataset = {v:.4f} (must be > 0, floored) -- "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_process_variance_is_drift_rate_squared():
    from ftc.calibration import CalibrationResult
    rate_result = CalibrationResult(value=0.2, source=MEASURED, n_samples=7, detail="fake rate for this check")
    q_result = fit_process_variance_per_cell(rate_result)
    ok = abs(q_result.value - 0.04) < TOLERANCE and q_result.source == MEASURED and q_result.n_samples == 7
    print(f"drift rate 0.2 -> process variance {q_result.value:.4f} (expected 0.04 = 0.2^2), "
          f"source/n_samples preserved: {q_result.source == MEASURED and q_result.n_samples == 7} -- "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_synthetic_apriltag_measurements_parses():
    measurements = synthetic_apriltag_measurements()
    ok = len(measurements) == 12 and all(m.range_in > 0 and m.position_error_in > 0 for m in measurements)
    print(f"synthetic_apriltag_measurements() -> {len(measurements)} rows, all range/error > 0 -- "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_load_calibration_apriltag_source_is_honest():
    with tempfile.TemporaryDirectory() as tmp:
        real_path = Path(tmp) / "apriltag.csv"
        with open(real_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["detection_id", "range_in", "incidence_deg", "position_error_in"])
            for i in range(5):
                writer.writerow([f"d{i}", 10.0 + i * 10, i * 2.0, 1.0 + i * 0.3])

        with_real = load_calibration(apriltag_csv_path=real_path)
        without_real = load_calibration()

        ok = (with_real.apriltag_variance.source == MEASURED and with_real.apriltag_variance.n_samples == 5
              and without_real.apriltag_variance.source == SYNTHETIC)
        print(f"real CSV -> source={with_real.apriltag_variance.source} n={with_real.apriltag_variance.n_samples}; "
              f"no CSV -> source={without_real.apriltag_variance.source} -- {'OK' if ok else 'FAIL'}")
        return ok


# --- pytest entry points --------------------------------------------------


def test_solver_recovers_exact_linear_relationship():
    assert check_solver_recovers_exact_linear_relationship()


def test_too_few_detections_returns_null_model():
    assert check_too_few_detections_returns_null_model()


def test_variance_floors_at_minimum():
    assert check_variance_floors_at_minimum()


def test_process_variance_is_drift_rate_squared():
    assert check_process_variance_is_drift_rate_squared()


def test_synthetic_apriltag_measurements_parses():
    assert check_synthetic_apriltag_measurements_parses()


def test_load_calibration_apriltag_source_is_honest():
    assert check_load_calibration_apriltag_source_is_honest()


if __name__ == "__main__":
    checks = [
        check_solver_recovers_exact_linear_relationship(),
        check_too_few_detections_returns_null_model(),
        check_variance_floors_at_minimum(),
        check_process_variance_is_drift_rate_squared(),
        check_synthetic_apriltag_measurements_parses(),
        check_load_calibration_apriltag_source_is_honest(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
