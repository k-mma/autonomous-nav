"""
Does ftc/match.py's trapezoidal drive-time model (_trapezoidal_drive_time_s)
actually implement a correct accelerate/cruise/decelerate physics
profile, and does it change match timing the way its own docstring
claims (strictly slower than the old naive distance/speed formula,
almost always via the triangular branch at this project's grid scale)?

1. A short distance (well under the accel-to-cruise threshold) uses the
   triangular branch and matches the closed-form kinematics answer
   (peak speed = sqrt(a*d), total time = 2*sqrt(d/a)) computed
   independently here, not just re-derived from the same formula.
2. A long distance (well over the threshold) uses the trapezoidal
   branch, and its total time decomposes correctly into accelerate +
   cruise + decelerate phases that individually cover the right
   distance in the right time.
3. At MAX_ACCEL_MPS2, a single grid cell (CELL_SIZE_IN, in meters) is
   short enough that _trapezoidal_drive_time_s never reaches
   MAX_DRIVE_SPEED_MPS -- i.e. every single-cell step in this project's
   actual grid falls in the triangular branch, confirming the module
   docstring's "nearly every step uses the triangular branch" claim
   with the project's real constants, not a hand-picked example.
4. The new drive time for one cell is strictly greater than the old
   naive distance/MAX_DRIVE_SPEED_MPS time -- finite acceleration can
   only make a step take longer or equal, never shorter.
5. distance_m=0 returns 0.0 (no divide-by-zero, no phantom drive time
   for a step that doesn't actually move).
"""
import math

from ftc.config import CELL_SIZE_IN, INCHES_PER_METER, MAX_ACCEL_MPS2, MAX_DRIVE_SPEED_MPS
from ftc.match import _trapezoidal_drive_time_s


def check_triangular_branch_matches_closed_form():
    d = 0.05  # well under accel_distance_m at this project's constants
    accel_distance = MAX_DRIVE_SPEED_MPS ** 2 / (2 * MAX_ACCEL_MPS2)
    assert d < 2 * accel_distance, "test distance must actually exercise the triangular branch"
    expected_time = 2 * math.sqrt(d / MAX_ACCEL_MPS2)
    peak_speed = math.sqrt(MAX_ACCEL_MPS2 * d)
    got = _trapezoidal_drive_time_s(d)
    ok = abs(got - expected_time) < 1e-9 and peak_speed < MAX_DRIVE_SPEED_MPS
    print(f"  distance={d}m, expected_time={expected_time:.4f}s, got={got:.4f}s, peak_speed={peak_speed:.3f}m/s "
          f"(< {MAX_DRIVE_SPEED_MPS} m/s cruise)")
    print(f"triangular branch matches the independently-derived closed-form kinematics answer: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_trapezoidal_branch_decomposes_correctly():
    accel_distance = MAX_DRIVE_SPEED_MPS ** 2 / (2 * MAX_ACCEL_MPS2)
    d = 2 * accel_distance + 1.0  # comfortably over the threshold, 1m of cruise
    got = _trapezoidal_drive_time_s(d)

    accel_time = MAX_DRIVE_SPEED_MPS / MAX_ACCEL_MPS2
    cruise_distance = d - 2 * accel_distance
    cruise_time = cruise_distance / MAX_DRIVE_SPEED_MPS
    expected_time = 2 * accel_time + cruise_time
    # Distance actually covered by each phase, integrating v(t) = a*t
    # during accel/decel, should sum back to d -- a decomposition check,
    # not just re-running the same formula.
    distance_covered = 2 * (0.5 * MAX_ACCEL_MPS2 * accel_time ** 2) + cruise_distance

    ok = abs(got - expected_time) < 1e-9 and abs(distance_covered - d) < 1e-9
    print(f"  distance={d:.3f}m, expected_time={expected_time:.4f}s, got={got:.4f}s, "
          f"distance_covered_by_phases={distance_covered:.4f}m (should equal {d:.4f})")
    print(f"trapezoidal branch decomposes into accel+cruise+decel phases that cover the right distance: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_single_cell_never_reaches_cruise():
    accel_distance = MAX_DRIVE_SPEED_MPS ** 2 / (2 * MAX_ACCEL_MPS2)
    cell_m = CELL_SIZE_IN / INCHES_PER_METER
    diagonal_cell_m = cell_m * math.sqrt(2)
    ok = cell_m < 2 * accel_distance and diagonal_cell_m < 2 * accel_distance
    print(f"  accel_distance (0 to cruise)={accel_distance:.4f}m, one cell={cell_m:.4f}m, "
          f"one diagonal cell={diagonal_cell_m:.4f}m")
    print(f"a single orthogonal or diagonal grid step is too short to reach MAX_DRIVE_SPEED_MPS at this "
          f"project's real constants: {'OK' if ok else 'FAIL'}")
    return ok


def check_strictly_slower_than_naive_formula():
    cell_m = CELL_SIZE_IN / INCHES_PER_METER
    naive_time = cell_m / MAX_DRIVE_SPEED_MPS
    new_time = _trapezoidal_drive_time_s(cell_m)
    ok = new_time > naive_time
    print(f"  naive distance/speed time={naive_time:.4f}s, trapezoidal time={new_time:.4f}s "
          f"({new_time / naive_time:.2f}x slower)")
    print(f"finite acceleration makes a single cell's drive time strictly longer than the old naive "
          f"formula: {'OK' if ok else 'FAIL'}")
    return ok


def check_zero_distance_is_zero_time():
    ok = _trapezoidal_drive_time_s(0.0) == 0.0 and _trapezoidal_drive_time_s(-1.0) == 0.0
    print(f"  _trapezoidal_drive_time_s(0.0)={_trapezoidal_drive_time_s(0.0)}, "
          f"_trapezoidal_drive_time_s(-1.0)={_trapezoidal_drive_time_s(-1.0)}")
    print(f"zero (or invalid negative) distance returns 0.0 with no divide-by-zero: {'OK' if ok else 'FAIL'}")
    return ok


if __name__ == "__main__":
    checks = [
        check_triangular_branch_matches_closed_form(),
        check_trapezoidal_branch_decomposes_correctly(),
        check_single_cell_never_reaches_cruise(),
        check_strictly_slower_than_naive_formula(),
        check_zero_distance_is_zero_time(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
