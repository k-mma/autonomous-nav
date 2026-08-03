"""
variance_level (nav/field_variance.py) is an arbitrary [0, 1] knob --
good for sweeping the whole space (ftc/suite_benchmark.py), useless for
answering "where does *my* field and robot actually sit on this axis"
without some real measurement to anchor it. This module is that anchor:
given real measurements (nominal vs. actual field-element positions,
measured dead-reckoning drift over a real 30-second run), it fits the
corresponding variance_level component(s) -- so ftc/recommend.py can
turn "variance_level=0.4" into a number a team can actually check
their own field and robot against.

CRITICAL -- read this before trusting any output of this module. It
ships a clearly labeled SYNTHETIC placeholder dataset (see
SYNTHETIC_ELEMENT_MEASUREMENTS / SYNTHETIC_ODOMETRY_MEASUREMENTS below)
so every downstream tool has *something* to run against out of the
box. That placeholder is NOT a measurement of any real FTC field or
robot -- it is a documented, order-of-magnitude guess, chosen to be
roughly consistent with the already-documented ballpark constants in
ftc/config.py (DEAD_RECKONING_DRIFT_PER_CELL etc.), and nothing more.
Every fit_* function returns a CalibrationResult whose `.source` field
is either "measured" (real CSV data was supplied, non-empty, and used)
or "synthetic_placeholder" (no CSV given, or the given CSV was empty).
Every caller -- ftc/recommend.py, this module's own __main__ -- must
surface `.source` to the user, never just the fitted number. Drop your
own measured CSVs in and pass their paths; nothing here is more
trustworthy than a guess until you do.

CSV formats (see read_element_csv / read_odometry_csv for the exact
column contract):

  elements.csv:  element_id,nominal_x_in,nominal_y_in,actual_x_in,actual_y_in
    One row per surveyed field element: where the season's field setup
    guide/CAD says it goes, vs. where you actually measured it on your
    field, in inches from the same fixed reference corner.

  odometry.csv:  trial_id,distance_traveled_in,final_drift_in
    One row per real run with dead reckoning only (no vision/tag
    correction active): total commanded drive distance in inches, and
    the measured straight-line error between where the robot ended up
    and where it was commanded to be, at the end of the run.
"""
import csv
import math
from dataclasses import dataclass
from pathlib import Path

from nav.config import FIELD_VARIANCE_MAX_OBSTACLE_DRIFT_COUNT, FIELD_VARIANCE_MAX_START_DRIFT_RADIUS

from ftc.config import CELL_SIZE_IN, DEAD_RECKONING_DRIFT_PER_CELL

MEASURED = "measured"
SYNTHETIC = "synthetic_placeholder"

# A typical FTC autonomous route length, for converting a calibrated
# per-cell pose-drift rate into a start_drift variance_level component
# (see fit_start_drift_level) -- an assumption, not a measurement.
# Chosen as roughly half of ftc/field.py's 24-cell grid diagonal, a
# reasonable middle-of-the-season autonomous path length.
NOMINAL_PATH_CELLS = 15

# ---------------------------------------------------------------------
# SYNTHETIC PLACEHOLDER DATA -- NOT REAL MEASUREMENTS.
#
# element displacements: assumes careful-but-imperfect field setup --
# most elements 0.5-2.5in off their CAD nominal position, with 2 of the
# 8 displaced further (~4-5in, e.g. a scoring structure nudged during
# setup) so the synthetic obstacle_drift default isn't trivially zero.
# An order-of-magnitude guess for "a team taped the field carefully but
# didn't laser-survey it," not a citation of any real field audit.
#
# odometry drift: hand-picked so the *fitted* drift-per-cell rate lands
# close to ftc/config.py's own DEAD_RECKONING_DRIFT_PER_CELL (itself
# already labeled a ballpark estimate there) -- internally consistent
# with this project's other guesses, still a guess, still not data.
# ---------------------------------------------------------------------
SYNTHETIC_ELEMENT_MEASUREMENTS_CSV = (
    "element_id,nominal_x_in,nominal_y_in,actual_x_in,actual_y_in\n"
    "elem_1,24.0,24.0,24.8,23.5\n"
    "elem_2,72.0,24.0,71.6,25.9\n"
    "elem_3,120.0,24.0,120.4,24.2\n"
    "elem_4,24.0,72.0,26.3,71.8\n"
    "elem_5,72.0,72.0,72.1,71.9\n"
    "elem_6,120.0,72.0,117.0,75.5\n"
    "elem_7,24.0,120.0,24.6,120.4\n"
    "elem_8,72.0,120.0,75.5,116.5\n"
)
SYNTHETIC_ODOMETRY_MEASUREMENTS_CSV = (
    "trial_id,distance_traveled_in,final_drift_in\n"
    "trial_1,96.0,3.6\n"
    "trial_2,120.0,4.4\n"
    "trial_3,84.0,3.1\n"
    "trial_4,108.0,4.0\n"
    "trial_5,96.0,3.8\n"
)


@dataclass
class ElementMeasurement:
    element_id: str
    nominal_x_in: float
    nominal_y_in: float
    actual_x_in: float
    actual_y_in: float

    def displacement_in(self):
        return math.hypot(self.actual_x_in - self.nominal_x_in, self.actual_y_in - self.nominal_y_in)


@dataclass
class OdometryMeasurement:
    trial_id: str
    distance_traveled_in: float
    final_drift_in: float


@dataclass
class CalibrationResult:
    """`value`'s meaning depends on which fit_* function produced it --
    see each one's docstring. `source` is MEASURED or SYNTHETIC and
    must always be surfaced alongside `value`, never dropped. `detail`
    is a one-line human-readable explanation of how `value` was
    derived, for a printed report."""
    value: float
    source: str
    n_samples: int
    detail: str

    def is_measured(self):
        return self.source == MEASURED


def read_element_csv(path):
    """See module docstring for the required column names. Returns []
    on a missing/empty file rather than raising -- callers treat an
    empty list as "no real data supplied" and fall back to the
    synthetic placeholder (see load_calibration)."""
    path = Path(path)
    if not path.exists():
        return []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        return [
            ElementMeasurement(
                element_id=row["element_id"],
                nominal_x_in=float(row["nominal_x_in"]),
                nominal_y_in=float(row["nominal_y_in"]),
                actual_x_in=float(row["actual_x_in"]),
                actual_y_in=float(row["actual_y_in"]),
            )
            for row in reader
        ]


def read_odometry_csv(path):
    path = Path(path)
    if not path.exists():
        return []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        return [
            OdometryMeasurement(
                trial_id=row["trial_id"],
                distance_traveled_in=float(row["distance_traveled_in"]),
                final_drift_in=float(row["final_drift_in"]),
            )
            for row in reader
        ]


def _parse_csv_text(text, row_cls, field_names):
    reader = csv.DictReader(text.splitlines())
    return [row_cls(**{f: (row[f] if f.endswith("_id") else float(row[f])) for f in field_names}) for row in reader]


def synthetic_element_measurements():
    return _parse_csv_text(SYNTHETIC_ELEMENT_MEASUREMENTS_CSV, ElementMeasurement,
                             ["element_id", "nominal_x_in", "nominal_y_in", "actual_x_in", "actual_y_in"])


def synthetic_odometry_measurements():
    return _parse_csv_text(SYNTHETIC_ODOMETRY_MEASUREMENTS_CSV, OdometryMeasurement,
                             ["trial_id", "distance_traveled_in", "final_drift_in"])


def fit_obstacle_drift_level(measurements, source, cell_size_in=CELL_SIZE_IN,
                               max_count=FIELD_VARIANCE_MAX_OBSTACLE_DRIFT_COUNT):
    """variance_level's obstacle_drift component (nav/field_variance.py's
    obstacle_drift_scale axis, Phase 2): counts how many surveyed
    elements are displaced at least half a cell from nominal (the point
    where a planner's cell-grid obstacle layout would actually be
    wrong), and expresses that count as a fraction of
    FIELD_VARIANCE_MAX_OBSTACLE_DRIFT_COUNT -- the same unit
    nav/field_variance.py's obstacle_count already uses, so this fits
    directly onto that axis without a unit conversion.
    """
    if not measurements:
        return CalibrationResult(0.0, source, 0, "no element measurements available")
    threshold_in = cell_size_in / 2
    displaced = sum(1 for m in measurements if m.displacement_in() >= threshold_in)
    level = min(displaced / max_count, 1.0) if max_count > 0 else 0.0
    detail = (f"{displaced}/{len(measurements)} elements displaced >= {threshold_in:.1f}in "
               f"(half a {cell_size_in:.0f}in cell) from nominal -> {displaced}/{max_count} of "
               "FIELD_VARIANCE_MAX_OBSTACLE_DRIFT_COUNT")
    return CalibrationResult(level, source, len(measurements), detail)


def fit_pose_drift_rate(measurements, source, cell_size_in=CELL_SIZE_IN):
    """Fits a per-cell pose-drift rate in the same units as ftc/
    config.py's DEAD_RECKONING_DRIFT_PER_CELL / ODOMETRY_DRIFT_PER_CELL
    (stddev of pose error, in cells, injected per cell of real travel).

    The suite drift model (ftc/sensors.py, ftc/match.py) adds
    independent Gaussian noise of that stddev every step, so error
    accumulates like a random walk: after N cells of travel with
    constant per-cell stddev sigma, the expected magnitude of
    accumulated error is proportional to sigma * sqrt(N), not sigma * N.
    Inverting that relationship (sigma = measured_drift / sqrt(N)) is
    what this function does, rather than the simpler-but-wrong linear
    "drift / distance" you'd get from treating it as constant-rate
    accumulation.
    """
    if not measurements:
        return CalibrationResult(0.0, source, 0, "no odometry measurements available")
    rates = []
    for m in measurements:
        if m.distance_traveled_in <= 0:
            continue
        n_cells = m.distance_traveled_in / cell_size_in
        drift_cells = m.final_drift_in / cell_size_in
        rates.append(drift_cells / math.sqrt(n_cells))
    if not rates:
        return CalibrationResult(0.0, source, 0, "no usable odometry measurements (all zero distance)")
    mean_rate = sum(rates) / len(rates)
    detail = (f"mean of {len(rates)} trials' drift_cells / sqrt(distance_cells) -> "
               f"{mean_rate:.4f} cells stddev per cell traveled "
               f"(ftc/config.py's DEAD_RECKONING_DRIFT_PER_CELL default is "
               f"{DEAD_RECKONING_DRIFT_PER_CELL})")
    return CalibrationResult(mean_rate, source, len(rates), detail)


def fit_start_drift_level(pose_drift_rate_result, nominal_path_cells=NOMINAL_PATH_CELLS,
                            max_radius=FIELD_VARIANCE_MAX_START_DRIFT_RADIUS):
    """Converts a fitted per-cell pose-drift rate into variance_level's
    start_drift component: expected accumulated drift (cells) over
    `nominal_path_cells` of travel, as a fraction of
    FIELD_VARIANCE_MAX_START_DRIFT_RADIUS. This is an approximation --
    nav/field_variance.py's start_drift models a one-time initial
    placement offset, not drift accumulated while driving -- but by the
    time an open-loop run reaches its scoring position, "wrong from the
    start" and "drifted the whole way there" produce the same kind of
    failure (the executed path lands somewhere other than planned), so
    treating accumulated drift as an equivalent start-offset magnitude
    is a defensible stand-in until Phase 5 has a reason to do better.
    """
    expected_drift_cells = pose_drift_rate_result.value * math.sqrt(nominal_path_cells)
    level = min(expected_drift_cells / max_radius, 1.0) if max_radius > 0 else 0.0
    detail = (f"{pose_drift_rate_result.value:.4f} cells/sqrt(cell) rate * sqrt({nominal_path_cells}) "
               f"nominal path cells = {expected_drift_cells:.2f} expected drift cells -> "
               f"{expected_drift_cells:.2f}/{max_radius} of FIELD_VARIANCE_MAX_START_DRIFT_RADIUS "
               "(approximation: treats accumulated drive drift as start-offset-equivalent -- see docstring)")
    return CalibrationResult(level, pose_drift_rate_result.source, pose_drift_rate_result.n_samples, detail)


@dataclass
class Calibration:
    obstacle_drift: CalibrationResult
    pose_drift_rate: CalibrationResult
    start_drift: CalibrationResult

    def describe(self):
        lines = []
        for name, result in [("obstacle_drift", self.obstacle_drift),
                               ("pose_drift_rate", self.pose_drift_rate),
                               ("start_drift", self.start_drift)]:
            tag = "REAL MEASURED DATA" if result.is_measured() else "SYNTHETIC PLACEHOLDER -- NOT REAL DATA"
            lines.append(f"[{tag}] {name} = {result.value:.4f}  (n={result.n_samples})  {result.detail}")
        return "\n".join(lines)


def load_calibration(element_csv_path=None, odometry_csv_path=None):
    """The one entry point ftc/recommend.py (and anything else) should
    use. Reads real CSVs if paths are given and non-empty; falls back
    to the labeled synthetic placeholder for whichever one wasn't
    supplied. Never mixes silently -- the returned Calibration's three
    CalibrationResults each carry their own independent `.source`, so a
    caller with real element data but no odometry data (or vice versa)
    gets an honest mixed report, not a single flag that hides which
    half is real.
    """
    element_measurements = read_element_csv(element_csv_path) if element_csv_path else []
    element_source = MEASURED
    if not element_measurements:
        element_measurements = synthetic_element_measurements()
        element_source = SYNTHETIC

    odometry_measurements = read_odometry_csv(odometry_csv_path) if odometry_csv_path else []
    odometry_source = MEASURED
    if not odometry_measurements:
        odometry_measurements = synthetic_odometry_measurements()
        odometry_source = SYNTHETIC

    obstacle_drift = fit_obstacle_drift_level(element_measurements, element_source)
    pose_drift_rate = fit_pose_drift_rate(odometry_measurements, odometry_source)
    start_drift = fit_start_drift_level(pose_drift_rate)

    return Calibration(obstacle_drift=obstacle_drift, pose_drift_rate=pose_drift_rate, start_drift=start_drift)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--elements", help="CSV of nominal vs. actual field-element positions", default=None)
    parser.add_argument("--odometry", help="CSV of measured dead-reckoning drift over real runs", default=None)
    args = parser.parse_args()

    calibration = load_calibration(args.elements, args.odometry)
    print(calibration.describe())
    if not calibration.obstacle_drift.is_measured() or not calibration.pose_drift_rate.is_measured():
        print("\nNOTE: one or more inputs used the SYNTHETIC placeholder above, not real measurements.")
        print("Pass --elements/--odometry with your own CSVs (see this file's module docstring for the")
        print("exact column format) once you have real field/robot data.")
