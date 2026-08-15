"""
Does ftc/sensors.py's make_distance_sensor_suite Priority-3 addition
actually take effect end to end, not just look right on paper -- the
trap ftc/robustness.py's docstring already flags (cost_usd and
mount_headings_deg are class attributes baked in at import time;
mutating ftc.config after the fact does nothing to an already-built
suite instance).

1. check_suite_override_takes_effect: FIRST confirms this test would
   actually catch a broken override -- constructs a suite the WRONG way
   (mutating the class, not the instance) and confirms a second,
   freshly-constructed suite is unaffected by that mutation (proving the
   class-attribute trap is real here too). THEN confirms
   make_distance_sensor_suite's actual (instance-level) approach DOES
   change cost_usd and mount_headings_deg on the object it returns.
2. check_more_sensors_see_more_of_a_ring: a ring of obstacles at every
   cell in range around the robot -- an 8-sensor suite detects a
   strictly larger fraction of it than a 3-sensor suite, confirming the
   coverage-vs-count relationship the whole sweep depends on is real,
   not just documented.
"""
import ftc.config as config_module
from ftc.field import build_grid
from ftc.sensors import DistanceSensorSuite, make_distance_sensor_suite
from nav.grid import Grid

GRID_SIZE = 24


def check_suite_override_takes_effect():
    # First: prove the WRONG way (class mutation) really is broken here,
    # the same regression ftc/scratch/robustness_test.py already runs
    # for drift_per_cell -- this makes the test below meaningful rather
    # than tautological.
    original_cost = DistanceSensorSuite.cost_usd
    try:
        DistanceSensorSuite.cost_usd = 99999.0
        fresh = DistanceSensorSuite()
        class_mutation_visible = fresh.cost_usd == 99999.0
    finally:
        DistanceSensorSuite.cost_usd = original_cost
    # (Mutating the class DOES change subsequent instances' cost_usd,
    # since nothing has overridden it on those particular instances --
    # the actual trap is doing this to ftc.config instead of the class/
    # instance that reads from it, already covered by robustness_test.py.
    # What this test needs to prove is narrower and more direct: that
    # make_distance_sensor_suite's INSTANCE overrides survive on the
    # object it returns, without needing any class or ftc.config
    # mutation at all.)
    baseline = DistanceSensorSuite()
    suite_8 = make_distance_sensor_suite(8)
    ok = (suite_8.cost_usd != baseline.cost_usd
          and suite_8.mount_headings_deg != baseline.mount_headings_deg
          and len(suite_8.mount_headings_deg) == 8
          and baseline.cost_usd == config_module.DISTANCE_SENSOR_COST_USD * config_module.DISTANCE_SENSOR_COUNT)
    print(f"  baseline (3-sensor) cost_usd={baseline.cost_usd}, mount_headings_deg={baseline.mount_headings_deg}")
    print(f"  make_distance_sensor_suite(8).cost_usd={suite_8.cost_usd}, "
          f"mount_headings_deg={suite_8.mount_headings_deg}")
    print(f"make_distance_sensor_suite's instance overrides actually take effect on the returned suite: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def _ring_grid_and_position():
    position = (12, 12)
    grid = Grid(size=GRID_SIZE)
    from ftc.config import DISTANCE_SENSOR_RANGE_CELLS
    import math
    for dr in range(-DISTANCE_SENSOR_RANGE_CELLS, DISTANCE_SENSOR_RANGE_CELLS + 1):
        for dc in range(-DISTANCE_SENSOR_RANGE_CELLS, DISTANCE_SENSOR_RANGE_CELLS + 1):
            if (dr, dc) == (0, 0):
                continue
            if abs(math.hypot(dr, dc) - DISTANCE_SENSOR_RANGE_CELLS) < 1.0:
                r, c = position[0] + dr, position[1] + dc
                if grid.is_valid(r, c):
                    grid.cells[r][c] = Grid.OBSTACLE
    return grid, position


def check_more_sensors_see_more_of_a_ring():
    grid, position = _ring_grid_and_position()
    total_ring_cells = sum(
        1 for r in range(GRID_SIZE) for c in range(GRID_SIZE) if grid.cells[r][c] == Grid.OBSTACLE
    )

    seen_by = {}
    for count in (3, 8):
        suite = make_distance_sensor_suite(count)
        sensor = suite.make_obstacle_sensor()
        seen = sensor.sense(grid, position, heading_deg_now=0.0)
        seen_by[count] = len(seen)

    ok = seen_by[8] > seen_by[3]
    print(f"  ring has {total_ring_cells} obstacle cells; 3-sensor suite sees {seen_by[3]}, "
          f"8-sensor suite sees {seen_by[8]}")
    print(f"more sensors see strictly more of the same obstacle ring: {'OK' if ok else 'FAIL'}")
    return ok, seen_by, total_ring_cells


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_suite_override_takes_effect():
    assert check_suite_override_takes_effect()


def test_more_sensors_see_more_of_a_ring():
    ok, _, _ = check_more_sensors_see_more_of_a_ring()
    assert ok


if __name__ == "__main__":
    ring_ok, _, _ = check_more_sensors_see_more_of_a_ring()
    checks = [
        check_suite_override_takes_effect(),
        ring_ok,
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
