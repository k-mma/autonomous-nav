"""
Do ftc/sensors.py's suites actually respect the physical limits they
claim to model, not just produce plausible-looking numbers downstream?

1. ConeSensor never reports a cell outside its mounts' angular cones --
   the whole point of modeling narrow ToF cones instead of reusing
   nav/sensor.py's LidarSensor disc scan unmodified is that a cone
   sensor has real blind spots (see benchmark_results/
   ftc_suite_writeup.md's "Honest findings" section, which depends on
   this being true); if ConeSensor secretly saw everything in range
   regardless of angle, that finding would be fabricated.
2. ConeSensor never reports a cell beyond its configured range.
3. AprilTagSuite.tag_correction returns None whenever no tag is
   actually within range + FOV + line of sight -- pose correction that
   fired without a tag in view would be exactly the kind of "free"
   correction this project's whole cost/benefit framing depends on not
   existing.
4. AprilTagSuite.tag_correction returns a real correction when a tag
   genuinely is in view (range + FOV + LOS all satisfied).
5. An obstacle sitting directly between the robot and an otherwise-
   visible tag blocks the correction (line-of-sight actually matters,
   not just range + FOV).
"""
import math

from nav.grid import Grid

from ftc.config import DISTANCE_SENSOR_HALF_ANGLE_DEG, DISTANCE_SENSOR_RANGE_CELLS
from ftc.field import FieldElement, FieldLayout, TagSite, build_grid, in_to_cell
from ftc.sensors import AprilTagSuite, ConeSensor, angular_diff, heading_deg

GRID_SIZE = 24


def _grid_with_obstacles(cells):
    g = Grid(size=GRID_SIZE)
    for r, c in cells:
        g.cells[r][c] = Grid.OBSTACLE
    return g


def check_cone_sensor_stays_in_cone():
    """Ring the sensor with obstacles at every cell within range (a
    dense field, not a sparse one, so a leaky cone check would almost
    certainly catch at least one out-of-cone false positive)."""
    position = (12, 12)
    obstacles = [
        (position[0] + dr, position[1] + dc)
        for dr in range(-DISTANCE_SENSOR_RANGE_CELLS, DISTANCE_SENSOR_RANGE_CELLS + 1)
        for dc in range(-DISTANCE_SENSOR_RANGE_CELLS, DISTANCE_SENSOR_RANGE_CELLS + 1)
        if (dr, dc) != (0, 0) and 0 <= position[0] + dr < GRID_SIZE and 0 <= position[1] + dc < GRID_SIZE
    ]
    grid = _grid_with_obstacles(obstacles)
    sensor = ConeSensor([0.0, -90.0, 90.0], DISTANCE_SENSOR_HALF_ANGLE_DEG, DISTANCE_SENSOR_RANGE_CELLS)
    heading_now = 30.0  # an arbitrary, non-axis-aligned heading -- the harder case
    seen = sensor.sense(grid, position, heading_now)

    violations = 0
    for r, c in seen:
        dr, dc = r - position[0], c - position[1]
        angle = math.degrees(math.atan2(dr, dc))
        in_any_cone = any(
            abs(angular_diff(angle, heading_now + rel)) <= DISTANCE_SENSOR_HALF_ANGLE_DEG
            for rel in (0.0, -90.0, 90.0)
        )
        if not in_any_cone:
            violations += 1
            print(f"  cell ({r},{c}) at angle {angle:.1f}deg reported, outside every cone")
    ok = violations == 0 and len(seen) > 0
    print(f"ConeSensor only ever reports cells inside a mount's cone: "
          f"{'OK' if ok else f'FAIL ({violations} violations, {len(seen)} total seen)'}")
    return ok


def check_cone_sensor_respects_range():
    # A grid big enough that "range + 5" from the center never clips
    # against the boundary -- FTC_GRID_SIZE (24) is too small for that
    # relative to DISTANCE_SENSOR_RANGE_CELLS (~13), which would let a
    # boundary-clamped "far" obstacle land back inside range and make
    # this check pass for the wrong reason.
    size = 4 * DISTANCE_SENSOR_RANGE_CELLS
    position = (size // 2, size // 2)
    far_r = position[0] + DISTANCE_SENSOR_RANGE_CELLS + 5
    grid = Grid(size=size)
    grid.cells[far_r][position[1]] = Grid.OBSTACLE
    sensor = ConeSensor([0.0], 90.0, DISTANCE_SENSOR_RANGE_CELLS)  # wide-open cone, range is the only filter here
    seen = sensor.sense(grid, position, heading_deg((0, 0), (1, 0)))
    ok = len(seen) == 0
    print(f"ConeSensor never reports an obstacle beyond its configured range: {'OK' if ok else 'FAIL'}")
    return ok


def _corridor_grid_with_tag():
    """An obstacle-free grid with one AprilTag on the left wall (x=0,
    facing +col/east), matching ftc/field.py's DEFAULT_TAG_SITES
    convention -- see TagSite's docstring for the heading convention."""
    grid = build_grid(FieldLayout(elements=[]))
    tag = TagSite(x_in=0.0, y_in=72.0, heading_deg=0.0)
    return grid, tag


def check_no_correction_without_a_tag_in_view():
    grid, tag = _corridor_grid_with_tag()
    suite = AprilTagSuite()
    import random
    rng = random.Random(0)

    # Out of range: far from the tag along its facing direction.
    far_position = (12, 23)
    far = suite.tag_correction(grid, far_position, 0.0, [tag], rng)

    # In range, but outside the tag's FOV (approaching from the side,
    # not from roughly in front of it).
    tag_cell = in_to_cell(tag.x_in, tag.y_in)
    side_position = (tag_cell[0] + 3, tag_cell[1] + 3)  # ~45deg off the tag's 0deg-facing normal
    side = suite.tag_correction(grid, side_position, 0.0, [tag], rng)

    ok = far is None and side is None
    print(f"  far={far}, side={side}")
    print(f"AprilTagSuite.tag_correction returns None with no tag actually in view: {'OK' if ok else 'FAIL'}")
    return ok


def check_correction_when_tag_is_in_view():
    grid, tag = _corridor_grid_with_tag()
    suite = AprilTagSuite()
    import random
    rng = random.Random(0)

    tag_cell = in_to_cell(tag.x_in, tag.y_in)
    in_view_position = (tag_cell[0], tag_cell[1] + 6)  # straight out from the tag's facing direction, in range
    correction = suite.tag_correction(grid, in_view_position, 0.0, [tag], rng)
    ok = correction is not None and 0.0 < correction <= 1.0
    print(f"  correction={correction}")
    print(f"AprilTagSuite.tag_correction returns a real correction with a tag genuinely in view: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_obstacle_blocks_line_of_sight():
    grid, tag = _corridor_grid_with_tag()
    tag_cell = in_to_cell(tag.x_in, tag.y_in)
    in_view_position = (tag_cell[0], tag_cell[1] + 6)

    # Drop a wall directly between the tag and the robot.
    blocker_col = tag_cell[1] + 3
    for dr in range(-1, 2):
        r = tag_cell[0] + dr
        if grid.is_valid(r, blocker_col):
            grid.cells[r][blocker_col] = Grid.OBSTACLE

    suite = AprilTagSuite()
    import random
    rng = random.Random(0)
    correction = suite.tag_correction(grid, in_view_position, 0.0, [tag], rng)
    ok = correction is None
    print(f"  correction with a wall between robot and tag: {correction}")
    print(f"an obstacle between the robot and an otherwise-visible tag blocks the correction: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_cone_sensor_stays_in_cone():
    assert check_cone_sensor_stays_in_cone()


def test_cone_sensor_respects_range():
    assert check_cone_sensor_respects_range()


def test_no_correction_without_a_tag_in_view():
    assert check_no_correction_without_a_tag_in_view()


def test_correction_when_tag_is_in_view():
    assert check_correction_when_tag_is_in_view()


def test_obstacle_blocks_line_of_sight():
    assert check_obstacle_blocks_line_of_sight()


if __name__ == "__main__":
    checks = [
        check_cone_sensor_stays_in_cone(),
        check_cone_sensor_respects_range(),
        check_no_correction_without_a_tag_in_view(),
        check_correction_when_tag_is_in_view(),
        check_obstacle_blocks_line_of_sight(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
