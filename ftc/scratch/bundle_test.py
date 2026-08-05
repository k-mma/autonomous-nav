"""
Does ftc/bundle.py's composition actually reproduce ftc/sensors.py's
suites, rather than approximate them?

That's the load-bearing claim of the whole optimizer: if a bundle of
{distance_sensors, apriltag, odometry_pods} isn't EXACTLY FullSuite,
then every number ftc/optimizer.py prints lives in a slightly different
universe from the published headline results, and the two can't be
compared. These checks are written to fail loudly if that ever stops
holding.

1. check_part_costs_reproduce_suite_costs: every suite's parts reprice
   to its own published cost_usd -- the part model is a refactor of
   ftc/sensors.py's flat prices, not a second source of truth that can
   drift.
2. check_single_component_bundle_is_identical: a one-suite bundle
   produces byte-for-byte identical MatchResults to that suite on the
   same seed, for every suite in SUITES -- same rng draws, same
   capabilities, same cost.
3. check_bundle_reproduces_full_suite: the three-component bundle IS
   FullSuite, across a batch of seeded matches.
4. check_shared_hardware_is_not_double_counted: apriltag + apriltag_imu
   costs one camera, not two, and is recognized as the same robot as
   apriltag_imu alone.
5. check_conflicting_parts_rejected: two different ToF layouts on one
   robot is refused, not silently priced.
6. check_composite_sensor_unions_coverage: a bundle of two obstacle
   sensors sees a superset of what either sees alone, with no
   double-reporting of an already-known cell.
"""
import random

from nav.field_variance import generate_ground_truth

from ftc.bundle import (
    CompositeObstacleSensor, enumerate_bundles, make_bundle, part_cost_mismatches, parts_for,
)
from ftc.field import build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import SUITES, DistanceSensorSuite, FullSuite, LidarSuite, make_distance_sensor_suite


def _scenario(seed, layout="cluttered", level=0.6):
    from nav.algorithms import astar
    grid = build_grid(layout)
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    rng = random.Random(seed)
    for _ in range(50):
        start, goal = rng.sample(free, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= 8:
            break
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, level, seed=seed)
    return grid, start, goal, ground_truth, actual_start, tag_sites_for(layout)


def _run(suite, seed, fidelity=None):
    grid, start, goal, ground_truth, actual_start, tag_sites = _scenario(seed)
    return run_match(suite, grid, start, goal, ground_truth, actual_start, tag_sites,
                     random.Random(seed), fidelity=fidelity)


def check_part_costs_reproduce_suite_costs():
    mismatches = part_cost_mismatches()
    for name, declared, from_parts in mismatches:
        print(f"  MISMATCH {name}: cost_usd={declared} but parts price to {from_parts}")
    ok = not mismatches
    print(f"Every suite's parts reprice to its own published cost_usd: {'OK' if ok else 'FAIL'}")
    return ok


def check_single_component_bundle_is_identical():
    ok = True
    # "realistic" as well as the default tier -- the camera-FOV gate and
    # heading drift are live there, so a bundle mishandling
    # camera_mount_headings_deg or heading correction would show up
    # here and nowhere else.
    for fidelity in (None, "realistic"):
        for name in SUITES:
            for seed in (3, 11, 29):
                direct = _run(SUITES[name](), seed, fidelity=fidelity)
                bundled = _run(make_bundle(name), seed, fidelity=fidelity)
                same = (direct.success == bundled.success and direct.steps == bundled.steps
                        and direct.collisions == bundled.collisions and direct.replans == bundled.replans
                        and direct.elapsed_s == bundled.elapsed_s
                        and direct.final_pose_error_in == bundled.final_pose_error_in
                        and direct.final_heading_error_deg == bundled.final_heading_error_deg)
                cost_same = make_bundle(name).cost_usd == SUITES[name].cost_usd
                if not (same and cost_same):
                    ok = False
                    print(f"  DIFFERS: {name} seed={seed} fidelity={fidelity}")
                    print(f"    direct  {direct}")
                    print(f"    bundled {bundled}")
    print(f"A one-suite bundle is byte-for-byte identical to that suite ({len(SUITES)} suites x 3 seeds x "
          f"2 fidelity tiers): {'OK' if ok else 'FAIL'}")
    return ok


def check_bundle_reproduces_full_suite():
    bundle = make_bundle("distance_sensors", "apriltag", "odometry_pods")
    cost_ok = bundle.cost_usd == FullSuite.cost_usd
    drift_ok = bundle.drift_per_cell == FullSuite.drift_per_cell
    caps_ok = (bundle.senses_obstacles == FullSuite.senses_obstacles
               and bundle.fixes_pose == FullSuite.fixes_pose)
    print(f"  bundle cost=${bundle.cost_usd:.2f} vs FullSuite ${FullSuite.cost_usd:.2f}; "
          f"drift {bundle.drift_per_cell} vs {FullSuite.drift_per_cell}")

    match_ok = True
    for seed in (2, 7, 13, 41, 97):
        direct = _run(FullSuite(), seed)
        bundled = _run(make_bundle("distance_sensors", "apriltag", "odometry_pods"), seed)
        if (direct.success, direct.steps, direct.elapsed_s, direct.collisions, direct.replans,
                direct.final_pose_error_in) != (bundled.success, bundled.steps, bundled.elapsed_s,
                                                 bundled.collisions, bundled.replans,
                                                 bundled.final_pose_error_in):
            match_ok = False
            print(f"  DIFFERS at seed={seed}: {direct} vs {bundled}")
    ok = cost_ok and drift_ok and caps_ok and match_ok
    print(f"distance_sensors + apriltag + odometry_pods IS FullSuite, cost and match-for-match: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_shared_hardware_is_not_double_counted():
    overlapping = make_bundle("apriltag", "apriltag_imu")
    alone = make_bundle("apriltag_imu")
    naive = SUITES["apriltag"].cost_usd + SUITES["apriltag_imu"].cost_usd
    cost_ok = overlapping.cost_usd == alone.cost_usd
    same_robot = overlapping.part_signature() == alone.part_signature()
    print(f"  apriltag + apriltag_imu: parts-union cost ${overlapping.cost_usd:.2f}, "
          f"naive sum ${naive:.2f}, apriltag_imu alone ${alone.cost_usd:.2f}")

    # And the enumerator drops the redundant one rather than offering a
    # team two names for the same purchase.
    bundles = enumerate_bundles(["apriltag", "apriltag_imu", "imu"], min_size=1)
    signatures = [b.part_signature() for b in bundles]
    deduped = len(signatures) == len(set(signatures))
    print(f"  enumerate_bundles over 3 overlapping suites -> {len(bundles)} distinct robots "
          f"(of 7 raw combinations)")
    ok = cost_ok and same_robot and deduped
    print(f"Shared hardware is costed once and duplicate robots are dropped: {'OK' if ok else 'FAIL'}")
    return ok


def check_conflicting_parts_rejected():
    three = make_distance_sensor_suite(3)
    six = make_distance_sensor_suite(6)
    raised = False
    try:
        make_bundle(three, six)
    except ValueError as exc:
        raised = True
        print(f"  refused as expected: {exc}")
    # And the parts of a swept variant are what its own cost says.
    from ftc.bundle import part_cost
    priced_ok = abs(part_cost(parts_for(six)) - six.cost_usd) < 0.005
    ok = raised and priced_ok
    print(f"Two conflicting ToF layouts on one robot are refused, and swept variants price correctly: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_composite_sensor_unions_coverage():
    grid = build_grid("cluttered")
    position = (grid.size // 2, grid.size // 2)
    heading = 0.0

    cones = DistanceSensorSuite().make_obstacle_sensor()
    lidar = LidarSuite().make_obstacle_sensor()
    cone_seen = cones.sense(grid, position, heading)
    lidar_seen = lidar.sense(grid, position, heading)

    composite = CompositeObstacleSensor([DistanceSensorSuite().make_obstacle_sensor(),
                                          LidarSuite().make_obstacle_sensor()])
    first = composite.sense(grid, position, heading)
    second = composite.sense(grid, position, heading)  # nothing moved -- nothing is "newly" seen

    union_ok = first == (cone_seen | lidar_seen)
    superset_ok = cone_seen <= first and lidar_seen <= first
    no_repeat_ok = second == set()
    print(f"  cones saw {len(cone_seen)}, lidar saw {len(lidar_seen)}, composite saw {len(first)} "
          f"(union {len(cone_seen | lidar_seen)}); second identical scan reported {len(second)} new")
    ok = union_ok and superset_ok and no_repeat_ok
    print(f"A composite sensor sees the union of its children and never re-reports a known cell: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


if __name__ == "__main__":
    checks = [
        check_part_costs_reproduce_suite_costs(),
        check_single_component_bundle_is_identical(),
        check_bundle_reproduces_full_suite(),
        check_shared_hardware_is_not_double_counted(),
        check_conflicting_parts_rejected(),
        check_composite_sensor_unions_coverage(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
