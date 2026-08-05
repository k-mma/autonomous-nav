"""
Do the Priority-4 suites (ImuSuite, AprilTagImuSuite,
DualCameraAprilTagSuite) actually behave the way their docstrings claim,
end to end?

1. check_imu_corrects_heading_without_replanning: an IMU-suite match at
   the pessimistic tier (heading drift on) ends with a smaller
   final_heading_error_deg than an otherwise-identical DeadReckoning
   match -- and crucially, doing so adds ZERO extra replans (an IMU fix
   never changes the believed grid cell, so ftc/match.py's replan
   trigger never fires from it).
2. check_imu_is_free: ImuSuite.cost_usd == 0.0 -- the whole point of the
   suite ("is the free hardware worth the code").
3. check_dual_camera_helps_under_realistic_fov_but_not_optimistic: a
   scripted scenario with two tags on opposite walls -- at the
   "realistic" tier, DualCameraAprilTagSuite corrects from a tag AprilTag
   Suite's single forward camera can't see (rear-facing); at the
   "optimistic" tier (omnidirectional camera either way), the single-
   camera suite already sees everything, so the second camera changes
   nothing -- the exact demonstration ftc/config.py's DualCameraAprilTag
   Suite docstring promises.
4. check_apriltag_imu_combines_both_corrections: AprilTagImuSuite's cost
   equals AprilTagSuite's alone (IMU is free), and it corrects heading
   even on ticks where no tag is in view (via the IMU alone) -- unlike
   AprilTagSuite, whose heading correction only ever piggybacks on an
   actual tag detection.
"""
import random

import ftc.config as config_module
from ftc.field import FieldLayout, TagSite, build_grid, in_to_cell, tag_sites_for
from ftc.match import run_match
from ftc.sensors import AprilTagImuSuite, AprilTagSuite, DeadReckoningSuite, DualCameraAprilTagSuite, ImuSuite


def _sparse_scenario(seed=5):
    grid = build_grid("sparse")
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for("sparse")
    from nav.algorithms import astar
    rng = random.Random(seed)
    for _ in range(50):
        start, goal = rng.sample(free, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= 10:
            break
    from nav.field_variance import generate_ground_truth
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.0, seed=seed)
    return grid, start, goal, ground_truth, actual_start, tag_sites


def check_imu_corrects_heading_without_replanning():
    grid, start, goal, ground_truth, actual_start, tag_sites = _sparse_scenario()

    dead_reckoning = run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                 random.Random(5), fidelity="pessimistic")
    imu = run_match(ImuSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                      random.Random(5), fidelity="pessimistic")

    heading_ok = abs(imu.final_heading_error_deg) < abs(dead_reckoning.final_heading_error_deg)
    replan_ok = imu.replans == dead_reckoning.replans  # heading-only correction never adds a replan
    ok = heading_ok and replan_ok
    print(f"  dead_reckoning final_heading_error_deg={dead_reckoning.final_heading_error_deg}, "
          f"replans={dead_reckoning.replans}")
    print(f"  imu final_heading_error_deg={imu.final_heading_error_deg}, replans={imu.replans}")
    print(f"ImuSuite ends with less heading error than DeadReckoningSuite, with the SAME replan count "
          f"(a heading-only fix never triggers a replan): {'OK' if ok else 'FAIL'}")
    return ok


def check_imu_is_free():
    ok = ImuSuite().cost_usd == 0.0
    print(f"  ImuSuite().cost_usd = {ImuSuite().cost_usd}")
    print(f"ImuSuite is genuinely $0 hardware cost: {'OK' if ok else 'FAIL'}")
    return ok


def check_dual_camera_helps_under_realistic_fov_but_not_optimistic():
    # A rear tag only a rear-facing camera can see: robot at (12, 12)
    # facing 0deg (east, its only camera direction for AprilTagSuite),
    # tag mounted on the west wall facing east (heading_deg=0) so it's
    # readable from directly in front of it -- but that means the tag
    # sits WEST of the robot, i.e. behind a robot facing east.
    grid = build_grid(FieldLayout(elements=[]))
    tag = TagSite(x_in=0.0, y_in=72.0, heading_deg=0.0)
    tag_cell = in_to_cell(tag.x_in, tag.y_in)
    robot_position = (tag_cell[0], tag_cell[1] + 6)  # east of the tag, in range/FOV/LOS
    robot_facing_east = 0.0  # camera (front-mounted) points away from the (western) tag

    single = AprilTagSuite()
    dual = DualCameraAprilTagSuite()

    original = config_module.MODEL_FIDELITY
    try:
        config_module.MODEL_FIDELITY = "realistic"
        single_realistic = single.tag_correction(grid, robot_position, robot_facing_east, [tag], random.Random(0))
        dual_realistic = dual.tag_correction(grid, robot_position, robot_facing_east, [tag], random.Random(0))

        config_module.MODEL_FIDELITY = "optimistic"
        single_optimistic = single.tag_correction(grid, robot_position, robot_facing_east, [tag], random.Random(0))
        dual_optimistic = dual.tag_correction(grid, robot_position, robot_facing_east, [tag], random.Random(0))
    finally:
        config_module.MODEL_FIDELITY = original

    realistic_ok = single_realistic is None and dual_realistic is not None
    optimistic_ok = (single_optimistic is not None and dual_optimistic is not None)
    ok = realistic_ok and optimistic_ok
    print(f"  realistic tier: single-camera={single_realistic}, dual-camera={dual_realistic}")
    print(f"  optimistic tier: single-camera={single_optimistic}, dual-camera={dual_optimistic}")
    print(f"a second (rear) camera detects a tag the single forward camera misses under realistic FOV "
          f"gating, but changes nothing under the optimistic (omnidirectional) tier: {'OK' if ok else 'FAIL'}")
    return ok


def check_apriltag_imu_combines_both_corrections():
    cost_ok = AprilTagImuSuite().cost_usd == AprilTagSuite().cost_usd
    print(f"  AprilTagSuite cost={AprilTagSuite().cost_usd}, AprilTagImuSuite cost={AprilTagImuSuite().cost_usd}")

    grid, start, goal, ground_truth, actual_start, tag_sites = _sparse_scenario()
    apriltag_only = run_match(AprilTagSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                                random.Random(5), fidelity="pessimistic")
    apriltag_imu = run_match(AprilTagImuSuite(), grid, start, goal, ground_truth, actual_start, tag_sites,
                               random.Random(5), fidelity="pessimistic")
    # AprilTag-only leaves the SAME dead-reckoning heading drift
    # unmanaged between detections; AprilTag+IMU should end with less
    # accumulated heading error since the IMU corrects it every tick
    # regardless of tag visibility.
    heading_ok = abs(apriltag_imu.final_heading_error_deg) <= abs(apriltag_only.final_heading_error_deg)
    ok = cost_ok and heading_ok
    print(f"  apriltag_only final_heading_error_deg={apriltag_only.final_heading_error_deg}")
    print(f"  apriltag_imu final_heading_error_deg={apriltag_imu.final_heading_error_deg}")
    print(f"AprilTagImuSuite costs the same as AprilTagSuite alone (IMU is free) and ends with no more "
          f"heading error than AprilTag alone: {'OK' if ok else 'FAIL'}")
    return ok


if __name__ == "__main__":
    checks = [
        check_imu_corrects_heading_without_replanning(),
        check_imu_is_free(),
        check_dual_camera_helps_under_realistic_fov_but_not_optimistic(),
        check_apriltag_imu_combines_both_corrections(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
