"""
Standalone sanity check: does PyBullet even work here? Load the empty
world, drop the r2d2 example robot into it, let physics settle, confirm
nothing explodes. Stop here -- the grid/A*/robot-driving work is
pybullet_app/sim3d/ and pybullet_main.py, not this file.

Run with a real GUI window (the actual visual sanity check):
    python3 -m pybullet_app.scratch.pybullet_setup_test

Run headless, e.g. in CI or over SSH with no display (--headless swaps
p.GUI for p.DIRECT and skips the "press enter" pause):
    python3 -m pybullet_app.scratch.pybullet_setup_test --headless
"""
import argparse
import time

import pybullet as p
import pybullet_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="use DIRECT mode, no GUI window")
    args = parser.parse_args()

    p.connect(p.DIRECT if args.headless else p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5])
    print(f"plane body id: {plane_id}")
    print(f"r2d2 body id: {robot_id}")

    seconds = 3
    for _ in range(240 * seconds):
        p.stepSimulation()
        if not args.headless:
            time.sleep(1 / 240)

    pos, _ = p.getBasePositionAndOrientation(robot_id)
    print(f"r2d2 settled at {pos} after {seconds}s")
    print("r2d2 is standing still in an empty world -- sanity check passed.")

    if not args.headless:
        input("GUI window should be open. Press Enter to close...")
    p.disconnect()


if __name__ == "__main__":
    main()
