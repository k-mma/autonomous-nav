"""
Standalone sanity check: does pybullet.rayTestBatch actually work the way
pybullet_app/sim3d/lidar.py assumes? Hardcoded 3D layout, one scan from a fixed
position, print which rays hit something and what grid cell each hit
maps back to. No robot, no driving, no visualizer -- just the raycast.

    python3 -m pybullet_app.scratch.pybullet_lidar_test [--headless]
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import pybullet as p

from nav.grid import Grid
from pybullet_app.sim3d.lidar import Lidar3D
from pybullet_app.sim3d.world import connect, build_obstacles
from pybullet_app.sim3d.coords import grid_to_world


def build_test_grid():
    grid = Grid()
    for row in range(8, 15):
        grid.cells[row][12] = Grid.OBSTACLE
    return grid


EXPECTED_WALL = {(row, 12) for row in range(8, 15)}


def run_scan(headless):
    connect(gui=not headless)
    grid = build_test_grid()
    build_obstacles(grid)

    scan_row, scan_col = 10, 8
    x, y, _ = grid_to_world(scan_row, scan_col)
    lidar = Lidar3D(num_rays=36, max_range=6.0)
    hits = lidar.scan((x, y), gui=not headless)

    print(f"scanning from grid cell ({scan_row}, {scan_col}) / world ({x}, {y})")
    print(f"rays cast: {lidar.num_rays}, max range: {lidar.max_range}m")
    print(f"newly discovered obstacle cells: {sorted(hits)}")
    print(f"total known_obstacles: {sorted(lidar.known_obstacles)}")
    return hits


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    hits = run_scan(args.headless)
    # all of EXPECTED_WALL is within 6m of (10, 8): |12-8|=4 <= 6
    found_expected = hits & EXPECTED_WALL
    print(f"expected wall cells within range that were actually found: {sorted(found_expected)}")
    assert found_expected, "lidar found none of the known wall cells -- raycast setup is broken"
    print("raycast sanity check passed.")

    if not args.headless:
        input("Press Enter to close...")
    p.disconnect()


# --- pytest entry points --------------------------------------------------

def test_raycast_finds_the_known_wall():
    try:
        hits = run_scan(headless=True)
        assert hits == EXPECTED_WALL, f"expected exactly {sorted(EXPECTED_WALL)}, got {sorted(hits)}"
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
