"""
Two robots navigating simultaneously without colliding, including a
forced conflict through a single-width corridor and a documented
deadlock-resolution policy.

The layout: a wall with exactly one row-tall gap in it (the corridor).
Robot A starts on the west side, robot B on the east side, and each one's
goal is the other's start -- they have to swap sides through the same
one-cell-wide opening, a head-on conflict, not just "somewhere on the
same grid."

Coordination policy (see WRITEUPS.md for the full writeup):
- Robot A has strict right-of-way: it plans once against the static grid
  and never replans or reacts to B at all.
- Robot B always plans against the static grid *plus* a block placed
  around A's current cell, so it naturally detours around, or waits for,
  wherever A currently is. It replans every REPLAN_PERIOD_S.
- If A is sitting inside the corridor (the only route), B's planner
  reports "no path" -- B holds position and retries next tick rather
  than crashing or looping forever. Since A never yields and always has
  somewhere to go, this can't turn into a true two-way deadlock: exactly
  one robot (B) always yields, by construction.
- A hard safety stop is layered on top: if the two robots' actual
  distance ever falls below SAFETY_STOP_RADIUS, B is forced to stop that
  frame regardless of what its plan says. Grid-based replanning runs
  every REPLAN_PERIOD_S, not every physics tick, so this is the failsafe
  against a fast-moving robot closing that gap in between replans.

    python3 pybullet_multi_robot_main.py
    python3 pybullet_multi_robot_main.py --headless --max-seconds 60
"""
import argparse
import math
import time

import pybullet as p

from nav.algorithms import find_path
from nav.grid import Grid
from pybullet_main import build_drive_waypoints
from nav.sim3d.coords import grid_to_world, world_to_grid
from nav.sim3d.robot import Robot
from nav.sim3d.world import connect, build_obstacles, mark_cell, ROBOT_A_COLOR, ROBOT_B_COLOR

SIM_HZ = 240
REPLAN_PERIOD_S = 0.2
BLOCK_RADIUS = 1
SAFETY_STOP_RADIUS = 1.0

ROBOT_A_START, ROBOT_A_GOAL = (12, 2), (12, 22)
ROBOT_B_START, ROBOT_B_GOAL = (12, 22), (12, 2)
CORRIDOR_ROW = 12
WALL_COLS = (10, 11)


def build_corridor_grid():
    """A wall at cols 10-11 spanning rows 8-16, except CORRIDOR_ROW, left
    open -- the one-cell-wide (in row) gap both robots must funnel
    through from opposite ends."""
    grid = Grid()
    for row in range(8, 17):
        if row == CORRIDOR_ROW:
            continue
        for col in WALL_COLS:
            grid.cells[row][col] = Grid.OBSTACLE
    return grid


def blocked_grid(base_grid, blocked_cells):
    """A cheap copy of base_grid with `blocked_cells` additionally marked
    as obstacles -- the same technique the pygame MovingObstacle
    replanning logic used to make A* treat a moving thing as a temporary
    wall, just with a robot instead of a scripted bouncing obstacle."""
    g = Grid()
    g.cells = [row[:] for row in base_grid.cells]
    for r, c in blocked_cells:
        if g.is_valid(r, c) and g.cells[r][c] == Grid.FREE:
            g.cells[r][c] = Grid.OBSTACLE
    return g


def cell_block(cell, radius):
    r0, c0 = cell
    return {(r0 + dr, c0 + dc) for dr in range(-radius, radius + 1) for dc in range(-radius, radius + 1)}


class NavAgent:
    """One robot's navigation state. `blocking` robots (A) never look at
    anyone else; yielding robots (B) are handed the current set of cells
    to additionally avoid every time they (re)plan."""

    def __init__(self, name, body_id, base_grid, goal, smooth_method):
        self.name = name
        self.robot = Robot(body_id)
        self.base_grid = base_grid
        self.goal = goal
        self.smooth_method = smooth_method
        self.waypoints = []
        self.idx = 0
        self.waiting = False
        self.arrived = False

    def current_cell(self):
        pos = self.robot.position()
        return world_to_grid(pos[0], pos[1])

    def _park(self):
        """Goal(A) and start(B) are the same cell (and vice versa) --
        that's inherent to a "swap sides" scenario, not a bug, but it
        means a robot resting exactly on arrival would permanently
        occupy the other robot's destination. Nudge it a meter off to
        the side (out of the corridor's row, into open space that was
        never part of either grid path) once it's done, so it's out of
        the way both physically and for the other robot's planning."""
        pos = self.robot.position()
        p.resetBasePositionAndOrientation(self.robot.body_id, [pos[0], pos[1] + 1.5, pos[2]], [0, 0, 0, 1])

    def plan(self, blocked_cells=None):
        grid = blocked_grid(self.base_grid, blocked_cells) if blocked_cells else self.base_grid
        cell = self.current_cell()
        if cell == self.goal:
            self.arrived = True
            self.robot.stop()
            self._park()
            return
        path, _, reason, _ = find_path(grid, "astar", cell, self.goal)
        if path is None:
            self.waiting = True
            self.robot.stop()
            return
        self.waiting = False
        self.waypoints = build_drive_waypoints(path, self.smooth_method)
        self.idx = 0

    def drive_step(self, force_stop=False):
        if self.arrived:
            return
        if force_stop or self.waiting or not self.waypoints:
            self.robot.stop()
            return
        if self.idx >= len(self.waypoints):
            self.arrived = True
            self.robot.stop()
            self._park()
            return
        if self.robot.drive_toward(self.waypoints[self.idx]):
            self.idx += 1
            if self.idx >= len(self.waypoints):
                self.arrived = True
                self.robot.stop()
                self._park()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--smooth", choices=["raw", "corner_cut", "spline"], default="spline")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=45.0)
    return parser.parse_args()


def main():
    args = parse_args()
    gui = not args.headless
    connect(gui=gui)

    grid = build_corridor_grid()
    build_obstacles(grid)
    mark_cell(*ROBOT_A_START, color=ROBOT_A_COLOR)
    mark_cell(*ROBOT_B_START, color=ROBOT_B_COLOR)

    ax, ay, _ = grid_to_world(*ROBOT_A_START)
    bx, by, _ = grid_to_world(*ROBOT_B_START)
    a_id = p.loadURDF("r2d2.urdf", basePosition=[ax, ay, 0.4])
    b_id = p.loadURDF("r2d2.urdf", basePosition=[bx, by, 0.4])

    agent_a = NavAgent("A (priority)", a_id, grid, ROBOT_A_GOAL, args.smooth)
    agent_b = NavAgent("B (yields)", b_id, grid, ROBOT_B_GOAL, args.smooth)

    def a_blockers():
        # Once A has arrived and parked, it's no longer occupying the
        # grid at all -- stop treating it as an obstacle.
        return cell_block(agent_a.current_cell(), BLOCK_RADIUS) if not agent_a.arrived else set()

    agent_a.plan()  # A never looks at B, ever -- planned once, done.
    agent_b.plan(blocked_cells=a_blockers())

    steps = 0
    max_steps = int(args.max_seconds * SIM_HZ)
    next_replan_b = 0
    was_waiting = False

    while steps < max_steps and not (agent_a.arrived and agent_b.arrived):
        if steps >= next_replan_b and not agent_b.arrived:
            next_replan_b = steps + int(REPLAN_PERIOD_S * SIM_HZ)
            agent_b.plan(blocked_cells=a_blockers())
            if agent_b.waiting and not was_waiting:
                print(f"  t={steps / SIM_HZ:.1f}s: B has no clear path (A is in/near the corridor) -- waiting")
            elif was_waiting and not agent_b.waiting:
                print(f"  t={steps / SIM_HZ:.1f}s: corridor clear -- B resuming")
            was_waiting = agent_b.waiting

        pa, pb = agent_a.robot.position(), agent_b.robot.position()
        too_close = (not agent_a.arrived) and math.hypot(pa[0] - pb[0], pa[1] - pb[1]) < SAFETY_STOP_RADIUS

        agent_a.drive_step()
        agent_b.drive_step(force_stop=too_close)

        p.stepSimulation()
        if not args.headless:
            time.sleep(1 / SIM_HZ)
        steps += 1

    print(f"robot A arrived: {agent_a.arrived}")
    print(f"robot B arrived: {agent_b.arrived}")
    if not (agent_a.arrived and agent_b.arrived):
        print(f"stopped after {args.max_seconds}s safety cap")

    if not args.headless:
        input("Press Enter to close...")
    p.disconnect()


if __name__ == "__main__":
    main()
