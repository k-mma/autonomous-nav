"""
Two robots navigating simultaneously without colliding, crossing paths at
a genuine intersection rather than sharing a start/goal cell.

The layout: four building blocks in the grid's four quadrants, leaving a
3-cell-wide "plus" of open street down the middle -- one street running
north-south, one running east-west, crossing at the grid's center. Robot
A drives the north-south street start to finish; robot B drives the
east-west street start to finish. Their start and goal cells are all
different from each other (unlike an earlier version of this demo, where
goal(A) == start(B) by construction and caused its own class of bugs --
see WRITEUPS.md). The two straight-line routes are the same length and
both robots are spawned already facing their first direction of travel,
so without any avoidance they arrive at the crossing at essentially the
same moment -- the near-collision is a property of the layout and
timing, not scripted.

Coordination policy (see WRITEUPS.md for the full writeup, including
several real bugs found by watching a live run rather than just checking
pass/fail at the end -- most recently, an earlier version of this file
tried to avoid collisions with a per-frame steering nudge instead of
replanning, which visibly reduced the crossing distance but didn't
reliably prevent actual contact):

- Both robots replan, symmetrically. Every REPLAN_PERIOD_S, each one
  runs A* against the real grid *plus* a block placed around the other
  robot's current cell (`cell_block`) -- the same technique the pygame
  `MovingObstacle` replanning logic used, just with a robot standing in
  for the "moving obstacle" on both sides at once. Detecting the other
  robot nearby produces a genuinely different, grid-verified route (using
  the street's spare width to slide into an adjacent lane), not a
  steering offset layered on top of an unrelated path -- so the result
  can never steer either robot into a wall the way a blind offset could.
  A replan is only actually issued when the blocked cells changed or the
  last attempt found nothing (see WRITEUPS.md for why replanning
  unconditionally every tick is its own bug).
- If neither robot's current cell nor the other's makes a route
  possible, the blocked one holds position (`waiting = True`) and
  retries on the next replan tick, rather than crashing on `None` or
  driving into a wall.
- A hard safety-distance stop is layered on top as an absolute last
  resort, not the primary mechanism: if the two robots' actual distance
  ever falls below SAFETY_STOP_RADIUS, both are forced to stop that
  exact frame regardless of what their plans say. Checked
  unconditionally, every step, no exceptions. With replanning doing its
  job, this should rarely fire in practice.

    python3 pybullet_app/pybullet_multi_robot_main.py
    python3 pybullet_app/pybullet_multi_robot_main.py --headless --max-seconds 60
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pybullet as p

from nav.algorithms import find_path
from nav.grid import Grid
from pybullet_app.pybullet_main import build_drive_waypoints
from pybullet_app.sim3d.coords import grid_to_world, world_to_grid
from pybullet_app.sim3d.hud import Hud, FollowLabel
from pybullet_app.sim3d.robot import Robot, DEFAULT_SPEED
from pybullet_app.sim3d.world import (
    connect, build_obstacles, mark_cell, mark_goal_cell, label_cell, draw_trigger_marker, LivePath,
    ROBOT_A_COLOR, ROBOT_B_COLOR,
)

SIM_HZ = 240
# Tight enough to track the other robot's cell as it moves at speed (at
# DEFAULT_SPEED it crosses roughly one cell every 0.05s) without
# replanning so often it interrupts its own progress every tick.
REPLAN_PERIOD_S = 0.05
BLOCK_RADIUS = 1
SAFETY_STOP_RADIUS = 0.55
# How often the HUD text / debug-parameter sliders actually get read and
# redrawn -- see pybullet_main.py's HUD_UPDATE_PERIOD_S for why this is
# throttled well below the 240Hz physics rate.
HUD_UPDATE_PERIOD_S = 0.1
# How long the "replanning..." HUD indicator stays lit after a real
# replan, mirroring pygame_app/visualizer.py's REPLAN_FLASH_MS -- matched to
# pybullet_app/sim3d/world.py's PATH_LINGER_SECONDS, same reasoning as
# pybullet_main.py's REPLAN_FLASH_S.
REPLAN_FLASH_S = 1.2
HUD_POSITION = (2, 2, 9)

# A drives straight down the middle column; B drives straight across the
# middle row. Every one of the four points is a different cell -- no
# shared start/goal anywhere, unlike the earlier swap-sides layout.
CENTER = 12
ROBOT_A_START, ROBOT_A_GOAL = (2, CENTER), (22, CENTER)
ROBOT_B_START, ROBOT_B_GOAL = (CENTER, 2), (CENTER, 22)

# The open "plus" of street is CENTER +/- STREET_HALF_WIDTH in both row
# and column; BUILDING_SPANS fills the four quadrants outside it, leaving
# a margin at the grid's outer edge too.
STREET_HALF_WIDTH = 1  # street is STREET_HALF_WIDTH*2 + 1 = 3 cells wide
BUILDING_SPANS = (range(5, 11), range(14, 20))


def build_intersection_grid():
    """Four square buildings, one per quadrant, leaving a 3-wide street
    down the middle in both directions -- a real cross-street
    intersection, not just a single corridor. Neither street is ever
    blocked (buildings only occupy the row/col ranges *outside* the
    street), so both A's and B's straight-line routes are guaranteed
    clear; only the crossing itself, where the two streets overlap, is
    ever contested -- and even there, the 3-cell width gives a replan
    somewhere to actually go instead of just "wait"."""
    grid = Grid()
    street_lo, street_hi = CENTER - STREET_HALF_WIDTH, CENTER + STREET_HALF_WIDTH
    for row_span in BUILDING_SPANS:
        for col_span in BUILDING_SPANS:
            for row in row_span:
                if street_lo <= row <= street_hi:
                    continue
                for col in col_span:
                    if street_lo <= col <= street_hi:
                        continue
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
    """One robot's navigation state. Fully symmetric -- both A and B use
    the exact same class and the exact same policy (plan around whatever
    cells are currently blocked, replan when that changes). Nothing here
    knows or cares which agent is "the other one"; that's the caller's
    job every replan tick."""

    def __init__(self, name, body_id, base_grid, goal, smooth_method, color, gui):
        self.name = name
        self.robot = Robot(body_id)
        self.base_grid = base_grid
        self.goal = goal
        self.smooth_method = smooth_method
        self.color = color
        self.gui = gui
        self.waypoints = []
        self.idx = 0
        self.waiting = False
        self.arrived = False
        self.live_path = LivePath(color[:3], gui, SIM_HZ, z=0.04)
        self.replan_flash_until_step = 0
        self.label = FollowLabel(name, gui, color=color)

    def current_cell(self):
        pos = self.robot.position()
        return world_to_grid(pos[0], pos[1])

    def update_label(self):
        self.label.update(self.robot.position())

    def _park(self, now_step):
        """An arrived robot resting exactly on its goal cell would remain
        a phantom obstacle there forever as far as the other robot's
        planning is concerned. Nudge it a meter off to the side once it's
        done, so it's out of the way both physically and for the other
        robot's blocked-cell calculations."""
        pos = self.robot.position()
        p.resetBasePositionAndOrientation(self.robot.body_id, [pos[0], pos[1] + 1.5, pos[2]], [0, 0, 0, 1])
        self.live_path.clear(now_step)

    def plan(self, blocked_cells=None, now_step=0):
        """Runs a fresh A* against `grid` and -- if it actually changes the
        route -- redraws the active-path debug line for it (item 4: a
        replan that doesn't change anything, or that finds no route at
        all, shouldn't leave a stale line from before on screen, and one
        that does find a fresh route should never leave the *old* line
        drawn alongside the new one). The redraw itself flashes in and
        the superseded route lingers rather than an instant swap -- see
        pybullet_app/sim3d/world.py: LivePath."""
        grid = blocked_grid(self.base_grid, blocked_cells) if blocked_cells else self.base_grid
        cell = self.current_cell()
        if cell == self.goal:
            self.arrived = True
            self.robot.stop()
            self._park(now_step)
            return

        path, _, reason, _ = find_path(grid, "astar", cell, self.goal)
        self.replan_flash_until_step = now_step + int(REPLAN_FLASH_S * SIM_HZ)
        if path is None:
            self.waiting = True
            self.robot.stop()
            self.live_path.clear(now_step)
            return

        self.waiting = False
        self.waypoints = build_drive_waypoints(path, self.smooth_method)
        self.idx = 0
        self.live_path.set_path(self.waypoints, now_step)

    def drive_step(self, now_step, force_stop=False):
        if self.arrived:
            return
        if force_stop or self.waiting or not self.waypoints:
            self.robot.stop()
            return
        if self.idx >= len(self.waypoints):
            self.arrived = True
            self.robot.stop()
            self._park(now_step)
            return
        if self.robot.drive_toward(self.waypoints[self.idx]):
            self.idx += 1
            if self.idx >= len(self.waypoints):
                self.arrived = True
                self.robot.stop()
                self._park(now_step)


def agent_status(agent, steps):
    if agent.arrived:
        return "arrived"
    if steps < agent.replan_flash_until_step:
        return "replanning..."
    if agent.waiting:
        return "waiting"
    return "driving"


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

    grid = build_intersection_grid()
    build_obstacles(grid)
    mark_cell(*ROBOT_A_START, color=ROBOT_A_COLOR)
    mark_goal_cell(*ROBOT_A_GOAL, color=ROBOT_A_COLOR)
    mark_cell(*ROBOT_B_START, color=ROBOT_B_COLOR)
    mark_goal_cell(*ROBOT_B_GOAL, color=ROBOT_B_COLOR)
    if gui:
        label_cell(*ROBOT_A_START, "A start", ROBOT_A_COLOR)
        label_cell(*ROBOT_A_GOAL, "A goal", ROBOT_A_COLOR)
        label_cell(*ROBOT_B_START, "B start", ROBOT_B_COLOR)
        label_cell(*ROBOT_B_GOAL, "B goal", ROBOT_B_COLOR)

    ax, ay, _ = grid_to_world(*ROBOT_A_START)
    bx, by, _ = grid_to_world(*ROBOT_B_START)
    # Spawn each robot already facing its first direction of travel (A
    # south, B east) instead of both defaulting to the URDF's neutral
    # heading. Left at the default, B already happens to face the way it
    # needs to go while A has to turn 90 degrees first -- a head start
    # for B that's an accident of geometry, not the routes. Since both
    # routes are the same length at the same speed, spawning both
    # pre-aimed makes their arrival at the crossing genuinely
    # simultaneous, which is what actually produces a close call instead
    # of a comfortable miss.
    a_orientation = p.getQuaternionFromEuler([0, 0, math.pi / 2])  # facing +y (south)
    b_orientation = p.getQuaternionFromEuler([0, 0, 0])  # facing +x (east)
    a_id = p.loadURDF("r2d2.urdf", basePosition=[ax, ay, 0.4], baseOrientation=a_orientation)
    b_id = p.loadURDF("r2d2.urdf", basePosition=[bx, by, 0.4], baseOrientation=b_orientation)

    agent_a = NavAgent("A", a_id, grid, ROBOT_A_GOAL, args.smooth, ROBOT_A_COLOR, gui)
    agent_b = NavAgent("B", b_id, grid, ROBOT_B_GOAL, args.smooth, ROBOT_B_COLOR, gui)

    def blockers_for(mover, other):
        # Once the other robot has arrived and parked, it's no longer
        # occupying the grid at all -- stop treating it as an obstacle.
        return cell_block(other.current_cell(), BLOCK_RADIUS) if not other.arrived else set()

    last_blockers_a = blockers_for(agent_a, agent_b)
    last_blockers_b = blockers_for(agent_b, agent_a)
    agent_a.plan(blocked_cells=last_blockers_a)
    agent_b.plan(blocked_cells=last_blockers_b)

    hud = Hud(HUD_POSITION, gui)
    speed_param = p.addUserDebugParameter("robot speed", 5.0, 40.0, DEFAULT_SPEED) if gui else None

    steps = 0
    max_steps = int(args.max_seconds * SIM_HZ)
    next_replan = 0
    next_hud_step = 0
    was_waiting_a = agent_a.waiting
    was_waiting_b = agent_b.waiting

    while steps < max_steps and not (agent_a.arrived and agent_b.arrived):
        if steps >= next_replan:
            next_replan = steps + int(REPLAN_PERIOD_S * SIM_HZ)
            # Only actually replan when the obstacle picture changed, or
            # the last attempt found nothing at all -- replanning
            # unconditionally every tick throws away perfectly good
            # progress each time, since a fresh plan starts from the
            # *rounded* current cell and resets the current target.
            if not agent_a.arrived:
                current_a = blockers_for(agent_a, agent_b)
                if agent_a.waiting or current_a != last_blockers_a:
                    agent_a.plan(blocked_cells=current_a, now_step=steps)
                    last_blockers_a = current_a
                    # B's current cell is *why* A just replanned -- call
                    # it out instead of leaving the viewer to infer cause
                    # and effect purely from the path bending.
                    draw_trigger_marker(*agent_b.current_cell(), gui=gui)
                    if agent_a.waiting != was_waiting_a:
                        state = "no route right now -- holding" if agent_a.waiting else "replanned around B"
                        print(f"  t={steps / SIM_HZ:.2f}s: A {state}")
                    was_waiting_a = agent_a.waiting
            if not agent_b.arrived:
                current_b = blockers_for(agent_b, agent_a)
                if agent_b.waiting or current_b != last_blockers_b:
                    agent_b.plan(blocked_cells=current_b, now_step=steps)
                    last_blockers_b = current_b
                    draw_trigger_marker(*agent_a.current_cell(), gui=gui)
                    if agent_b.waiting != was_waiting_b:
                        state = "no route right now -- holding" if agent_b.waiting else "replanned around A"
                        print(f"  t={steps / SIM_HZ:.2f}s: B {state}")
                    was_waiting_b = agent_b.waiting

        pa = agent_a.robot.position()[:2]
        pb = agent_b.robot.position()[:2]
        too_close = math.hypot(pa[0] - pb[0], pa[1] - pb[1]) < SAFETY_STOP_RADIUS

        agent_a.drive_step(steps, force_stop=too_close)
        agent_b.drive_step(steps, force_stop=too_close)

        p.stepSimulation()
        if not args.headless:
            time.sleep(1 / SIM_HZ)
        steps += 1

        if steps >= next_hud_step:
            next_hud_step = steps + int(HUD_UPDATE_PERIOD_S * SIM_HZ)
            agent_a.live_path.tick(steps)
            agent_b.live_path.tick(steps)
            if speed_param is not None:
                shared_speed = p.readUserDebugParameter(speed_param)
                agent_a.robot.speed = shared_speed
                agent_b.robot.speed = shared_speed
            agent_a.update_label()
            agent_b.update_label()

            hud.update([
                f"multi-robot avoidance | A* | t={steps / SIM_HZ:.1f}s",
                f"A: {agent_status(agent_a, steps)} | wp {agent_a.idx}/{len(agent_a.waypoints)}",
                f"B: {agent_status(agent_b, steps)} | wp {agent_b.idx}/{len(agent_b.waypoints)}",
            ])

    print(f"robot A arrived: {agent_a.arrived}")
    print(f"robot B arrived: {agent_b.arrived}")
    if not (agent_a.arrived and agent_b.arrived):
        print(f"stopped after {args.max_seconds}s safety cap")

    if not args.headless:
        input("Press Enter to close...")
    p.disconnect()


if __name__ == "__main__":
    main()
