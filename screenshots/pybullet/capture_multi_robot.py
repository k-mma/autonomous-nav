"""Captures the two multi-robot poster screenshots (two-robot corridor
avoidance and four-robot CBS intersection crossing), the same way
capture.py captures the single-robot demos: a real GUI connection
(DIRECT has no debug-visualizer camera to read back from), same camera
framing (distance 22, yaw 45, pitch -55, target [12, 12, 0]) a live
viewer would see, screenshot grabbed via getCameraImage.

Both scenarios are re-run here headlessly (no time.sleep, no
input()/interactive loop) rather than imported as a black box, since
what's actually worth capturing is a specific *moment* mid-run --
robots near the crossing, paths visibly diverging -- not the first or
last frame.

    python3 screenshots/pybullet/capture_multi_robot.py     # writes screenshots/pybullet/*.png
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import numpy as np
import pybullet as p
from PIL import Image

from nav.cbs import cbs
from pybullet_app.sim3d.coords import grid_to_world
from pybullet_app.sim3d.hud import Hud
from pybullet_app.sim3d.world import (
    connect, build_obstacles, mark_cell, mark_goal_cell, label_cell,
    ROBOT_A_COLOR, ROBOT_B_COLOR,
)
from pybullet_app.pybullet_cbs_main import build_agents, CBSAgent, PALETTE
from pybullet_app.pybullet_multi_robot_main import (
    build_intersection_grid, NavAgent, cell_block,
    ROBOT_A_START, ROBOT_A_GOAL, ROBOT_B_START, ROBOT_B_GOAL,
    SIM_HZ, REPLAN_PERIOD_S, BLOCK_RADIUS, SAFETY_STOP_RADIUS,
    HUD_UPDATE_PERIOD_S, HUD_POSITION,
)

OUT_DIR = Path(__file__).resolve().parent


def screenshot(name, width=1600, height=1200):
    """Same approach as capture.py's screenshot(): settle a few physics
    steps, then read back the debug visualizer's own camera matrices so
    every shot is framed identically to what connect() set up.

    Re-enables GUI rendering first (see begin_fast_forward's comment) --
    it's off for the whole simulation loop leading up to this call, and
    getCameraImage needs the debug visualizer's own camera state, which
    resetDebugVisualizerCamera set up back in connect()."""
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    for _ in range(5):
        p.stepSimulation()
    cam = p.getDebugVisualizerCamera()
    view_matrix, proj_matrix = cam[2], cam[3]
    img = p.getCameraImage(width, height, view_matrix, proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgba = np.reshape(img[2], (height, width, 4))
    out_path = OUT_DIR / name
    Image.fromarray(rgba[:, :, :3].astype("uint8")).save(out_path)
    print(f"wrote {out_path}")


def begin_fast_forward():
    """Both scenarios here simulate several sim-seconds of two/four
    robots driving before the actual moment worth capturing -- unlike
    capture.py's shots, which are posed once and screenshotted
    immediately. GUI mode syncs its window to every stepSimulation call
    by default, which makes running that many steps to get there far
    slower wall-clock than headless DIRECT mode would be, even with no
    time.sleep in the loop. Turning rendering off for the run-up (kept
    on only for the final screenshot -- see screenshot()) keeps the
    physics/replanning correct while skipping the redraw work, since
    nothing before the capture moment is ever actually looked at."""
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)


def capture_corridor():
    """Two robots, one corridor (pybullet_multi_robot_main.py). Runs the
    exact same symmetric replan-around-each-other policy that file's
    main() does, tracking the two robots' closest approach, then stops
    and captures shortly after it -- both robots near the crossing with
    their live-replanned paths visibly bent into the adjacent lane,
    which is the actual moment the avoidance policy is doing something
    (the start/end frames just show a robot driving a straight line)."""
    connect(gui=True)
    grid = build_intersection_grid()
    build_obstacles(grid)
    mark_cell(*ROBOT_A_START, color=ROBOT_A_COLOR)
    mark_goal_cell(*ROBOT_A_GOAL, color=ROBOT_A_COLOR)
    mark_cell(*ROBOT_B_START, color=ROBOT_B_COLOR)
    mark_goal_cell(*ROBOT_B_GOAL, color=ROBOT_B_COLOR)
    label_cell(*ROBOT_A_START, "A start", ROBOT_A_COLOR)
    label_cell(*ROBOT_A_GOAL, "A goal", ROBOT_A_COLOR)
    label_cell(*ROBOT_B_START, "B start", ROBOT_B_COLOR)
    label_cell(*ROBOT_B_GOAL, "B goal", ROBOT_B_COLOR)

    ax, ay, _ = grid_to_world(*ROBOT_A_START)
    bx, by, _ = grid_to_world(*ROBOT_B_START)
    a_orientation = p.getQuaternionFromEuler([0, 0, math.pi / 2])  # facing +y (south)
    b_orientation = p.getQuaternionFromEuler([0, 0, 0])  # facing +x (east)
    a_id = p.loadURDF("r2d2.urdf", basePosition=[ax, ay, 0.4], baseOrientation=a_orientation)
    b_id = p.loadURDF("r2d2.urdf", basePosition=[bx, by, 0.4], baseOrientation=b_orientation)

    agent_a = NavAgent("A", a_id, grid, ROBOT_A_GOAL, "spline", ROBOT_A_COLOR, True)
    agent_b = NavAgent("B", b_id, grid, ROBOT_B_GOAL, "spline", ROBOT_B_COLOR, True)

    def blockers_for(mover, other):
        return cell_block(other.current_cell(), BLOCK_RADIUS) if not other.arrived else set()

    last_blockers = {"a": blockers_for(agent_a, agent_b), "b": blockers_for(agent_b, agent_a)}
    agent_a.plan(blocked_cells=last_blockers["a"])
    agent_b.plan(blocked_cells=last_blockers["b"])

    hud = Hud(HUD_POSITION, True)
    begin_fast_forward()

    state = {"steps": 0, "next_replan": 0, "next_hud_step": 0, "next_progress_step": 0}

    def sim_step():
        """One replan-check + drive + tick cycle, shared by the
        detection phase below and the settle phase after it -- both need
        the exact same live sim, just for a different number of steps."""
        steps = state["steps"]
        if steps >= state["next_replan"]:
            state["next_replan"] = steps + int(REPLAN_PERIOD_S * SIM_HZ)
            if not agent_a.arrived:
                current_a = blockers_for(agent_a, agent_b)
                if agent_a.waiting or current_a != last_blockers["a"]:
                    agent_a.plan(blocked_cells=current_a, now_step=steps)
                    last_blockers["a"] = current_a
            if not agent_b.arrived:
                current_b = blockers_for(agent_b, agent_a)
                if agent_b.waiting or current_b != last_blockers["b"]:
                    agent_b.plan(blocked_cells=current_b, now_step=steps)
                    last_blockers["b"] = current_b

        pa = agent_a.robot.position()[:2]
        pb = agent_b.robot.position()[:2]
        dist = math.hypot(pa[0] - pb[0], pa[1] - pb[1])

        too_close = dist < SAFETY_STOP_RADIUS
        agent_a.drive_step(steps, force_stop=too_close)
        agent_b.drive_step(steps, force_stop=too_close)

        p.stepSimulation()
        state["steps"] += 1
        steps += 1

        if steps >= state["next_hud_step"]:
            state["next_hud_step"] = steps + int(HUD_UPDATE_PERIOD_S * SIM_HZ)
            agent_a.live_path.tick(steps)
            agent_b.live_path.tick(steps)
            agent_a.update_label()
            agent_b.update_label()

        if steps >= state["next_progress_step"]:
            state["next_progress_step"] = steps + SIM_HZ
            print(f"  corridor t={steps / SIM_HZ:.1f}s: dist={dist:.2f}m "
                  f"A.waiting={agent_a.waiting} B.waiting={agent_b.waiting}")

        return dist

    min_dist = float("inf")
    min_dist_step = 0
    # Both routes are ~10 cells to the crossing at DEFAULT_SPEED=20 m/s --
    # a few sim-seconds is generous; this is a safety cap, not the
    # expected stopping point (see the min_dist break condition below).
    max_steps = int(8 * SIM_HZ)

    while state["steps"] < max_steps:
        dist = sim_step()
        if dist < min_dist:
            min_dist = dist
            min_dist_step = state["steps"]
        # Stop shortly after closest approach, once both robots are moving
        # apart again -- captures the crossing itself, not a straight-line
        # approach or a robot that's already reached its goal.
        if state["steps"] > min_dist_step + int(0.3 * SIM_HZ) and dist > min_dist + 1.0:
            break

    # A replan can easily have fired within the last PATH_FLASH_SECONDS
    # (0.4s) -- e.g. exactly the crossing-triggered one that made this the
    # closest-approach moment in the first place -- which means both
    # paths could still be mid-flash (shared FLASH_COLOR, not each
    # robot's own steady color) right at the break point above, making it
    # hard to tell A's route from B's at a glance. Waiting out the flash
    # in real sim-time isn't an option -- at DEFAULT_SPEED=20 m/s even
    # 0.4s is 8m of further travel, well past the 3-wide street, which
    # was tried and did land both robots off in their own corners with
    # barely any path visible. Force each flash timer to expire right now
    # instead and re-tick once: same position, same lingering-old-path
    # comparison, just each active path already resolved to its real
    # steady color instead of caught mid-flash.
    agent_a.live_path.flash_until_step = state["steps"]
    agent_b.live_path.flash_until_step = state["steps"]
    agent_a.live_path.tick(state["steps"])
    agent_b.live_path.tick(state["steps"])

    hud.update([
        "multi-robot avoidance | A* | both robots replan around each other's cell",
        f"closest approach: {min_dist:.2f}m (safety stop at {SAFETY_STOP_RADIUS}m)",
        f"A: wp {agent_a.idx}/{len(agent_a.waypoints)} | B: wp {agent_b.idx}/{len(agent_b.waypoints)}",
    ])
    screenshot("multi_robot_corridor.png")
    p.disconnect()


def capture_cbs():
    """Four robots, one intersection (pybullet_cbs_main.py). CBS plans
    every robot's whole route once, offline, as a conflict-free set of
    time-indexed paths, then this drives all four in lockstep. Captured
    partway through the shortest robot's route, once every robot has
    reached that shared timestep -- with four compass-symmetric arms,
    that lands all four somewhere around the shared intersection, which
    is the whole point of this demo (CBS's joint plan, not any one
    robot's leg of it)."""
    connect(gui=True)
    grid = build_intersection_grid()
    build_obstacles(grid)

    agent_specs = build_agents(4)
    colors = {name: PALETTE[i % len(PALETTE)] for i, name in enumerate(agent_specs)}
    for name, (start, goal) in agent_specs.items():
        mark_cell(*start, color=colors[name])
        mark_goal_cell(*goal, color=colors[name])

    result = cbs(grid, agent_specs, max_expansions=5000)
    assert result is not None, "CBS failed to find a conflict-free solution for the poster scenario"

    agents = []
    for name, (start, goal) in agent_specs.items():
        x, y, _ = grid_to_world(*start)
        body_id = p.loadURDF("r2d2.urdf", basePosition=[x, y, 0.4])
        agents.append(CBSAgent(name, body_id, result[name], colors[name], True))

    hud = Hud(HUD_POSITION, True)
    target_step = min(len(a.waypoints) for a in agents) // 2
    begin_fast_forward()

    steps = 0
    next_hud_step = 0
    next_progress_step = 0
    max_steps = int(15 * SIM_HZ)

    while steps < max_steps and not all(a.step >= target_step for a in agents):
        ready = [a.tick() for a in agents]
        p.stepSimulation()
        steps += 1
        if all(ready):
            for a in agents:
                a.advance()

        if steps >= next_hud_step:
            next_hud_step = steps + int(HUD_UPDATE_PERIOD_S * SIM_HZ)
            for a in agents:
                a.update_label()
                a.live_path.tick(steps)

        if steps >= next_progress_step:
            next_progress_step = steps + SIM_HZ
            print(f"  cbs t={steps / SIM_HZ:.1f}s: " + " ".join(f"{a.name}={a.step}/{target_step}" for a in agents))

    hud.update([
        f"CBS | {len(agents)} robots | jointly planned, conflict-free, driven in lockstep",
        "each robot's entire route was solved once, offline -- no live replanning here",
        " | ".join(f"{a.name}: step {a.step}/{len(a.waypoints) - 1}" for a in agents),
    ])
    screenshot("cbs_intersection.png")
    p.disconnect()


def main():
    capture_corridor()
    capture_cbs()


if __name__ == "__main__":
    main()
