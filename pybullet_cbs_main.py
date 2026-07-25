"""
N robots crossing the same 4-way intersection at once, coordinated by
Conflict-Based Search (nav/cbs.py) instead of pybullet_multi_robot_main.py's
two-robot "replan around the other's current cell" policy, which has no
clean way to generalize past exactly two agents (see nav/cbs.py's module
docstring for why). CBS plans every robot's full route *once*, offline,
as a jointly conflict-free set of time-indexed paths -- there's no live
replanning loop here at all, which is the actual architectural
difference from pybullet_multi_robot_main.py: that file's whole design
is built around continuous re-planning as robots move; this one commits
to a single upfront plan and then just has to *execute* it faithfully.

Executing it faithfully is the one real wrinkle 3D driving adds that the
abstract algorithm doesn't have to think about. CBS's conflict-free
guarantee is a claim about *discrete timesteps*: "no two agents occupy
the same cell at the same timestep, and no two agents swap cells between
consecutive timesteps." A continuous physics simulation has no built-in
notion of "timestep" at all -- if every robot just drove its own
waypoint list independently at whatever speed it individually achieves,
their real-world timing would drift apart from the plan's discrete
clock, and CBS's guarantee stops applying (a robot that's supposed to be
one cell behind schedule because it's a hair slower than the others is
now, from the guarantee's point of view, at the wrong timestep). This
file drives all agents in **lockstep**: every robot advances to its next
per-timestep waypoint only once *every* robot has reached its current
one, so the whole fleet's simulated position always corresponds to
exactly one shared CBS timestep at a time -- reconstructing the discrete
clock the guarantee actually depends on. Paths are driven raw (no
corner-cutting/spline smoothing, unlike pybullet_main.py/
pybullet_multi_robot_main.py) for the same reason: smoothing shifts
where along the path a robot actually is at a given moment, which would
undermine the same synchronization.

    python3 pybullet_cbs_main.py                  # 4 robots, one per compass arm
    python3 pybullet_cbs_main.py --robots 6        # 6 robots, doubled up in adjacent lanes
    python3 pybullet_cbs_main.py --headless --max-seconds 60
"""
import argparse
import time

import pybullet as p

from nav.cbs import cbs, solution_cost
from nav.sim3d.coords import grid_to_world
from nav.sim3d.hud import Hud, FollowLabel
from nav.sim3d.robot import Robot, DEFAULT_SPEED
from nav.sim3d.world import connect, build_obstacles, mark_cell, mark_goal_cell, LivePath
from pybullet_multi_robot_main import build_intersection_grid, CENTER, STREET_HALF_WIDTH

SIM_HZ = 240
HUD_UPDATE_PERIOD_S = 0.1
HUD_POSITION = (2, 2, 9)

ARM_DIRECTIONS = ["N", "S", "W", "E"]  # starts at that compass arm, heads to the opposite one
PALETTE = [
    (0.10, 0.70, 0.90, 1.0),
    (0.95, 0.55, 0.10, 1.0),
    (0.45, 0.80, 0.20, 1.0),
    (0.85, 0.20, 0.75, 1.0),
    (0.95, 0.85, 0.15, 1.0),
    (0.55, 0.35, 0.95, 1.0),
    (0.20, 0.75, 0.60, 1.0),
    (0.80, 0.15, 0.15, 1.0),
]


def build_agents(num_robots):
    """Assign `num_robots` start/goal pairs across the intersection's four
    compass arms, cycling to an adjacent lane (the street is
    2*STREET_HALF_WIDTH + 1 = 3 cells wide) once more than 4 robots are
    asked for. Every agent's start and goal are independent cells -- the
    only thing CBS is told about them is which cell each one starts and
    ends at; it has no notion of "arm" or "lane" at all, that's purely
    this scenario-building step's structure."""
    lanes = [CENTER - STREET_HALF_WIDTH, CENTER, CENTER + STREET_HALF_WIDTH]
    agents = {}
    for i in range(num_robots):
        direction = ARM_DIRECTIONS[i % 4]
        lane = lanes[(i // 4) % len(lanes)]
        name = f"{direction}{i // 4 + 1}"
        if direction == "N":
            agents[name] = ((2, lane), (22, lane))
        elif direction == "S":
            agents[name] = ((22, lane), (2, lane))
        elif direction == "W":
            agents[name] = ((lane, 2), (lane, 22))
        else:
            agents[name] = ((lane, 22), (lane, 2))
    return agents


class CBSAgent:
    """One robot's execution of its (already fully planned, already
    conflict-free) CBS path. Purely mechanical -- no planning happens
    here at all, just driving raw grid-cell waypoints in lockstep with
    every other agent (see this module's docstring for why)."""

    def __init__(self, name, body_id, path_cells, color, gui):
        self.name = name
        self.robot = Robot(body_id)
        self.waypoints = [grid_to_world(r, c)[:2] for r, c in path_cells]
        self.color = color
        self.gui = gui
        self.step = 0
        self.label = FollowLabel(name, gui, color=color)
        # CBS never replans live (see the module docstring), so this path
        # never changes after being set once -- LivePath is still worth
        # using for its flash-in reveal (see nav/sim3d/world.py), so the
        # route actually announces itself onscreen instead of just
        # appearing, same as every other demo's paths now do.
        self.live_path = LivePath(color[:3], gui, SIM_HZ, z=0.04)
        self.live_path.set_path(self.waypoints, 0)

    @property
    def arrived(self):
        return self.step >= len(self.waypoints) - 1

    def _current_target(self):
        idx = min(self.step + 1, len(self.waypoints) - 1)
        return self.waypoints[idx]

    def tick(self):
        """Drive one physics step toward this tick's target. Returns
        True once this agent has reached it (i.e. it's ready for the
        whole fleet's shared timestep to advance), False otherwise."""
        if self.arrived:
            self.robot.stop()
            return True
        return self.robot.drive_toward(self._current_target())

    def advance(self):
        if not self.arrived:
            self.step += 1

    def update_label(self):
        self.label.update(self.robot.position())


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", type=int, default=4,
                         help="number of robots, cycling through the intersection's 4 compass "
                              "arms and (past 4) its 3 street lanes (default: 4, one per arm)")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=90.0)
    parser.add_argument("--max-expansions", type=int, default=5000,
                         help="CBS constraint-tree node budget -- a heavily contested scenario "
                              "(many robots, few lanes) can exceed this and fail to find a "
                              "solution; that's an expected property of CBS's worst-case "
                              "complexity, not a bug (see WRITEUPS.md)")
    return parser.parse_args()


def main():
    args = parse_args()
    gui = not args.headless
    connect(gui=gui)

    grid = build_intersection_grid()
    build_obstacles(grid)

    agent_specs = build_agents(args.robots)
    colors = {name: PALETTE[i % len(PALETTE)] for i, name in enumerate(agent_specs)}

    for name, (start, goal) in agent_specs.items():
        color = colors[name]
        mark_cell(*start, color=color)
        mark_goal_cell(*goal, color=color)

    print(f"solving CBS for {len(agent_specs)} robots...")
    t0 = time.perf_counter()
    result = cbs(grid, agent_specs, max_expansions=args.max_expansions)
    solve_ms = (time.perf_counter() - t0) * 1000
    if result is None:
        print(f"CBS failed to find a conflict-free solution within {args.max_expansions} "
              f"constraint-tree expansions ({solve_ms:.1f}ms) -- try fewer robots or a higher "
              f"--max-expansions.")
        p.disconnect()
        return
    print(f"CBS solved in {solve_ms:.1f}ms, total cost {solution_cost(result)} "
          f"(sum of every robot's path length)")
    for name in agent_specs:
        print(f"  {name}: {len(result[name]) - 1} steps")

    agents = []
    for name, (start, goal) in agent_specs.items():
        x, y, _ = grid_to_world(*start)
        body_id = p.loadURDF("r2d2.urdf", basePosition=[x, y, 0.4])
        agents.append(CBSAgent(name, body_id, result[name], colors[name], gui))

    hud = Hud(HUD_POSITION, gui)
    speed_param = p.addUserDebugParameter("robot speed", 5.0, 40.0, DEFAULT_SPEED) if gui else None

    steps = 0
    max_steps = int(args.max_seconds * SIM_HZ)
    next_hud_step = 0

    while steps < max_steps and not all(a.arrived for a in agents):
        ready = [a.tick() for a in agents]
        p.stepSimulation()
        if not args.headless:
            time.sleep(1 / SIM_HZ)
        steps += 1

        if all(ready):
            for a in agents:
                a.advance()

        if steps >= next_hud_step:
            next_hud_step = steps + int(HUD_UPDATE_PERIOD_S * SIM_HZ)
            if speed_param is not None:
                shared_speed = p.readUserDebugParameter(speed_param)
                for a in agents:
                    a.robot.speed = shared_speed
            for a in agents:
                a.update_label()
                a.live_path.tick(steps)

            arrived_count = sum(1 for a in agents if a.arrived)
            lines = [f"CBS | {len(agents)} robots | t={steps / SIM_HZ:.1f}s",
                     f"arrived: {arrived_count}/{len(agents)}"]
            lines.extend(
                f"{a.name}: {'arrived' if a.arrived else f'step {a.step}/{len(a.waypoints) - 1}'}"
                for a in agents
            )
            hud.update(lines)

    for a in agents:
        print(f"{a.name} arrived: {a.arrived}")
    if not all(a.arrived for a in agents):
        print(f"stopped after {args.max_seconds}s safety cap")

    if not args.headless:
        input("Press Enter to close...")
    p.disconnect()


if __name__ == "__main__":
    main()
