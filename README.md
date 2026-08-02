# autonomous-nav

A pygame grid-pathfinding visualizer comparing Dijkstra, A\*, RRT, RRT\*, and
D\* Lite, with weighted terrain, a simulated lidar sensor, dynamic
obstacles, automatic replanning, and a benchmark suite -- plus a PyBullet
port that drives real 3D robots along the *same* A* planning code, with
path smoothing, cost-map-aware routing, a raycast lidar sensor, two robots
coordinating through a shared corridor via a priority policy, and N robots
crossing an intersection at once coordinated by Conflict-Based Search.
Built as the first three legs of a longer autonomous-navigation project
that continues toward ROS 2 / Nav2 (see `WRITEUPS.md` for the full
narrative and the algorithm/interview explanations behind the code).

## What it does

- Click to draw obstacles on a 25x25 grid, place a start and goal, and run
  Dijkstra, A\*, or RRT to watch the search expand cell by cell (or, for
  RRT, watch its tree grow edge by edge).
- Handles the edge cases a naive search would crash on: start/goal on an
  obstacle, no path to the goal, start == goal.
- Toggle a weighted-terrain cost map: obstacles get an inflated-cost
  "buffer zone" (like Nav2's costmap), and A\*/Dijkstra route around them
  with clearance instead of hugging every wall.
- Toggle a simulated lidar sensor: the robot only knows about obstacles
  within its sensor radius, plans against that partial knowledge, and
  replans live as it discovers new obstacles -- real obstacles it hasn't
  sensed yet are drawn as free with a faint outline, so you can see the
  gap between what's true and what the robot knows.
- Drop moving obstacles that random-walk between free neighboring cells;
  send a robot down the computed path and watch it automatically replan
  when an obstacle blocks its route.
- Cycle A\*'s heuristic live (Manhattan / Euclidean / Octile / an
  intentionally inadmissible one) to see search effort and path quality
  change.
- Toggle 8-directional movement, or generate a fresh maze with recursive
  backtracking.
- `nav/rrt_star.py` extends RRT with rewiring toward asymptotic
  optimality, using the same `nav/kdtree.py` k-d tree RRT uses for its
  nearest-neighbor/within-radius queries. `nav/dstar_lite.py` is an
  incremental replanner (Koenig & Likhachev, 2002) that repairs a
  persistent search around a changed edge instead of resolving from
  scratch on every replan -- `nav/replan_benchmark.py` measures it against
  fresh A* on this project's own moving-obstacle and sensor-discovery
  replanning scenarios (see `benchmark_results/replan_writeup.md`).
- `pygame_app/scenarios/*.py` are standalone launchers that open the
  visualizer straight into a preset scene (e.g.
  `pygame_app/scenarios/scenario_maze.py`,
  `pygame_app/scenarios/scenario_costmap.py`) instead of needing manual
  clicks to reach it -- built on `pygame_app/scenario.py`'s
  `ScenarioConfig`.
- `nav/benchmark.py` runs all three original algorithms across 20 random
  grids and plots the comparison (see results below); `nav/scale_benchmark.py`
  reruns the comparison holding density fixed and scaling grid size instead
  (20x20 through 200x200) -- see "Scale benchmark" below.
- `pybullet_app/pybullet_main.py` ports the grid into a real 3D PyBullet
  world: the same `find_path`/A* code plans a route around a 3D obstacle
  block, a Catmull-Rom spline smooths it into something a robot base can
  actually follow, and a Husky robot drives it with velocity control (not
  teleportation). `--sensor` swaps that for a real raycast lidar
  (`pybullet.rayTestBatch`) that only knows what it's actually seen and
  replans as it explores. See "PyBullet 3D port" below.
- `pybullet_app/pybullet_multi_robot_main.py` runs two robots at once
  through a single-width corridor, forcing a head-on conflict, resolved
  with a priority policy (one robot always has right of way; the other
  detours around or waits for it) plus a hard safety-distance stop as a
  failsafe. See "Multiple robots" below.
- `pybullet_app/pybullet_cbs_main.py` runs N robots (default 4) through a
  shared 4-way intersection at once, coordinated by `nav/cbs.py`'s
  Conflict-Based Search -- every robot's route is planned jointly,
  offline, as a single conflict-free set of time-indexed paths, rather
  than replanning live like the two-robot corridor demo.

## How to run

```bash
python3 -m venv nav-env
source nav-env/bin/activate
pip install -r requirements.txt

python3 pygame_app/main.py                       # the pygame visualizer
python3 pygame_app/scenarios/scenario_maze.py    # ... or straight into a preset scenario
python3 -m nav.benchmark                         # regenerate benchmark_results/
python3 -m nav.scale_benchmark                   # regenerate the grid-size scaling results
python3 -m nav.replan_benchmark                  # regenerate the D* Lite vs A* replanning results
python3 pybullet_app/pybullet_main.py             # the PyBullet 3D demo (single robot)
python3 pybullet_app/pybullet_main.py --sensor    # ... with the raycast lidar instead of a perfect map
python3 pybullet_app/pybullet_multi_robot_main.py # two robots, forced corridor conflict
python3 pybullet_app/pybullet_cbs_main.py         # N robots through an intersection, coordinated by CBS
```

Pygame-only files live under `pygame_app/`, PyBullet-only files under
`pybullet_app/`; `nav/` holds the shared, framework-agnostic core (grid,
search algorithms, sensor model, benchmarks) both of them import from.
See "Repo layout" below. (Neither app directory is literally named
`pygame` or `pybullet` -- that would shadow the real installed libraries
of the same name for any script that adds the repo root to `sys.path`.)

If `pip install` fails building `pybullet` from source (no prebuilt wheel
for your platform/Python combo, common on very new macOS/Xcode Command
Line Tools), see "A build problem worth documenting" in `WRITEUPS.md` for
the exact fix -- it's a one-line patch to a bundled third-party header,
not an issue with this repo's code.

### Controls

| Key / click | Action |
|---|---|
| Left-click | Toggle obstacle |
| Shift + Left-click | Place/remove a moving obstacle |
| Right-click | Place start |
| Shift + Right-click | Place goal |
| `D` / `A` / `R` | Switch active algorithm (Dijkstra / A\* / RRT) |
| Space | Run the active algorithm |
| `W` | Start/stop the robot walking the current path |
| `H` | Cycle A\*'s heuristic |
| `X` | Toggle 8-directional movement |
| `M` | Generate a new maze |
| `K` | Toggle the weighted-terrain cost map |
| `S` | Toggle the lidar sensor model |
| `C` | Clear the grid |

## The algorithms, briefly

**Dijkstra** always expands the cheapest-so-far cell. It's guaranteed
optimal and doesn't need any notion of "closer to the goal" -- it just
explores outward in cost order, which is why it explores in ripples that
don't obviously point at the goal.

**A\*** expands the cell with the lowest `f = g + h`, where `g` is cost so
far (same as Dijkstra) and `h` is a heuristic estimate of the remaining
cost. As long as `h` never overestimates the true remaining cost
("admissible"), A\* is still guaranteed optimal, but explores dramatically
fewer cells because the heuristic steers it toward the goal instead of
outward in every direction.

**RRT** (Rapidly-exploring Random Tree) doesn't search a fixed neighbor
graph at all: it grows a tree from the start by repeatedly sampling a
random free point, stepping toward it from the tree's nearest node, and
keeping that step if it doesn't cross an obstacle, until a node lands near
the goal. It finds *a* path fast in open space and isn't restricted to
grid-aligned moves, but -- unlike Dijkstra/A\* here -- it gives up
optimality and determinism: the same grid produces a different tree, and a
different path, every run.

Full mechanics, the admissibility argument, the replanning policy, the
cost-map/sensor-model design, and the heuristic-breaking experiments are
written up in `WRITEUPS.md`.

## Dijkstra vs A\* vs RRT: benchmark results

20 random 25x25 grids, 10-35% obstacle density, all three algorithms run on
the identical grid/start/goal. Full data in `benchmark_results/results.csv`,
plot in `benchmark_results/comparison.png`, analysis in
`benchmark_results/writeup.md`.

| Trial | Density | Path len | Dijkstra cells | A\* cells | RRT nodes | RRT waypoints | RRT path len | Dijkstra ms | A\* ms | RRT ms |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 0.260 | 11 | 79 | 20 | 32 | 12 | 21.666 | 0.25 | 0.07 | 0.64 |
| 2 | 0.154 | 33 | 455 | 163 | 29 | 15 | 30.416 | 1.28 | 0.51 | 0.24 |
| 3 | 0.327 | 27 | 292 | 76 | 50 | 15 | 27.895 | 0.77 | 0.23 | 1.08 |
| 4 | 0.178 | 27 | 402 | 81 | 34 | 12 | 24.125 | 1.10 | 0.26 | 0.25 |
| 5 | 0.214 | 33 | 450 | 152 | 117 | 17 | 32.724 | 1.62 | 0.49 | 3.27 |
| 6 | 0.193 | 24 | 390 | 115 | 28 | 11 | 20.601 | 1.10 | 0.36 | 0.26 |
| 7 | 0.234 | 5 | 30 | 11 | 4 | 2 | 4.236 | 0.08 | 0.03 | 0.02 |
| 8 | 0.171 | 7 | 85 | 12 | 6 | 3 | 6.236 | 0.24 | 0.04 | 0.02 |
| 9 | 0.298 | 19 | 108 | 40 | 179 | 19 | 38.125 | 0.28 | 0.11 | 6.37 |
| 10 | 0.111 | 21 | 212 | 62 | 23 | 12 | 23.18 | 0.57 | 0.20 | 0.15 |
| 11 | 0.172 | 12 | 153 | 29 | 21 | 6 | 11.122 | 0.42 | 0.09 | 0.18 |
| 12 | 0.158 | 29 | 350 | 97 | 59 | 14 | 27.309 | 1.17 | 0.34 | 0.86 |
| 13 | 0.137 | 18 | 244 | 25 | 24 | 10 | 20.067 | 0.66 | 0.08 | 0.18 |
| 14 | 0.177 | 15 | 286 | 52 | 20 | 8 | 16.122 | 0.78 | 0.18 | 0.15 |
| 15 | 0.180 | 25 | 310 | 91 | 42 | 13 | 28.597 | 0.83 | 0.28 | 0.53 |
| 16 | 0.330 | 7 | 34 | 15 | 13 | 3 | 5.064 | 0.10 | 0.04 | 0.18 |
| 17 | 0.231 | 14 | 257 | 33 | 23 | 7 | 13.301 | 0.70 | 0.12 | 0.17 |
| 18 | 0.306 | 43 | 399 | 321 | 132 | 20 | 36.674 | 1.04 | 0.95 | 4.91 |
| 19 | 0.151 | 8 | 88 | 19 | 129 | 8 | 14.537 | 0.25 | 0.06 | 3.09 |
| 20 | 0.203 | 22 | 187 | 84 | 75 | 9 | 16.715 | 0.51 | 0.25 | 1.38 |

A\* explored 68.9% fewer cells than Dijkstra on average; the gap collapses
on trial 18 -- the longest, most obstacle-dense route in the set -- because
a forced detour makes Manhattan distance a much weaker predictor of true
travel cost. RRT found *a* path in all 20/20 trials (3,000-iteration
budget), but averaged 9.1% longer paths than the optimal Dijkstra/A* route
-- and its own path length swings from -27.7% to +100.7% run to run on
grids of similar difficulty, since tree growth depends on where random
samples happen to land rather than on the grid itself. Full breakdown,
including why RRT sometimes beats "optimal" (it isn't grid-constrained the
way Dijkstra/A* are here), in `benchmark_results/writeup.md`.

## PyBullet 3D port

`pybullet_app/pybullet_main.py` builds the identical `nav.grid.Grid` the
pygame visualizer uses -- a 25x25 grid with a 9x9 obstacle block sitting
directly between a start and goal placed on the same row, forcing a
detour around it -- projects it into a 3D PyBullet world (one static box
per obstacle cell, 1 grid cell = 1 meter), and plans across it with
`nav.algorithms.find_path`, completely unmodified from the pygame version. It then:

1. Plans the route **twice**: once with the cost map off (binary
   obstacles) and once with it on, and draws both as debug lines in the
   GUI (red vs blue) so you can see the clearance routing directly.
2. Smooths the chosen route -- corner-cutting first, then a Catmull-Rom
   spline (`pybullet_app/sim3d/smoothing.py`) -- since A*'s sharp
   90-degree grid waypoints aren't something a robot base can track
   without stopping to pivot at every one.
3. Drives a Husky robot along the result using **velocity control**
   (`pybullet.resetBaseVelocity`, not teleportation) -- the same
   turn-then-drive controller for every smoothing mode; the visible
   smoothness difference comes entirely from how closely spaced the
   waypoints it's given are, not from anything robot-specific.

```bash
python3 pybullet_app/pybullet_main.py                     # cost-map path, spline-smoothed (default)
python3 pybullet_app/pybullet_main.py --smooth raw        # raw A* waypoints, sharp turns, no smoothing
python3 pybullet_app/pybullet_main.py --smooth corner_cut # Chaikin corner-cutting instead of a spline
python3 pybullet_app/pybullet_main.py --no-cost-map       # binary obstacles only, no clearance routing
python3 pybullet_app/pybullet_main.py --headless          # DIRECT mode, no GUI window
```

`pybullet_app/scratch/pybullet_setup_test.py` is the standalone sanity
check this was built on top of: load `plane.urdf` + `r2d2.urdf`, let it
settle, confirm the GUI window actually opens.

Full writeup -- the grid-to-world coordinate mapping, why velocity
control instead of teleporting, the corner-cutting/spline math, and why
this specific obstacle layout (not a maze) was needed to make the
cost-map routing visible -- is in `WRITEUPS.md`.

## Lidar sensor in 3D

`--sensor` swaps the perfect-map planning above for
`pybullet_app/sim3d/lidar.py`'s `Lidar3D`: a real raycast sensor
(`pybullet.rayTestBatch`, 48 rays in a circle) instead of pygame's 2D
radius circle -- a wall can block the view of what's behind it now,
which a radius circle can't represent. The robot plans against
`nav.sensor.KnownGrid` (the *exact* class the pygame sensor mode uses,
unchanged) built from whatever the lidar has actually hit, rescans every
0.3s as it moves, and replans the instant it discovers something new:

```bash
python3 pybullet_app/pybullet_main.py --sensor                # lidar-limited knowledge, replans on discovery
python3 pybullet_app/pybullet_main.py --sensor --smooth raw   # same, but undo the path smoothing too
```

`pybullet_app/scratch/pybullet_lidar_test.py` is the standalone sanity
check: one scan from a fixed position against a hardcoded wall, confirming the raycasts
actually find it. Building the real version surfaced a genuine bug worth
knowing about: a ray hits an obstacle's surface at an exact cell boundary
(e.g. `x=8.5` for a 1-meter cell), which is ambiguous to round to a grid
cell and can even round to the *free* cell in front of the wall instead
of the wall itself. Fixed by nudging the hit point slightly further along
the ray, past the surface, before converting it to a cell -- see
`WRITEUPS.md` for the full failure mode (it briefly made the robot
"discover" that its own current cell was an obstacle).

## Multiple robots

`pybullet_app/pybullet_multi_robot_main.py`: four buildings, one per quadrant, leave a
3-cell-wide "plus" of open street down the middle of the grid -- a real
cross-street intersection. Robot A drives the north-south street start to
finish; robot B drives the east-west street start to finish. All four
points (A's start/goal, B's start/goal) are different cells -- an earlier
version of this demo had goal(A) == start(B) by construction (a
"swap sides through one corridor" layout) and that coincidence turned out
to cause its own class of bugs, so the two robots' paths now only ever
meet at the crossing in the middle, not at either one's start or goal.
The two routes are the same length, so left alone they'd reach the
crossing at close to the same moment -- both robots' start *and* goal
cells are marked (a disc for start, a diamond for goal, colored and
labeled per robot) so it's clear at a glance where each is headed. The
coordination policy:

- **Both robots replan, symmetrically.** Every `REPLAN_PERIOD_S = 0.05` s,
  each one runs A* against the real grid *plus* a block placed around the
  *other's* current cell (`cell_block`) -- the same technique the pygame
  `MovingObstacle` replanning logic used, just with a robot standing in
  for the moving obstacle on both sides at once. Detecting the other robot
  nearby produces a genuinely different, grid-verified route -- using the
  street's spare width to slide into an adjacent lane -- not a steering
  offset layered on top of an unrelated path. A replan is only actually
  issued when the blocked cells changed or the last attempt found nothing,
  to avoid interrupting a perfectly good drive already in progress every
  single tick.
- **If a route genuinely isn't there, the blocked robot holds position**
  (`waiting = True`) and retries on the next replan tick, rather than
  crashing on `None` or driving into a wall.
- **A hard safety-distance stop is layered on top as an absolute last
  resort, not the primary mechanism**: if the two robots' actual distance
  ever closes below 0.55m (r2d2's own footprint radius is about 0.17m, so
  contact needs centers within roughly 0.34m), both are forced to stop
  that frame regardless of what their plans say -- checked
  unconditionally, every step, with no exceptions. With replanning doing
  its job, this rarely fires in practice.

An earlier version of this avoided collisions with a per-frame *steering*
nudge on top of a fixed plan, rather than replanning -- it visibly
narrowed the crossing distance, but didn't reliably prevent actual
contact, and nudging a robot's aim point risks steering it into a wall
its plan never accounted for (real, and it happened). Replanning doesn't
have that problem: a shifted route is A*-verified against the real grid
every time, for both robots, so it can never point either one through a
wall. `pybullet_app/scratch/pybullet_multi_robot_test.py` is the standalone sanity
check: two robots on non-conflicting paths, no avoidance needed,
confirming the basics work before adding the forced conflict.

Both robots are also spawned already facing their first direction of
travel (`baseOrientation` computed from each route's initial heading),
not the URDF's default -- without that, one robot always happened to
already face the right way while the other needed a real turn first,
giving it a head start that was enough on its own to make the two miss
each other by 7+ meters despite their routes crossing at the exact
center of the grid on paper. With spawn headings synchronized, both
robots genuinely detour around each other at the crossing -- confirmed by
watching the replanned waypoints themselves shift into the next lane over
and back -- landing a comfortable ~1.6m apart at closest, consistent
across repeated runs in every smoothing mode (there's no randomness
anywhere in this simulation, so identical inputs reliably reproduce the
same outcome).

Several real bugs turned up building this -- a permanently-parked robot
blocking the other's goal, a replan loop that looked like a robot was
frozen solid, a safety check with a loophole that let an actual collision
through, a steering-based avoidance layer that made a robot orbit forever
or steered it into a wall (and, even once those were fixed, still didn't
reliably prevent contact -- which is why it was replaced with replanning
entirely), a slow turn-in-place controller that made one robot sit
motionless for half a second before a 90-degree turn even started moving
it, and a second, smaller version of that same head-start problem hiding
underneath it -- see "Multiple robots" in `WRITEUPS.md` for the full
account of each one and its fix.

```bash
python3 pybullet_app/pybullet_multi_robot_main.py --headless --max-seconds 60
```

## Scale benchmark

`nav/scale_benchmark.py` holds obstacle density fixed (20%) and
varies grid size (20x20, 50x50, 100x100, 200x200; 8 trials each) instead
of the other way around -- the direct answer to "how does this scale to
a bigger grid?" Full data in `benchmark_results/scale_results.csv`, plot
in `benchmark_results/scale_comparison.png`, analysis in
`benchmark_results/scale_writeup.md`.

| Size | Cells | Dijkstra avg | A\* avg | RRT avg | RRT found path |
|---:|---:|---:|---:|---:|---:|
| 20x20 | 400 | 0.439ms | 0.211ms | 0.431ms | 8/8 |
| 50x50 | 2,500 | 2.610ms | 0.838ms | 3.964ms | 8/8 |
| 100x100 | 10,000 | 9.588ms | 1.225ms | 118.834ms | 7/8 |
| 200x200 | 40,000 | 61.985ms | 10.332ms | 267.127ms | 5/8 |

A 100x increase in cells grows Dijkstra's runtime 141x but A\*'s only
49x -- A\*'s search effort actually shrinks as a *fraction* of the grid
as it grows (15.4% of the grid at 20x20, 7.0% at 200x200), since a
heuristic-guided search tracks start-to-goal distance far more than
total grid area. RRT is the outlier: 620x slower over the same
increase, and its completeness degrades with scale (8/8 -> 8/8 -> 7/8 ->
5/8) even with its step size and iteration budget both scaled up for
fairness, because its unindexed nearest-neighbor search over a growing
tree list is the real bottleneck, not the tuning. Full breakdown in
`benchmark_results/scale_writeup.md`.

## Repo layout

Pygame-only and PyBullet-only code live in their own top-level
directories, separate from each other and from the shared `nav/` core.
(Neither is named literally `pygame/` or `pybullet/` -- a directory with
that exact name, sitting on `sys.path`, would shadow the real installed
library of the same name for `import pygame`/`import pybullet` anywhere
in the project.)

```
nav/               Framework-agnostic core: both pygame_app/ and pybullet_app/ import from this
  grid.py          Grid model: cells, obstacles, start/goal, neighbors, cost map, size param
  algorithms.py    Dijkstra, A*, edge-case handling, path cost
  rrt.py           RRT (Rapidly-exploring Random Tree)
  rrt_star.py      RRT* -- RRT plus rewiring toward asymptotic optimality
  kdtree.py        k-d tree over (row, col) points -- RRT/RRT*'s nearest/within-radius queries
  dstar_lite.py    D* Lite -- incremental replanner that repairs a persistent search
  cbs.py           Conflict-Based Search -- joint conflict-free planning for 3+ agents
  sensor.py        Simulated lidar (2D radius) + the KnownGrid the robot plans against
  heuristics.py    Manhattan / Euclidean / Chebyshev / Octile / scaled
  obstacles.py     Moving obstacles + the replanning policy
  maze.py          Recursive-backtracking maze generator
  scenario_helpers.py Shared obstacle/terrain-scattering helpers used by both
                       pygame_app/scenarios/*.py and pybullet_app/pybullet_main.py
  benchmark.py        20-trial Dijkstra vs A* vs RRT benchmark -> CSV + plot
  scale_benchmark.py  20x20 - 200x200 grid-size scaling benchmark -> CSV + plot
  replan_benchmark.py D* Lite vs fresh A* on moving-obstacle/sensor-discovery replanning -> CSV + plot
  scratch/         Standalone throwaway scripts used to prove each piece
                    works before it was wired into the visualizer/pybullet_main
                    (framework-agnostic tests only -- pygame/PyBullet-specific
                    ones live under pygame_app/ and pybullet_app/ instead)

pygame_app/        Everything that touches pygame
  main.py            Entry point: opens the interactive visualizer
  visualizer.py      The pygame app
  scenario.py        ScenarioConfig -- preset state for scenarios/*.py
  scenarios/         Standalone launchers that open the visualizer into a preset scene
                      (maze, cost map, bottleneck, noisy sensor, step replay, ...)

pybullet_app/      Everything that touches PyBullet
  pybullet_main.py             Single-robot PyBullet demo (plan -> smooth -> drive, + --sensor/--terrain)
  pybullet_multi_robot_main.py Two-robot corridor-conflict + priority/deadlock demo
  pybullet_cbs_main.py         N-robot intersection demo, coordinated by CBS
  sim3d/           PyBullet world-building, path smoothing, robot control
    coords.py        Grid-cell <-> world-meter conversion
    world.py         Ground plane, obstacle bodies, debug-line path drawing
    smoothing.py     Collinear simplification, Chaikin corner-cutting, Catmull-Rom spline
    robot.py         Robot: drives a body (Husky or r2d2) toward waypoints via velocity control
    lidar.py         Lidar3D: real raycast sensor (pybullet.rayTestBatch)
    hud.py           World-space debug-text HUD and per-robot follow labels
  scratch/         PyBullet-specific standalone sanity checks (setup, raycast
                    lidar, lidar noise, multi-robot) -- same role as nav/scratch/,
                    just for the pieces that need a real PyBullet connection

benchmark_results/  Generated CSVs, plots, and writeups from all three benchmarks
WRITEUPS.md         Algorithm explanations, replanning policy, cost map,
                     sensor model, PyBullet port, 3D lidar, multi-robot
                     coordination, and the heuristic experiments' findings
```
