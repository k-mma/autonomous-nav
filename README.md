# autonomous-nav

A pygame grid-pathfinding visualizer comparing Dijkstra, A\*, and RRT, with
weighted terrain, a simulated lidar sensor, dynamic obstacles, automatic
replanning, and a benchmark suite -- plus a PyBullet port that drives a
real 3D robot along the *same* A* planning code, with path smoothing and
cost-map-aware routing. Built as the first two legs of a longer
autonomous-navigation project that continues toward ROS 2 / Nav2 (see
`WRITEUPS.md` for the full narrative and the algorithm/interview
explanations behind the code).

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
- Drop moving obstacles that bounce between two cells; send a robot down
  the computed path and watch it automatically replan when an obstacle
  blocks its route.
- Cycle A\*'s heuristic live (Manhattan / Euclidean / Octile / an
  intentionally inadmissible one) to see search effort and path quality
  change.
- Toggle 8-directional movement, or generate a fresh maze with recursive
  backtracking.
- `nav/benchmark.py` runs all three algorithms across 20 random grids and
  plots the comparison (see results below).
- `pybullet_main.py` ports the grid into a real 3D PyBullet world: the
  same `find_path`/A* code plans a route around a 3D obstacle block, a
  Catmull-Rom spline smooths it into something a robot base can actually
  follow, and an r2d2 robot drives it with velocity control (not
  teleportation). See "PyBullet 3D port" below.

## How to run

```bash
python3 -m venv nav-env
source nav-env/bin/activate
pip install -r requirements.txt

python3 main.py              # the pygame visualizer
python3 -m nav.benchmark     # regenerate benchmark_results/
python3 pybullet_main.py     # the PyBullet 3D demo
```

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

`pybullet_main.py` builds the identical `nav.grid.Grid` the pygame
visualizer uses -- a 25x25 grid with a 9x9 obstacle block sitting
directly between a start and goal placed on the same row, forcing a
detour around it -- projects it into a 3D PyBullet world (one static box
per obstacle cell, 1 grid cell = 1 meter), and plans across it with
`nav.algorithms.find_path`, completely unmodified from Week 1-3. It then:

1. Plans the route **twice**: once with the cost map off (binary
   obstacles) and once with it on, and draws both as debug lines in the
   GUI (red vs blue) so you can see the clearance routing directly.
2. Smooths the chosen route -- corner-cutting first, then a Catmull-Rom
   spline (`nav/sim3d/smoothing.py`) -- since A*'s sharp 90-degree grid
   waypoints aren't something a robot base can track without stopping to
   pivot at every one.
3. Drives an r2d2 robot along the result using **velocity control**
   (`pybullet.resetBaseVelocity`, not teleportation) -- the same
   turn-then-drive controller for every smoothing mode; the visible
   smoothness difference comes entirely from how closely spaced the
   waypoints it's given are, not from anything robot-specific.

```bash
python3 pybullet_main.py                     # cost-map path, spline-smoothed (default)
python3 pybullet_main.py --smooth raw        # Days 17-18: raw A* waypoints, sharp turns
python3 pybullet_main.py --smooth corner_cut # Chaikin corner-cutting instead of a spline
python3 pybullet_main.py --no-cost-map       # binary obstacles only, no clearance routing
python3 pybullet_main.py --headless          # DIRECT mode, no GUI window
```

`nav/scratch/pybullet_setup_test.py` is the Day 16 mini-MVP this was
built on top of: load `plane.urdf` + `r2d2.urdf`, let it settle, confirm
the GUI window actually opens.

Full writeup -- the grid-to-world coordinate mapping, why velocity
control instead of teleporting, the corner-cutting/spline math, and why
this specific obstacle layout (not a maze) was needed to make the
cost-map routing visible -- is in `WRITEUPS.md`.

## Repo layout

```
nav/
  grid.py          Grid model: cells, obstacles, start/goal, neighbors, cost map
  algorithms.py    Dijkstra, A*, edge-case handling, path cost
  rrt.py           RRT (Rapidly-exploring Random Tree)
  sensor.py        Simulated lidar + the KnownGrid the robot plans against
  heuristics.py    Manhattan / Euclidean / Chebyshev / Octile / scaled
  obstacles.py     Moving obstacles + the replanning policy
  maze.py          Recursive-backtracking maze generator
  visualizer.py    The pygame app
  benchmark.py     20-trial Dijkstra vs A* vs RRT benchmark -> CSV + plot
  sim3d/           PyBullet world-building, path smoothing, robot control
    coords.py        Grid-cell <-> world-meter conversion
    world.py         Ground plane, obstacle bodies, debug-line path drawing
    smoothing.py     Collinear simplification, Chaikin corner-cutting, Catmull-Rom spline
    robot.py         Robot: drives an r2d2 body toward waypoints via velocity control
  scratch/         Standalone throwaway scripts used to prove each piece
                    works before it was wired into the visualizer/pybullet_main
benchmark_results/  Generated CSV, plot, and writeup from benchmark.py
pybullet_main.py    The PyBullet 3D demo (plan -> smooth -> drive)
WRITEUPS.md         Algorithm explanations, replanning policy, cost map,
                     sensor model, PyBullet port, and the heuristic
                     experiments' findings
```
