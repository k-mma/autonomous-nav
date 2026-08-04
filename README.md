# autonomous-nav

**Research question:** which sensing investment actually buys reliability
in a 30-second FTC (*FIRST* Tech Challenge) autonomous period, and at
what level of field/reality deviation does each one become necessary?

![Sensor suite success rate vs. deviation, one row per deviation type, with 95% bootstrap CI bands](benchmark_results/ftc_suite_comparison.png)

**The finding:** FullSuite (distance sensors + AprilTag + odometry, $230)
has the highest raw success rate, but **AprilTag alone ($40) delivers
more than double FullSuite's success-rate gain per dollar spent** over
the free dead-reckoning baseline -- and the most useful result is
negative: DistanceSensorSuite collides in roughly half its trials even
at *zero* field deviation, because 3 narrow ToF cones cover only ~75° of
the 360° around the robot, not because of anything the field did. Which
deviation type actually dominates depends on what a suite fixes -- pose
error vs. obstacle error are different failure modes with different
fixes, not one generic "uncertainty" axis. Full numbers in "FTC
sensor-suite study: results" below and `benchmark_results/
ftc_suite_writeup.md`; `ftc/suite_benchmark.py` is the study that
produced them.

An FTC team gets roughly 10 matches a season -- one noisy, unrepeatable
trial each, no ground truth to compare against, and no control over how
much the real field/robot deviates from what the autonomous routine
assumed. That's not enough data to answer the research question
empirically, no matter how many matches a team plays. A testbed
calibrated against real field/robot deviation lets them run the
hundreds of controlled, repeatable trials the physical process could
never supply. That's this project's actual thesis: **simulation here is
the only viable instrument for this question, not a stand-in for
hardware you'd use if only you had more of it.** See "Research
question" below for the full framing.

Everything else in this repo -- the pathfinding planners, the
weighted-terrain cost map, the sensor/occupancy/belief-planning models,
`nav/uncertainty_benchmark.py`'s open-loop-vs-reactive-vs-belief study
-- exists because it's the belief-planning machinery the FTC study is
built on top of. It grew as a tour of pathfinding algorithms first (see
"Planners the testbed swaps between" below and `WRITEUPS.md` for that
narrative, which continues toward ROS 2 / Nav2); `ftc/` is where that
machinery gets pointed at one specific, answerable question.

## Research question

FTC autonomous is a 30-second dash: drive from a known start to a
scoring position using a pre-programmed route, with no driver input.
Every team already senses *something* -- at minimum, motor encoders --
and can buy more: odometry pods, distance sensors, AprilTag-based
vision, or all three together. Each option costs real money and real
integration time a team could spend elsewhere. Nothing about which one
is worth it is obvious from specs alone, because the answer depends on
*which kind* of deviation actually shows up on a given field: a robot
that starts a few inches off its mark needs pose correction, not
obstacle sensing; a field with elements that don't quite match the
CAD, or an opponent robot parked somewhere unplanned, needs the
opposite. `ftc/suite_benchmark.py` sweeps 5 sensor suites
(`ftc/sensors.py`) against 3 independently-scaled deviation types
(`nav/field_variance.py`, Phase 2's ablation) at 11 deviation levels, on
a real 30-second time budget (`ftc/match.py`) -- the study a real season
can't run, because it would need hundreds of matches to get the same
statistical power this testbed gets in about a minute.

## nav/ vs ftc/: a deliberate boundary

`nav/` stays domain-neutral on purpose: `nav/policies.py`,
`nav/occupancy.py`, `nav/field_variance.py`, `nav/algorithms.py`,
`nav/grid.py`, and everything else under `nav/` read as a general
belief-planning library, with no FTC-specific identifiers anywhere in
them. `ftc/` is a separate top-level package that holds the domain
specialization -- FTC names are correct and expected there, because
that's literally what the code models (an 18in robot, a 144in field, a
30-second clock, AprilTags). Keeping the boundary at the package level
rather than scattering FTC-specific `if` branches through `nav/` is
what lets `nav/`'s machinery keep being reusable for a different robot,
a different field, or a different competition entirely -- the FTC study
is one specific consumer of it, not what it's *for*.

## Planners the testbed swaps between

This project started as a tour of pathfinding algorithms, and that tour
is still here -- but it's infrastructure the FTC study runs on top of,
not the point. Every planner below is a backend `nav/algorithms.py`'s
`find_path` can swap in; `ftc/`'s policies and sensor suites all use A*
exclusively (see "Research question" above), because A* is the correct
tool at this grid scale (25x25-ish) and RRT/RRT* are not. It's not
(only) a speed argument -- `nav/kdtree.py`'s spatial index means RRT is
no longer even the slow one (see "Scale benchmark" below) -- it's a
completeness-class argument: Dijkstra/A* are resolution-complete on a
grid (guaranteed to find a path at the grid's resolution if one
exists), RRT is only probabilistically complete (guaranteed as sample
count -> infinity, not at any fixed budget), and `nav/scale_benchmark.py`
shows that gap showing up as real, measured incompleteness (8/8 -> 5/8)
as grid size grows even with a fast nearest-neighbor search. RRT/RRT*
stay in this repo as explicit baselines -- useful for seeing *why* an
exhaustive heuristic-guided search beats a sampling-based one here --
not as something the FTC study, or a real autonomous routine built on
this code, should actually reach for at this scale.

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
  `ScenarioConfig`. `pygame_app/scenarios/scenario_uncertainty.py` is the
  one exception (its own standalone pygame loop, not built on
  `ScenarioConfig`): a live, watchable replay of nav/uncertainty_
  benchmark.py's headline open-loop/reactive/belief comparison, ground
  truth on the left and the policy's current belief on the right.
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
python3 -m nav.uncertainty_benchmark             # regenerate the open-loop/reactive/belief uncertainty study
python3 -m ftc.suite_benchmark                   # regenerate the FTC sensor-suite study -- the headline result
python3 -m ftc.robustness                        # tipping-point sweep on the headline study's estimated constants
python3 -m ftc.layout_benchmark                  # does the best-value suite change on a different field layout?
python3 -m ftc.budget_benchmark                  # sweep AUTONOMOUS_PERIOD_S -- when does the budget start to bind?
python3 -m ftc.opponent_benchmark                # static vs. moving opponent -- does it change which suite wins?
python3 -m ftc.calibration                       # fit variance_level from real CSVs (or the synthetic placeholder)
python3 -m ftc.recommend                         # decision CLI: suite ranking + predicted success/time/cost
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
different path, every run. **This is the wrong tool at this project's
grid scale** (see "Scale benchmark" below and "Planners the testbed
swaps between" above) -- kept as an explicit, measured baseline showing
why, not a planner anything downstream (`ftc/`, the replanning policies)
actually uses.

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

| Size | Cells | Dijkstra avg | A\* avg | RRT avg (k-d tree) | RRT found path |
|---:|---:|---:|---:|---:|---:|
| 20x20 | 400 | 0.465ms | 0.227ms | 0.443ms | 8/8 |
| 50x50 | 2,500 | 2.679ms | 0.762ms | 1.520ms | 8/8 |
| 100x100 | 10,000 | 9.732ms | 1.156ms | 15.162ms | 7/8 |
| 200x200 | 40,000 | 56.511ms | 9.968ms | 58.548ms | 5/8 |

A 100x increase in cells grows Dijkstra's runtime ~120x but A\*'s only
~44x -- A\*'s search effort actually shrinks as a *fraction* of the grid
as it grows (15.4% of the grid at 20x20, 7.0% at 200x200), since a
heuristic-guided search tracks start-to-goal distance far more than
total grid area. RRT's *speed* used to be the outlier here (620x slower
over the same increase with a linear-scan nearest-neighbor search) but
`nav/kdtree.py`'s spatial index closed most of that gap -- RRT is now
faster than Dijkstra at every size above. Its *completeness* is a
separate story the k-d tree doesn't touch: 8/8 -> 8/8 -> 7/8 -> 5/8 even
with step size and iteration budget both scaled up for fairness, because
Dijkstra/A\* are **resolution-complete** (guaranteed to find a path at
the grid's resolution if one exists) while RRT is only
**probabilistically complete** (guaranteed only as sample count ->
infinity) -- a fixed iteration budget against a growing space is exactly
the situation that guarantee doesn't cover. Full breakdown, including
the before/after k-d tree numbers, in `benchmark_results/
scale_writeup.md`.

## FTC sensor-suite study: results

The answer to this project's research question, from `ftc/suite_benchmark.py`
(25 trials x 5 suites x 3 independently-scaled deviation types x 11
deviation levels, on the "cluttered" field layout). Full breakdown,
including the per-deviation-type charts and the reliability-per-dollar
chart, in `benchmark_results/ftc_suite_writeup.md`,
`ftc_suite_comparison.png`, and `ftc_reliability_per_dollar.png`.

| Suite | Cost | Overall success rate (variance_level >= 0.3) |
|---|---:|---:|
| Full suite | $230 | 56% |
| Odometry pods | $100 | 45% |
| AprilTag | $40 | 35% |
| Distance sensors | $90 | 21% |
| Dead reckoning (baseline) | $0 | 19% |

(AprilTag's correction model accounts for range- and viewing-angle-
dependent degradation, not a flat correction whenever a tag is merely
in view -- see `ftc/sensors.py`'s `AprilTagSuite.tag_correction` and
`ftc/config.py`'s `APRILTAG_RANGE_DEGRADATION`/`APRILTAG_ANGLE_
DEGRADATION`. The headline finding below was re-checked against this
more pessimistic model specifically to see if it would survive a less
generous assumption about its own winner -- it did.)

FullSuite wins on raw success rate, but **AprilTag is the best value**:
its success-rate gain over the free dead-reckoning baseline, per $100
spent, is still more than double FullSuite's (40.0pp/$100 vs.
16.2pp/$100) even under that more pessimistic correction model -- the
suites FullSuite stacks on top of AprilTag run into diminishing returns
rather than each adding their standalone value again. Which deviation
type actually dominates depends on the suite: dead reckoning's worst
failure mode is pose error (start drift), not obstacle error, which is
exactly what AprilTag (a pose-only fix) targets.

This result was measured on the `'cluttered'` layout only; `ftc/
layout_benchmark.py` reruns the identical sweep on `ftc/field.py`'s
other two layouts (sparse, corridor) and finds AprilTag stays the
best-value suite on both -- see `benchmark_results/
ftc_layout_writeup.md` and "Threats to validity" below.

The most useful negative result: **DistanceSensorSuite collides in
roughly half its trials even at zero field deviation.** A controlled
check (same trials, pose drift forced to zero) shows about two-thirds
of those collisions persist regardless -- the dominant cause isn't pose
drift, it's that 3 narrow ToF cones at ~12.5&deg; half-angle each cover
only about 75&deg; of the 360&deg; around the robot. A sparse fixed-cone
sensor suite has real, geometry-driven blind spots that this project's
own `nav/sensor.py` LidarSensor (a full disc scan) doesn't have --
buying distance sensors without covering enough of the robot's
perimeter can be worse than not sensing at all.

`nav/uncertainty_benchmark.py`'s own (domain-neutral) study still holds
at the retrofit-statistical-rigor bar Phase 3 asked for: reactive
replanning's success-rate CI separates from open-loop's by
variance_level=0.2 and stays separated the rest of the way, and belief-
based planning still collides in a handful of trials even at
variance_level=0.0 (see `benchmark_results/uncertainty_writeup.md`) --
the same class of finding as DistanceSensorSuite above, just for a
different reason (stale occupancy belief crossing the hard-obstacle
threshold too late, not an unseen blind spot).

Turn a real field/robot's own measurements into where it actually sits
on this study's deviation axis with `ftc/calibration.py`, and get a
suite recommendation for it with `ftc/recommend.py` -- see "How to run"
above. Both ship a clearly labeled synthetic placeholder dataset and
print which one (real or placeholder) they're actually running on; see
the next section for why that distinction matters as much as the
numbers themselves.

## Threats to validity / limitations

Naming these plainly is what separates a research testbed from a demo
-- none of them are secret, and none of them are fixed by this repo
alone.

- **Synthetic ground truth.** Every trial's "ground truth" grid
  (`nav/field_variance.py`'s `generate_ground_truth`) is a
  procedurally-perturbed copy of the assumed map, not a measurement of
  a real field. The perturbation model (start drift, obstacle drift,
  an unplanned blocker) is a hypothesis about what kinds of deviation
  matter, not a validated model of what FTC fields actually do.
- **Uncalibrated variance -- BOUNDED, not closed, until real
  measurements are supplied.** `variance_level` and `ftc/sensors.py`'s
  drift-rate constants are order-of-magnitude engineering estimates
  (see ftc/config.py's per-constant source comments) until `ftc/
  calibration.py` is run against real CSVs. `ftc/robustness.py` sweeps
  every estimated constant from 0.25x-4x its documented value and
  checks whether the best-value recommendation (AprilTag) survives
  being wrong by that much -- see `benchmark_results/
  ftc_robustness_writeup.md` for exactly which parameters tip the
  ranking and at what multiplier, and which never do across the swept
  range. A tipping point bounds how wrong an estimate can be before the
  conclusion changes; it does not tell you whether the *real* value is
  inside or outside that bound. Only `ftc/calibration.py` run against
  real measurements closes this -- right now, none of it has been
  checked against a real field or robot; the synthetic placeholder
  dataset exists to make the pipeline runnable, not to make its output
  trustworthy.
- **Simplified kinematics -- BOUNDED.** `ftc/match.py` now charges
  drive time via a trapezoidal (accelerate/cruise/decelerate) velocity
  profile bounded by `MAX_ACCEL_MPS2` (`ftc/match.py`'s
  `_trapezoidal_drive_time_s`) rather than assuming instantaneous
  acceleration to `MAX_DRIVE_SPEED_MPS` -- a real, if still simplified,
  improvement (a single 6in cell step is almost always too short to
  reach cruise speed at all, so drive time per step is now ~4-5x the
  old naive distance/speed figure). What's still not modeled: velocity
  isn't carried across consecutive collinear steps (each step starts
  and ends at rest, the same assumption the existing per-90-degree turn
  cost already makes), no wheel slip beyond the modeled pose drift, and
  no mecanum-specific strafing advantage despite `ftc/field.py`
  defaulting to 8-directional movement on the assumption of a holonomic
  drivetrain. A real robot's actual time-to-goal will still differ from
  this model's prediction by some amount this repo doesn't measure.
- **No opponent modeling -- BOUNDED.** The existing `unplanned_blocker`
  deviation type (one static obstacle, dropped once and left in place)
  is now joined by a `moving_blocker` variant in `ftc/
  opponent_benchmark.py`, which reuses `nav/obstacles.py`'s
  `MovingObstacle` (a seeded random walk, ticked on simulated match
  time) for a genuinely moving opponent, added alongside the static
  version rather than replacing it. The finding: **a moving opponent
  changes which suite is the best value** (Odometry pods beats AprilTag
  and FullSuite against a moving opponent; FullSuite is best against a
  static one -- see `benchmark_results/ftc_opponent_writeup.md`), and
  suites that never sense obstacles at all still do substantially
  better against a moving opponent than a static one, purely from
  timing luck (a parked obstacle blocks a fixed plan deterministically;
  a wandering one often isn't there anymore by the time a blind suite's
  plan reaches that cell). Still not modeled: the opponent has no goals
  of its own and doesn't react to this robot's presence -- a random
  walk is a step up from a fixed point, not a full multi-agent model.
- **One field layout for the headline study -- CLOSED.** `ftc/
  suite_benchmark.py` still runs on the `'cluttered'` layout only, but
  `ftc/layout_benchmark.py` reruns the identical full-rigor sweep on
  all three layouts `ftc/field.py` ships (sparse, cluttered, corridor)
  and checks whether the best-value suite changes: it doesn't --
  AprilTag is the best-value suite on every layout tested (see
  `benchmark_results/ftc_layout_writeup.md`). This closes the "would it
  shift on a different layout" question for the three layouts this repo
  actually ships; a season-specific surveyed layout dropped in later
  (see `ftc/field.py`'s module docstring) is still unchecked.
- **Small, fast trials mean the 30-second budget rarely binds --
  CLOSED.** `ftc/budget_benchmark.py` sweeps `AUTONOMOUS_PERIOD_S`
  downward (30s to 1.5s) and finds it now binds starting around 15-20s
  under the trapezoidal kinematics model above (it barely bound at all
  under the old naive drive-time formula) -- see `benchmark_results/
  ftc_budget_writeup.md`. Tightening the budget far enough does change
  which suite wins by raw success rate (DistanceSensorSuite overtakes
  FullSuite at 2s), though the headline 30s budget itself still never
  binds in the actual headline sweep.

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
  occupancy.py        Log-odds occupancy grid + BeliefGrid -- plans against expected cost, not a binary known/unknown split
  field_variance.py   Turns "how far reality deviates from the assumed map" into a sweepable knob, 3 independently-scalable deviation types
  policies.py         OpenLoopPolicy / ReactivePolicy / BeliefPolicy behind one shared interface
  uncertainty_benchmark.py  Open-loop vs reactive vs belief under swept map/reality deviation -> CSV + plot + statistical crossover
  stats.py            Pure-stdlib bootstrap confidence intervals (no numpy/scipy in this venv)
  scratch/         Standalone throwaway scripts used to prove each piece
                    works before it was wired into the visualizer/pybullet_main
                    (framework-agnostic tests only -- pygame/PyBullet-specific
                    ones live under pygame_app/ and pybullet_app/ instead)

ftc/               FTC domain layer -- see "nav/ vs ftc/" above. The only place
                    in this repo where FTC-specific names/numbers belong.
  config.py          Field/robot/match/sensor constants, each with its real-world source noted
  field.py           Parameterized field layouts (not tied to one season's game) -> a real-footprint-inflated nav.grid.Grid
  sensors.py         5 sensor suites (dead reckoning / odometry / distance sensors / AprilTag / full) -- which fix POSE error vs OBSTACLE error
  match.py           30-second autonomous-period budget model: drive + turn + replan time, pose-error offset mechanic
  suite_benchmark.py The headline study -- suite x deviation type x deviation level x trials -> CSV + 2 plots + writeup
  robustness.py      Tipping-point sweep on suite_benchmark.py's own estimated constants -- does the best-value suite change if they're wrong? -> CSV + plot + writeup
  layout_benchmark.py Reruns the headline sweep on all 3 field layouts -- does the best-value suite change with the layout? -> CSV + plot + writeup
  budget_benchmark.py Sweeps AUTONOMOUS_PERIOD_S downward -- when does the budget start to bind, and does it change the ranking? -> CSV + plot + writeup
  opponent_benchmark.py Static vs. moving (nav/obstacles.py MovingObstacle) opponent -- does a real-ish opponent change which suite wins? -> CSV + plot + writeup
  calibration.py     Fits variance_level components from real measurement CSVs (or a clearly-labeled synthetic placeholder)
  recommend.py       Decision CLI -- suite ranking / predicted success rate + CI / time vs. budget / cost
  scratch/         Same role as nav/scratch/, for the FTC-specific pieces

pygame_app/        Everything that touches pygame
  main.py            Entry point: opens the interactive visualizer
  visualizer.py      The pygame app
  scenario.py        ScenarioConfig -- preset state for scenarios/*.py
  scenarios/         Standalone launchers that open the visualizer into a preset scene
                      (maze, cost map, bottleneck, noisy sensor, step replay, uncertainty comparison, ...)
  scratch/         Headless (SDL_VIDEODRIVER=dummy) smoke tests for pygame-specific rendering paths

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
