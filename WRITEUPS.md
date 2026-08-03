# Writeups

Plain-English explanations of how each piece works, plus the concrete
observations behind each design decision. Organized by topic. Where a
claim is backed by a script, the script and its actual output are
referenced rather than restated from memory -- rerun them if you want to
confirm a number.

## Dijkstra and A*

### How Dijkstra works

Dijkstra keeps a priority queue of `(cost_so_far, cell)`, always popping the
cheapest one. The first time a cell is popped, its cost is final -- nothing
cheaper can show up later, because every edge costs a non-negative amount
and the queue always pops the smallest thing available. So each cell is
"settled" exactly once, and the search just expands outward in growing
rings of cost, with no notion of where the goal is. That's why its explored
region looks like a blob centered on the start rather than something
pointed at the goal.

### How A* works

A* is Dijkstra plus a heuristic: it pops the cell with the lowest
`f = g + h`, where `g` is the real cost so far (same meaning as Dijkstra's
cost) and `h(cell)` is an estimate of the remaining cost to the goal. The
heuristic biases the search toward the goal instead of outward in every
direction, so it visits far fewer cells before finding it.

### Why Manhattan distance is admissible on a 4-directional grid (60-second version)

A heuristic is "admissible" if it never overestimates the true remaining
cost -- if it did, A* could settle for a path that only looks good because
the heuristic lied about how far there was left to go. On a 4-directional
grid every move changes exactly one of (row, col) by 1 and costs exactly 1.
Manhattan distance is `|dr| + dc|`, and every legal move reduces that sum
by at most 1 (a move toward the goal reduces it by 1; a move sideways or
away doesn't reduce it, or increases it). So the true cost to the goal is
always >= the Manhattan distance -- it can never be an overestimate. That's
the whole proof: one move can never close more than 1 unit of Manhattan
distance, and one move never costs less than 1.

### Edge cases (`nav/algorithms.py: validate_endpoints`, `find_path`)

Four things the raw search loop doesn't check for and would otherwise
either crash or silently do the wrong thing:

- **start == goal**: trivially return a length-0 path without searching.
- **start on an obstacle**: the raw algorithms don't check this -- since
  neither `dijkstra` nor `astar` ever tests whether the *start* cell itself
  is passable (only whether cells you move *into* are), a start planted on
  an obstacle would silently search as if it weren't blocked. Caught and
  reported explicitly as `"start_blocked"`.
- **goal on an obstacle**: `get_neighbors` never returns an obstacle cell
  as a neighbor, so a goal sitting on one is simply unreachable -- the
  search would exhaust itself and report `"no_path"`, which is technically
  true but a worse message than `"goal_blocked"`.
- **no path exists**: the priority queue empties without ever popping the
  goal. Both algorithms already handle this correctly (return `None`); the
  wrapper just labels it.

Verified in `nav/scratch/edge_cases_test.py` -- each case is forced on a
hand-built grid and prints its own reason with zero UI involved.

## RRT, cost map, and replanning

### How RRT works, and why it's fundamentally different from grid search

Dijkstra and A* both search a *fixed* graph: every grid cell is a node,
every adjacent free cell is an edge, decided in advance by `get_neighbors`.
RRT doesn't search a graph at all -- it builds one, on the fly, out of
randomness:

1. Sample a random free point (occasionally the goal itself, 10% of the
   time -- `RRT_GOAL_SAMPLE_RATE` -- which biases the tree to eventually
   grow toward it instead of wandering forever).
2. Find the tree's existing node nearest that sample.
3. Step from that nearest node toward the sample by a fixed distance
   (`RRT_STEP_SIZE`), landing on a new point.
4. Add the new point as a tree node -- *if* the straight edge from nearest
   to new doesn't cross an obstacle (checked cell-by-cell with Bresenham's
   line algorithm, `nav/rrt.py: _clear_line`, same corner-safety concern
   as the diagonal-movement collision check).
5. Repeat until some new node lands within `RRT_GOAL_RADIUS` of the goal,
   or `RRT_MAX_ITERS` is exhausted.

This is why RRT "explores" completely differently from Dijkstra/A*: it
doesn't expand outward from the start in any organized order, and it
isn't confined to the grid's 4/8-directional step rule -- it connects
points with straight lines through open space. Verified in
`nav/scratch/rrt_test.py`, which grows a tree on a hardcoded grid with a
wall and prints the resulting node count and path.

The tradeoff for that flexibility: no completeness or optimality
guarantee in a fixed iteration budget, and no determinism -- rerun it on
the identical grid and you get a different tree and a different path
every time (unless you pin the RNG, which the benchmark does). See the
benchmark writeup (`benchmark_results/writeup.md`) for exactly how much
that costs it against Dijkstra/A* on this domain.

### Cost map / weighted terrain (`Grid.compute_cost_map`)

Originally the grid was strictly binary: a cell was either free
(cost 1 to enter) or an obstacle (impassable). `compute_cost_map` adds a
third state in spirit, without adding a third cell type: free cells
*near* an obstacle get a cost between 1.0 and `1 + COST_MAX_EXTRA`,
decreasing linearly to the 1.0 baseline at `COST_INFLUENCE_RADIUS` cells
away. This is exactly what Nav2's costmap inflation layer does --
obstacles project an increasing-cost "buffer zone" outward, so a
cost-aware planner prefers routes with clearance over routes that hug a
wall, without needing a hard-coded minimum-distance rule.

Mechanically, `Grid.get_neighbors` already returned `(cell, step_cost)`
pairs (1 or `sqrt(2)` for diagonal); cost-map mode just multiplies that
step cost by `self.cost[r][c]`, the terrain weight of the cell being
entered. Dijkstra and A* need zero code changes to respect it -- they
already treat step cost as data, not a hardcoded constant. Admissibility
survives too: Manhattan/octile estimate the *minimum possible* cost
(assuming every step costs exactly 1 or `sqrt(2)`), and terrain weight
only ever raises the true cost above that baseline, never below it, so
the heuristic still never overestimates.

**Observed effect on RRT: none.** This RRT implementation has no notion
of edge cost at all -- it only asks "does this line cross an obstacle,"
never "how expensive is this line" -- so a cost map that leaves every
free cell passable (just pricier) doesn't change which edges RRT is
willing to add, and its tree grows identically with cost-map mode on or
off. This is a real, structural limitation of plain RRT versus RRT*
(which does incorporate edge cost into which parent it connects to), not
a bug -- worth being able to say cold if asked "does your cost map affect
all three planners."

### Moving obstacles and replanning (`nav/obstacles.py`)

(Full version lives as the docstring on `MovingObstacle` in
`nav/obstacles.py`, since that's the code whose behavior it's describing.)

- A moving obstacle bounces between two adjacent free cells on a timer.
- After it moves, if its new cell lies on the *remaining* portion of the
  robot's current path (not the part already walked), the visualizer
  immediately replans from the robot's current cell to the goal.
- If an obstacle lands exactly on the robot's current cell, the robot
  doesn't step onto/through it -- it holds position and tries to replan
  from where it stands. If no path exists, it stays put and retries on
  every later obstacle move (the visualizer shows a "blocked" indicator)
  until a route opens back up.
- An obstacle moving *away* (freeing a cell) never forces a replan by
  itself. Only a move that newly blocks the current path does. This keeps
  the robot from redundantly replanning on every single tick of every
  obstacle in the scene.

Why replan from the robot's current cell rather than the original start:
the robot has already paid the cost to get where it is, and re-running
the search from the true starting point (the original `grid.start`) would
send it needlessly backtracking through cells it's already past.

Verified two ways: `nav/scratch/moving_obstacle_test.py` runs this policy
with no UI -- one obstacle bouncing on a fixed straight-line path, printing
the path before and after each replan (it correctly detours around the
obstacle every time it lands on the route). The visualizer integration was
checked with a headless pixel-level test that sampled the live pygame
surface: it confirmed the moving obstacle actually alternates cells, the
robot marker actually advances across the grid, and it detours through an
adjacent row exactly when the obstacle blocks its planned path -- i.e. the
replanning isn't just "doesn't crash," it visibly reroutes.

## Sensor model, heuristics, and diagonal movement

### Why sensor uncertainty matters, and how the simulated lidar works

Every planner up to this point assumed perfect map knowledge -- the
robot's `grid.start`, its `grid.cells`, and the true obstacle layout were
all the same object. That's not how a real robot works: it only knows
what its sensors have actually reached. Once that gap exists, planning
stops being "find the shortest path on a known graph" and becomes
planning-under-uncertainty -- a plan that looks perfectly safe can turn
out to be wrong the moment the robot gets close enough to see further.

`nav/sensor.py` models this with two pieces:

- **`LidarSensor`**: `sense(grid, position)` reveals every real obstacle
  within `radius` cells of `position` (Euclidean, not Chebyshev -- a
  circle, not a square) and adds it to `known_obstacles`, a set that only
  ever grows. It returns just the *newly* seen obstacles, which is what
  the caller needs to decide whether to replan.
- **`KnownGrid`**: a `Grid` subclass built entirely from
  `known_obstacles` -- every cell the sensor hasn't seen is assumed free.
  That's the only assumption a robot without a perfect map can honestly
  make; it also means a route can plan straight through a cell that's
  actually an obstacle, right up until the robot gets close enough to
  discover otherwise. Because it's a `Grid` subclass, `dijkstra`/`astar`/
  `rrt` run against it completely unmodified -- none of them know or
  care whether the grid they were handed is the ground truth or a
  partial belief about it.

`nav/scratch/lidar_test.py` proves the discover-and-replan loop with no
UI: a robot moving toward a goal along a corridor with three obstacles
hidden until it's within sensor radius. It plans an initially-optimistic
straight path (nothing sensed yet), then replans twice as it advances and
each obstacle enters range, and reaches the goal having discovered
exactly the obstacles that were actually in its path -- confirmed by
diffing `sensor.known_obstacles` against the hardcoded ground truth at
the end of the run.

**Known simplification:** `known_obstacles` only grows -- a cell once
seen as an obstacle is never un-sensed, even if (in the visualizer, with
moving obstacles + sensor mode both on) it later moves away. A real
sensor would see it's gone; this one remembers stale information
forever. Left as-is because handling it properly means tracking sensed
*free* cells too, not just obstacles, which is real added complexity for
a case the project's moving obstacles don't actually create in practice
(the robot re-senses its surroundings every step it takes, so a stale
belief only lingers for cells outside current sensor range).

### Sensor noise: from perfect detection to imperfect, and what changes about trusting it

Both sensor models above are, by default, perfect: every real obstacle
within range gets detected, at its exact cell, and nothing else does.
That's a much stronger assumption than any real range sensor gets to
make, and it's worth being honest that everything in the section above
(the discover-and-replan loop, the "stale belief" simplification) was
built and verified against that unrealistically clean signal. `noisy=True`
(`LidarSensor` in `nav/sensor.py`, `Lidar3D` in `pybullet_app/sim3d/lidar.py` --
same three parameters, same behavior, one per 2D radius-cell and one per
3D raycast) replaces it with three independent failure modes, each
governed by its own rate in `nav/config.py`:

- **False negative** (`NOISE_MISS_RATE`, default 0.15): a real obstacle
  in range isn't detected this scan.
- **Position noise** (`NOISE_POSITION_RATE`, default 0.15): a detected
  obstacle is reported at a random *adjacent* cell (2D: one of its 8
  neighbors; 3D: the raycast hit point nudged by up to
  `POSITION_JITTER_METERS` before being converted to a cell) instead of
  its true one.
- **False positive** (`NOISE_FALSE_POSITIVE_RATE`, default 0.02): a free
  cell (2D) or a ray that hit nothing (3D, which hallucinates a phantom
  hit at a random point along that ray instead) gets "detected" as an
  obstacle that isn't there.

Both default to `noisy=False`, so every existing caller, test, and demo
keeps its exact current deterministic behavior unless it explicitly asks
for noise -- nothing above needed to change to stay true.

**Why a single noisy reading can't just be trusted the way a perfect
one was.** `known_obstacles` still means exactly what it always did --
every cell ever *reported*, right or wrong -- because noise can plant a
wrong cell just as permanently as a correct one (nothing here un-senses
anything, same simplification as before, now compounded by the fact
that some of what gets remembered forever was never true). Both sensors
also track `detection_counts` (how many separate scans reported each
cell) and expose `confirmed_obstacles(min_detections)`: with
`min_detections=1` (the default, and exactly what a non-noisy sensor's
`known_obstacles` already was), a single report is trusted immediately
-- safe only because a perfect sensor is never wrong. With noise on, a
caller should ask for `min_detections=2` or higher instead: requiring
the *same* cell to be independently (mis)reported more than once is a
meaningfully rarer coincidence than seeing it once, for exactly the
reason repeated confirmation is trustworthy in general -- each scan's
noise is an independent roll, so the chance of the identical false
reading landing twice is roughly the single-scan rate *squared*, not
the same rate again.

**Does this actually matter, concretely, or is it a theoretical nicety?**
Measured, not asserted (`nav/scratch/lidar_noise_test.py`,
`pybullet_app/scratch/pybullet_lidar_noise_test.py` -- both scanning a hidden
obstacle repeatedly from a fixed position, then replanning against
`confirmed_obstacles` at increasing thresholds and comparing the
resulting path cost to the true optimum):

| min_detections | 2D corridor test | 3D wall test |
|---:|---|---|
| 1 (= raw `known_obstacles`) | **FAILED outright** (`start_blocked`) | cost 34.00 (+13% vs optimal) |
| 2 (`CONFIRMATION_THRESHOLD` default) | cost 29.00 (+16% vs optimal) | cost 30.00 (= optimal) |
| 3 | cost 25.00 (= optimal) | cost 30.00 (= optimal) |
| 4+ | cost 25.00 (= optimal) | cost 30.00 (= optimal) |

The 2D case is the more dramatic one: in that exact run, a single false
positive happened to land on the robot's own start cell, and
`validate_endpoints` correctly (if unhelpfully) reports "start_blocked"
-- a perfectly literal reading of a noisy sensor's output makes planning
fail *outright*, not just suboptimally, the instant bad luck puts a
phantom obstacle somewhere it really matters. Requiring even one repeat
(`min_detections=2`) was enough to avoid that specific outright failure
in both scripts, though the 2D run still paid 16% extra path length at
that threshold from other surviving noise; a couple more repeats closed
the rest of the gap to the true optimum in both. That's the actual
shape of the tradeoff a confirmation threshold buys: higher means fewer
false alarms and less wasted detour, at the direct cost of needing more
scans -- more time spent near a real obstacle -- before the robot will
act on it at all. A robot that needs to react instantly to a single
sensor ping cannot also demand five confirmations first; this project's
`CONFIRMATION_THRESHOLD = 2` is a middle-of-the-road default, not a
tuned optimum, and callers that want a different point on that tradeoff
(`pygame_app/visualizer.py`'s `N` key, `pybullet_app/pybullet_main.py --sensor
--noisy-sensor`) can pass a different `min_detections` to
`confirmed_obstacles` directly.

**A confirmation threshold does not turn noisy sensing back into perfect
sensing, and it's worth being precise about exactly how it falls short**
(found by actually inspecting *which* cells survived a low threshold,
not assumed): in the 3D wall test, most of the "confirmed" cells that
weren't real turned out to be position-jittered *neighbors* of a real
wall cell -- the same true obstacle, reported one cell off, repeatedly,
often enough on its own to independently clear the threshold. That's a
structurally different, more benign failure than a random phantom
elsewhere: it blurs a real obstacle's boundary by about a cell rather
than inventing one in open space, but it's still a real limit of
counting hits *per exact cell* -- a sensor whose position noise spreads
a real detection's signal across several neighboring cells can leave
every individual cell under-confirmed even though the area is clearly
occupied, since each neighbor only gets a fraction of the repeat counts
the true cell would have accumulated alone. A more sophisticated belief
representation (e.g. an occupancy grid that spreads confidence
across nearby cells instead of requiring an exact repeat) would handle
this better; per-cell counting is the simplest thing that could
possibly work, chosen deliberately over that added complexity the same
way the "known_obstacles only grows" simplification was, and both
tradeoffs are recorded here rather than hidden.

**Wiring**: both `pygame_app/visualizer.py` (`N` key, alongside `S` for the
sensor itself) and `pybullet_app/pybullet_main.py --sensor --noisy-sensor` gained a
noise toggle that's off by default, and both switched their replanning
trigger from "any newly reported cell" to "any newly *confirmed* cell"
(computing `confirmed_cells()` before and after each scan and diffing
the two sets) -- with noise off this is provably identical to the old
behavior, since `confirmed_obstacles(1) == known_obstacles` always; with
noise on, it's what keeps a single stray false positive from kicking off
a wasted replan on its own. The live "hidden obstacle" visual in both
(`pygame_app/visualizer.py`'s outline, `pybullet_app/pybullet_main.py`'s dim/reveal) is
deliberately left showing *raw* sensor output, not confirmed -- so a
human watching the demo can see the sensor's actual noisy behavior in
real time, while the planner underneath only ever acts on what it's
decided to trust.

### Inadmissible heuristics: does 1.5x actually break anything?

The experiment was to multiply Manhattan by 1.5 and see what actually
breaks in practice. First finding, which took a wrong turn to get to: **a "perfect"
maze (recursive backtracking, no loops) has exactly one route between any
two cells.** There's nothing for a bad heuristic to get wrong when there's
only one possible path -- `nav/scratch/heuristic_experiment.py` originally
ran on maze grids and found 0/8 suboptimal results across every heuristic,
including the inflated one, simply because there was no alternate route to
wrongly prefer. Suboptimality needs *competing* routes of different
lengths, which only shows up with scattered obstacles (loops, open areas),
not a spanning-tree maze. Switching the test bed to random 35%-density
obstacle grids was necessary before the experiment meant anything.

Second finding: even on grids with real alternate routes, **1.5x almost
never broke optimality** in 25 trials (0/25). It did roughly halve the
number of cells explored, though -- a real speed win for zero cost in these
cases. Cranking the inflation to **3x** did break it, in 2/25 trials, e.g.:

```
seed 0: optimal cost 39  ->  1.5x found 39 (optimal, explored 87 cells)
                          ->  3x   found 45 (13% worse, explored 70 cells)
seed 3: optimal cost 31  ->  1.5x found 31 (optimal, explored 64 cells)
                          ->  3x   found 35 (13% worse, explored 65 cells)
```

Why it takes a large inflation factor to see it with this implementation:
`astar`/`dijkstra` here settle a cell permanently the first time it's
popped and never reopen it (a "closed set with no reopening"). That's
provably safe *only* if the heuristic is consistent (`h(n) - h(n') <=
cost(n, n')` for every edge). Manhattan is consistent; scaling it by a
constant `k > 1` is not, in general -- a move that reduces true Manhattan
distance by the full 1 reduces the *scaled* heuristic by `k`, so
`h'(n) - h'(n') = k > 1 =` the edge cost whenever `k > 1`, violating
consistency. So the algorithm genuinely can lock in a worse path -- it's
just that with `k = 1.5` the violation is small enough that it rarely
matters on a 25x25 grid with modest obstacle density. `k = 3` makes the
effect obvious and reproducible. This is the classic "weighted A*"
speed/optimality tradeoff: inflating the heuristic buys search speed by
spending some (usually small, occasionally real) amount of solution
quality.

### Diagonal movement and why Manhattan breaks

Added 8-directional movement to `Grid.get_neighbors` (`nav/grid.py`):
diagonal steps cost `sqrt(2)`, and are only allowed when *both* orthogonal
cells adjacent to the move are free -- otherwise a "diagonal" step would
clip through a solid wall corner, which isn't a real move. That corner
check wasn't obvious up front; it came up while implementing and is
worth calling out since it's a genuine correctness bug in naive
8-directional implementations, not just a style choice.

Manhattan distance is `dr + dc` -- it assumes you can only ever move one
axis at a time, which is exactly untrue once diagonal moves are allowed.
The correct heuristic for a `sqrt(2)`-cost diagonal is **octile distance**:
`max(dr, dc) + (sqrt(2) - 1) * min(dr, dc)`. Since `sqrt(2) - 1 < 1`,
octile distance is always <= Manhattan distance (equal only when `dr` or
`dc` is 0), meaning **Manhattan systematically overestimates the true cost
whenever a diagonal shortcut exists** -- it's inadmissible the moment
diagonal movement is on, for the same reason as the inflated heuristic
above, just for a structural reason instead of an arbitrary multiplier.

`nav/scratch/diagonal_heuristic_test.py` confirms this on 29 random
25%-density 8-directional grids: Manhattan produced a suboptimal path in
2/29 trials, octile in 0/29. Concrete example:

```
seed 4: optimal cost 16.657  ->  manhattan found 17.243 (SUBOPTIMAL, 18 explored)
                              ->  octile    found 16.657 (optimal,    35 explored)
```

Notice Manhattan explored *fewer* cells than octile even while being
wrong -- same pattern as the inflated-heuristic case: overestimating the
remaining cost makes the search more aggressively greedy, which is faster
but no longer guaranteed correct. Octile is both correct and only
modestly more expensive, which is why it's the visualizer's default
heuristic whenever diagonal movement is on (`X` key), while Manhattan
stays default for 4-directional movement.

### Maze generator (`nav/maze.py`)

Recursive backtracking on the even-coordinate lattice of the grid (cells
at even row/col, walls at odd row/col in between), which requires an odd
`GRID_SIZE` -- 25 satisfies that. It produces a "perfect" maze: exactly one
route between any two open cells, no loops. That property is exactly what
made it the wrong test bed for the heuristic experiments above, but it's
useful on its own as an interesting, always-solvable stress test for the
visualizer and the replanning logic.

## PyBullet port

### What actually had to be new code

The goal was a robot navigating a 3D environment using the same A* code
as the pygame visualizer, and it's worth being precise about how literal
that is: `pybullet_app/pybullet_main.py` imports `nav.grid.Grid` and
`nav.algorithms.find_path` directly, unmodified, and calls them exactly
the way `pygame_app/visualizer.py` does. Every new file lives under
`pybullet_app/sim3d/` and is strictly about the physics interface -- turning grid
cells into 3D bodies, turning a cell path into a driveable trajectory,
and turning that trajectory into motor commands. Planning logic and 3D
plumbing never touch the same file.

### Grid-to-world coordinates (`pybullet_app/sim3d/coords.py`)

`grid_to_world(row, col)` maps `col -> x`, `row -> y`, `z = 0` at
`WORLD_CELL_SIZE = 1.0` meter per cell -- the same `col`-is-horizontal,
`row`-is-vertical convention `pygame_app/visualizer.py` uses for pixels, just
with meters instead of pixels and an explicit up-axis. Every obstacle
body, path debug-line, and waypoint in `pybullet_app/sim3d/world.py` goes through
this one function, so the 3D obstacle layout lines up with the grid A*
actually searched, cell for cell.

### Why `resetBaseVelocity`, not teleporting the robot

The goal was to drive the robot along A* waypoints using velocity
control, not to make it look smooth on the first pass (that's what the
smoothing step below is for). The tempting shortcut is
`p.resetBasePositionAndOrientation(robot_id,
waypoint, ...)` every frame -- it would "work" in the sense of the robot
visibly moving along the path, but it's not driving, it's teleporting;
PyBullet's physics engine never gets a velocity to integrate, so nothing
about the motion is actually simulated. `Robot.drive_toward`
(`pybullet_app/sim3d/robot.py`) instead computes a heading error to the target and
calls `p.resetBaseVelocity(body_id, linearVelocity=[...],
angularVelocity=[...])` every step -- a real (if simplified) velocity
command the physics engine has to integrate into position over time, the
same category of control a differential-drive base actually uses.

It turns in place before driving forward (`FACE_TARGET_TOLERANCE`)
specifically because `resetBaseVelocity` has no concept of "forward" --
without that check the robot would happily strafe sideways toward a
waypoint behind it, which no real wheeled base can do.

### Path smoothing: corner-cutting, then a spline (`pybullet_app/sim3d/smoothing.py`)

Three functions, used in sequence:

1. **`simplify_collinear`** -- A* on a 4-directional grid emits one
   waypoint *per cell*, so a straight 10-cell run is 10 collinear points.
   Collapsing runs like that to their two endpoints (cross-product test:
   `(p1-p0) x (p2-p0) == 0` means collinear) gives the smoothers below a
   handful of real corners to work with instead of dozens of redundant
   knots along every straight stretch. Not smoothing by itself -- pure
   cleanup before smoothing.
2. **`chaikin_smooth`** -- simple corner-cutting, done first because it's
   the cheapest thing that could possibly work. Each pass replaces every
   corner with two points 1/4 and 3/4 of the way along its adjacent
   edges; repeated a few times this rounds every corner into a curve.
   Cheap, easy to reason about, and the result only *approaches* the
   original path -- except the two endpoints, which are kept exact here
   (unlike the textbook version of Chaikin, which cuts those too) so the
   robot still starts and ends in the actual start/goal cell instead of
   near it.
3. **`catmull_rom_spline`** -- the follow-up upgrade once corner-cutting
   proved the concept. Unlike Chaikin, a Catmull-Rom spline passes
   *exactly* through every control point, not just the endpoints, while
   still arriving at each one smoothly instead of on a sharp corner.
   This is what `pybullet_app/pybullet_main.py` drives by default (`--smooth
   spline`); `--smooth corner_cut` and `--smooth raw` (no smoothing at
   all, the pre-smoothing baseline) are there specifically so the
   difference is easy to see and describe, not just claimed.

The one thing that never changes across all three modes is
`Robot.drive_toward` -- same controller, same heading-error logic, every
time. The robot looks jerky on `--smooth raw` and smooth on `--smooth
spline` purely because of how far apart consecutive waypoints are and
how sharply the heading has to change between them, not because of
anything mode-specific in the driving code. That's the actual argument
for why smoothing matters, demonstrated rather than asserted.

### Porting the cost map, and why this demo grid isn't a maze

`grid.cost_map_enabled` / `grid.refresh_cost_map()` (`nav/grid.py`) were
already built to work through `get_neighbors`, independent of who's
rendering the grid -- so "porting" the cost map to PyBullet took zero
new code. `pybullet_app/pybullet_main.py` just plans twice, once with
`cost_map_enabled = False` and once `True`, and draws both routes as
PyBullet debug lines (red = binary-obstacle route, blue = cost-map
route) so the difference in path *shape* is directly visible in the GUI
instead of inferred from numbers.

That comparison needs room to actually differ, which is why
`build_demo_grid()` places one large 9x9 obstacle block instead of
reusing `nav/maze.py`'s recursive-backtracking maze (the same reason it
was the wrong test bed for the heuristic experiments above): a perfect
maze's corridors are exactly one cell wide everywhere,
so there's no free space *around* an obstacle to route through with
clearance -- inflating obstacle cost there mostly just makes the one
available corridor pricier, not differently shaped. An open block with
room on every side is what actually exercises the feature.

It also needs start and goal placed so the block is unavoidable, which
the first version of this demo got wrong: start and goal in opposite
corners of the grid let a 4-directional Manhattan-optimal search route
along the *outer* boundary of the grid, arbitrarily far from the block,
since every monotone staircase path between two corners has identical
Manhattan cost -- the binary and cost-map searches picked the exact same
40-step outer-edge route and there was nothing to compare (confirmed by
diffing the two returned paths: identical, cell for cell). Fixed by
putting start and goal on the *same row*, with the block sitting
directly between them -- now the straight route is genuinely blocked,
a detour is mandatory, and the block is placed off-center on that row
(2 rows below its top edge, 6 above its bottom) so going around the top
edge is the unambiguously shorter option instead of a coin-flip between
two equally-good detours. With that fix, the binary search hugs the
block's top edge by exactly one cell (the closest legal cell to it),
while the cost-map search bows out to a row fully outside the
`COST_INFLUENCE_RADIUS = 3` inflation zone -- a real, visibly different
detour, not just a different cell count.

### A build problem worth documenting: `pip install pybullet` didn't just work

On this machine (a very new macOS/Xcode Command Line Tools combination),
`pip install pybullet` fails building from source -- there's no
prebuilt wheel for this platform/Python combination, so pip falls back
to compiling PyBullet's bundled C++ sources, and that build hits a real
compiler error, not a flaky one:

```
_stdio.h:322:7: error: expected identifier or '('
FILE *fdopen(int, const char *) __DARWIN_ALIAS_STARTING(...);
        ^
zutil.h:128:26: note: expanded from macro 'fdopen'
#define fdopen(fd, mode) NULL /* No fdopen() */
```

PyBullet vendors an old fork of zlib (`examples/ThirdPartyLibs/zlib/`)
whose `zutil.h` `#define`s `fdopen` to `NULL` on `MACOS`/`TARGET_OS_MAC`
-- a workaround from the classic (pre-OS X) Mac Toolbox era, when there
genuinely was no POSIX `fdopen`. Modern Darwin very much has one, and
this macro get textually substituted into `_stdio.h`'s own *declaration*
of `fdopen` the next time any translation unit includes zutil.h before
stdio.h -- turning `FILE *fdopen(...)` into `FILE *NULL(...)`, a syntax
error, not a linker or logic issue. It's a genuine bug in ~15-year-old
vendored code that happens to have gone unnoticed on toolchains where
header inclusion order didn't provoke it.

Fix: download the sdist (`pip download pybullet==3.2.7 --no-binary=:all:
--no-deps`), delete that one obsolete macro block from
`examples/ThirdPartyLibs/zlib/zutil.h` (Darwin doesn't need a stub for a
function it actually has), and `pip install` the patched source tree
directly. Worth recording here rather than just fixing silently, since
"the install didn't work out of the box" is itself a real thing to be
able to explain if asked about the PyBullet leg of this project.

## 3D lidar, multiple robots, scale

### Making the robots faster

`pybullet_app/sim3d/robot.py`'s defaults were raised well above the original
2.0 m/s / 4.0 rad/s starting point, both for a snappier demo. Before
raising them, the headroom was checked experimentally: 6, 8, and 10 m/s
tested head-to-head (same `drive_toward` controller, same r2d2), and all
three reached the target with the base staying flat the whole time
(`max_z` never left its resting height -- no bouncing, no tipping),
confirming there's plenty of margin above the original default before
anything physically breaks.

### Porting the sensor model to real raycasts (`pybullet_app/sim3d/lidar.py`)

The 2D sensor was "every obstacle within radius R" -- a circle, with no
concept of line of sight. `Lidar3D.scan` replaces that with
`pybullet.rayTestBatch`: 48 rays cast outward from the robot's position,
and whatever each ray's *first* hit is becomes a sensed obstacle. A wall
can now block the view of what's behind it, which a radius circle
structurally cannot represent -- this is a real capability upgrade, not
just a reimplementation in a new coordinate space.

Two things had to be right for this to work at all, both found by
actually running it, not by reasoning about the API in the abstract:

- **Rays can't originate at the robot's own center.** A ray that starts
  literally inside (or on the surface of) the sensing robot's own
  collision shape hits *itself* first, every time, regardless of
  `ignore_body_id` -- rayTestBatch reports the first hit along the ray,
  and the robot's own hull is that hit. Fixed by starting each ray
  `ORIGIN_OFFSET = 0.35` m out from center, in the ray's own direction,
  rather than at the exact center point.
- **A hit lands exactly on a cell boundary, and rounding that is
  ambiguous.** An obstacle box's near face sits at, e.g., `x = 8.5` for
  a 1.0m cell size -- precisely the boundary between free cell 8 and
  obstacle cell 9. `pybullet_app/scratch/pybullet_lidar_test.py`'s standalone scan
  didn't surface this because its one hardcoded scan never happened to
  land exactly on a boundary; the real integration in
  `pybullet_app/pybullet_main.py --sensor` did, immediately: `newly_seen` sets came
  back containing cells like `(12, 8)` -- the *free* cell directly in
  front of the real wall at column 9-17, not the wall itself. Python's
  `round()` is round-half-to-even, so a hit at exactly `x=8.5` doesn't
  even consistently round toward the obstacle. Left unfixed, this
  poisons `known_obstacles` with phantom obstacles in real free space,
  and eventually one of those phantom cells was the robot's *own*
  current cell -- every subsequent replan immediately failed with
  `start_blocked`, because `KnownGrid` correctly (if confusingly) saw
  the robot's own position as an obstacle. Fixed in `Lidar3D.scan` by
  nudging the hit point `HIT_NUDGE = 0.1` m further along the ray, past
  the surface, before converting it to a grid cell -- a hit at `x=8.5`
  moving in the `+x` direction becomes `x=8.6`, which rounds to 9
  unambiguously.

With both fixed, `pybullet_app/pybullet_main.py --sensor` reliably explores roughly a
quarter of the grid (21-24 of 81 real obstacle cells, across repeated
runs) -- only what it actually needed to see to solve the specific
route -- and replans live as each new obstacle enters view, verified by
the printed "sensed N new obstacle cell(s) -- replanning" trail matching
up with the robot actually changing course rather than driving through
where the (still just-discovered) wall is.

### Multiple robots (`pybullet_app/pybullet_multi_robot_main.py`)

The layout is a real cross-street intersection: four square buildings,
one per grid quadrant, leaving a 3-cell-wide "plus" of open street down
the middle in both directions. Robot A drives the north-south street
start to finish; robot B drives the east-west street start to finish.
All four points -- A's start, A's goal, B's start, B's goal -- are
different cells, and the two routes are the same length, so left to
their own devices the robots reach the crossing at close to the same
moment. This replaced an earlier layout (a single corridor with
goal(A) == start(B), a "swap sides" scenario) specifically because that
coincidence turned out to cause its own class of bugs -- see below.

**Coordination policy (current version -- see the bug list below for two
earlier versions that didn't hold up):**

- **Both robots replan, symmetrically.** Every `REPLAN_PERIOD_S = 0.05` s,
  each one runs A* against the real grid *plus* a block placed around the
  *other's* current cell (`cell_block`, a 1-cell buffer) -- the exact
  same technique the pygame `MovingObstacle` replanning logic used
  (mark the moving thing as a temporary wall, replan around it), just
  with a robot standing in for the moving obstacle on both sides at once.
  A replan is only actually issued when the blocked cells changed or the
  last attempt found nothing (`agent.waiting or current != last`) --
  replanning unconditionally every tick was an earlier bug (see below),
  and the fix generalizes cleanly to both robots being symmetric now.
- **If a route genuinely isn't there, the blocked robot holds position**
  (`waiting = True`) and retries on the next replan tick, rather than
  crashing on `None` or driving into a wall.
- **Neither robot gets a local steering nudge anymore.** An earlier
  version had one robot (B) steer its aim point sideways every physics
  step when the other got close, faster-reacting than the 0.05s replan
  cadence. It's gone now -- see the bug entry below for why steering
  turned out to be the wrong mechanism entirely, not just something that
  needed better tuning.

**Why symmetric replanning doesn't deadlock here, unlike the old
corridor layout.** A *symmetric* "both treat the other as an obstacle"
policy is exactly what could deadlock face to face in a single
one-cell-wide corridor: each one sees the other blocking the only route
and waits forever, since neither ever decides to go first, which is why
the corridor-layout version of this demo used a strict priority order
instead (see below). The intersection layout removes the reason that
mattered: streets are 3 cells wide, so when A* finds the direct route
blocked it almost always finds a real alternate route through the
adjacent lane rather than reporting failure at all -- there's usually
somewhere to go besides "wait." `waiting = True` still exists as a
fallback for the rare moment neither lane is free, but it's a transient
state on the way to a route reopening, not a standoff between two
robots that refuse to yield.

**A hard safety-distance stop is layered on top, deliberately not relied
on as the primary mechanism:** replanning runs every `REPLAN_PERIOD_S`,
not every physics tick, so a fast robot could in principle close
real-world distance in the gap between replans. If the two robots'
actual distance ever drops below `SAFETY_STOP_RADIUS`, both are forced to
stop that exact frame regardless of what their plans say. It exists as a
failsafe against replanning latency, the same role an emergency stop
plays underneath a real path planner -- and it checks real physical
distance unconditionally, every step, with no exceptions (see the
collision bug below for what happened when it briefly wasn't
unconditional).

**One scenario-design bug worth recording:** the first version of this
demo had robot A permanently parked exactly on top of robot B's goal
cell after "arriving," because goal(A) == start(B) by construction (a
swap-sides scenario makes that coincidence unavoidable) and a robot that
finishes just sits at its exact goal position forever. B's own
destination was therefore permanently blocked by A's corpse, and B never
finished. Fixed by nudging an arrived robot 1.5m off to the side
(`NavAgent._park`) and excluding arrived robots from the other's blocked-
cell set entirely -- once a robot is done, it stops being an obstacle for
anyone.

**A second bug, found by watching a live run rather than just the
end-of-run pass/fail:** even after that fix, B looked stuck for several
seconds right after A cleared the corridor, when the plan should already
have been open. The cause was replanning B *unconditionally* every
`REPLAN_PERIOD_S`, even when nothing had changed. Every fresh `plan()`
call starts the new route from B's *rounded* current cell
(`world_to_grid`), which snaps to a point slightly behind or off B's
actual continuous position -- so a needless replan doesn't just waste
work, it makes B briefly steer toward that snapped point instead of
smoothly continuing, over and over, once per replan tick. Averaged over
many ticks this looked exactly like "stuck," when it was actually
"constantly restarting." Fixed by only calling `plan()` when B is
currently waiting (no valid route yet) or when the blocked-cell set
has actually changed since the last plan (`current_blockers !=
last_blockers`) -- once A clears the corridor and the blocked set
stabilizes at empty, B is simply left alone to drive the route it
already has. (The same unconditional-replan mistake reappeared in a
second form once the staging-cell fallback was added: the trigger
condition included "replan whenever B isn't on a *final* plan," which
fired every single tick for the entire drive to the staging point, not
just when something changed. Same symptom, same fix -- drop that extra
condition and rely purely on "waiting, or the blocked set changed.")

**A third bug: the safety stop had a loophole that caused a real
collision.** The distance check was originally `(not agent_a.arrived)
and distance < SAFETY_STOP_RADIUS` -- deliberately skipped once A had
"arrived," on the reasoning that a stationary, parked A poses no risk.
In practice: B spends most of a fast run either blocked or held back by
the safety stop itself, which means when the corridor *finally* clears
-- the instant A reaches its goal -- B is often still sitting close by,
about to lurch back into motion right as A crosses the finish line.
That's exactly the moment the two are most likely to be near each
other, and it's exactly the moment the one check that would have caught
it switched itself off. Fixed by removing the `not agent_a.arrived`
condition entirely -- the check now runs unconditionally, every step,
regardless of A's state. A parked far away simply never trips it; nothing
is lost by leaving it on.

**A fourth bug, from the version of this demo that gave the priority
robot (A) its own avoidance steering too:** the first version of the
local avoidance layer applied to *both* robots,
and used raw proximity ("is the other robot within AVOID_RADIUS") rather
than checking whether it was actually in the way. Two failure modes came
out of that, found by watching, not by reading the code:

- A stationary B sitting near A's own goal caused A to *orbit* it
  indefinitely -- the sideways push from raw proximity never turns off
  near a stationary point, so A kept getting deflected before it could
  close the last bit of distance to arrive. Fixed by projecting the
  other robot onto the mover's straight-line path and only reacting when
  it's both close *and* roughly ahead (`avoidance_target`'s perpendicular-
  distance-and-"along" check) -- once the mover has drawn level with or
  passed the obstacle, it's no longer "ahead" and the push stops on its
  own, instead of persisting forever against something that isn't moving.
- Even with that fixed, giving A a steering nudge at all is risky in a
  way B's isn't: A's route was verified clear of every wall once, at
  plan time, and never rechecked. Nudging it sideways for *any* reason,
  including dodging B, can push it directly into a wall it has no idea
  is there (this actually happened right at the corridor's mouth,
  wedging A against the wall segment it was never routed through).
  B doesn't have this problem because B *does* replan against the real
  grid, so a shifted position is still a validated one next tick. The
  fix was to only ever steer B -- A drives its original, verified route
  with zero deflection, exactly as its "never reacts to B" design already
  promised. B alone ends up carrying three independent, complementary
  layers of collision avoidance (grid-level blocking, per-frame steering,
  and the hard distance stop), which turns out to be enough: nothing
  needs A's cooperation to stay clear of it.

**Why the layout changed from a corridor to an intersection.** All four
bugs above were found and fixed against the original "single corridor,
goal(A) == start(B)" layout, and the fixes made that layout genuinely
collision-free. But the coincidence itself kept generating new edge
cases even after each individual bug was fixed -- A's goal and B's start
(and vice versa) being the *same physical point* meant the two robots'
zones of relevance always overlapped near both ends of the run, not just
at the corridor. The more robust fix wasn't another patch, it was
removing the coincidence: the current layout is a real cross-street
intersection (four quadrant buildings, a 3-wide open street each way)
where all four points -- both starts, both goals -- are different cells,
and the only place the two robots' paths have any reason to come near
each other is the crossing in the middle. The coordination policy above
(priority, staging, steering, safety stop) didn't need to change at all
to move to this layout -- it was already layout-agnostic -- which is
itself a decent sign it was the right level to fix things at.

**A fifth bug, found only after that move: a slow turn-in-place
controller.** The corridor layout never exposed this, because A happened
to spawn already facing the direction it needed to drive (straight down
the one row that mattered). The intersection layout doesn't: A spawns
facing its URDF's default heading and has to turn 90 degrees before it
can drive south at all. `Robot.drive_toward` picked the turn rate with
plain proportional control (`wz = yaw_error * 4`) for *any* size of
error, including a full 90-degree one -- and proportional-only control
decays multiplicatively, never linearly, so large errors shrink very
slowly at first. Measured directly: it took over 130 simulation steps
(0.5+ seconds) of turning in place before the heading error dropped
enough to start driving forward at all, during which A sat motionless at
its exact spawn point. B, meanwhile, started moving immediately (its own
first turn was small), so by the time A finally got going B already had
a half-second head start -- enough that their paths, despite crossing at
the exact center of the grid on paper, missed each other by 7+ meters in
practice. Fixed with a two-phase turn controller in `pybullet_app/sim3d/robot.py`:
turn at the full `turn_speed` while the heading error is large
(`TURN_EASE_THRESHOLD = 0.5` rad), and only switch to proportional easing
close to the target heading, where smooth settling actually matters and
the correspondingly larger gain (raised from 4 to 12) no longer risks
overshoot. After the fix, A starts translating within about 0.15s instead
of 0.5+, and the two robots' closest approach at the crossing drops from
7.8m (never really interacting) to under 2m.

**A sixth issue, once that gap was mostly closed: a leftover, smaller
timing asymmetry.** Fixing the turn controller closed most of the gap,
but not all of it -- min approach distance was still around 1.9m,
suspiciously identical whether or not the local avoidance layer was even
active, which was the tell that avoidance wasn't the thing determining
the outcome. The remaining cause: B's very first waypoint direction
(east) happens to match r2d2's default spawn heading exactly, so B
*also* needed zero turning, while A -- even with the fixed controller --
still needed a real (if now fast) 90-degree turn. That's a second,
smaller version of the identical head-start problem, from the same root
cause: nothing about either robot's spawn orientation was ever chosen to
match its route, both times by accident rather than design. Confirmed
by disabling avoidance entirely and measuring the "natural" closest
approach: 1.97m with default spawn headings, 0.44m once both robots are
explicitly spawned already facing their first direction of travel
(`baseOrientation` passed to `loadURDF`, computed from each route's
initial heading) -- a genuinely synchronized crossing, both routes being
equal length at equal speed. With that fixed and the local avoidance
layer re-enabled, the closest approach lands around 0.5m (r2d2's own
footprint radius is about 0.17m, so actual contact would need centers
within roughly 0.34m -- 0.5m is close, not a graze). At that distance
`SAFETY_STOP_RADIUS` (tightened from 1.0m to 0.55m specifically to allow
this) engages: B's trail visibly curves as it approaches, then holds
still for a beat exactly as A crosses in front of it, then resumes --
confirmed consistent across five repeated runs in each smoothing mode
(this simulation has no randomness anywhere, so identical inputs
reliably reproduce the same near-miss, not just "usually").

**A seventh issue: steering narrowed the crossing distance but didn't
reliably prevent contact, so it was replaced with replanning entirely.**
The 0.5m result above still relied on `SAFETY_STOP_RADIUS` actually
catching every case -- a single hard distance check, unconditional but
still just one layer, with no margin for physics-step timing variance
(a real position update happens once per 1/240s step; the check only
sees where a robot already is, not where it's about to be). In practice,
runs of the live simulation showed the two robots actually making
contact -- the steering nudge reduced how often the hard stop needed to
save the day, but it didn't change what happens in the cases where it
doesn't quite. Steering was always a patch on top of a fixed plan, not a
plan itself; the fix was to stop treating "how do I not hit it" as a
steering problem and go back to what the rest of this project already
does well: when you detect something new, replan. Both robots now run
the identical symmetric policy above -- detect the other's current cell,
treat it as a temporary obstacle, replan a real A*-verified route around
it, every `REPLAN_PERIOD_S = 0.05` s (tightened from the corridor
layout's 0.2s specifically so the block tracks a fast-moving robot's
cell before it's already moved through it). The printed waypoints during
a run show this working directly -- e.g. robot A's route visibly bends
through row 13 and back for a few replan ticks while B is in the way,
then straightens back to a direct line the moment the block clears. The
result: closest approach settles around 1.6m, consistently, across
repeated runs in every smoothing mode -- not as dramatic a near-miss as
the steering version's 0.5m, but the actual point was "don't collide,"
and a route that's re-verified against the real grid every time it
changes is a fundamentally more trustworthy way to guarantee that than
a steering offset ever was.

### Scale benchmark (`nav/scale_benchmark.py`)

`nav.grid.Grid` gained an optional `size` parameter (defaulting to the
usual 25, so every existing caller is unaffected) specifically so this
benchmark could reuse the exact same `Grid`/`dijkstra`/`astar`/`rrt` code
at 20x20 through 200x200 instead of writing a parallel implementation.

The headline number: going from 400 to 40,000 cells (100x) grows
Dijkstra's runtime 141x, A*'s only 49x, and RRT's 620x. The A* number is
the interesting one -- its cells-explored *as a fraction of the grid*
actually shrinks as the grid grows (15.4% at 20x20, down to 7.0% at
200x200), because a heuristic search's effort tracks the distance
between start and goal, which doesn't grow as fast as total grid area
does. RRT's numbers are the cautionary one: even after scaling its
`step_size` and `max_iters` up with grid size specifically to keep the
comparison fair, its completeness still degraded under scale (8/8 at
20x20 and 50x50, down to 7/8 at 100x100, 5/8 at 200x200) -- because the
actual bottleneck (an unindexed linear scan over every tree node to find
the nearest one each iteration) gets worse the more the tree has to grow
to cross a bigger space, and no amount of parameter tuning removes an
`O(n)`-per-iteration search itself. Full numbers and the specific worst
case (a 200x200 trial that grew a 1,504-node tree and still never found
the goal) are in `benchmark_results/scale_writeup.md`.

## Advanced planners: k-d tree, RRT*, D* Lite, CBS

Four additions on top of the four planners above, each targeting a
specific, already-measured weak point: RRT's O(n) nearest-neighbor scan
(the scale benchmark's own conclusion), plain RRT's lack of an
optimality guarantee, every replanning scenario's "start over from
scratch" cost, and the two-robot coordination policy's inability to
generalize past two agents.

### k-d tree for RRT's nearest-neighbor search (`nav/kdtree.py`)

`nav/rrt.py`'s `_nearest` used to be `min(nodes, key=...)` -- a linear
scan over every node in the tree, every single iteration. Both
`benchmark_results/writeup.md` and `benchmark_results/scale_writeup.md`
had already identified this as RRT's actual bottleneck, not obstacle
density or grid size directly: an `O(n)`-per-iteration search means total
search cost grows *faster* than linearly in how large the tree gets, and
a bigger space needs a bigger tree to cross it.

`KDTree` (`nav/kdtree.py`) is a standard 2D k-d tree, built incrementally
-- `insert(point)` one node at a time, exactly how RRT grows its tree --
supporting both `nearest(point)` (RRT's per-iteration query) and
`within_radius(point, radius)` (RRT*'s neighbor-radius query, see below;
one data structure serves both new planners). No rebalancing: a naive
recursive k-d tree can degrade toward a linked list under an adversarial
insertion order (e.g. points fed in sorted order), but RRT's insertion
order -- each new node steered toward a uniformly random sample -- is
nowhere near adversarial, so this stays close enough to balanced without
the maintenance a general-purpose incremental k-d tree would need.
Verified against a brute-force linear scan over 2,000 random points: 500
nearest-neighbor queries and 100 radius queries, zero mismatches.

**The before/after, rerunning `nav/scale_benchmark.py` on the identical
code otherwise:**

| Size | RRT (linear scan, before) | RRT (k-d tree, after) | Speedup | Completeness (same, either way) |
|---:|---:|---:|---:|---:|
| 100x100 | 116.716ms | 15.162ms | **7.7x** | 7/8 |
| 200x200 | 258.820ms | 58.548ms | **4.4x** | 5/8 |

Completeness is identical in both columns -- the k-d tree changes
nothing about *what* RRT finds, only how fast it finds it, exactly as
expected from replacing one implementation of the same query with a
faster one. Full table (all four sizes) and discussion in
`benchmark_results/scale_writeup.md`.

### RRT*: rewiring for asymptotic optimality (`nav/rrt_star.py`)

Plain RRT connects every new node to its single nearest existing
neighbor and never revisits that decision -- `benchmark_results/writeup.md`
measured the cost of that at 9.1% average path-length overhead versus
the optimal grid-search path. RRT* (Karaman & Frazzoli) fixes this with
two changes, both implemented in `nav/rrt_star.py`:

1. **Cheapest parent, not nearest parent.** When adding a new node, look
   at every existing node within `neighbor_radius` (a k-d tree
   `within_radius` query) and connect to whichever gives the lowest
   total cost-to-come, not whichever happens to be geometrically
   closest -- as long as the straight edge is collision-free.
2. **Rewire nearby nodes through the new one.** After adding it, check
   those same nearby nodes again: if routing through the just-added node
   is now cheaper than a node's current parent, switch its parent. This
   is the step plain RRT has no equivalent of, and it's what lets
   earlier, locally-suboptimal connections get corrected as the tree
   fills in -- the actual mechanism behind "asymptotically optimal."

Unlike `rrt()`, `rrt_star()` never stops early at the first node that
reaches the goal radius -- it keeps iterating for the entire budget,
since later rewiring can still improve a path found early. `neighbor_radius`
is a fixed multiple of `step_size` (`RRT_STAR_NEIGHBOR_FACTOR = 2.0`)
rather than the textbook shrinking-ball formula (`gamma * (log n / n) **
(1/d)`) -- a common practical simplification, traded for simplicity at
the cost of not being the asymptotically tightest possible radius.
Rewiring also does *not* cascade a cost improvement to a rewired node's
own descendants (a full implementation would) -- a rewired node's own
cost is always correct, its descendants' just may lag until *they*
happen to get rewired directly. Both are documented, deliberate
simplifications, not oversights.

One real correctness hazard that *is* fully handled: rewiring a node `n`
to point through the just-added node would create a cycle in the tree
(and an infinite loop in `reconstruct`) if `n` happens to already be one
of that new node's own ancestors -- geometrically close in Euclidean
distance but topologically far away in the tree. `_is_ancestor` walks
the parent chain before every rewire to rule this out; without it, a
sufficiently winding tree could eventually loop.

Wired into `pygame_app/visualizer.py` as a fourth selectable algorithm (`T` key,
alongside `D`/`A`/`R`) -- same explored-region suppression and tree-edge
drawing RRT already gets, extended to check for `"rrt_star"` everywhere
`"rrt"` was special-cased. Verified end to end with a headless pixel-
level render (same technique the moving-obstacle integration used): a
real RRT* run drawn to an off-screen pygame surface, confirmed non-white
pixels actually appear where the tree and path should be.

**Benchmarked head-to-head against plain RRT on the identical 20 grids
`benchmark_results/writeup.md` already used, with both algorithms fed the
*identical* random-sample sequence per trial** (same seed,
`random.Random(trial_num * 1000 + attempt)`, so any difference in the
result is attributable to the algorithm, not to random variance between
separate runs):

- **RRT* produced a shorter path in 20/20 trials -- never longer, never
  tied -- averaging 25.2% shorter**, ranging from 0.9% (a trial where
  RRT's own tree already grew a fairly direct route) to 69.5% (the trial
  `benchmark_results/writeup.md` already flagged as RRT's worst case,
  where its path was literally double the cardinal-optimal length --
  RRT*, given the identical samples, closes almost all of that gap).
- **The cost is runtime, and it's a real, structural one:** RRT*
  averaged 67.1ms per trial against RRT's 0.97ms -- almost 70x slower on
  the identical 3,000-iteration budget. Most of that isn't extra work
  per sample (parent selection/rewiring over a handful of nearby nodes,
  cheap even with the k-d tree); it's that RRT* never stops early the
  way plain RRT does the instant it reaches the goal, so it spends its
  *entire* budget on every trial where plain RRT often used a small
  fraction of its own.

Full numbers, the comparison against the cardinal-only "optimal" path
(where RRT*'s lack of a movement-direction constraint makes it come out
*shorter* than optimal, for the same reason plain RRT sometimes does --
not a contradiction, see the writeup), and the complete honest takeaway
are in `benchmark_results/writeup.md`.

### D* Lite: incremental replanning (`nav/dstar_lite.py`)

Every planner up to this point solves from scratch, every single call --
which is exactly what happens today, over and over, in
`nav/obstacles.py`'s moving-obstacle replanning, `nav/sensor.py`'s
discovery-triggered replanning, and both pybullet multi-robot/sensor
demos. D* Lite (Koenig & Likhachev, 2002) instead keeps one persistent
search around and repairs it incrementally when the grid changes.

**The mechanism.** The search runs backward, from `goal` outward, and
tracks two costs per cell: `g(s)` (best known cost-to-goal) and `rhs(s)`
(a one-step lookahead, `min` over neighbors of `edge_cost + g(neighbor)`).
A cell is "consistent" when `g == rhs`; only inconsistent cells ever sit
on the open queue. When an edge's cost changes, only the cells whose
`rhs` could actually depend on that edge go inconsistent -- everywhere
else on the grid, previously computed `g` values stay exactly as valid
as before. Since the search is anchored at the fixed goal rather than
the moving start, the robot advancing one cell doesn't invalidate
anything either -- it only needs a single scalar correction (`km`, the
heuristic distance moved since the last query) to keep the priority
queue correctly ordered, not a fresh search. Grid adjacency being
symmetric (true even with the diagonal corner-cut rule -- the same two
corner cells get checked regardless of which direction you're moving
between two cells) is what lets one function (`_neighbor_cells`) serve as
both the successor set (for computing `rhs`) and the predecessor set
(for deciding what to re-examine after a change), with no separate
"reverse graph" needed.

**Correctness**, checked exhaustively rather than just on a couple of
hand-picked cases (`nav/scratch/dstar_lite_test.py`): 40/40 random grids
match `astar`'s path cost exactly on a fresh, one-shot plan, and across
15 trials of 15 steps each (robot advances, a random cell's obstacle
state flips, D* Lite repairs incrementally) -- 207/207 individual steps
match a completely fresh `astar` recomputation from the robot's exact
current position on the exact current grid. Every single step, not just
the final result.

**Does it actually replan faster? Benchmarked on both named scenarios
specifically** (`nav/replan_benchmark.py`, recreating `nav/obstacles.py`'s
bouncing `MovingObstacle` and `nav/sensor.py`'s `LidarSensor`/`KnownGrid`
discovery, including pygame_app/visualizer.py's exact replan-trigger condition
for the sensor case), swept across grid size the same way
`nav/scale_benchmark.py` does, since this project's actual interactive
grid is a fixed 25x25 and the honest answer turned out to depend on
scale:

| Size | Moving obstacle speedup | Sensor discovery speedup |
|---:|---:|---:|
| 25x25 | **2.42x** | **0.49x** (slower) |
| 50x50 | **2.90x** | **0.63x** (slower) |
| 100x100 | **8.49x** | **1.31x** |
| 200x200 | **16.34x** | **1.97x** |

**Moving obstacle: D* Lite wins at every size, growing fast** -- a single
bouncing obstacle only ever invalidates a small, localized neighborhood
each time it moves, exactly the case incremental repair is built for.

**Sensor discovery is the more honest result: D* Lite is *slower* at
this project's actual 25x25 scale**, only becoming a net win at 100x100
and above. Two real reasons, not artifacts: sensor discovery can reveal
several newly-blocked cells in one event (unlike one bouncing obstacle),
so each replan touches more vertices; and D* Lite's own per-vertex
bookkeeping (heap push/pop with lazy deletion, several dict lookups, a
full neighbor scan per vertex update) is genuine Python-level constant-
factor overhead that a small grid's already-cheap `astar` search doesn't
have enough cost to amortize away. That overhead matters less as the
grid grows, because a from-scratch search's cost keeps climbing with
grid size while the number of vertices a single sensor update actually
touches doesn't. **This is the same "wrong tool at this project's actual
scale, right tool at a bigger one" shape of conclusion
`benchmark_results/writeup.md` already reached for plain RRT** -- arrived
at independently, for a structurally different reason (constant-factor
overhead here, an `O(n)` search there). Full discussion in
`benchmark_results/replan_writeup.md`.

`KnownGrid` (`nav/sensor.py`) gained an optional `size` parameter to make
this benchmark possible at all -- it previously always built a fixed
25x25 grid regardless of the underlying grid's actual size (fine for
every existing caller, which only ever runs at that default), the same
gap `Grid` itself had before `nav/scale_benchmark.py` needed a `size`
parameter added for exactly this reason.

### Conflict-Based Search for 3+ robots (`nav/cbs.py`, `pybullet_app/pybullet_cbs_main.py`)

`pybullet_app/pybullet_multi_robot_main.py`'s two-robot policy -- each robot treats the
*other's* current cell as a temporary obstacle and replans around it,
symmetrically -- has no clean symmetric extension past exactly two
agents: with three or more, whose cell does agent A treat as blocked
when B and C are both nearby and might each move differently depending
on what A does? Real multi-agent conflicts are inherently joint, not
pairwise-reactive. A fully joint search over N agents' combined state
space is exponential in N (roughly `(cells)^N`) and intractable past two
or three agents on this project's 625-cell grid -- CBS (Sharon, Stern,
Felner & Sturtevant, 2012) avoids that by searching a tree of
*constraints* instead, only ever running single-agent searches.

**The mechanism**, fully described in `nav/cbs.py`'s module docstring:
plan every agent independently first (no constraints); find the first
conflict between any two agents' paths (`first_conflict` -- either a
*vertex* conflict, both at the same cell at the same time, or an *edge*
conflict, two agents swapping cells between consecutive timesteps, which
a vertex check alone would miss entirely); branch into two child nodes,
each forbidding one of the two conflicting agents from that cell/edge at
that time and replanning *only that agent* (every other agent's path is
reused unchanged); repeat on the lowest-total-cost node in the queue
until one has no conflicts left. The low-level search itself is
time-expanded A* -- state is `(cell, time)`, not just `cell`, since a
constraint like "can't be here at time 7" is meaningless without a time
axis, and an explicit wait-in-place action is what lets one agent yield
to another instead of being forced into head-on contact.

**Correctness**, verified independently of CBS's own termination
condition (`nav/scratch/cbs_test.py`): 2, 3, and 4-agent crossings on the
same 4-way intersection layout, each solution re-checked from scratch
against `first_conflict` (the same function CBS uses internally, so a
pass here means that function's "no conflicts left" answer is
trustworthy) plus per-agent assertions that every path starts at its
start, ends at its goal, only takes legal single-cell steps, and never
crosses an obstacle. All conflict-free.

**Scales past two agents in practice, with the expected caveat about
worst-case complexity.** On the intersection grid, cycling robots through
its four compass arms and (past four) its three street lanes: 4 agents
solve in 42ms, 5 in 42ms, 6 in 388ms, all genuinely conflict-free. A
deliberately adversarial stress case -- 8 agents, all four arms, two
robots per arm, all crossing at once with only two of the three lanes
used -- failed to find a solution within a 3,000-node budget (2.4s spent
trying). That's CBS's well-documented worst-case behavior showing up
under maximal contention, not a bug: the constraint tree can grow
exponentially when conflicts keep cascading into new conflicts, the same
way a joint search would, just deferred to a smaller subset of hard
cases instead of showing up on every instance.

**pybullet_app/pybullet_cbs_main.py drives N robots through the CBS plan in lockstep**
-- the one real complication continuous 3D simulation adds that the
discrete algorithm doesn't have to consider. CBS's guarantee ("no two
agents at the same cell at the same *timestep*") is a claim about a
discrete clock; if every robot just drove its own waypoints
independently at whatever speed it individually achieved, real-world
timing would drift from the plan's discrete clock and the guarantee
would stop applying. Every robot advances to its next per-timestep
waypoint only once *every* robot has reached its current one -- an
agent that's supposed to wait one timestep keeps "reporting ready"
every physics tick without advancing, until the rest of the fleet
catches up, at which point they all advance together. Paths are driven
raw, with no corner-cutting/spline smoothing (unlike
`pybullet_app/pybullet_main.py`/`pybullet_app/pybullet_multi_robot_main.py`), since smoothing
would shift where along the path a robot actually is at a given moment
-- exactly the synchronization lockstep driving exists to preserve.

**Does the discrete guarantee actually survive translating into
continuous physics?** Measured directly, not assumed: tracking every
pair of robots' real Euclidean distance throughout a full lockstep run,
the closest approach across the whole simulation was 0.46m, for both a
4-robot and a 6-robot run -- comfortably above the ~0.34m two r2d2
footprints (radius ~0.17m each, per the two-robot demo's own numbers)
would need to actually touch. Zero contact, confirmed by measurement,
not inferred from the discrete plan being conflict-free on paper.

## The FTC sensor-suite study: pose error vs. obstacle error

Everything above is a domain-neutral belief-planning toolkit. `ftc/` is
where it gets pointed at one specific, answerable question: *which
sensing investment actually buys reliability in a 30-second FTC
autonomous period, and at what level of field/reality deviation does
each one become necessary?* See README.md's "Research question" and
"nav/ vs ftc/" sections for the framing and the reason the FTC-specific
code lives in its own package instead of leaking into `nav/`.

**The central modeling distinction this study is built around: not all
deviation is the same kind of deviation.** A robot can be wrong about
where *it* is (pose error -- it started a little off its mark, or its
wheels slipped over the course of the run) or wrong about what the
*field* looks like (obstacle error -- a game element sits somewhere
other than the CAD says, or an opponent robot parked somewhere
unplanned). Nothing about a sensor suite's marketing tells you which
one it fixes. `ftc/sensors.py`'s five suites split cleanly along that
line: `OdometryPodSuite` and `AprilTagSuite` only ever touch pose error
(`fixes_pose = True`, `senses_obstacles = False`); `DistanceSensorSuite`
only ever touches obstacle error (the reverse); `FullSuite` is the only
one that touches both; `DeadReckoningSuite` touches neither and is what
every FTC team already has for free. `nav/field_variance.py`'s Phase 2
ablation (`start_drift_scale` / `obstacle_drift_scale` / `blocker_scale`,
added on top of the pre-existing bundled `variance_level` knob) exists
specifically so a sweep can isolate one deviation type at a time instead
of only ever seeing their combined effect -- without that split there'd
be no way to explain *why* a given suite wins or loses, only that it
does.

**Pose error is modeled as a continuous drift, not a one-shot offset.**
nav/'s own uncertainty study (`nav/uncertainty_benchmark.py`) only ever
applies a single fixed start-position offset per trial, via
`OpenLoopPolicy`'s `_offset` mechanic. A real robot's pose estimate
keeps drifting for the whole match, and can be *corrected* mid-match by
a suite that senses something absolute (an AprilTag). `ftc/match.py`
generalizes the offset mechanic into a continuous vector `error` such
that `true_position = believed_position + error`: every cell of real
travel nudges `error` by a suite-specific `drift_per_cell` (Gaussian,
matching the ftc/config.py-documented physical reasoning that
wheel-encoder slip accumulates with distance traveled, not with the
clock), and every successful AprilTag detection shrinks it back down by
`APRILTAG_CORRECTION_FACTOR`. Planning and obstacle-sensing both happen
entirely in the robot's own *believed* frame -- exactly what a real
robot does, since it only ever has its own possibly-wrong idea of where
it is -- and the resulting motion command gets translated into the true
frame by the *current* `error` before being checked against ground
truth. That's also why `ftc/scratch/match_test.py` has to reach for
`PLANNING_OVERHEAD_S`-patching rather than a straight elapsed_s
comparison across suites to prove replanning costs real time: two
suites that replan a different number of times also, in general, drive
different paths, so a naive "the one that replanned more took longer"
comparison is confounded by route length and can point the wrong way.

**The headline numbers** (25 trials x 5 suites x 3 deviation types x 11
deviation levels, `ftc/suite_benchmark.py`, full methodology and tables
in `benchmark_results/ftc_suite_writeup.md`): FullSuite has the highest
raw success rate (56% at variance_level >= 0.3, averaged across all
three deviation types), but AprilTag -- the cheapest suite that fixes
anything at all -- has more than double FullSuite's success-rate gain
per dollar spent over the free DeadReckoningSuite baseline. Which
deviation type actually dominates a given suite's failures depends on
what that suite fixes: DeadReckoningSuite's worst failure mode is pose
error (start drift), the thing a $40 AprilTag setup targets directly,
not obstacle error.

**The most useful result is the negative one, and it very nearly got
mis-attributed.** DistanceSensorSuite collides in roughly half its
trials even at variance_level=0.0 -- ground truth cell-for-cell
identical to the assumed map, every deviation type at exactly zero. The
first-draft writeup blamed this on a SLAM-style consistency problem
(sensed-obstacle positions recorded in the robot's believed frame going
stale as pose error drifts between detection and use) -- a real,
plausible-sounding mechanism, and one this project's own nav/ study
already has a documented precedent for (BeliefPolicy's nonzero collision
rate at variance_level=0.0, `benchmark_results/uncertainty_writeup.md`).
It was wrong, or at least not the dominant cause: a controlled check
(the same trials, `drift_per_cell` forced to 0 so pose error can't be a
factor at all) showed roughly two-thirds of the collisions persisting
anyway. The real, dominant cause is geometry -- `DISTANCE_SENSOR_COUNT`
narrow ToF cones (`DISTANCE_SENSOR_HALF_ANGLE_DEG` half-angle each,
mounted front/left/right) cover only about 75 of the 360 degrees around
the robot; anything in the remaining ~285-degree gap, a very plausible
place for an obstacle to sit relative to a robot mid-turn on a diagonal
grid, is simply never seen until the next planned step walks straight
into it. `ftc/scratch/sensors_test.py`'s `check_cone_sensor_stays_in_cone`
exists specifically to keep this honest going forward -- it rings a
sensor with obstacles at every cell in range and asserts every single
reported detection actually falls inside a mount's cone, so this
blind-spot finding can never quietly become an artifact of a sensor
model that secretly sees more than it claims to. The corrected finding
is blunter than the original SLAM-consistency story: a sparse fixed-cone
suite has real, geometry-driven blind spots a full lidar-style disc scan
(`nav/sensor.py`'s `LidarSensor`, which `ReactivePolicy` never collides
with) doesn't have, and buying distance sensors without covering enough
of the robot's perimeter can be worse than not sensing at all.

**Calibration (`ftc/calibration.py`) and the decision tool
(`ftc/recommend.py`)** are the other half of making `variance_level`
mean something. Every chart above is indexed by an arbitrary [0, 1]
number; `ftc/calibration.py` fits the corresponding real-world
components from two kinds of measurement (nominal-vs-actual field-
element positions, for obstacle_drift; measured dead-reckoning drift
over a real 30-second run, for a pose-drift rate and, from that, a
start_drift-equivalent level) rather than requiring you to guess where
your own field/robot sits on that axis. It ships a clearly labeled
synthetic placeholder dataset so `ftc/recommend.py` has something to run
against immediately, and every single output -- from `Calibration.
describe()`'s per-component `[REAL MEASURED DATA]` / `[SYNTHETIC
PLACEHOLDER]` tags on up through `ftc/recommend.py`'s CLI banner --
states plainly which one it's looking at. No number produced by either
tool should be read as "the real answer" until real measurement CSVs
have actually been dropped in; see both modules' docstrings for the
exact CSV column contract.
