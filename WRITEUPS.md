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

- start == goal: trivially return a length-0 path without searching.
- start on an obstacle: the raw algorithms don't check this -- since
  neither `dijkstra` nor `astar` ever tests whether the *start* cell itself
  is passable (only whether cells you move *into* are), a start planted on
  an obstacle would silently search as if it weren't blocked. Caught and
  reported explicitly as `"start_blocked"`.
- goal on an obstacle: `get_neighbors` never returns an obstacle cell
  as a neighbor, so a goal sitting on one is simply unreachable -- the
  search would exhaust itself and report `"no_path"`, which is technically
  true but a worse message than `"goal_blocked"`.
- no path exists: the priority queue empties without ever popping the
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

Observed effect on RRT: none. This RRT implementation has no notion
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

- `LidarSensor`: `sense(grid, position)` reveals every real obstacle
  within `radius` cells of `position` (Euclidean, not Chebyshev -- a
  circle, not a square) and adds it to `known_obstacles`, a set that only
  ever grows. It returns just the *newly* seen obstacles, which is what
  the caller needs to decide whether to replan.
- `KnownGrid`: a `Grid` subclass built entirely from
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

Known simplification: `known_obstacles` only grows -- a cell once
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

- False negative (`NOISE_MISS_RATE`, default 0.15): a real obstacle
  in range isn't detected this scan.
- Position noise (`NOISE_POSITION_RATE`, default 0.15): a detected
  obstacle is reported at a random *adjacent* cell (2D: one of its 8
  neighbors; 3D: the raycast hit point nudged by up to
  `POSITION_JITTER_METERS` before being converted to a cell) instead of
  its true one.
- False positive (`NOISE_FALSE_POSITIVE_RATE`, default 0.02): a free
  cell (2D) or a ray that hit nothing (3D, which hallucinates a phantom
  hit at a random point along that ray instead) gets "detected" as an
  obstacle that isn't there.

Both default to `noisy=False`, so every existing caller, test, and demo
keeps its exact current deterministic behavior unless it explicitly asks
for noise -- nothing above needed to change to stay true.

Why a single noisy reading can't just be trusted the way a perfect
one was. `known_obstacles` still means exactly what it always did --
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

Does this actually matter, concretely, or is it a theoretical nicety?
Measured, not asserted (`nav/scratch/lidar_noise_test.py`,
`pybullet_app/scratch/pybullet_lidar_noise_test.py` -- both scanning a hidden
obstacle repeatedly from a fixed position, then replanning against
`confirmed_obstacles` at increasing thresholds and comparing the
resulting path cost to the true optimum):

| min_detections | 2D corridor test | 3D wall test |
|---:|---|---|
| 1 (= raw `known_obstacles`) | FAILED outright (`start_blocked`) | cost 34.00 (+13% vs optimal) |
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

A confirmation threshold does not turn noisy sensing back into perfect
sensing, and it's worth being precise about exactly how it falls short
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

Wiring: both `pygame_app/visualizer.py` (`N` key, alongside `S` for the
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
breaks in practice. First finding, which took a wrong turn to get to: a "perfect"
maze (recursive backtracking, no loops) has exactly one route between any
two cells. There's nothing for a bad heuristic to get wrong when there's
only one possible path -- `nav/scratch/heuristic_experiment.py` originally
ran on maze grids and found 0/8 suboptimal results across every heuristic,
including the inflated one, simply because there was no alternate route to
wrongly prefer. Suboptimality needs *competing* routes of different
lengths, which only shows up with scattered obstacles (loops, open areas),
not a spanning-tree maze. Switching the test bed to random 35%-density
obstacle grids was necessary before the experiment meant anything.

Second finding: even on grids with real alternate routes, 1.5x almost
never broke optimality in 25 trials (0/25). It did roughly halve the
number of cells explored, though -- a real speed win for zero cost in these
cases. Cranking the inflation to 3x did break it, in 2/25 trials, e.g.:

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
The correct heuristic for a `sqrt(2)`-cost diagonal is octile distance:
`max(dr, dc) + (sqrt(2) - 1) * min(dr, dc)`. Since `sqrt(2) - 1 < 1`,
octile distance is always <= Manhattan distance (equal only when `dr` or
`dc` is 0), meaning Manhattan systematically overestimates the true cost
whenever a diagonal shortcut exists -- it's inadmissible the moment
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

1. `simplify_collinear` -- A* on a 4-directional grid emits one
   waypoint *per cell*, so a straight 10-cell run is 10 collinear points.
   Collapsing runs like that to their two endpoints (cross-product test:
   `(p1-p0) x (p2-p0) == 0` means collinear) gives the smoothers below a
   handful of real corners to work with instead of dozens of redundant
   knots along every straight stretch. Not smoothing by itself -- pure
   cleanup before smoothing.
2. `chaikin_smooth` -- simple corner-cutting, done first because it's
   the cheapest thing that could possibly work. Each pass replaces every
   corner with two points 1/4 and 3/4 of the way along its adjacent
   edges; repeated a few times this rounds every corner into a curve.
   Cheap, easy to reason about, and the result only *approaches* the
   original path -- except the two endpoints, which are kept exact here
   (unlike the textbook version of Chaikin, which cuts those too) so the
   robot still starts and ends in the actual start/goal cell instead of
   near it.
3. `catmull_rom_spline` -- the follow-up upgrade once corner-cutting
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

- Rays can't originate at the robot's own center. A ray that starts
  literally inside (or on the surface of) the sensing robot's own
  collision shape hits *itself* first, every time, regardless of
  `ignore_body_id` -- rayTestBatch reports the first hit along the ray,
  and the robot's own hull is that hit. Fixed by starting each ray
  `ORIGIN_OFFSET = 0.35` m out from center, in the ray's own direction,
  rather than at the exact center point.
- A hit lands exactly on a cell boundary, and rounding that is
  ambiguous. An obstacle box's near face sits at, e.g., `x = 8.5` for
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

Coordination policy (current version -- see the bug list below for two
earlier versions that didn't hold up):

- Both robots replan, symmetrically. Every `REPLAN_PERIOD_S = 0.05` s,
  each one runs A* against the real grid *plus* a block placed around the
  *other's* current cell (`cell_block`, a 1-cell buffer) -- the exact
  same technique the pygame `MovingObstacle` replanning logic used
  (mark the moving thing as a temporary wall, replan around it), just
  with a robot standing in for the moving obstacle on both sides at once.
  A replan is only actually issued when the blocked cells changed or the
  last attempt found nothing (`agent.waiting or current != last`) --
  replanning unconditionally every tick was an earlier bug (see below),
  and the fix generalizes cleanly to both robots being symmetric now.
- If a route genuinely isn't there, the blocked robot holds position
  (`waiting = True`) and retries on the next replan tick, rather than
  crashing on `None` or driving into a wall.
- Neither robot gets a local steering nudge anymore. An earlier
  version had one robot (B) steer its aim point sideways every physics
  step when the other got close, faster-reacting than the 0.05s replan
  cadence. It's gone now -- see the bug entry below for why steering
  turned out to be the wrong mechanism entirely, not just something that
  needed better tuning.

Why symmetric replanning doesn't deadlock here, unlike the old
corridor layout. A *symmetric* "both treat the other as an obstacle"
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

A hard safety-distance stop is layered on top, deliberately not relied
on as the primary mechanism: replanning runs every `REPLAN_PERIOD_S`,
not every physics tick, so a fast robot could in principle close
real-world distance in the gap between replans. If the two robots'
actual distance ever drops below `SAFETY_STOP_RADIUS`, both are forced to
stop that exact frame regardless of what their plans say. It exists as a
failsafe against replanning latency, the same role an emergency stop
plays underneath a real path planner -- and it checks real physical
distance unconditionally, every step, with no exceptions (see the
collision bug below for what happened when it briefly wasn't
unconditional).

One scenario-design bug worth recording: the first version of this
demo had robot A permanently parked exactly on top of robot B's goal
cell after "arriving," because goal(A) == start(B) by construction (a
swap-sides scenario makes that coincidence unavoidable) and a robot that
finishes just sits at its exact goal position forever. B's own
destination was therefore permanently blocked by A's corpse, and B never
finished. Fixed by nudging an arrived robot 1.5m off to the side
(`NavAgent._park`) and excluding arrived robots from the other's blocked-
cell set entirely -- once a robot is done, it stops being an obstacle for
anyone.

A second bug, found by watching a live run rather than just the
end-of-run pass/fail: even after that fix, B looked stuck for several
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

A third bug: the safety stop had a loophole that caused a real
collision. The distance check was originally `(not agent_a.arrived)
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

A fourth bug, from the version of this demo that gave the priority
robot (A) its own avoidance steering too: the first version of the
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

Why the layout changed from a corridor to an intersection. All four
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

A fifth bug, found only after that move: a slow turn-in-place
controller. The corridor layout never exposed this, because A happened
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

A sixth issue, once that gap was mostly closed: a leftover, smaller
timing asymmetry. Fixing the turn controller closed most of the gap,
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

A seventh issue: steering narrowed the crossing distance but didn't
reliably prevent contact, so it was replaced with replanning entirely.
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

The before/after, rerunning `nav/scale_benchmark.py` on the identical
code otherwise:

| Size | RRT (linear scan, before) | RRT (k-d tree, after) | Speedup | Completeness (same, either way) |
|---:|---:|---:|---:|---:|
| 100x100 | 116.716ms | 15.162ms | 7.7x | 7/8 |
| 200x200 | 258.820ms | 58.548ms | 4.4x | 5/8 |

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

1. Cheapest parent, not nearest parent. When adding a new node, look
   at every existing node within `neighbor_radius` (a k-d tree
   `within_radius` query) and connect to whichever gives the lowest
   total cost-to-come, not whichever happens to be geometrically
   closest -- as long as the straight edge is collision-free.
2. Rewire nearby nodes through the new one. After adding it, check
   those same nearby nodes again: if routing through the just-added node
   is now cheaper than a node's current parent, switch its parent. This
   is the step plain RRT has no equivalent of, and it's what lets
   earlier, locally-suboptimal connections get corrected as the tree
   fills in -- the actual mechanism behind "asymptotically optimal."

Unlike `rrt()`, `rrt_star()` never stops early at the first node that
reaches the goal radius -- it keeps iterating for the entire budget,
since later rewiring can still improve a path found early. `neighbor_radius`
is a fixed multiple of `step_size` (`RRT_STAR_NEIGHBOR_FACTOR = 2.0`)
rather than the textbook shrinking-ball formula (`gamma * (log n / n) ** (1/d)`) -- a common practical simplification, traded for simplicity at
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

Benchmarked head-to-head against plain RRT on the identical 20 grids
`benchmark_results/writeup.md` already used, with both algorithms fed the
*identical* random-sample sequence per trial (same seed,
`random.Random(trial_num * 1000 + attempt)`, so any difference in the
result is attributable to the algorithm, not to random variance between
separate runs):

- RRT* produced a shorter path in 20/20 trials -- never longer, never
  tied -- averaging 25.2% shorter, ranging from 0.9% (a trial where
  RRT's own tree already grew a fairly direct route) to 69.5% (the trial
  `benchmark_results/writeup.md` already flagged as RRT's worst case,
  where its path was literally double the cardinal-optimal length --
  RRT*, given the identical samples, closes almost all of that gap).
- The cost is runtime, and it's a real, structural one: RRT*
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

The mechanism. The search runs backward, from `goal` outward, and
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

Correctness, checked exhaustively rather than just on a couple of
hand-picked cases (`nav/scratch/dstar_lite_test.py`): 40/40 random grids
match `astar`'s path cost exactly on a fresh, one-shot plan, and across
15 trials of 15 steps each (robot advances, a random cell's obstacle
state flips, D* Lite repairs incrementally) -- 207/207 individual steps
match a completely fresh `astar` recomputation from the robot's exact
current position on the exact current grid. Every single step, not just
the final result.

Does it actually replan faster? Benchmarked on both named scenarios
specifically (`nav/replan_benchmark.py`, recreating `nav/obstacles.py`'s
bouncing `MovingObstacle` and `nav/sensor.py`'s `LidarSensor`/`KnownGrid`
discovery, including pygame_app/visualizer.py's exact replan-trigger condition
for the sensor case), swept across grid size the same way
`nav/scale_benchmark.py` does, since this project's actual interactive
grid is a fixed 25x25 and the honest answer turned out to depend on
scale:

| Size | Moving obstacle speedup | Sensor discovery speedup |
|---:|---:|---:|
| 25x25 | ~1x (a tie) | ~0.6x (slower) |
| 50x50 | ~1.9x | ~0.7x (slower) |
| 100x100 | ~3.7x | ~1.5x |
| 200x200 | ~6.7x | ~2.3x |

These are wall-clock microbenchmarks and the small-grid figures move a
few tens of percent run to run, so they're quoted approximately here;
`benchmark_results/replan_writeup.md` carries the exact numbers from the
most recent run and is regenerated automatically by
`python3 -m nav.replan_benchmark`. What's stable across runs is the
*shape*: a tie at 25x25 on moving obstacle, a clear loss at 25x25 on
sensor discovery, and both crossing over as the grid grows.

Moving obstacle: a dead heat at this project's actual 25x25 scale, then
D* Lite pulls away fast as the grid grows -- a single bouncing obstacle
only ever invalidates a small, localized neighborhood each time it
moves, exactly the case incremental repair is built for, but at 25x25
that repair costs about what a fresh search does.

Sensor discovery is the starker result: D* Lite is outright *slower* at
this project's actual 25x25 scale, only becoming a net win at 100x100
and above. Two real reasons, not artifacts: sensor discovery can reveal
several newly-blocked cells in one event (unlike one bouncing obstacle),
so each replan touches more vertices; and D* Lite's own per-vertex
bookkeeping (heap push/pop with lazy deletion, several dict lookups, a
full neighbor scan per vertex update) is genuine Python-level constant-
factor overhead that a small grid's already-cheap `astar` search doesn't
have enough cost to amortize away. That overhead matters less as the
grid grows, because a from-scratch search's cost keeps climbing with
grid size while the number of vertices a single sensor update actually
touches doesn't. This is the same "wrong tool at this project's actual
scale, right tool at a bigger one" shape of conclusion
`benchmark_results/writeup.md` already reached for plain RRT -- arrived
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

The mechanism, fully described in `nav/cbs.py`'s module docstring:
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

Correctness, verified independently of CBS's own termination
condition (`nav/scratch/cbs_test.py`): 2, 3, and 4-agent crossings on the
same 4-way intersection layout, each solution re-checked from scratch
against `first_conflict` (the same function CBS uses internally, so a
pass here means that function's "no conflicts left" answer is
trustworthy) plus per-agent assertions that every path starts at its
start, ends at its goal, only takes legal single-cell steps, and never
crosses an obstacle. All conflict-free.

Scales past two agents in practice, with the expected caveat about
worst-case complexity. On the intersection grid, cycling robots through
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

pybullet_app/pybullet_cbs_main.py drives N robots through the CBS plan in lockstep
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

Does the discrete guarantee actually survive translating into
continuous physics? Measured directly, not assumed: tracking every
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

Formally, `nav/occupancy.py` + `BeliefPolicy`'s "plan against an
expected-cost map instead of an assumed-true one" is a tractable
approximation of belief-space planning: instead of planning a
single path through the one grid the robot assumes is real, plan
through the space of *distributions over grids* the robot's sensor
history is consistent with. The fully general version of that problem
-- acting optimally under both state uncertainty and future observation
uncertainty -- is a POMDP (partially observable Markov decision
process), which is famous for being intractable to solve exactly at any
real-world scale. `OccupancyGrid`'s log-odds update sidesteps that
intractability by not actually solving the POMDP: it collapses the full
distribution over grids down to one number per cell (the marginal
occupancy probability, tracked independently per cell) and plans a
single path against the resulting expected-cost map, rather than
reasoning jointly over the exponentially large space of ways the whole
grid's true state and the robot's future observations of it could
unfold together. That's a real approximation, not a free lunch -- see
`benchmark_results/uncertainty_writeup.md` and the DistanceSensorSuite
discussion below for two separate, concrete ways a per-cell-independent
belief can still get a robot into trouble a jointly-reasoned one
wouldn't.

The central modeling distinction this study is built around: not all
deviation is the same kind of deviation. A robot can be wrong about
where *it* is (pose error -- it started a little off its mark, or its
wheels slipped over the course of the run) or wrong about what the
*field* looks like (obstacle error -- a game element sits somewhere
other than the CAD says, or an opponent robot parked somewhere
unplanned). Nothing about a sensor suite's marketing tells you which
one it fixes. `ftc/sensors.py`'s original five headline suites split
cleanly along that
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

Pose error is modeled as a continuous drift, not a one-shot offset.
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
a fraction that itself degrades with range and viewing obliquity (see
`AprilTagSuite.tag_correction` and `ftc/config.py`'s
`APRILTAG_CORRECTION_FACTOR_MAX`/`APRILTAG_RANGE_DEGRADATION`/
`APRILTAG_ANGLE_DEGRADATION`) rather than a flat constant applied
uniformly whenever a tag merely clears the range/FOV/line-of-sight
gates -- real fiducial pose estimation doesn't stay equally good
everywhere inside that envelope, and the headline reliability-per-
dollar result (AprilTag as the best-value suite) was specifically
re-checked against this more pessimistic model rather than assumed to
survive it; it did, by roughly the same margin. Planning and
obstacle-sensing both happen
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

The headline numbers (25 trials x 5 suites x 3 deviation types x 11
deviation levels, `ftc/suite_benchmark.py`, full methodology and tables
in `benchmark_results/ftc_suite_writeup.md`): Odometry pods has the
highest raw success rate (30% at variance_level >= 0.3, averaged across
all three deviation types -- FullSuite isn't even the runner-up
anymore, landing behind both AprilTag and Rear camera at 16%), but
AprilTag -- the cheapest suite that fixes anything at all -- has over
40 times FullSuite's success-rate gain per dollar spent over the free
DeadReckoningSuite baseline. Which
deviation type actually dominates a given suite's failures depends on
what that suite fixes: DeadReckoningSuite's worst failure mode is pose
error (start drift), the thing a $25 AprilTag setup (a single Logitech
C270 -- see ftc/config.py's APRILTAG_COST_USD) targets directly,
not obstacle error.

The most useful result is the negative one, and it very nearly got
mis-attributed. DistanceSensorSuite collides in the large majority
(77%) of its trials even at variance_level=0.0 -- ground truth cell-for-cell
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
suite has real, geometry-driven blind spots, and buying distance
sensors without covering enough of the robot's perimeter can be worse
than not sensing at all. `ftc/coverage_benchmark.py` (see below) later
found this blind spot isn't just currently unmet but structurally
unclosable by any FTC-legal ToF sensor count -- there is no purchasable
full-360-degree option in this project's model to compare against at
all, lidar-class hardware not being legal FTC equipment.

Calibration (`ftc/calibration.py`) and the decision tool
(`ftc/recommend.py`) are the other half of making `variance_level`
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

### Does the headline finding depend on the field layout? (`ftc/layout_benchmark.py`)

The headline sweep above runs on `ftc/field.py`'s `'cluttered'` layout
only -- deliberately, per `ftc/suite_benchmark.py`'s own docstring, as
"the richest test of both obstacle-sensing suites ... and the hard-
footprint-inflation routing" this project's grids add over `nav/`'s
point-robot ones. But "richest test" is also exactly the kind of
layout choice that could be doing unacknowledged work in a result:
would AprilTag still be the best-value suite on a field with almost
nothing to route around, or one with a single forced bottleneck instead
of a scatter of obstacles?

`ftc/layout_benchmark.py` answers that by rerunning the *identical*
full-rigor sweep -- all 11 `variance_level` steps, all 3 deviation
types, all 25 trials/point, nothing reduced -- on `'sparse'` and
`'corridor'` as well, reusing `ftc/suite_benchmark.py`'s own
`run_combo`/`aggregate`/`overall_success_rate` rather than
reimplementing the sweep logic a second time. It deliberately does not
touch `ftc/suite_benchmark.py` or anything it writes; the headline
numbers this README and the conference poster cite come from a module
this one only ever imports from, never modifies.

The result: AprilTag is the best-value suite on all three layouts.
Its per-$100 margin over the runner-up is actually *wider* on sparse
and corridor (+78.4 and +63.4pp/$100) than on cluttered (+8.0pp/$100)
-- with fewer or more concentrated obstacles to route around, obstacle-
sensing suites like DistanceSensorSuite and the obstacle-sensing half of
FullSuite have less to buy, while AprilTag's pose-only fix keeps paying
off regardless of what the obstacles look like. Full per-layout tables
and the reliability-per-dollar chart are in `benchmark_results/
ftc_layout_writeup.md` and `ftc_layout_comparison.png`.

What this does and doesn't prove. This closes the "one field layout"
question for the three layouts `ftc/field.py` actually ships -- it does
not prove the finding survives an arbitrary future layout (a real
season's surveyed field, dropped in as its own `FieldLayout`, per that
module's docstring), only that it isn't an artifact of the specific
"cluttered" choice this repo happened to headline with. A built-in
consistency check (`ftc/layout_benchmark.py`'s `'cluttered'` pass reuses
`ftc/suite_benchmark.py`'s exact seed formula) confirms the two modules
agree trial-for-trial on what "cluttered" means, which
`ftc/scratch/layout_benchmark_test.py` asserts directly rather than
leaving as an unverified claim in a docstring.

### How wrong would the estimated constants have to be? (`ftc/robustness.py`)

The headline recommendation (AprilTag as the best-value suite) rests on
`ftc/config.py`'s own documented "ballpark engineering estimates," not
measurements -- drift rates, AprilTag's correction-quality falloff, and
every suite's cost. `ftc/robustness.py` sweeps each one independently
from 0.25x to 4x its estimated value (a reduced sweep -- 15 trials/point
at 4 variance levels instead of the headline's 25/11 -- since a
tipping-point search needs "did the ranking flip," not a publication-
grade curve at every multiplier) and asks whether the best-value suite
actually changes.

The gotcha this module's own docstring leads with: `SensorSuite.
drift_per_cell` and `.cost_usd` are CLASS attributes, evaluated once at
import time from `ftc.config`'s values -- patching `ftc.config.
DEAD_RECKONING_DRIFT_PER_CELL` after `ftc.sensors` has already been
imported does *nothing* to a freshly-constructed suite (verified
directly in `ftc/scratch/robustness_test.py`'s
`check_config_patch_does_nothing`, which confirms the broken approach
really is broken rather than just documenting a scare story). The fix
is an INSTANCE-attribute override on each constructed suite object,
which Python's normal instance-shadows-class lookup honors without
touching `ftc.config` at all. `AprilTagSuite.tag_correction` has the
same trap one level deeper: it reads `APRILTAG_CORRECTION_FACTOR_MAX`
etc. as `ftc.sensors` MODULE globals (bound there by `ftc/sensors.py`'s
own `from ftc.config import ...`), so varying those requires patching
the attribute directly on the `ftc.sensors` module object, not
`ftc.config`. A sweep built on the naive monkeypatch would run cleanly,
print plausible numbers, and measure nothing at all -- exactly the
failure mode a passing test suite can hide.

The result has changed materially twice since `ftc/config.py`'s sensor
costs and AprilTag's detection range were first corrected against real
vendor/FTC-doc sources (AprilTag's cost dropped from an unsourced ~$40
to a real Logitech C270's $25; its range grew from an understated 6ft
to FTC's own documented 10ft figure) -- for a time, AprilTag's margin
over every other suite was wide enough that nothing swept tipped the
ranking anywhere in [0.25x, 4x]. Fixing `ftc/field.py`'s own grid-
boundary bug (README.md's "Threats to validity") reintroduced several
statistically-clean tipping points; fixing the newer rotated-footprint-
collision gap (also "Threats to validity" -- both change which free
cells every scenario samples start/goal from) moved the ground again,
this time toward apparent tips that turn out NOT to survive a
statistical check: dead-reckoning drift rate nominally tips the
recommendation to Rear camera at 2.0x its estimated value, Odometry
pods' cost nominally tips it to Odometry pods at 0.25x, and AprilTag's
own cost nominally tips it to Rear camera at 4.0x -- but all three are
"maybe, not confirmed": the new winner's confidence interval still
overlaps the old one's at that multiplier in every case, plausibly
sampling noise from only 15 trials/point at this project's now-lower
absolute success rates. Odometry-pod drift rate, AprilTag's own
max-correction parameter, AprilTag's range/angle degradation, Distance
sensors' cost, and Full suite's cost never tip the ranking anywhere in
[0.25x, 4x] at all. Tipping points -- and whether they survive a
significance check -- are sensitive to exactly the kind of underlying-
model correction this project keeps making, not a property fixed once
and for all -- rerunning `ftc/robustness.py` after any future constant
(or model) correction is the way to find out whether they've moved
again, not assuming a past run still applies. Full table and
plain-language summary in `benchmark_results/ftc_robustness_writeup.md`,
chart in `ftc_robustness.png`.

What this does and doesn't prove. A tipping point bounds how wrong
an estimate can be before the conclusion changes -- it says nothing
about whether the *real* value is inside or outside that bound. This
BOUNDS the "uncalibrated variance" limitation (README.md's "Threats to
validity"); it does not CLOSE it. Only `ftc/calibration.py` run against
real measured field/robot data closes it.

### Does the 30-second budget ever actually bind? (`ftc/budget_benchmark.py`)

`ftc_suite_writeup.md`'s own "Honest findings" section flagged this as
open: no suite ran out of `AUTONOMOUS_PERIOD_S` anywhere in the headline
sweep, which made the budget model arguably decorative -- `ftc/
match.py` charges elapsed time carefully (drive, turn, per-replan
overhead) for a constraint nothing had ever actually hit.
`ftc/budget_benchmark.py` sweeps the budget downward (30s to 1.5s,
patching `ftc.match.AUTONOMOUS_PERIOD_S` -- a plain module global
`run_match` looks up fresh on every call, following the exact pattern
`ftc/scratch/match_test.py` already established, unlike `ftc/sensors.py`'s
class-attribute trap) and reruns the full-rigor sweep at every point.

The finding depends on which of this project's own priorities has run
first: under the *original* naive drive-time model (distance /
`MAX_DRIVE_SPEED_MPS`, instantaneous acceleration), the budget barely
bound anywhere in the prompt's own example range (30s down to 7s) --
median match time under 2 seconds even at high field deviation. Once
`ftc/match.py` gained a trapezoidal acceleration profile (see below),
every step got charged more realistically and the same sweep now finds
the budget starts binding at 10s, with a real (not
noise-explainable) ranking change at very tight budgets: Full suite
overtakes Odometry pods as the #1 suite by raw success rate at 4s.

A "which suites replan more, and therefore feel a tight budget first"
hypothesis is checked against measured `avg_replans`/trial data rather
than assumed from suite category -- the first draft of this module
assumed the replan-heavy suites were the obstacle-sensing ones
(DistanceSensorSuite, FullSuite, since they replan on every newly-sensed
obstacle). That assumption was incomplete: AprilTag also replans on
every successful pose correction (`ftc/match.py`'s `replan_needed = not
planned_once or tag_corrected`), and empirically out-replans
DistanceSensorSuite (0.69 vs. 0.32 replans/trial at the real 30s
budget) despite never sensing obstacles at all. But at the tightest
budget actually tested (1.5s), it isn't AprilTag that has the highest
over-budget rate -- it's Odometry pods (77%), a suite that replans 0.00
times/trial on average. This does NOT confirm the replan-heavy-suites-
degrade-first hypothesis at all: `PLANNING_OVERHEAD_S` isn't the
dominant cost near the budget edge for Odometry pods -- total
`elapsed_s` (drive + turn time over whatever path length that suite's
own pose/obstacle error forces) matters at least as much as replan
count. Full table, replan-count data, and the ranking-change CI check
in `benchmark_results/ftc_budget_writeup.md`, chart in
`ftc_budget_comparison.png`.

This CLOSES the "budget rarely binds" limitation for the range actually
swept -- it's no longer an untested claim that the budget model is
decorative, it's a measured curve showing exactly where it stops being
decorative and what changes when it does.

### A real (moving) opponent, not a fixed point (`ftc/opponent_benchmark.py`)

`unplanned_blocker` (the existing deviation type) drops one STATIC
obstacle on the planned route, once, before the match starts, and
leaves it there -- an opponent with no motion and no reaction to this
robot's presence. `nav/obstacles.py` already has `MovingObstacle` (a
seeded random walk used by `pygame_app`'s interactive visualizer);
`ftc/opponent_benchmark.py` reuses it completely unmodified rather than
writing a second one, and adds a `moving_blocker` deviation type
alongside the existing static one rather than replacing it, so the two
can be compared directly.

The integration detail that had to be solved carefully: `MovingObstacle.
tick(grid, now_ms, blocked)` is driven by millisecond WALL-CLOCK timing,
while `ftc/match.py` accrues SIMULATED `elapsed_s` that has no fixed
relationship to real time. `run_match` gained an optional
`moving_obstacles` parameter (default `()`, so every existing caller --
every priority before this one -- is completely unaffected) that ticks
each obstacle once per loop iteration with `now_ms = elapsed_s *
1000.0`: simulated match time, not wall-clock time, so a given seed
always reproduces the exact same sequence of obstacle moves regardless
of how fast the process executes. `ftc/scratch/opponent_benchmark_test.py`
checks this with a spy object that records every `now_ms` it's ticked
with, rather than depending on a real `MovingObstacle` happening to have
a free neighbor and the match happening to run long enough for a real
move to occur (an earlier version of the test relied on that scenario
luck and flaked when a match ended in collision before the obstacle's
first scheduled tick).

Both the static and moving paths use the identical probability gate
(`min(variance_level, 1.0)`) and identical candidate-cell selection (an
interior cell of the assumed start->goal path, via the same
`nav.algorithms.astar` call `nav/field_variance.py`'s own
`_place_unplanned_blocker` uses) -- verified directly in
`ftc/scratch/opponent_benchmark_test.py` by seeding both functions
identically and confirming they choose the same cell, not just assumed
from reading the code.

The result: a moving opponent DOES appear to change which suite is the
best value, though not cleanly. Distance sensors is the best-value
suite against a static blocker (+10.1pp/$100, off a 6% free-baseline
floor) -- the FIRST time in this project that obstacle-sensing suite
has won a best-value comparison outright, since a parked blocker on the
planned route is exactly the failure mode it exists to catch. Against a
moving blocker, AprilTag (front camera) edges out Rear camera
(+26.0pp/$100 vs. +12.0pp/$100), but the margin is close enough that
the two suites' success-rate confidence intervals still overlap at this
trial count -- treat "AprilTag beats Rear camera against a moving
opponent" as plausible, not confirmed. What doesn't need hedging:
neither winner is Full suite, and Distance sensors's static-blocker win
isn't close. This is a different, messier finding than an earlier run
of this same module produced, when AprilTag's cost and detection range
were both under-modeled (an unsourced ~$40 and an understated 6ft,
corrected against a real Logitech C270 price and FTC's own documented
10ft AprilTag detection range respectively) and, later, than a version
measured before `ftc/field.py`'s grid-boundary and rotated-footprint-
collision bugs were fixed (README.md's "Threats to validity") -- each
correction has genuinely moved which suite this comparison names,
which is itself the lesson: this specific ranking is not settled
science, it is conditional on the model underneath it exactly as much
as the headline sweep is. More surprising, and consistent across every
version of this study so far: suites that never sense obstacles at all
(dead reckoning, odometry, AprilTag) still do substantially better
against a moving opponent than a static one, purely from timing luck --
a parked obstacle sits on a blind suite's fixed route for the entire
match, so a plan that ever crosses that cell collides deterministically;
a wandering obstacle often isn't there anymore by the time that same
fixed plan actually reaches the cell (the largest such gain: Odometry
pods, +31 percentage points). A moving opponent is harder to
*reason about*, but this specific deviation type makes it easier to
*physically avoid* for a suite that isn't reasoning about it at all.
Full tables (including collision/replan counts, which show the
obstacle-sensing suites' *active* version of the same underlying
advantage) in `benchmark_results/ftc_opponent_writeup.md`, chart in
`ftc_opponent_comparison.png`.

This BOUNDS the "no opponent modeling" limitation -- a random walk with
no goals and no reaction to this robot's presence is still a long way
from a real opponent, but it's a measured step up from a fixed point.
A moving vs. static opponent now DOES change the headline-style
recommendation this specific comparison names (Distance sensors static,
AprilTag moving, neither Full suite) -- the useful result lives at both
levels: which suite wins, and how much each suite's raw numbers move
between the two blocker types.

### Trapezoidal drive-time kinematics (`ftc/match.py`'s `_trapezoidal_drive_time_s`)

`ftc/match.py` charged drive time as `distance / MAX_DRIVE_SPEED_MPS`
plus a flat per-90-degree turn cost -- implicitly assuming the robot
reaches top speed instantaneously. `_trapezoidal_drive_time_s` replaces
that with a proper accelerate/cruise/decelerate profile bounded by a new
documented constant, `MAX_ACCEL_MPS2` (`ftc/config.py`, sized the same
ballpark way `TURN_TIME_PER_90DEG_S` already is: 0-to-top-speed in
roughly half a second). Each step is still charged independently,
starting and ending at rest -- the same assumption the existing flat
turn cost already makes at every direction change -- rather than
carrying velocity across consecutive collinear steps, which would need
restructuring how `elapsed_s` accrues across the whole match loop, a
larger change than this deliberately-scoped addition.

The practical consequence: at this project's actual constants (1.5 m/s
top speed, 3.0 m/s^2 acceleration), the distance needed to reach cruise
speed is 0.375m -- well over a single 6in grid cell (0.1524m) or even a
diagonal step (0.2155m). So almost every step in this project's grids
falls in the triangular (never-reaches-cruise) branch of the profile,
not the trapezoidal one, and drive time per step comes out roughly
4-5x the old naive figure. `ftc/scratch/kinematics_test.py` checks both
branches against an independently-derived closed-form kinematics answer
(not just a re-run of the same formula), confirms a real grid step never
reaches cruise speed at this project's constants, and confirms the new
time is always >= the old naive one, never less.

The headline success-rate numbers (as published in `benchmark_results/
ftc_suite_writeup.md` at the time) did not change -- despite drive time
per step roughly quadrupling (median total match time went from
~1.5-2.5s to ~5.3s,
max from single digits to ~21s), no trial in the 4,125-match headline
sweep crossed the 30-second budget, so every success/collision outcome
(governed by pose drift and sensor geometry, not elapsed time) came out
identical. The effect instead shows up entirely in `ftc/
budget_benchmark.py`'s sweep (see above): the *interaction* with a
tighter budget is where realistic kinematics actually changes a
conclusion, exactly as anticipated going into this addition.

### Model fidelity tiers: camera FOV and heading error (`ftc/config.py`'s `MODEL_FIDELITY`)

Two unmodeled optimisms sat underneath every number in this project
until now, both flattering AprilTag specifically (the headline best-
value winner) and every suite generally: `AprilTagSuite.tag_correction`
accepted `heading_deg_now` -- the robot's current heading -- as an
argument and never used it, so a tag was "detected" regardless of which
way the robot's camera actually pointed, as if every AprilTag setup
were an omnidirectional camera; and `ftc/match.py` tracked pose error
as a `(row, col)` translation vector only, with no heading error
anywhere, as if the robot always knew exactly which way it was
pointing. Real dead reckoning doesn't work that way -- small angular
error compounds into large lateral error over distance (a robot 2
degrees off heading drifts about 3.5in laterally over just one field-
length traverse), which is why gyro/heading drift is usually the
*dominant* real-world dead-reckoning failure mode, not a footnote next
to translation drift.

The fix couldn't just be "re-tune the model pessimistically" -- the
replacement parameters (how narrow is a real camera's FOV, how fast
does heading really drift) are themselves uncalibrated ballpark
estimates, exactly the same status as every other constant in `ftc/
config.py`. Silently swapping one set of unvalidated numbers for
another wouldn't have closed anything, and it would have invalidated
the published headline numbers with no way to reproduce them. Instead,
`ftc/config.py`'s `MODEL_FIDELITY` picks between three named tiers,
each with its own values for `CAMERA_FOV_DEG_BY_TIER`, `HEADING_DRIFT_
DEG_PER_CELL_BY_TIER`, `APRILTAG_HEADING_CORRECTION_FACTOR_BY_TIER`,
and `APRILTAG_DETECTION_DROPOUT_RATE_BY_TIER`:

- *optimistic* (the default): camera FOV >= 360deg (omnidirectional),
  zero heading drift, zero AprilTag detection dropout. This isn't a
  new, more honest baseline -- it's the OLD, pre-fix behavior, given a
  name specifically so later tiers can be compared against it.
  `ftc/scratch/fidelity_test.py`'s
  `check_optimistic_tier_reproduces_headline_exactly` reruns a sample
  of the exact seeded trials behind `benchmark_results/
  ftc_suite_results.csv` and diffs every column except
  `planning_time_ms` (unseeded wall-clock time, never a valid
  comparison signal in this project) against the checked-in CSV --
  passing with zero mismatches across all columns checked. This
  regression check is rerun after every constant correction (see
  README.md's "Threats to validity" for the vendor/FTC-doc corrections
  applied to `ftc/config.py`'s sensor costs and AprilTag range) -- the
  headline success-rate numbers have changed since this tier was first
  built, but every time, they've changed because a genuinely wrong
  constant was fixed against a real source, never because the
  optimistic tier's own byte-for-byte reproduction broke.
- *realistic*: a 60deg camera FOV (the Logitech C270/C310's own
  published FOV -- the low end of the cameras FTC's VisionPortal docs
  list as having built-in AprilTag calibrations, and the specific
  camera those docs call the workhorse of the FTC program), nonzero
  heading drift, an 85% AprilTag heading-correction factor, still zero
  detection dropout.
- *pessimistic*: a 45deg FOV -- below every FTC-supported camera's
  nominal spec, standing in for the gap between a camera's nominal FOV
  and its actually-usable one (a tag at the very edge of frame is
  geometrically "in view" but is exactly where lens distortion and
  partial cropping are worst) -- faster heading drift, a lower (70%)
  heading-correction factor, and a nonzero AprilTag detection dropout
  rate (a fraction of otherwise-valid detections just fail to resolve a
  pose that tick, e.g. motion blur).

Two mechanisms had to actually change behavior, not just exist as
config:

1. Camera FOV gating (`ftc/sensors.py`'s `_tag_in_camera_fov`): a NEW
   gate, separate from the existing tag-side FOV check (is the robot
   standing somewhere the tag itself can be read from). This one asks
   whether the robot's own camera, given its current TRUE heading, is
   actually pointed at the tag -- `angle_robot_to_tag` (the direction
   from the robot to the tag) has to fall within `camera_fov_deg/2` of
   one of the suite's `camera_mount_headings_deg`, offset by the
   robot's heading. At `camera_fov_deg >= 360` this returns `True`
   unconditionally without even looking at the heading arguments --
   the optimistic tier's exactness guarantee in code, not just in
   intent. `ftc/scratch/fidelity_test.py`'s
   `check_camera_fov_gate_rejects_tags_behind_the_robot` confirms a tag
   that's otherwise perfectly in range/FOV/line-of-sight gets rejected
   once the robot's camera faces away from it at the realistic tier,
   and `check_camera_fov_gate_is_a_noop_at_optimistic_tier` confirms
   the IDENTICAL scenario still corrects at the optimistic tier.
2. Heading error that actually rotates executed motion, not just a
   reported number (`ftc/match.py`). Pose error grew a third component,
   `heading_error` (degrees), alongside the existing `(row, col)`
   translation `error` -- both still "true = believed + error," the
   translation term added, the heading term *rotating*. Every step, the
   commanded motion is computed as a vector in the believed frame (the
   next grid cell the plan wants, relative to the current one); that
   vector gets rotated by `heading_error` (`ftc/match.py`'s `_rotate`)
   before being applied to the robot's TRUE position -- a wrong heading
   belief doesn't just mis-report where the robot thinks it is, it
   steers the robot somewhere it didn't intend to go, exactly the
   compounding-lateral-error mechanism real dead reckoning has.
   `_rotate` short-circuits to an exact identity at `deg == 0.0` rather
   than computing through `cos`/`sin`, which is what lets
   `heading_error` staying exactly `0.0` at the optimistic tier
   guarantee byte-identical `next_true` cells, not just numerically
   close ones. `ftc/scratch/fidelity_test.py`'s
   `check_heading_error_rotates_execution` confirms both the identity
   case and that a real (25-degree) heading error rotates a hand-built
   step vector to a measurably different cell, then confirms the same
   thing end to end through `run_match`: `final_heading_error_deg`
   stays exactly `0.0` at the optimistic tier across a real match and
   is nonzero at the pessimistic one.

A trap worth naming for whoever touches this next: every new random
draw this feature adds (`rng.gauss` for heading drift, `rng.random()`
for AprilTag dropout) is explicitly guarded behind `if <tier value> >
0:` rather than called unconditionally with a zero-valued parameter.
`random.gauss` consumes the same number of underlying `random()` calls
regardless of `sigma` (it caches every other call's second Box-Muller
value internally), so calling it even with `sigma=0.0` would have
silently shifted every subsequent random draw in the match -- rng
state, elapsed_s, everything downstream -- relative to the pre-
Priority-1 code, breaking the optimistic tier's byte-identical
reproduction guarantee in a way that would have been very hard to spot
(the guarded caller looks correct; only the underlying rng stream
position is wrong). This is exactly why the optimistic-tier regression
test compares against the actual checked-in CSV rather than trusting
the guard was applied everywhere it needed to be.

`ftc/fidelity_benchmark.py` reruns the identical full-rigor headline
sweep at all three tiers side by side. The finding: the best-value
suite does not survive even the first step off optimistic -- AprilTag
(+16.0pp/$100) is best at the optimistic tier, but Odometry pods
(+6.1pp/$100) takes over at realistic and stays there at pessimistic
(+5.8pp/$100) -- only two distinct winners across all three tiers this
time, not three. AprilTag's own overall success rate drops from 18% to
12% (realistic), then holds essentially flat into pessimistic (13%), as
the camera stops being omnidirectional and heading drift starts
mattering; Rear camera's second, rear-facing camera absorbs part of the
same FOV-gating hit but still declines steadily (18% -> 15% -> 14%) --
it no longer overtakes AprilTag on value at any tier the way it once
did, since Odometry pods gets there first this time. Odometry pods
(which fixes pose without ever needing a camera pointed anywhere)
doesn't move at all across tiers (30% at every one) -- unsurprising
given it has no camera-FOV or heading-drift exposure to begin with, and
its own pp/$100 value climbs to the top of the ranking simply because
everything camera-dependent is losing ground around it, not because
Odometry pods itself is doing anything differently. Full suite's own
pp/$100 return stays the lowest of any suite that ever leads a tier
(+0.4, +0.8, +0.5) at every fidelity level -- "best raw performer" and
"best value" remain two different questions regardless of which tier
is in view. Full table in `benchmark_results/ftc_fidelity_writeup.md`.
This BOUNDS the
camera-FOV/heading-error gap in README.md's "Threats to validity" -- it
does not CALIBRATE it; every non-optimistic tier's constants are the
same class of ballpark engineering estimate as everything else in
`ftc/config.py`, not a measured replacement for them.

### New suites: IMU and dual-camera AprilTag (`ftc/sensors.py`)

Two suites only exist, or only matter, because of the fidelity-tier
model above:

`ImuSuite` corrects HEADING error only -- an IMU has no absolute
position reference at all, so translation drift is untouched, same
rate as `DeadReckoningSuite` -- continuously, every tick, with no need
for anything to be "in view" the way an AprilTag detection is. Its
hardware cost is genuinely `$0` (every REV Control Hub already ships
one); the only real cost is integration effort, which this project's
dollar-based cost model has no way to price. `ftc/newsuites_
benchmark.py` reports that case explicitly as "undefined," not
`inf` -- `ftc_suite_writeup.md`'s existing `per_100 = ... if cost > 0
else float("inf")` pattern was written for a suite that's the free
BASELINE (`dead_reckoning`, always excluded from its own ranking); a
free suite that ISN'T the baseline needed a genuinely different
treatment, since "infinite value per dollar" isn't a meaningful claim
about a suite whose real cost is engineering time this project can't
price at all. A heading-only correction also never triggers a replan
(`ftc/match.py` only replans on a position correction or a newly-sensed
obstacle -- a heading fix doesn't change which grid cell the robot
believes it's at), which is the other half of "is the free hardware
worth the code": unlike AprilTag, an IMU fix costs nothing in
`PLANNING_OVERHEAD_S` either. `AprilTagImuSuite` stacks both
corrections (AprilTag's position fix, the IMU's continuous heading fix)
at AprilTag's cost alone.

`DualCameraAprilTagSuite` mounts a second (rear-facing) camera on the
same detection pipeline, priced the same as the first one -- another
Logitech C270, ~$25 more (`ftc/config.py`'s
`DUAL_CAMERA_APRILTAG_COST_USD`). Under the OLD omnidirectional-
camera model this suite would have done *literally nothing* -- a second
omnidirectional camera can't see anything the first one didn't already
see. Under the fidelity-tier model's real FOV gating it roughly doubles
angular tag coverage, and `ftc/scratch/newsuites_test.py`'s
`check_dual_camera_helps_under_realistic_fov_but_not_optimistic`
confirms both halves of that claim directly: a scripted tag placed
behind a robot facing away from it is invisible to the single-camera
suite and visible to the dual-camera one at the realistic tier, and
BOTH suites see it at the optimistic tier (where "behind" doesn't mean
anything to an omnidirectional camera). `ftc/newsuites_benchmark.py`'s
sweep confirms the same shape end to end: a +0% gap at the optimistic
tier, a real +2pp gap at the realistic one -- a clean demonstration
that the Priority 1 fidelity fix is what makes this suite meaningful to
model at all, not just a more expensive AprilTag.

### Tank vs. mecanum drivetrain (`ftc/drivetrain.py`)

`ftc/field.py` has defaulted to `diagonal=True` "on the assumption of a
holonomic drivetrain" since Phase 2, and README.md named "no mecanum-
specific strafing advantage" as an open limitation ever since: every
suite paid the same flat `TURN_TIME_PER_90DEG_S` cost on every
direction change, whether or not the robot it was modeling could
actually translate without turning. `ftc/drivetrain.py` adds `TANK` and
`MECANUM` as a `Drivetrain` axis orthogonal to sensor suite -- any
suite from `ftc/sensors.py` can run on either one, so this is swept
like `ftc/layout_benchmark.py`'s field layout or `ftc/
budget_benchmark.py`'s budget, not folded into `SUITE_ORDER`.

TANK must physically rotate to face its direction of travel before
every direction change -- this IS the existing (pre-this-addition)
behavior, which is why `ftc/match.py`'s `drivetrain=None` default
reproduces it exactly, and an explicit `TANK` instance is required to
be functionally identical to that default
(`ftc/scratch/drivetrain_test.py`'s `check_tank_matches_legacy_default`
verifies this to the bit, on a real seeded match, not just by
inspecting the two code paths). MECANUM can translate in any direction
while holding a fixed chassis heading, paying zero turn cost -- but
runs at `MECANUM_STRAFE_SPEED_FACTOR` (0.8x, ballpark) of forward speed
and accrues `MECANUM_STRAFE_DRIFT_MULTIPLIER` (1.6x, ballpark, wheel
scrub) extra pose drift whenever its direction of travel isn't roughly
"forward" relative to whatever heading it's holding. Both wheel sets
are priced directly from goBILDA's current listed prices at the same
96mm diameter MAX_DRIVE_SPEED_MPS is itself derived from: $39.96 for
four Hogback Traction Wheels ($9.99 each), $169.99 for a goBILDA 96mm
Mecanum Wheel Set.

The heading policy had to be chosen and documented, not left implicit:
MECANUM holds a fixed heading for the whole match, picked once at the
start, aimed at whichever `TagSite` is nearest the robot's true
starting position (`ftc/match.py`, `run_match`). This is the natural
choice for a robot that's investing in an AprilTag-reading camera at
all -- and it's the specific interaction `ftc/drivetrain_benchmark.py`
was built to check: does holding that heading (keeping the camera
aimed at the tag wall for the entire match, instead of swinging away
from it every time the robot changes direction the way TANK's camera
does) recover some of AprilTag's realistic-tier success-rate loss from
the fidelity-tier section above?

The answer, measured rather than assumed: mecanum loses at both tiers.
The camera-FOV theory predicted the tank-vs-mecanum gap for AprilTag
would narrow going from optimistic to realistic (mecanum's camera stays
aimed at the tag wall; tank's swings away every direction change);
measured, it does narrow slightly, from -15 percentage points at
optimistic (19% tank / 4% mecanum) to -13 at realistic (15% / 2%) -- a
small but real move in the direction the camera-FOV mechanism predicts,
nowhere near enough to overcome the strafe penalty at this trial
count. The strafe penalty is the whole story here, not a partial
offset to a real camera-FOV win: a route's travel
direction changes on nearly every leg (up to 8 different directions on
this project's diagonal grid), the held heading is picked once and
never updates, so unless a route happens to run roughly parallel to
whichever tag wall was nearest the start cell, most of its steps are
strafes relative to that heading -- paying the 1.6x drift multiplier on
close to every step, not just the occasional sideways one, and that
extra drift itself degrades AprilTag's own tag-detection geometry (a
drifted position is more likely to fall outside a tag's range/FOV/LOS
envelope) enough to swamp whatever the held heading was supposed to
buy. That compounding drift penalty drags down EVERY suite's success
rate under mecanum, not just AprilTag's (`benchmark_results/
ftc_drivetrain_writeup.md`'s per-suite table), and mecanum's $130
premium over tank is not repaid by ANY suite tested at this trial count
-- not even Odometry pods, which came closest under an earlier revision
of this model but now loses 17 points under mecanum (32% tank vs. 15%
mecanum) same as everything else. This is a real, measured limitation
of the *specific* heading
policy implemented here (hold one heading, chosen once, for the whole
match) -- not a closed verdict on mecanum drivetrains in general. A
policy that re-picks its held heading periodically (toward whichever
tag wall is nearest the CURRENT position, say, or toward a route's own
dominant direction) would strafe far less and isn't tested here; README
records this as CLOSED for "does the drivetrain model itself account
for strafing" and open for "which heading policy should a real team
actually run."

### Closing the loop: two alternative heading policies, actually tested

The paragraph above names its own untested alternative explicitly --
that was the brief for this addition: build the policy it names, and
report whatever it actually does, favorable or not.

`heading_policy` became a real field on `Drivetrain` (`ftc/
drivetrain.py`, default `"fixed_at_start"` -- the exact original
behavior, verified byte-for-byte: `resolve_held_heading_deg` computes
the SAME nearest-tag-from-`actual_start` formula the original code
computed once, just called fresh every tick instead of cached, which
for a pure function of an unchanging input is a no-op, not a
refactor-and-hope). Two new policies sit alongside it: `nearest_tag_
current` re-aims toward whichever tag is nearest wherever the robot
ACTUALLY is right now, recomputed every tick; `route_dominant` aims
along the currently-planned route's own circular mean travel
direction -- a unit vector per remaining leg, summed, converted back to
an angle (the standard way to average angles without wraparound error:
averaging 359deg and 1deg has to give 0deg, not 180deg) -- which
directly targets the quantity `speed_and_drift_factor` actually
penalizes (total strafe against the route this robot is actually
driving), rather than targeting tag visibility as a proxy for it.

Wiring this in meant moving the heading computation from a ONE-TIME
pre-loop calculation to something `ftc/match.py` re-resolves every
tick, from the current position and the current plan -- not a large
change, but one that had to preserve the original default exactly
while making the other two policies live. `ftc/scratch/
heading_policy_test.py` proves the pieces in isolation before trusting
any of this end-to-end: an exact hand-computed circular-mean case
(one east step plus one north step averages to precisely -45deg, not
"something between the two"), a check that `fixed_at_start` genuinely
ignores the robot's current position while `nearest_tag_current`
genuinely tracks it, and a byte-for-byte regression check reproducing
the OLD inline formula independently and diffing against the new
resolver's output on a real scenario.

The result, from `ftc/drivetrain_benchmark.py`'s second sweep
(`benchmark_results/ftc_drivetrain_heading_policy_writeup.md` --
deliberately a SEPARATE study with its own output files and its own
base seed, so the original `ftc_drivetrain_writeup.md` numbers already
cited above and in README.md stay byte-for-byte untouched): `route_
dominant` is a real, paired-bootstrap-significant improvement over
`fixed_at_start` (7% -> 9% pooled success rate at the `realistic`
fidelity tier) -- re-aiming genuinely helps, confirming the mechanism
the original writeup predicted but had no policy to demonstrate.
`nearest_tag_current`, on the other hand, is NOT a significant
improvement (7% -- exactly tied with the 7% baseline at this
rounding): optimizing for tag visibility doesn't reliably reduce strafe
against wherever the robot is actually trying to go, which is a
different target than `route_dominant` optimizes for. Neither policy
closes the gap to tank (18%), and neither changes which sensor suite is
the best
buy on mecanum -- Odometry pods stays the best-value suite under every
heading policy tested, AprilTag never recovers the lead it holds on
tank. Re-aiming helps; it does not flip either headline verdict.
Reported at that strength, not rounded up to "mecanum's problem is
solved" or down to "re-aiming doesn't matter" -- both would have been
wrong.

A separate, full-rigor question neither sweep above is designed to
answer: does the *headline* best-value recommendation itself (AprilTag,
measured under the default tank-equivalent drivetrain) survive on
mecanum, for every suite, not just AprilTag? `ftc/
drivetrain_suite_benchmark.py` answers it the same way `ftc/
layout_benchmark.py` answers the equivalent question for field layout
-- rerunning `ftc_suite_writeup.md`'s exact full-rigor sweep (all 11
levels, all 3 deviation types, 25 trials/point, nothing reduced) once
per drivetrain, for all 7 headline suites. The "tank" pass reproduces
`ftc_suite_results.csv` trial-for-trial (the same consistency check
`ftc/layout_benchmark.py` runs for its own "cluttered" pass), so the
"mecanum" pass is a genuinely paired comparison, not a separately-tuned
guess. The answer: no -- the best-value suite is drivetrain-dependent,
AprilTag under tank but Odometry pods under mecanum. Every suite's raw
success rate drops substantially on mecanum regardless, from -8 points
(Distance sensors) up to -15 (AprilTag (front camera) and Rear camera,
the largest drops), and this time the RANKING of which suite is worth
its price changes too, not just the raw numbers. See `benchmark_results/
ftc_drivetrain_suite_writeup.md` for the full per-suite table.

### Sensor coverage: can you buy out the distance-sensor blind spot? (`ftc/coverage_benchmark.py`)

`ftc_suite_writeup.md`'s strongest negative finding never answered the
obvious follow-up question: DistanceSensorSuite's 3 narrow ToF cones
cover only ~75 of the 360 degrees around the robot and collide in the
large majority of trials (77%) even at zero field deviation, but does buying
MORE sensors actually fix that, or is a sparse fixed-cone suite doomed
regardless of count? `ftc/coverage_benchmark.py` answers directly:
sweeping `DISTANCE_SENSOR_COUNT` over {3, 4, 6, 8} (`ftc/sensors.py`'s
`make_distance_sensor_suite`, an instance-override factory -- see `ftc/
robustness.py`'s docstring for why this has to be instance-level, not a
class or `ftc.config` mutation, and `ftc/scratch/coverage_test.py`'s
`check_suite_override_takes_effect` for the test written to fail if it
weren't).

Mount-heading placement is documented per count in `ftc/config.py`,
since even coverage vs. front-weighted is itself a real design choice,
not an afterthought: 3 stays front/left/right (the existing headline
layout, front-weighted toward the direction of travel); 4 adds a rear
sensor for full cardinal coverage; 6 and 8 switch to EVEN spacing
(60deg and 45deg respectively) once there are enough sensors that
picking a side to leave uncovered stops making sense.

The result: more coverage measurably helps, and there's a hard ceiling
on how far that goes. Going from 3 to 8 sensors drops the
zero-deviation collision rate from 87% to 80% -- confirming the
mechanism is real -- but even 8 sensors, the largest count this project
prices, only covers 200 of 360 degrees; a 160-degree blind arc survives
no matter how many of these specific sensors get bought, because each
one only ever adds its own narrow cone and never closes a gap faster
than it opens a new one at its own edge. An earlier version of this
study used a full-360-degree disc-scan suite (a "lidar" option) as the
direct head-to-head this comparison originally wanted -- that comparison
is gone, not because it stopped being interesting, but because
lidar-class hardware isn't legal FTC equipment and this project no
longer models it anywhere. The honest replacement question is sharper,
not weaker: "can you buy full coverage" now has a clean NO for every
option this project can legally price, not just an unmeasured maybe.
That the improvement tops out well short of eliminating collisions
even at the highest coverage tested is itself
consistent with `ftc_suite_writeup.md`'s own controlled check, which
already found that roughly a third of DistanceSensorSuite's collisions
persist even with pose drift completely disabled -- coverage angle is
the DOMINANT cause of the zero-deviation collisions, not the only one.

### Drivetrain speed / gearing (optional, `ftc/gearing_benchmark.py`)

The lowest-priority, explicitly optional addition: `MAX_DRIVE_SPEED_MPS`
and `MAX_ACCEL_MPS2` were plain constants, but once `ftc/
budget_benchmark.py` showed `AUTONOMOUS_PERIOD_S` genuinely starts
binding at 10s under the trapezoidal kinematics model, "buy a
faster motor" became an actually testable purchase for the first time
-- a robot that never runs out of time has nothing to gain from more
speed, and prior to that finding this would have been a pure paper
exercise. `ftc/config.py`'s `GEARING_OPTIONS` ("stock"/"fast"/"faster")
is built directly from goBILDA's own published 5203-series RPM/torque
table (`ftc/match.py`'s `run_match` gained an optional `gearing`
parameter, `None`/`"stock"` reproducing `MAX_DRIVE_SPEED_MPS`/
`MAX_ACCEL_MPS2` exactly): stock is the 19.2:1 ratio (312 RPM, 338
oz-in) `MAX_DRIVE_SPEED_MPS`/`MAX_ACCEL_MPS2` are themselves derived
from; "fast" is 13.7:1 (435 RPM, 260 oz-in); "faster" is 5.2:1 (1150
RPM, 109 oz-in) -- a genuine catalog option, included as a deliberate
far-end anchor, not a configuration this project recommends. Every
ratio costs the same $54.99 per motor regardless of speed, so a 4-motor
gearing swap is $219.96 whichever ratio is picked -- what a team buys
with a "faster" ratio's identical price is nothing; the tradeoff is
paid entirely in torque, via `slip_factor` scaling `drift_per_cell` up.

An early version of this used invented speed/accel multipliers that
both moved the SAME direction (faster AND harder-accelerating), which
is backwards for a real gearmotor: torque falls as RPM rises for a
fixed motor. Rebuilding `GEARING_OPTIONS` from goBILDA's actual spec
table surfaced a second, non-obvious consequence: every option's
accel-to-cruise distance (`speed^2 / (2*accel)`) is far larger than one
6in grid cell (stock: 0.375m; fast: 0.948m; faster: 15.8m), so a single
step ALWAYS falls in `_trapezoidal_drive_time_s`'s triangular branch --
`2*sqrt(distance/accel)` -- where top speed never enters the formula at
all, only acceleration does. Since real motor torque falls faster than
RPM rises across the 5203 lineup, a faster gearing choice is strictly
SLOWER per short hop in this model, not faster, on top of costing more
drift. This is not a speed-vs-slip tradeoff; it is a lose-lose at
FTC's typical short-hop distances -- a real, sourced finding, not a bug
to route around.

`ftc/scratch/gearing_test.py` verifies this two ways: directly against
`_trapezoidal_drive_time_s`'s own closed-form kinematics (the same
style `ftc/scratch/kinematics_test.py` already established) -- "faster"
gearing's own accel value produces a strictly LONGER single-cell drive
time, not shorter -- and end to end through `run_match`, with a
drift-zeroed suite instance so the comparison isn't confounded by a
subtler, real effect this addition surfaced: more slip changes the
accumulated pose error, which changes which cells actually get visited
and how much turning happens, which can shift a *stochastic* multi-step
match's total elapsed_s in either direction independent of the
per-step kinematics. An early version of the end-to-end check assumed
faster gearing WAS faster per cell and compared full, undoctored
matches directly; once the motor-spec correction reversed that
assumption, the same test was rewritten to check (and confirm) the
opposite -- no gearing option ever rescues a budget stock already
fails, since none of them save per-cell time at this grid scale.

The result, crossed with budget (30s -- the real, non-binding budget;
15s -- right at the binding point per `ftc/budget_benchmark.py`; 10s --
binds hard), averaged across all 7 headline suites: faster gearing
loses at every budget tested, by a wide and CI-clean margin (roughly
-5 percentage points at 30s, widening to roughly -7 at 15s and -7
at 10s --
exact figures in `benchmark_results/ftc_gearing_writeup.md`, which
regenerates them fresh each run). Unlike the pre-correction version of
this finding, the mechanism is not "slip cost outweighs a real time
saving" -- there was never a time saving to weigh it against.

What this does and does not prove: this is a reduced-rigor sweep,
averaged across every suite rather than reported per suite -- a team
running a specific suite that already fixes pose (AprilTag, odometry
pods) would plausibly absorb the extra slip-driven drift better than
dead reckoning does, and this module doesn't check that per-suite
breakdown. The per-cell-slower finding itself is not a ballpark
estimate -- it follows directly from goBILDA's own published RPM/torque
table and this project's own fixed grid cell size, not from a tuned
constant. `slip_factor` remains the one number in `GEARING_OPTIONS`
that is an explicit ballpark engineering estimate: no vendor publishes
slip-vs-gearing data.

## The bundle optimizer: from "which suite" to "which combination"

Every study above this one compares a fixed list of suites. That list
was always the real limitation, and it took a while to see it: seven
suites is a *comparison*, not a *search*, and one of the seven
(`FullSuite`) is a hand-written class that happens to hardcode one
particular combination of three others. There are eight suites in
`ftc/sensors.py` now. Nobody was going to hand-write the other
combinations, so nobody could ask whether any of them were worth
buying.

`ftc/bundle.py` composes them on demand instead. The interesting part
wasn't the composition -- OR the capability flags, take the min of the
drift rates, union the obstacle sensors -- it was the two things that
would have quietly produced wrong answers if I'd done the obvious
thing.

The first is cost. Suites price themselves with a flat `cost_usd`, and
summing those over a bundle double-counts every shared part:
`AprilTagSuite` ($25, one webcam) plus `AprilTagImuSuite` ($25, the
same webcam and a free IMU) is not a $50 robot. It's a $25 robot,
described twice. So a bundle is costed over the *union of its parts*
(`ftc/config.py`'s `PART_COSTS_USD`), which does more than fix the
arithmetic: it gives every bundle a part signature, and two bundles
with the same signature are the same purchase. 63 raw combinations of
7 suites collapse to 23 genuinely distinct robots, and the search never
pays to simulate the same robot twice or offers a team two names for
one option. A "cost model" that started as a bookkeeping fix turned
into the deduplication key for the whole search.

The second is that composition had to be *exact*, not approximate. If a
bundle of {distance sensors, AprilTag, odometry pods} isn't
byte-for-byte `FullSuite`, then every number the optimizer prints lives
in a slightly different universe from the published headline results
and the two can't be compared. `ftc/scratch/bundle_test.py` enforces
that: a one-suite bundle reproduces that suite's `MatchResult` exactly
(all 8 suites x 3 seeds x 2 fidelity tiers), and the three-component
bundle reproduces `FullSuite` match for match. That constraint is what
forced the one genuinely non-obvious design decision in the module.
A bundle does NOT call each pose-fixing component's `tag_correction`
in turn -- that would consume one rng draw per component per tick, so a
two-camera bundle would diverge from an identical single-camera robot
on a shared seed for reasons having nothing to do with its second
camera. It runs one detection pipeline over the *union* of its camera
mounts, which is both what the robot physically has and the only
version that stays exact.

### The statistics were the actual upgrade

`ftc/optimizer.py` searches the space, but the part I'd defend hardest
is the significance test, because "this bundle is better" is the claim
the whole module exists to make and it's easy to make badly.

Every benchmark in this repo already runs each candidate against the
identical seeded scenarios -- shared scenarios are the reason a gap
between suites is attributable to the suite. But every comparison in
this repo then *threw that structure away*, computing two independent
bootstrap CIs and eyeballing whether they overlap. That's leaving a lot
on the table. Scenario difficulty is the dominant source of variance
here: two candidates can differ reliably by 15 points on the same
trials while each one's own success rate has a 30-point CI.

`nav/stats.py`'s new `bootstrap_paired_diff_ci` resamples trial
*indices* instead, so a resample takes trial i's outcome from both
candidates or from neither. `ftc/scratch/optimizer_test.py` has the
constructed case: A=14/40 and B=20/40 with independent CIs of [20%,
50%] and [35%, 65%] -- heavily overlapping, a clear "call it a wash"
under the old test -- have a paired difference of [+5.0%, +27.5%],
p=0.004. Same data. The check that this cuts both ways is in the same
file: identical vectors return [0, 0] with p=1.0, and two independent
coin flips are not called significant.

### What it found

The headline is that bundling works, but only in a specific way: every
bundle that significantly beat its own best single component spans more
than one *capability category* (pose fixing, obstacle sensing, drift
reduction, heading holding) AND adds a category that single component
didn't have. That isn't a claim I wrote into the writeup and hoped for
-- `ftc/optimizer_benchmark.py` checks it against the data and prints a
hedged version instead if it doesn't hold. The mechanism is the one
this project's own deviation-type analysis has been pointing at since
the headline study: a match is lost to whichever deviation the robot
has no answer for, so two sensors fixing the *same* failure mode mostly
don't stack -- the second is correcting an error the first already
removed.

Three results I didn't expect:

- **The frontier gap shrank once the odometry-pod price was fixed.**
  The best robot at a $50 budget and a $150 budget is still the same
  $25 front camera -- nothing between $25 and $195 beats it. But under
  the old ($280/$305) pricing, the top-of-frontier bundle needed a $500
  budget to reach; under the corrected ($195/$220) pricing it's
  affordable at $300 with $80 to spare (`ftc_optimizer_writeup.md`'s
  "Best robot at each budget"). There's no longer a budget tier where
  odometry pods alone, rather than the full bundle, is the right call.
  CORRECTION: odometry pods used to be priced at goBILDA's $279.99
  2-pod-plus-Pinpoint-computer bundle; see README.md's "FTC
  sensor-suite study: results" for why that was wrong.
- **The best-average robot and the most-robust robot keep
  almost-but-not-quite coinciding, for a different reason each time.**
  Optimizing the mean across scenarios and optimizing the *worst*
  scenario (minimax -- the right objective when you can't predict your
  division) used to pick two different robots that merely tied on
  worst-case success. The first draft of the writeup asserted they were
  "different robots, which is the whole reason this study reports both"
  -- true in general, false in that run, and it was only false because
  `rank()` breaks worst-case ties toward the cheaper robot: an artifact
  of a tiebreaker, not a finding, and the prose was fixed to compare
  worst-case *rates* and say plainly when the two objectives merely
  agree rather than literally coincide. Removing lidar as a candidate
  component (see README.md's "Threats to validity") changed the
  situation again: for a while the best-average and most-robust robots
  resolved to the exact same $330 bundle. Fixing `ftc/field.py`'s own
  grid-boundary bug (also "Threats to validity" -- it changes which
  free cells every scenario samples start/goal from) separated them
  again, for a while, into a $330 best-average robot and a $25 most-
  robust one tied on an 8% worst case. Fixing the newer rotated-
  footprint-collision gap (same section) moved the ground once more:
  the best-average robot is now the cheaper odometry pods + front
  camera ($220, 34%, no rear camera needed to lead the ranking anymore),
  and the most-robust is the free baseline itself (encoders only, $0)
  -- both tied at a 0% worst case, since nearly every robot in this
  catalog now has SOME scenario it fails completely. Same lesson, yet
  again -- check whether "different" claims about ranked objects are
  actually about identity, not just an equal score, and don't expect a
  coincidence found once to survive the next correction to the
  underlying model.
- **Greedy search happens to be enough here, and its one step is the
  whole story.** Forward selection lands on the same robot as
  exhaustive enumeration (odometry pods + front camera): starting from
  odometry pods alone (26%, $195), its one addition -- AprilTag (front
  camera) -- is itself statistically significant (+8.8%, 95% CI [+4.0%,
  +14.4%], p<0.001), and nothing further improves on it. "Stop when the
  mean stops going up" and `--require-significant` agree here without
  needing to demonstrate a difference between them.

The honest limitation, stated in the study itself: fusion conflict
isn't modeled. Capabilities merge optimistically -- sensors union their
detections, the best localization hardware sets the drift rate -- so
two sensors *disagreeing* about where the robot is, and the filter work
of resolving that, costs nothing here. Every bundle number is therefore
an upper bound on what combining actually buys.

## Giving bundles their own visualizer, not a mode of the suite one

The first pass at watching bundles run put the feature inside
`scenario_ftc_suites.py` -- a `"+"`-join syntax in `--suites` for a
bundle panel, plus a `--live-bundle` panel toggled with number keys.
It worked, but it was the wrong home for it. That file's whole design
-- fixed panel count decided once at startup, a keybinding scheme built
around scrubbing one scenario across a handful of named suites -- is
right for "compare these specific suites side by side" and wrong for
"browse a combinatorial space," which has a different natural
interaction (page through an ordered list) and a different natural
constraint (the thing you're looking at can be a different size on
every step). Bolting the second onto the first meant reserving a
number-key range that had nothing to do with the file's existing
controls and a live-bundle panel whose presence/absence the rest of the
layout code had to special-case. Separating them into
`scenario_ftc_bundles.py` isn't just tidiness: it let the browsing UI
be *shaped* around what it's actually for, particularly the one thing
the old design couldn't do at all -- resize the window to fit however
many components the currently-selected combination has, from a 2-suite
pair (3 panels: two ingredients + the bundle) up to the largest
combination the candidate pool builds, just by pressing Left/Right.

The result is 19 buildable 2+-suite combinations (from the 6 candidate
suites `ftc/optimizer.py`'s `DEFAULT_COMPONENTS` offers, minus
dead-reckoning, which never changes a bundle's drift rate or
capabilities -- down from 7 candidates/42 combinations before lidar was
removed as a purchasable component, see README.md's "Threats to
validity") at the default candidate pool -- the exact same
enumeration `ftc/bundle.py`'s `enumerate_bundles` and
`ftc/optimizer_benchmark.py`'s study already use, so the visualizer's
list and the writeup's numbers are provably talking about the same
robots, not two independently-maintained catalogs. Left/Right steps to
the previous/next one; PageUp/PageDown jump 5; Home/End jump to the
cheapest/priciest. There's no other control needed to see all of them
-- which was the actual ask -- and the status bar always says which one
you're on ("BUNDLE 7/19") so paging through never loses its place.

One correctness fix fell out of building this that had nothing to do
with bundles per se: the single-suite visualizer's sensor-visual
drawing only ever showed ONE sensor kind per panel (cone or camera --
whichever the code checked first), which meant `FullSuite` --
distance sensors AND AprilTag together, senses obstacles AND fixes pose
-- never drew its camera FOV wedge, even in the original single-suite
tool. A bundle can combine arbitrarily many sensor types, so
`field_view.py`'s `sensor_kind`/`draw_sensor_visual` had to generalize
into `suite_sensor_visuals`/`draw_sensor_visual` accepting a LIST of
active visuals rather than picking one -- and since that's a strict
generalization (a suite with one active sensor type still gets a
one-element list, drawn identically to before), it's shared by both
visualizers and fixes the FullSuite gap in the original one too, for
free. Cosmetic only -- nothing in `ftc/match.py` or any published
number depends on what gets drawn.

## Parallelizing the sweeps

Every full-rigor sweep in `ftc/` and `nav/uncertainty_benchmark.py`
loops over the same shape: a grid of independent points (deviation
type x variance level, or just variance level), each running
`TRIALS_PER_COMBO` trials for every suite/policy, all written into one
CSV at the end. Nothing about one combo depends on another finishing
first -- the obvious next question is whether that means they can run
at the same time.

### The thing that had to be checked before touching anything

Running trials in a different order, or on different workers, is only
safe if nothing in the sweep depends on order. The risk is a single
shared `random.Random()` instance consumed once per trial in a fixed
loop -- trial 40 would then depend on trials 1 through 39 having
already drawn from it in exactly that sequence, and parallelizing would
silently change every trial after the first.

That's not what this repo does, and it was worth actually reading the
code to confirm rather than assuming it either way.
`ftc/suite_benchmark.py`'s `run_combo` computes `trial_seed = base_seed
+ t` and hands each trial its own fresh `random.Random(trial_seed)` --
`base_seed` itself comes from `6_000_000 +
DEVIATION_TYPE_ORDER.index(deviation_type) * 1_000_000 + round(level *
100)`, a pure function of the combo's own coordinates, not a running
counter. Every trial's random state is fully determined by
`(deviation_type, variance_level, trial_index)` alone. `nav/
uncertainty_benchmark.py`'s `run_variance_level` does the same thing
one axis simpler (`1_000_000 + round(level * 100)`). Both were already
safe to parallelize before any of this work started -- which meant this
task really was mostly "write the runner, and write the test that
proves it," exactly as it looked going in.

### The runner

`ftc/suite_benchmark.py` gets `run_sweep(deviation_types, levels,
num_trials, grid, free_cells, tag_sites, fidelity=None,
max_workers=None)`: it builds the full list of `(deviation_type,
level)` combos, submits each as its own `run_combo(...)` call to a
`ProcessPoolExecutor`, and reassembles the results back into the fixed
`combos` order before returning -- not completion order, which
`ProcessPoolExecutor` makes no promise about, and which running the
same sweep twice could easily return in a different sequence purely
from OS scheduling noise. Reassembling into a fixed order is what makes
the output order independent of how many workers ran it or how fast
each one happened to finish. `max_workers=1` skips the process pool
entirely and runs every combo serially in-process -- this is the
baseline the determinism tests compare against, and it exists so
proving "parallel gives the same answer as serial" doesn't also have to
account for subprocess startup noise on the serial side.

`nav/uncertainty_benchmark.py` gets the same shape, one axis simpler
(`run_sweep(levels, num_trials, ...)` over `run_variance_level`
instead of over `run_combo`).

Three of `ftc/suite_benchmark.py`'s siblings --
`ftc/layout_benchmark.py`, `ftc/fidelity_benchmark.py`, and (with one
exception, below) `ftc/budget_benchmark.py` -- already imported
`run_combo` directly from `ftc/suite_benchmark.py` and looped over it
themselves with the identical seed formula copy-pasted in each file.
Pulling that formula out into a named `base_seed()` function and
routing all three through the shared `run_sweep()` instead of their own
copy of the loop wasn't scope creep -- it was three duplicate copies of
the same nine-line loop collapsing into zero once the shared version
existed, which is a smaller diff than adding a fourth copy for
`run_sweep` to sit next to would have been.

### The one sweep that couldn't just switch over

`ftc/budget_benchmark.py` runs its sweep once per candidate
`AUTONOMOUS_PERIOD_S` value by monkeypatching
`ftc.match.AUTONOMOUS_PERIOD_S` -- a plain module-level global --
before calling into the sweep, then restoring it in a `finally` block.
That works when everything runs in one process, because `run_match`
looks the global up fresh on every call from the same patched module.
It does not obviously work once combos start running in separate
processes: a `ProcessPoolExecutor` worker under Python's `spawn` start
method (the default on macOS and Windows) gets its own fresh import of
`ftc.match`, with the *original*, unpatched value of
`AUTONOMOUS_PERIOD_S` -- the parent process's monkeypatch never crosses
the process boundary. `fork` (Linux's default, and so CI's) would
happen to copy the patched value into each worker, because forking
duplicates the whole process's memory including the patch already
applied. That divergence -- correct by accident under `fork`, silently
wrong under `spawn` -- is exactly the kind of platform-dependent bug
that passes on a Linux CI runner and breaks the moment someone runs it
locally on a Mac, which made it worth catching in review rather than
finding out about it that way. `ftc/budget_benchmark.py` calls
`run_sweep(..., max_workers=1)`, forcing every combo to run in-process
where the monkeypatch is guaranteed to apply, on every platform,
without relying on which start method happens to be in effect.
Parallelizing this particular sweep properly would mean threading
`budget_s` through `run_combo`/`run_match` as an explicit argument
instead of a patched global -- a real refactor, not done here, to keep
this change to its intended scope.

### Proving it, not just arguing it

`ftc/scratch/suite_sweep_parallel_test.py` and `nav/scratch/
uncertainty_sweep_parallel_test.py` each run a small sweep (a few
combos, few trials -- fast enough for every `pytest` run) three ways:
serial (`max_workers=1`), the default parallel worker count, and two
different explicit worker counts against each other. All comparisons
are row-for-row over every column except the wall-clock timing one
(`planning_time_ms`/`planning_time_s`) -- excluded for the same reason
`ftc/scratch/fidelity_test.py`'s `COMPARE_COLUMNS` already excludes it:
it measures how long the trial actually took to execute on this
machine, this run, and was never reproducible run-to-run even before
any of this, parallel or not. Beyond the reduced test sweeps, the real
check that mattered most was rerunning the *actual* 4,125-row headline
sweep (all 3 deviation types, all 11 variance levels, all 25 trials, in
parallel) and diffing it against the checked-in
`benchmark_results/ftc_suite_results.csv` -- zero mismatches across
every row, which is the thing every number in this README's "FTC
sensor-suite study" section actually depends on staying true.

### What it bought, honestly

On the 8-core machine this was measured on, the headline sweep dropped
from an 11.8s median (3 repeats, serial) to 6.2s (3 repeats, default
worker count) -- about 1.9x, not 8x. Scaling isn't linear with worker
count: 2 workers already reaches 8.4s, 4 workers 7.2s, 8 workers only
6.2s. The reason is combo size, not process overhead -- the 33
`(deviation_type, variance_level)` combos are not equal-sized work
units (higher variance levels trigger more replanning, more
collisions, more retried scenarios before a solvable one is found), so
splitting 33 uneven jobs across 8 workers leaves some workers idle
waiting on whichever worker drew the biggest combo. A finer-grained
split (per-trial rather than per-combo) would likely scale better, at
the cost of more process-pool overhead per unit of work and a bigger
change to `run_combo`'s own structure -- not done here, both because
1.9x already meaningfully speeds up local iteration on every sweep in
this repo and because the smaller, easier-to-verify change was the
right tradeoff for what this addition needed to prove.

## Sensor fusion: does AprilTag+odometry's advantage survive disagreement?

`ftc/bundle.py`'s own module docstring already says the honest thing
about how it combines suites: capabilities merge "the physically
honest way," but a tag detection is applied at face value, with no
concept of it conflicting with what odometry already believes. README.md's
"Threats to validity" named the consequence -- the optimizer's bundle
results are an upper bound, size unknown -- without measuring it. This
is the addition that measures it, for exactly one pairing (AprilTag vs.
odometry pods) rather than building a general multi-sensor filter.

### Why a plain weighted average, not a Kalman filter or a particle filter

The obvious "correct" answer here is a Kalman filter: it's the standard
tool for fusing two Gaussian estimates of the same quantity, and this
project already tracks pose error as a 2D vector, which is exactly the
state a Kalman filter wants. It was deliberately not used, for a reason
that's really about honesty rather than difficulty: this project has no
actual covariance model. `error`'s uncertainty is never tracked as a
number anywhere in `ftc/match.py` -- there's a *value* (the accumulated
drift vector) but no *variance* attached to it. A real Kalman filter's
gain comes from comparing the prior's variance to the observation's
variance; faking a variance number just to plug it into a Kalman
update would be inventing precision this project doesn't have and
dressing it up in the right equations. A plain confidence-weighted
average makes the same honesty visible instead of hiding it: the
weights (`ODOMETRY_FUSION_CONFIDENCE`, `APRILTAG_FUSION_CONFIDENCE`)
are named as what they are -- engineering estimates -- not disguised as
a covariance this project never computed.

A particle filter was the second option seriously considered, mostly
because it's the more "correct" answer for representing a genuinely
multi-modal belief (two sources that disagree could both be
independently right, at two different unresolved candidate positions,
until later evidence sorts it out). It was rejected for scope reasons
that turned out to matter more the more the tradeoff was considered:
`nav/` has zero numpy anywhere in it (`nav/stats.py`'s own docstring
says so explicitly, hand-rolling bootstrap CIs in stdlib instead), and
a particle filter without vectorized array operations means hundreds of
Python-object particles, resampled every tick, in a simulation that
already runs the full headline sweep in a few seconds. That's a real
performance regression for a feature whose entire point was supposed
to be answering ONE question well, not becoming the next thing the
project has to keep fast. The deeper reason, though, is that a
multi-modal belief is the right tool for "the robot might genuinely be
in one of two places," and that's not actually the question being
asked here -- the question is "does a reading disagree with what's
already believed enough to be distrusted," which a single point
estimate with a confidence attached answers perfectly well.

### How the confidence values were actually chosen

Not tuned to produce a particular result -- chosen first, from numbers
already in the codebase, before ever running the benchmark. AprilTag's
per-detection confidence reuses `frac` -- `AprilTagSuite.tag_correction`
already computes a range/angle-degraded correction fraction for every
detection, and building a SECOND geometry-quality signal that would
have to agree with the first one seemed like exactly the kind of
duplicated, driftable logic this project's own conventions (`ftc/
robustness.py`'s warnings about class-attribute traps, `ftc/bundle.py`'s
`part_cost_mismatches` check) exist to avoid. `APRILTAG_FUSION_
CONFIDENCE = 1.0` was picked so that a clean, ideal detection
(`frac` near `APRILTAG_CORRECTION_FACTOR_MAX = 0.90`) ends up weighted
roughly 9:1 against `ODOMETRY_FUSION_CONFIDENCE = 0.1` -- close to how
strongly the OLD, un-fused model trusted a clean detection (`error *
(1 - 0.9)` already discards 90% of the prior). The goal was for fusion
to behave like the old model in the easy case and only diverge from it
where the old model had no way to represent something at all (a
biased-but-working detection, an outright wrong one) -- not to build a
system that disagrees with the established baseline everywhere just to
look more sophisticated.

### The bug the disagreement threshold caught before it shipped

The first design for the disagreement check compared the incoming
observation directly against the raw prior (`error`, the full
accumulated drift). That's wrong, and it's wrong in a way that would
have been very easy to ship without noticing: a large, perfectly
LEGITIMATE correction naturally looks like a big disagreement under
that comparison, purely because a good detection is SUPPOSED to differ
a lot from a badly-drifted prior -- that's what correcting pose means.
Comparing against the raw prior would have flagged confident, correct
detections as suspicious purely because a lot of drift had accumulated
first, which has nothing to do with whether the detection itself was
trustworthy.

The fix was reframing what gets compared: `ftc/fusion.py` computes
`plain_corrected = error * (1 - frac)` -- the geometry model's own
prediction of what a well-functioning detection of this quality should
report -- and constructs the actual observation as a deviation FROM
that prediction (a small constant bias, or a wide-sigma random jump for
a bad reading), not as an independent value compared against the raw
prior. This is a simplified version of what Kalman-style filters call
innovation gating: gate on the residual from the PREDICTED observation,
not on raw distance from the state. Before finalizing the constants, a
5,000-sample sanity script (typical error magnitudes 0-4 cells, typical
`frac` 0.1-0.9) confirmed the fix actually worked: about 6.5% of
ordinary detections still cross `FUSION_DISAGREEMENT_THRESHOLD_CELLS`
purely from correction magnitude, against about 87% of genuinely bad
ones. Real separation, not perfect separation -- reported as exactly
that in `benchmark_results/ftc_fusion_writeup.md`, not rounded up to
"solved."

### What the study found, and checking it wasn't a bug before believing it

The result was more dramatic than expected: pooled across `ftc/
optimizer.py`'s 5 scenario profiles, the AprilTag+odometry bundle
succeeds in 35% of trials under the existing optimistic merge and 22%
under confidence-weighted fusion -- a statistically significant drop
(paired 95% CI [-18.5%, -9.0%]) that doesn't just shrink the bundle's
advantage over its best single component, it inverts it: the fused
bundle (22%) ends up BELOW its own best single component, Odometry
pods (25%).

A result that large is exactly the kind that deserves a second look for
a bug before being written up as a finding, so before trusting it: is
`is_bad` (the per-detection bad-reading roll) only drawn when a
detection actually happens, or could it be drawn every tick regardless?
Checked directly -- `fused_tag_correction` is only ever called from
inside `ftc/match.py`'s `if frac is not None:` branch, which only
executes once `suite.tag_correction` has already passed its own range/
FOV/line-of-sight/dropout gates. So the mechanism behind the size of
the drop is real, not a bug: this project's own `ftc/budget_
benchmark.py` docstring already establishes that AprilTag replans (and
therefore corrects pose) unusually often in this simulation's matches,
which means the PER-MATCH chance of at least one bad detection compounds
well above the 5% per-detection rate (roughly 40% over 10 detections),
and this project's match model has no recovery from a single badly
wrong correction by default (`on_collision="halt"` -- a corrupted
`error` can point the robot's next commanded step straight at an
obstacle it never sees coming, ending the match on the spot).

The honest framing, and the one actually written into `ftc/config.py`'s
comments and the benchmark's own writeup: this is what these SPECIFIC,
never-measured constants produce, not a calibrated claim about real
AprilTag hardware. The constants were fixed before the benchmark ran,
the result wasn't adjusted afterward to look more moderate, and the
magnitude is reported exactly as computed -- consistent with this
project's own rule that a finding that doesn't flatter its own
machinery (DistanceSensorSuite's blind spot, mecanum's unpaid premium)
gets reported with the same confidence as one that does.

### Closing the loop: real variance tracking, and a Kalman path after all

The reasoning above ("Why a plain weighted average, not a Kalman
filter") was correct as far as it went, but it left a specific,
nameable gap: this project had never tracked a real variance anywhere,
so a Kalman filter was properly out of reach, not permanently ruled
out. Closing that gap meant doing the actual work the earlier decision
deferred, in the order that matters -- fit real variances FIRST, only
then write the filter -- rather than jumping straight to "add a Kalman
filter" and quietly inventing the numbers it needs, which would have
been the exact mistake the original decision was written to avoid.

`ftc/calibration.py` gained two new fits. `fit_process_variance_per_cell`
needed no new data or new math at all -- it's the existing `fit_pose_
drift_rate` (itself derived from the random-walk relation `Var(total) =
n_cells * sigma^2`) squared into the shape a Kalman predict step
actually consumes, since that random-walk assumption already IS
"variance grows linearly with cells traveled." `fit_apriltag_
measurement_variance` needed a genuinely new dataset this project never
had before: real AprilTag detection SCATTER (range, incidence angle,
measured position error), independent of any assumed correction-quality
formula, fit by ordinary least squares -- `squared_error = b0 +
b1*range_in + b2*incidence_deg` -- solved with a hand-written 3x3
Gaussian elimination (`_solve_3x3`) rather than adding a NumPy
dependency this project has deliberately never had (`nav/stats.py`'s
own hand-written bootstrap is the same choice, for the same reason).
Proven against a synthetic dataset built from a KNOWN exact linear
relationship first (`ftc/scratch/calibration_test.py`'s `check_solver_
recovers_exact_linear_relationship`) -- the only way to actually verify
a hand-rolled linear-algebra solver is solving the right system, not
just returning *something*.

`nav/kalman.py` is the estimator itself, domain-neutral like every
other `nav/` module -- `predict`/`update`/`gated_update`, the standard
scalar Kalman math, isotropic (one variance for both axes) the same way
`nav/estimation.py`'s `PositionEstimate` is already one confidence for
both axes. `fuse()` was deliberately left completely untouched rather
than retrofitted to accept a variance: its whole contract is a plain,
honestly-labeled `confidence` weight, and blurring that into "sometimes
a real variance, sometimes an invented one" would have undone the exact
honesty the original design was protecting. `nav/scratch/kalman_test.py`
proves the estimator standalone -- including a deliberate cross-check
against `fuse()` itself (`check_degenerates_to_confidence_weighted_
fuse_at_equal_uncertainty`): fed EQUAL variances/confidences, the two
independently-implemented tools produce the exact same 50/50 blend,
which is either a coincidence or a sign both are computing something
real. It isn't a coincidence.

Wiring it in (`ftc/fusion.py`'s `fused_tag_correction_kalman`, `ftc/
match.py`'s `fusion="kalman"`) needed one honest approximation, stated
plainly rather than hidden: `AprilTagSuite.tag_correction` collapses a
detection's raw range/incidence into a single scalar `frac` before
`ftc/fusion.py` ever sees it, and changing that public method's
signature to plumb raw geometry through would have meant touching every
override (`FullSuite`, `DualCameraAprilTagSuite`, ...) for a change
scoped to one fusion path. Instead, `_apriltag_observation_variance_
cells2` evaluates the fitted variance model once at its own best case
(range=0, incidence=0) and scales it by how far `frac` sits below its
own achievable maximum -- physically sound (worse `frac` really does
mean a worse detection) but coarser than the fitted model could be:
two geometrically different detections that happen to produce the same
`frac` get treated as equally uncertain, even though real range/
incidence data would tell them apart. `_apriltag_observation_variance_
cells2`'s own docstring says this every time it's read, not just once
at the top -- the same discipline this project applies to every other
documented simplification.

The result, in `ftc/fusion_kalman_benchmark.py` (deliberately a
SEPARATE study writing SEPARATE output files, not a rewrite of `ftc_
fusion_writeup.md` in place -- that file's 35%-to-22% finding is
already published and cited elsewhere, and forcing every one of its
sentences to carry a "this part is on synthetic variance" caveat that
has nothing to do with what it's actually about would have made it
worse, not more honest): at this project's SYNTHETIC placeholder
variance (the pipeline is built and proven; the real measurement still
doesn't exist), Kalman fusion succeeds in 26% of trials, a real,
paired-bootstrap-significant improvement over confidence-weighted
fusion's 22% (+5.0%, 95% CI [+2.0%, +8.5%]) -- the properly gated,
variance-aware update genuinely does recover some of what a fixed
confidence weight throws away. It edges narrowly back above its own
best single component's rate (Odometry pods, 25%) -- the inversion the
plain-weighted-average study found doesn't just shrink under Kalman
specifically, it reverses, if only barely (+2%, not separately tested
for significance against the single-component floor), well short of
the 35% optimistic-merge figure the whole disagreement-modeling
investigation set out to check. The headline finding from the
plain-weighted-average study survives in weakened form under Kalman,
not overturned -- exactly the kind of result this project reports at
full strength either way, not adjusted toward whichever answer would
look better for the fancier tool.

## Planning latency at the tail

The brief for this addition was explicit about order of operations:
read `ftc/match.py` before writing anything, and describe what it
actually does rather than assuming a per-tick deadline exists. That
turned out to matter, because the real structure isn't what a first
guess would produce.

### What reading the code actually found

`run_match` tracks `elapsed_s` as a purely SIMULATED time accumulator
-- drive time, turn time, collision recovery, all added as computed
quantities, never compared against a wall clock. The only REAL
wall-clock measurement anywhere in the function is a `time.perf_counter()`
pair bracketing each `astar()` call, accumulated into
`planning_time_s`. That measured value is returned on `MatchResult` and
used elsewhere purely for reporting (`avg_planning_ms` in other
benchmarks' aggregate tables) -- it is never added to `elapsed_s`. What
IS added to `elapsed_s` is a flat constant, `PLANNING_OVERHEAD_S`
(0.05s), charged once per replan, and only on replans AFTER the first
-- the very first planning call in a match, which real matches often
spend on the longest search of the whole run, costs `elapsed_s` exactly
nothing, charged or measured.

This is a deliberate design decision, not an oversight -- `PLANNING_
OVERHEAD_S`'s own comment in `ftc/config.py` says so directly: raw
Python `astar()` "is sub-millisecond... and would understate what a
real re-plan actually costs" on FTC-legal onboard compute. But it has
a consequence that comment doesn't spell out, and that this addition's
whole job was to check: under the model exactly as implemented, no
measured planning latency, however large, can change `over_budget` or
`success`. That's not a hypothesis to test with a sweep -- it's a fact
about the code, and the first thing this addition did was PROVE it
directly rather than just cite it: `ftc/scratch/planning_latency_
test.py`'s `check_flat_charge_ignores_measured_latency` reruns the
identical scenario and seed twice, once against real `astar()` and once
against a version that sleeps an extra 0.1s on every call (more than
double `PLANNING_OVERHEAD_S`, more than a match with several replans
would even notice), and confirms `elapsed_s` comes back byte-for-byte
identical either way -- 14.1052s both times, regardless of the real,
injected delay (measured `planning_time_s` goes from 2.3ms to 212.5ms,
roughly a 100x increase, and `elapsed_s` doesn't move at all).

### The actual question, and why it has to be a counterfactual

Given that structural fact, "does the tail matter" can't be asked by
just looking at whether any published `over_budget` value moves --
none ever could, by construction. The only way to ask the real
question is a counterfactual: for each match, what would `elapsed_s`
have been if it had used THIS match's own real measured planning total
instead of the flat charge? That's `result.elapsed_s - (replans *
PLANNING_OVERHEAD_S) + measured_total` -- subtract out what the flat
model charged for planning, add back in what planning actually took,
including the first call the flat model never charges at all. A match
that flips from under-budget to over-budget under that substitution is
one where the tail would have mattered, if this project's match model
charged for it.

### Getting the latency numbers without touching ftc/match.py

The measurement itself needed to not risk anything published. Rather
than add instrumentation to `ftc/match.py` -- which would mean auditing
every existing call site for a behavior change, exactly the kind of
risk this project's own conventions exist to avoid -- `ftc/planning_
latency_benchmark.py`'s `capture_astar_latencies()` is a context
manager that monkeypatches `ftc.match.astar` (not `nav.algorithms.astar`
-- Python binds `from nav.algorithms import astar` into `ftc.match`'s
own namespace at import time, so patching the original module's
attribute after that would silently intercept nothing) for the
duration of one `run_match()` call, records each call's wall-clock
time, and restores the original function in a `finally` block. Zero
lines of `ftc/match.py` changed; zero risk to any existing benchmark.

The sanity check this needed before trusting any of its numbers: does
the captured total actually match what `run_match` itself measured and
returned (`MatchResult.planning_time_s`)? The first version of this
check used a tight, fixed tolerance and failed partway through the real
sweep -- not because the mechanism was wrong, but for two compounding,
genuinely small reasons worth naming rather than papering over. First,
once `ftc.match.astar` is patched, `run_match`'s OWN internal timer is
now bracketing a call to the wrapper, not the raw function -- so
`run_match`'s measured interval includes the wrapper's own overhead (a
second `perf_counter()` pair, a list append) on top of the real
`astar()` time, while the wrapper's own recorded latency excludes that.
Second, wall-clock timing on a real, shared machine has genuine noise
(OS scheduling, GC pauses) that doesn't scale cleanly with call count.
The fix was a tolerance that scales with the measured value itself
(`max(2e-4, 0.05 * planning_time_s)`) rather than a fixed guess per
call -- loose enough to absorb real timing noise, tight enough that an
actual instrumentation bug (timing a different set of calls entirely)
would still fail it by orders of magnitude, not a few percent.

### Grid size, and the constraint discovered while designing the sweep

The brief asked for a grid-size sweep reusing `ftc/field.py`'s existing
layouts rather than inventing new ones. The first instinct was to vary
`build_grid`'s `cell_size_in` parameter -- same physical field, finer
resolution, which `build_grid` already accepts. Prototyping that
revealed a real problem before it could contaminate any data: every
`*_CELLS` sensor-range constant in `ftc/config.py`
(`APRILTAG_RANGE_CELLS`, `DISTANCE_SENSOR_RANGE_CELLS`, ...) is computed
ONCE at import time from the module-level, native `CELL_SIZE_IN` --
none of them read whatever `cell_size_in` a particular `build_grid()`
call actually used. Varying `cell_size_in` would have silently made
every sensor's range wrong relative to the grid it was actually driving
on, at every resolution except the native one -- exactly the kind of
confound that would have made every suite-comparison number in this
specific study quietly meaningless without an obvious symptom pointing
back to the cause. The fix was reframing what "grid size" means for
this study: vary `size` alone, holding `cell_size_in` fixed at its
native value -- a bigger physical field, same real 6-inch cells,
identical to how `nav/scale_benchmark.py` already scales `nav/`'s own
domain-neutral grid. This is now flagged explicitly in both the
benchmark's own writeup and README.md's limitations entry as a real
constraint on `ftc/field.py`'s `cell_size_in` parameter that this
study surfaced, not something already known and simply being avoided.

### The result was flatter than expected in the median, though not in the max, and that needed checking too

The full sweep (5 grid sizes from 24 to 384 cells/side, 3 layouts, 7
suites, 12 trials each, 1843 individual planning calls) found zero
outcome flips, at every single size tested -- not just at native scale.
That's a stronger negative result than expected going in, and a flat
result across every condition is exactly the kind of thing worth
doubting before writing up, not the kind of thing to take at face value
because it's convenient.

The check: does latency actually grow with grid size at all in this
setup, or does something about the experimental design suppress it? The
MEDIAN says no -- it stays small and roughly flat at every size (0.35ms
at 24 cells/side up to 0.73ms at 384), well under `PLANNING_OVERHEAD_S`
regardless of grid size. The MAX says otherwise: 4.26ms at 24
cells/side, but 309.99ms at 192 and 1351.35ms at 384 -- real,
substantial tail growth with grid size, directly visible in the bounded
sweep's own numbers now, without needing anything else to reveal it.
(An earlier version of this study, before `ftc/field.py`'s own grid-
boundary bug was fixed -- see README.md's "Threats to validity" -- found
the bounded sweep's own max stayed small and flat across every size
too, and reached for a quick, separate, unbounded-path comparison
(uniform random start/goal across the whole grid, the same sampling the
very first timing prototype for this study used) to show that growth
was being suppressed by the bounded sampler rather than genuinely
absent -- at size=384, unbounded sampling produced path lengths up to
315 cells and a max latency of 174.5ms, well above what the bounded
sweep produced at that grid size at the time. That specific unbounded
comparison hasn't been rerun against the corrected grid, so its exact
old figures are not repeated here as current -- but the underlying
mechanism it demonstrated, that path LENGTH drives A*'s cost here more
than raw cell count, and that this study's own scenario sampler
deliberately bounds path length at every grid size specifically to keep
DRIVE time from swamping the budget before planning latency could
matter, still holds regardless of which specific numbers illustrate
it.) Both facts stay true at once regardless: the tail latency, however
large, never once flips a match's outcome at any size tested here --
1351.35ms is still three orders of magnitude below the 30-second
budget. Reporting a clean, structural "planning latency's tail doesn't
matter, and doesn't even grow with grid size" finding would now be
reporting something narrower than what the regenerated data actually
shows -- the correct, current claim is "the tail grows with grid size
but stays nowhere near large enough to matter at any size tested," not
"the tail doesn't grow."

## Scripted auto: what if the robot never replans at all?

Every study in this repo up to this point shares one unstated
assumption: the robot plans with A* and keeps replanning throughout the
match. Real FTC teams mostly don't do that -- a season's actual
autonomous program is usually a fixed, hand-tuned sequence of moves a
team worked out ahead of time and never reconsiders live, closer to
`nav/policies.py`'s `OpenLoopPolicy` (whose own docstring already says
so: "This is what a standard FTC autonomous routine does today") than
to `ftc/match.py`'s own live-replanning loop. That gap sat unstated in
this project until it was asked about directly.

### Why OpenLoopPolicy itself couldn't just be reused

The honest first instinct was to reuse `OpenLoopPolicy` outright --
it's already the right CONCEPT, already implemented, already proven.
It doesn't fit as code, though: `Policy.step(true_grid, current_cell)`
is `nav/`'s minimal grid-cell abstraction, built for `nav/
uncertainty_benchmark.py`'s comparison of open-loop/reactive/belief
strategies over a bare grid with no drivetrain, no real elapsed-time
accounting, no sensor suites, no pose/heading error, no fusion, no
collision recovery. `ftc/match.py`'s `run_match` already owns all of
that for the live-replanning case; wrapping a second, parallel FTC-
aware loop around `OpenLoopPolicy` would have meant either duplicating
all of it a second time or bolting `run_match`'s machinery onto
`OpenLoopPolicy` from the outside, neither of which is smaller or
cleaner than what actually shipped: `scripted_auto=True`, a single new
`run_match` parameter that narrows the function's OWN existing replan
trigger (already computed every tick, already named `replan_needed`)
down to `not planned_once`, unconditionally. Same concept as
`OpenLoopPolicy`, expressed as the smallest possible change to
machinery that already existed, rather than new machinery duplicating
it.

### The bug a first pass at this would have shipped

The first version of `scripted_auto` didn't touch WHICH grid the one
allowed plan gets computed against -- it just forced `replan_needed =
not planned_once` and left the existing `source_grid` selection alone
(`KnownGrid(known_obstacles_believed, ...)` for an obstacle-sensing
suite, `assumed_grid` otherwise). That's wrong in a way a quick scratch
test caught immediately: at tick 0, `known_obstacles_believed` is
still EMPTY (nothing has been sensed yet), so an obstacle-sensing
suite's one-and-only plan under that first version was computed against
a nearly blank map, treating every not-yet-seen obstacle as free --
structurally WORSE than a non-sensing suite's plan against the full
assumed map, for a reason that has nothing to do with the actual
question ("does obstacle sensing help without ever rerouting?"). The
scratch check exposed it concretely: DistanceSensorSuite scored
measurably BELOW DeadReckoningSuite on the identical scenarios under
that first version -- a suite that senses obstacles doing worse than
one that doesn't, purely from where its one plan happened to be
computed against. The fix: `scripted_auto=True` always plans against
`assumed_grid`, regardless of `suite.senses_obstacles` -- a stand-in
for a team authoring a routine against the field's known/CAD layout
ahead of time, not against one instant of live sensor data. After the
fix, the same scenarios come back exactly tied (7% vs. 7%,
`ftc/scratch/scripted_auto_test.py`'s
`check_obstacle_sensing_buys_nothing_under_scripted_auto`) -- neutral,
which is what "no avenue to act on what it senses" should actually
look like, not a hidden penalty.

### The result, and the one place pooling the wrong data would have hidden it

`ftc/scripted_auto_benchmark.py` crosses `scripted_auto` against the 5
headline suites and this project's usual 3 deviation types. The
headline comparison (DistanceSensorSuite vs. DeadReckoningSuite) is
computed over `obstacle_drift`/`unplanned_blocker` only, not pooled
across all three deviation types the way most other studies in this
repo pool their axes: `start_drift` is PURE pose error (`ftc/
suite_benchmark.py`'s own `DEVIATION_TYPES` sets `obstacle_drift_scale=
0, blocker_scale=0` for it), so an obstacle sensor has structurally
nothing to detect differently from the assumed map under that
deviation type alone -- pooling it in would have diluted the exact
mechanism the question is about, the same trap `fit_apriltag_
measurement_variance` in the Kalman work above was careful to avoid
in a different form.

Even restricted to the two deviation types where it could matter,
distance sensing's live-replanning advantage over dead reckoning turned
out to already be too small to separate from noise at this trial
count -- consistent with, not contradicting, this project's own
earlier finding (the DistanceSensorSuite blind-spot result, `ftc_
suite_writeup.md`) that the three modeled distance sensors only cover
about 75 degrees of the full 360 around the robot and collide in the
large majority of their trials (77%) even at zero deviation. A first draft of
this writeup asserted the live-replanning advantage "IS significant"
without actually checking it, then printed the correct "not
significant" conclusion for the SCRIPTED row right next to that false
claim about the REACTIVE row -- caught by reading the two paired-CI
numbers the code had already computed, not by assumption, and fixed by
writing out all four reactive/scripted x significant/not-significant
cases explicitly instead of assuming only one shape of result was
possible.

What the pooled comparison couldn't cleanly show, the STRUCTURAL
comparison confirms directly: both pose-fixing suites (AprilTag,
Odometry pods) stay measurably ahead of dead reckoning even under
`scripted_auto` -- pose correction still helps the exact same fixed
route land closer to where it was planned, since that needs no reroute
at all, only a better estimate of where the robot actually is relative
to a route it's already committed to. FullSuite (the only suite
combining both capabilities) loses roughly the obstacle-sensing half of
its advantage under `scripted_auto` and keeps the pose-fixing half --
the same mechanism, visible in one suite at once. Pose error and
obstacle error remain, under yet another lens, genuinely different
failure modes with genuinely different dependence on live replanning --
not a matter of degree, a matter of mechanism.
