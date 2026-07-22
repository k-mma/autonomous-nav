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

### Replanning policy

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
that is: `pybullet_main.py` imports `nav.grid.Grid` and
`nav.algorithms.find_path` directly, unmodified, and calls them exactly
the way `nav/visualizer.py` does. Every new file lives under
`nav/sim3d/` and is strictly about the physics interface -- turning grid
cells into 3D bodies, turning a cell path into a driveable trajectory,
and turning that trajectory into motor commands. Planning logic and 3D
plumbing never touch the same file.

### Grid-to-world coordinates (`nav/sim3d/coords.py`)

`grid_to_world(row, col)` maps `col -> x`, `row -> y`, `z = 0` at
`WORLD_CELL_SIZE = 1.0` meter per cell -- the same `col`-is-horizontal,
`row`-is-vertical convention `nav/visualizer.py` uses for pixels, just
with meters instead of pixels and an explicit up-axis. Every obstacle
body, path debug-line, and waypoint in `nav/sim3d/world.py` goes through
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
(`nav/sim3d/robot.py`) instead computes a heading error to the target and
calls `p.resetBaseVelocity(body_id, linearVelocity=[...],
angularVelocity=[...])` every step -- a real (if simplified) velocity
command the physics engine has to integrate into position over time, the
same category of control a differential-drive base actually uses.

It turns in place before driving forward (`FACE_TARGET_TOLERANCE`)
specifically because `resetBaseVelocity` has no concept of "forward" --
without that check the robot would happily strafe sideways toward a
waypoint behind it, which no real wheeled base can do.

### Path smoothing: corner-cutting, then a spline (`nav/sim3d/smoothing.py`)

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
   This is what `pybullet_main.py` drives by default (`--smooth
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
new code. `pybullet_main.py` just plans twice, once with
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

`nav/sim3d/robot.py`'s defaults were raised well above the original
2.0 m/s / 4.0 rad/s starting point, both for a snappier demo. Before
raising them, the headroom was checked experimentally: 6, 8, and 10 m/s
tested head-to-head (same `drive_toward` controller, same r2d2), and all
three reached the target with the base staying flat the whole time
(`max_z` never left its resting height -- no bouncing, no tipping),
confirming there's plenty of margin above the original default before
anything physically breaks.

### Porting the sensor model to real raycasts (`nav/sim3d/lidar.py`)

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
  obstacle cell 9. `nav/scratch/pybullet_lidar_test.py`'s standalone scan
  didn't surface this because its one hardcoded scan never happened to
  land exactly on a boundary; the real integration in
  `pybullet_main.py --sensor` did, immediately: `newly_seen` sets came
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

With both fixed, `pybullet_main.py --sensor` reliably explores roughly a
quarter of the grid (21-24 of 81 real obstacle cells, across repeated
runs) -- only what it actually needed to see to solve the specific
route -- and replans live as each new obstacle enters view, verified by
the printed "sensed N new obstacle cell(s) -- replanning" trail matching
up with the robot actually changing course rather than driving through
where the (still just-discovered) wall is.

### Multiple robots (`pybullet_multi_robot_main.py`)

The layout is a wall with exactly one row-tall gap in it. Robot A starts
west of the gap, robot B starts east of it, and each one's *goal* is the
other's *start* -- forcing a genuine head-on conflict through the same
one-cell-wide corridor, not just two robots that happen to share a grid.

**Coordination policy:**

- **A has strict right-of-way.** It plans once, against the static grid
  only, and never looks at B again for the rest of the run.
- **B always treats A's current cell (plus a 1-cell buffer,
  `cell_block`) as a dynamic obstacle** and replans every
  `REPLAN_PERIOD_S = 0.2` s -- the exact same technique the pygame
  `MovingObstacle` replanning logic used (mark the moving thing as a
  temporary wall, replan around it), just with a robot as the "moving
  obstacle" instead of a scripted bouncer.
- **When A is in the corridor, B's planner reports no path.** B holds
  position (`waiting = True`) and just retries on the next replan tick,
  rather than crashing on `None` or spinning in place trying to reach an
  unreachable target.

**Why this can't turn into a true deadlock:** a *symmetric* version of
this policy -- both robots treating each other as an obstacle and
neither one ever committing to go first -- genuinely can deadlock face
to face in a corridor this narrow: each one sees the other blocking its
only route and waits, forever, since neither ever decides to move first.
Breaking the symmetry with a strict priority order rules this out
structurally, not by luck: A never checks B's position at all, so A
always has somewhere to go; B always yields when it must. There is no
state where both are simultaneously waiting on each other, because only
one of them (B) is ever capable of waiting in the first place.

**The resolution policy chosen is "B waits," not "B backs up."** Backing
up (reversing along the already-driven path) would need its own argument
for why it's always safe to reverse through cells already confirmed
clear -- true here, but it's solving a problem ("B is already committed
partway into a blocked corridor") that waiting avoids ever creating: B
never enters the corridor while A occupies it in the first place, because
its own replanning refuses to route it there. Waiting in place is simply
the strictly simpler policy given that guarantee.

**A hard safety-distance stop is layered on top, deliberately not relied
on as the primary mechanism:** grid-based replanning runs every 0.2s, not
every physics tick, so a fast robot could in principle close real-world
distance in the gap between replans. If the two robots' actual distance
ever drops below `SAFETY_STOP_RADIUS = 1.0` m, B is forced to stop that
exact frame regardless of what its current plan says. This never actually
triggers in normal runs (the grid-level policy keeps them well clear
first) -- it exists as a failsafe against replanning latency, the same
role an emergency stop plays underneath a real path planner.

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
