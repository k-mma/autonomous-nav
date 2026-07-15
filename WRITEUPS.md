# Writeups

Plain-English explanations for everything the project plan's "learn" days
asked me to be able to explain cold, plus the concrete observations behind
each design decision. Organized by week. Where a claim is backed by a
script, the script and its actual output are referenced rather than
restated from memory -- rerun them if you want to confirm a number.

## Week 1 -- Dijkstra and A*

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

## Week 2 -- Replanning

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

## Week 3 -- heuristics and diagonal movement

### Inadmissible heuristics: does 1.5x actually break anything?

The plan's experiment was to multiply Manhattan by 1.5 and "watch what
breaks." First finding, which took a wrong turn to get to: **a "perfect"
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
check wasn't part of the original plan; it came up while implementing and
is worth calling out since it's a genuine correctness bug in naive
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
