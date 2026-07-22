# Scale benchmark: Dijkstra vs A* vs RRT on 20x20 through 200x200 grids

8 random trials per size, 20% obstacle density, all three algorithms run
on the identical grid/start/goal at each size. Raw data in
`scale_results.csv`, plot in `scale_comparison.png`. Where
`nav/benchmark.py` holds grid size fixed (25x25) and varies obstacle
density, this holds density fixed and varies grid size -- the direct
answer to "how does this scale to a bigger grid?"

## What the data shows

| Size | Cells | Dijkstra avg | A* avg | RRT avg | RRT found path |
|---:|---:|---:|---:|---:|---:|
| 20x20 | 400 | 0.439ms | 0.211ms | 0.431ms | 8/8 |
| 50x50 | 2,500 | 2.610ms | 0.838ms | 3.964ms | 8/8 |
| 100x100 | 10,000 | 9.588ms | 1.225ms | 118.834ms | 7/8 |
| 200x200 | 40,000 | 61.985ms | 10.332ms | 267.127ms | 5/8 |

**Dijkstra and A* both scale sub-linearly relative to raw cell count, but
by very different margins.** Going from 400 to 40,000 cells (100x more)
grows Dijkstra's runtime 141x -- close to proportional -- but A*'s only
49x. The reason shows up in cells explored, not just time: A*'s explored
cells actually shrink *as a fraction of the grid* as it grows (about
15.4% of the grid at 20x20, only 7.0% at 200x200), because a heuristic-
guided search's effort tracks the straight-line distance between start
and goal far more than it tracks total grid area -- a bigger grid with
the same obstacle density doesn't automatically mean a proportionally
harder search, as long as there's a heuristic pointing the way. Dijkstra
has no such heuristic, so it keeps expanding in cost-ordered rings
regardless of where the goal is, and its cost scales much closer to the
grid's raw size.

**RRT is the real story here, and it's not a good one at scale.** Its
runtime grew 620x over the same 100x cell-count increase -- worse than
even Dijkstra's raw growth -- and its completeness *degraded*: it found a
path in every 20x20 and 50x50 trial, dropped to 7/8 at 100x100, and only
5/8 at 200x200, even with `step_size` and `max_iters` both scaled up with
grid size specifically to give it a fair shot. The worst single case
(200x200, trial 4) grew a 1,504-node tree without ever reaching the goal.
The mechanism is the same one `benchmark_results/writeup.md` already
identified at fixed grid size: RRT's nearest-neighbor search scans every
existing tree node, so its cost grows faster than linearly in tree size
-- and a bigger grid needs a bigger tree to cross it. Scaling the tuning
parameters delays this problem; it doesn't remove it, because the
underlying nearest-neighbor search is still unindexed and still scans
the whole tree every iteration.

**The honest takeaway for scaling this to a 1000x1000 grid or a real
outdoor environment:** A* is the clear answer for a static, fully-known
grid at any of these sizes -- its
relative search effort *improves* with scale here, not degrades. RRT
would need a proper spatial index (a k-d tree over tree nodes, which is
what real RRT implementations use instead of the linear scan this
project's does) before it's viable at 1000x1000; without one, its
runtime growth alone rules it out well before 200x200 does, let alone a
grid 25x larger.
