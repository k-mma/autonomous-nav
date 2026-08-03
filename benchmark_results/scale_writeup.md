# Scale benchmark: Dijkstra vs A* vs RRT on 20x20 through 200x200 grids

8 random trials per size, 20% obstacle density, all three algorithms run
on the identical grid/start/goal at each size. Raw data in
`scale_results.csv`, plot in `scale_comparison.png`. Where
`nav/benchmark.py` holds grid size fixed (25x25) and varies obstacle
density, this holds density fixed and varies grid size -- the direct
answer to "how does this scale to a bigger grid?"

## What the data shows

**Updated after adding a k-d tree spatial index for RRT's nearest-neighbor
search (`nav/kdtree.py`, see WRITEUPS.md) -- the numbers below are the
*current* code. The original linear-scan numbers are kept in the row
below each current one for the direct before/after comparison; raw data
for the old version is saved as `scale_results_before_kdtree.csv`.**

| Size | Cells | Dijkstra avg | A* avg | RRT avg (k-d tree) | RRT avg (linear scan, old) | RRT found path |
|---:|---:|---:|---:|---:|---:|---:|
| 20x20 | 400 | 0.465ms | 0.227ms | 0.443ms | 0.438ms | 8/8 |
| 50x50 | 2,500 | 2.679ms | 0.762ms | 1.520ms | 3.964ms | 8/8 |
| 100x100 | 10,000 | 9.732ms | 1.156ms | **15.162ms** | 116.716ms | 7/8 |
| 200x200 | 40,000 | 56.511ms | 9.968ms | **58.548ms** | 258.820ms | 5/8 |

The k-d tree makes no difference to *completeness* (found path 8/8, 8/8,
7/8, 5/8 at each size, identical to before) -- it's purely a speed change,
exactly as expected from replacing one nearest-neighbor implementation
with a faster one that returns the same answer. What it does change is
dramatic: **7.7x faster at 100x100 (116.7ms -> 15.2ms) and 4.4x faster at
200x200 (258.8ms -> 58.5ms)**. RRT goes from "by far the slowest of the
three planners, worse than Dijkstra's raw growth" to "faster than
Dijkstra, still behind A*" at both sizes.

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

**RRT used to be the real story here, and it wasn't a good one at
scale -- this is now fixed, and it's worth recording both halves.** With
the original linear-scan nearest-neighbor search, RRT's runtime grew
620x over the 100x cell-count increase from 20x20 to 200x200 -- worse
than even Dijkstra's raw growth. The mechanism `benchmark_results/writeup.md`
already identified at fixed grid size: an unindexed nearest-neighbor
search scans every existing tree node, so its cost grows faster than
linearly in tree size, and a bigger grid needs a bigger tree to cross
it. Scaling `step_size`/`max_iters` up with grid size (still done today)
delayed that problem; it never removed it, because the search itself was
still O(n) per iteration regardless of tuning. Replacing that linear scan
with a k-d tree (`nav/kdtree.py`, see WRITEUPS.md) fixed the mechanism
directly rather than tuning around it: **116.7ms -> 15.2ms at 100x100
(7.7x), 258.8ms -> 58.5ms at 200x200 (4.4x)**, with zero change to how
many trials actually found a path (7/8 and 5/8 either way -- the index
changes speed, not behavior).

Completeness *itself* still degrades with scale, independent of the
nearest-neighbor fix: 8/8 at 20x20 and 50x50, 7/8 at 100x100, 5/8 at
200x200, even with `step_size`/`max_iters` scaled up with grid size to
give it a fair shot. The worst single case (200x200, trial 4) grew a
1,504-node tree without ever reaching the goal. That's a separate,
harder problem than raw search speed -- it's about how much of a bigger
space a fixed sampling/iteration budget can actually cover -- and a
faster nearest-neighbor search doesn't change it, since the k-d tree
returns the *same* answer as the linear scan did, just faster to compute.

This isn't a tuning problem to be fixed with a bigger iteration budget --
it's the two algorithms belonging to different completeness classes.
Dijkstra/A* on a grid are **resolution-complete**: if a path exists at
the grid's resolution, the search is guaranteed to find it, full stop,
because it's an exhaustive (heuristically-ordered, but exhaustive)
search of a finite graph. RRT is only **probabilistically complete**:
the probability of finding an existing path approaches 1 as the sample
count approaches infinity, which is a guarantee *in the limit*, not at
any fixed budget. `MAX_ITERS`/`step_size` scaling up with grid size
(what this benchmark already does, "to give it a fair shot") narrows the
gap between a finite budget and that limit, but never closes it --
which is exactly the 8/8 -> 8/8 -> 7/8 -> 5/8 trend above: not a bug,
not an implementation gap the k-d tree could have fixed, but the
predicted behavior of a probabilistically-complete planner run under a
budget that stays fixed while the space it has to cover keeps growing.

**The honest takeaway for scaling this to a 1000x1000 grid or a real
outdoor environment:** A* is still the clear answer for a static,
fully-known grid at any of these sizes -- its relative search effort
*improves* with scale here, not degrades. RRT's runtime is no longer the
blocker it was (the k-d tree closes most of that gap), but its
completeness under a fixed iteration budget still degrades with scale,
which no spatial index fixes -- that would need a bigger sampling budget
or a fundamentally denser tree, a different problem from where the time
was going.
