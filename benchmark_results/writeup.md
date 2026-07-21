# Dijkstra vs A* vs RRT: 20-trial benchmark

20 random 25x25 grids, obstacle density 10-35%, start/goal picked from free
cells, all three algorithms run on the identical grid (RRT capped at 3,000
iterations). Raw data in `results.csv`, plot in `comparison.png`.

## Dijkstra vs A*

Across the 20 trials A* explored 68.9% fewer cells than Dijkstra on
average (4,811 total cells for Dijkstra vs 1,498 for A*), most individual
trials landing in the 55-90% range. That's the Manhattan heuristic doing
its job: it steers the search almost straight at the goal instead of
expanding outward in every direction.

The gap narrows sharply when the obstacle field forces a long, winding
path rather than a direct one. Trial 18 (30.6% density, a 43-step path,
the longest of the run) is the clear outlier: A* explored only 19.5%
fewer cells than Dijkstra. Density alone doesn't hurt A* much -- density
that forces detours does, because Manhattan distance stops correlating
with true travel cost once the route has to zigzag.

## Where RRT wins, and where it loses

RRT found *a* path in all 20/20 trials within its 3,000-iteration budget,
but "found a path" and "found a good path" are different claims. Its tree
size per trial ranged from 4 nodes (a nearly-direct trial 7) to 179 nodes
(trial 9, on a mere 30% density grid) -- more than an order of magnitude
of variance for grids of similar difficulty, because tree growth depends
entirely on where random samples happen to land. Dijkstra and A* never
vary like this on a fixed grid; RRT does, every single run.

**RRT loses on path quality most of the time.** Averaged over all 20
trials it produced a path 9.1% longer than the optimal Dijkstra/A* route.
On individual trials the overhead spiked as high as 100.7% (trial 9,
29.8% density -- literally double the optimal length) and 97.0% (trial
1, 26.0% density), because RRT has no mechanism to straighten a path
after finding one; it keeps whatever zig-zagging route the tree happened
to grow.

**RRT sometimes "wins" on raw path length, which is not a contradiction.**
12 of the 20 trials show RRT *shorter* than the "optimal" grid path -- as
much as -27.7% (trial 16) and -24.0% (trial 20). This benchmark runs
Dijkstra/A* with diagonal movement off, so their optimal path is
cardinal-only. RRT isn't constrained to the grid's 4-directional step
rule at all -- it connects tree nodes with straight lines through open
space, so it can cut diagonally across open cells no cardinal-only search
is allowed to. Its "path length" is optimal relative to a *different,
less constrained* movement model, not proof it beat A* at the same
problem.

**Runtime** is RRT's clearest loss: 0.02-6.37ms per trial with no
correlation to obstacle density at all -- it tracks tree size instead.
Trial 9 (179 nodes) took 6.37ms and trial 7 (4 nodes, the smallest tree
of the run) took 0.02ms, a >300x spread that has nothing to do with how
hard either grid was to solve. Nearest-neighbor search over a growing,
unindexed node list is the dominant cost, so runtime scales with how much
the tree happened to grow -- a property of RRT's own randomness, not of
the problem. A*/Dijkstra's runtime is boringly proportional to cells
explored, every time, on the same grid.

**The honest takeaway:** on this domain -- a small, fully-known, static
grid -- RRT is strictly worse than A*: slower, less predictable, and
producing longer paths. It exists here to demonstrate the algorithm and
its tradeoffs, not because it's the right tool for this problem. RRT
earns its keep in the regime A*/Dijkstra can't touch: continuous,
high-dimensional configuration spaces (e.g. a robot arm's joint angles)
where building an explicit graph to search is computationally infeasible.
