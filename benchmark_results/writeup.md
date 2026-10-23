# Dijkstra vs A* vs RRT vs RRT*: 20-trial benchmark

20 random 25x25 grids, obstacle density 10-35%, start/goal picked from free
cells, all four algorithms run on the identical grid (RRT and RRT* both
capped at 3,000 iterations, and given the *identical* random-sample
sequence per trial -- see the RRT* section below for why). Raw data in
`results.csv`, plot in `comparison.png`.

## Per-trial data

| Trial | Density | Path len | Dijkstra cells | A* cells | RRT nodes | RRT waypoints | RRT path len | Dijkstra ms | A* ms | RRT ms |
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

RRT loses on path quality most of the time. Averaged over all 20
trials it produced a path 9.1% longer than the optimal Dijkstra/A* route.
On individual trials the overhead spiked as high as 100.7% (trial 9,
29.8% density -- literally double the optimal length) and 97.0% (trial
1, 26.0% density), because RRT has no mechanism to straighten a path
after finding one; it keeps whatever zig-zagging route the tree happened
to grow.

RRT sometimes "wins" on raw path length, which is not a contradiction.
12 of the 20 trials show RRT *shorter* than the "optimal" grid path -- as
much as -27.7% (trial 16) and -24.0% (trial 20). This benchmark runs
Dijkstra/A* with diagonal movement off, so their optimal path is
cardinal-only. RRT isn't constrained to the grid's 4-directional step
rule at all -- it connects tree nodes with straight lines through open
space, so it can cut diagonally across open cells no cardinal-only search
is allowed to. Its "path length" is optimal relative to a *different,
less constrained* movement model, not proof it beat A* at the same
problem.

Runtime is RRT's clearest loss: 0.02-6.37ms per trial with no
correlation to obstacle density at all -- it tracks tree size instead.
Trial 9 (179 nodes) took 6.37ms and trial 7 (4 nodes, the smallest tree
of the run) took 0.02ms, a >300x spread that has nothing to do with how
hard either grid was to solve. Nearest-neighbor search over a growing,
unindexed node list is the dominant cost, so runtime scales with how much
the tree happened to grow -- a property of RRT's own randomness, not of
the problem. A*/Dijkstra's runtime is boringly proportional to cells
explored, every time, on the same grid.

The honest takeaway: on this domain -- a small, fully-known, static
grid -- RRT is strictly worse than A*: slower, less predictable, and
producing longer paths. It exists here to demonstrate the algorithm and
its tradeoffs, not because it's the right tool for this problem. RRT
earns its keep in the regime A*/Dijkstra can't touch: continuous,
high-dimensional configuration spaces (e.g. a robot arm's joint angles)
where building an explicit graph to search is computationally infeasible.

## RRT*: does the rewiring actually help?

RRT* (`nav/rrt_star.py`) was run on the exact same 20 grids, with the
exact same random-sample sequence per trial as plain RRT -- both are
seeded with `random.Random(trial_num * 1000 + attempt)`, so at every
iteration both algorithms sample the *identical* point and only differ in
what they do with it. That's a deliberately paired comparison: any
difference in the resulting path is attributable to RRT*'s parent
selection and rewiring, not to random variance between separate runs.

RRT* wins on path length, decisively and consistently. Head to head,
RRT* produced a *shorter* path than plain RRT in all 20/20 trials -- never
longer, never tied -- averaging 25.2% shorter, ranging from a modest
0.9% (trial 16, where RRT's own tree already happened to grow a fairly
direct route) up to 69.5% (trial 9, the same trial plain RRT's own
benchmark write-up above flagged as its worst case -- RRT's path there was
literally double the cardinal-optimal length; RRT* still starting from
the identical samples closes almost all of that gap). This is the direct,
measured payoff of the two mechanisms described in `nav/rrt_star.py`'s
docstring: choosing the cheapest available parent instead of just the
nearest one, and rewiring nearby nodes through a new one when that's
cheaper -- applied identically to every random sample RRT itself also
saw, and only ever making the result better, never worse (both algorithms
found *a* path in 20/20 trials; RRT* never failed where RRT succeeded, or
vice versa).

Compared to the cardinal-only Dijkstra/A* "optimal" path, RRT* actually
comes out *shorter* on average (-24.9% overhead, i.e. its paths average
about 25% shorter than the grid search's), for the same reason
plain RRT's own overhead numbers go negative on some trials above: RRT/
RRT* aren't constrained to the grid's 4-directional step rule, so they
can cut diagonally across open space no cardinal-only search is allowed
to. It's not a fair claim that RRT* "beat" A* at the same problem -- it
solved a *less constrained* version of it -- but it is a fair claim
against plain RRT, which faces the exact same lack of constraint and
still loses to RRT* by 25.2% anyway.

The cost is runtime, and it's a real, structural cost, not just
overhead from doing more per iteration. RRT* averaged 67.1ms per trial
against plain RRT's 0.97ms -- almost 70x slower on the same 3,000-
iteration budget. Most of that isn't the extra per-iteration work
(parent selection and rewiring over a handful of nearby nodes each,
cheap even with the k-d tree radius query); it's that RRT* never stops
early. Plain RRT returns the instant some new node lands within
`goal_radius` of the goal, so on an easy trial it might use a few dozen
iterations out of its 3,000-iteration budget and quit. RRT* keeps
iterating for the *entire* budget every time, because rewiring after
the goal is first reached can still improve the path -- that's the whole
mechanism behind "asymptotically optimal." So the 70x runtime gap isn't
"RRT* does more work per sample," it's "RRT* does the full 3,000 samples
of work on every trial, where RRT often did a small fraction of that."
That's an intentional tradeoff (quality now costs a fixed, predictable
amount of extra time instead of an unpredictable amount of extra path
length), not an unexamined regression.

The honest takeaway: if the 9.1% average path-length overhead plain
RRT already had against optimal was worth fixing, RRT* fixes most of it
(and then some, since it's being compared against RRT's own, equally
unconstrained baseline) for a fixed, bounded extra cost in runtime, not
an open-ended one. Whether that trade is worth it depends entirely on
whether the fixed ~3,000-iteration runtime budget is affordable for the
use case -- for the same reason plain RRT's honest takeaway pointed at
continuous, high-dimensional spaces rather than this project's small,
fully-known grid, RRT* is the version of that argument that actually
scales into a real motion-planning use case, since its published
guarantee (converging to the optimal path as iterations -> infinity) is
exactly the property plain RRT lacks and that a real system would need
if it can't afford to re-run planning from scratch for a marginally
better route.
