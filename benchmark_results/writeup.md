# Dijkstra vs A*: 20-trial benchmark

20 random 25x25 grids, obstacle density 10-35%, start/goal picked from free
cells, both algorithms run on the identical grid. Raw data in `results.csv`,
plot in `comparison.png`.

## What the data shows

Across the 20 trials A* explored 68.9% fewer cells than Dijkstra on
average (4,811 total cells for Dijkstra vs 1,498 for A*), most individual
trials landing in the 55-90% range. That's the Manhattan heuristic doing
its job: it steers the search almost straight at the goal instead of
expanding outward in every direction.

The gap narrows sharply when the obstacle field forces a long, winding
path rather than a direct one. Trial 18 (30.6% density, a 43-step path,
the longest of the run) is the clear outlier: A* explored only 19.5%
fewer cells than Dijkstra, and its runtime was actually slightly higher.
Trials with similarly high density but a shorter, straighter path (3 and
16) kept a 55%+ advantage. Density alone doesn't hurt A* much -- density
that forces detours does, because Manhattan distance stops correlating
with true travel cost once the route has to zigzag, and the heuristic
gets less informative exactly when the search gets harder.
