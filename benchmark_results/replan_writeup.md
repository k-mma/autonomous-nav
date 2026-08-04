# D* Lite vs from-scratch A*: moving-obstacle and sensor-discovery replanning

6 trials per grid size (25x25, 50x50, 100x100, 200x200), 15% obstacle
density, both scenarios recreating the exact replanning policy this
project already uses live (see `nav/replan_benchmark.py`'s docstring for
the precise correspondence to `nav/obstacles.py` and `nav/sensor.py`).
Raw data in `replan_results.csv`, plot in `replan_comparison.png`.
Correctness (does D* Lite actually agree with A*, not just run faster) is
checked separately and exhaustively in `nav/scratch/dstar_lite_test.py`:
40/40 one-shot trials and 207/207 individual incremental repair steps
match A*'s path cost exactly.

## What the data shows

| Size | Moving obstacle: astar | D* Lite | speedup | Sensor discovery: astar | D* Lite | speedup |
|---:|---:|---:|---:|---:|---:|---:|
| 25x25 | 2.054ms | 0.848ms | 2.42x | 1.312ms | 2.695ms | 0.49x |
| 50x50 | 7.181ms | 2.480ms | 2.90x | 3.310ms | 5.268ms | 0.63x |
| 100x100 | 64.940ms | 7.647ms | 8.49x | 30.399ms | 23.153ms | 1.31x |
| 200x200 | 329.304ms | 20.159ms | 16.34x | 222.716ms | 112.853ms | 1.97x |

(Both totals are summed over every replan in the trial, not per-replan --
"speedup" is total astar time / total D* Lite time for the whole trial.)

Moving obstacle: D* Lite wins at every size tested, and the margin
grows fast. 2.4x faster even at the small 25x25 grid this project
actually runs interactively at, climbing to 16.3x by 200x200. This is
the scenario where the mechanism plays out cleanly: an obstacle bouncing
between two fixed cells only ever invalidates a small, localized
neighborhood of the search each time it moves, and D* Lite's
`update_edge_costs` + `compute_shortest_path` only ever touches that
neighborhood -- not the whole grid, regardless of how big the grid is.
A* has no way to reuse anything between calls; it re-explores from the
robot's current cell outward every single time, and that cost grows with
distance-to-goal (hence with grid size), while D* Lite's repair cost
stays close to flat.

Sensor discovery tells a more honest, two-part story: D* Lite is
*slower* at this project's actual scale, and only becomes faster once
the grid is considerably bigger than anything this project runs. At
25x25 and 50x50, D* Lite loses (0.49x and 0.63x -- roughly 2x and 1.6x
*slower* than just calling astar fresh). It crosses over to a genuine
win only at 100x100 (1.31x) and 200x200 (1.97x). Two things are going on
here, both real, not artifacts:

1. Sensor discovery only ever adds obstacles, never removes them
   (`known_obstacles` is monotonically growing -- see WRITEUPS.md's
   sensor-model section), so each replan event tends to touch more newly
   -blocked cells at once than the single bouncing obstacle in the other
   scenario does, and D* Lite's per-vertex bookkeeping (heap push/pop
   with lazy-deletion, dict lookups for g/rhs/entry, a full neighbor scan
   per `update_vertex` call) is real, non-trivial Python-level constant-
   factor overhead. On a small grid, A*'s search itself is *so* cheap
   (a full search barely explores more than the direct path when the
   grid is this size) that D* Lite's bookkeeping overhead per event
   costs more than the search it's replacing.
2. Building a `KnownGrid` from scratch every replan -- exactly what
   `pygame_app/visualizer.py`'s `planning_grid()` and this benchmark's astar
   baseline both do -- also isn't very expensive at 25x25 (comparable to
   the search itself), so there's less baseline cost to begin with for
   an incremental approach to beat.

Both effects shrink relative to D* Lite's real advantage as the grid
grows -- a from-scratch A* search's cost keeps climbing (nearly 170x
from 25x25 to 200x200), while D* Lite's incremental repair cost climbs
far more slowly (about 42x over the same range), because the number of
*newly relevant* cells per sensor update doesn't grow with total grid
size, only with how far the robot has moved and the sensor's fixed
radius -- so the crossover point (somewhere between 50x50 and 100x100
here) is a real property of this specific scenario, not noise.

## The honest takeaway

Neither scenario supports a flat "D* Lite is just faster" claim -- the
real, useful finding is that its advantage is asymptotic, and where the
crossover sits genuinely depends on how localized a single replan event
is. The moving-obstacle scenario (one bouncing obstacle, a strictly
local change every time) wins for D* Lite even at this project's actual
25x25 scale. The sensor-discovery scenario (potentially several newly-
sensed cells at once, plus real overhead in D* Lite's own bookkeeping)
does *not* win at that scale -- it only pays off once the grid is 4-8x
larger than anything this project's pygame visualizer or pybullet demos
actually use. That's not a reason to dismiss the algorithm; it's the
same "wrong tool at this project's actual scale, right tool at a bigger
one" conclusion `benchmark_results/writeup.md` already reached for plain
RRT, arrived at independently and for a structurally different reason
(constant-factor bookkeeping overhead here, versus an O(n) nearest-
neighbor scan there) -- and it's exactly the kind of claim this project's
own stated ethos requires being run and measured rather than assumed.
