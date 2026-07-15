# autonomous-nav

A pygame grid-pathfinding visualizer comparing Dijkstra and A\*, with dynamic
obstacles, automatic replanning, and a benchmark suite. Built as the first
leg of a longer autonomous-navigation project that continues into ROS 2 /
Nav2 (see `WRITEUPS.md` for the full narrative and the algorithm/interview
explanations behind the code).

## What it does

- Click to draw obstacles on a 25x25 grid, place a start and goal, and run
  Dijkstra or A\* to watch the search expand cell by cell.
- Handles the edge cases a naive search would crash on: start/goal on an
  obstacle, no path to the goal, start == goal.
- Drop moving obstacles that bounce between two cells; send a robot down
  the computed path and watch it automatically replan when an obstacle
  blocks its route.
- Cycle A\*'s heuristic live (Manhattan / Euclidean / Octile / an
  intentionally inadmissible one) to see search effort and path quality
  change.
- Toggle 8-directional movement, or generate a fresh maze with recursive
  backtracking.
- `nav/benchmark.py` runs both algorithms across 20 random grids and plots
  the comparison (see results below).

## How to run

```bash
python3 -m venv nav-env
source nav-env/bin/activate
pip install -r requirements.txt

python3 main.py              # the visualizer
python3 -m nav.benchmark     # regenerate benchmark_results/
```

### Controls

| Key / click | Action |
|---|---|
| Left-click | Toggle obstacle |
| Shift + Left-click | Place/remove a moving obstacle |
| Right-click | Place start |
| Shift + Right-click | Place goal |
| `D` / `A` | Switch active algorithm |
| Space | Run the active algorithm |
| `R` | Start/stop the robot walking the current path |
| `H` | Cycle A\*'s heuristic |
| `X` | Toggle 8-directional movement |
| `M` | Generate a new maze |
| `C` | Clear the grid |

## The algorithms, briefly

**Dijkstra** always expands the cheapest-so-far cell. It's guaranteed
optimal and doesn't need any notion of "closer to the goal" -- it just
explores outward in cost order, which is why it explores in ripples that
don't obviously point at the goal.

**A\*** expands the cell with the lowest `f = g + h`, where `g` is cost so
far (same as Dijkstra) and `h` is a heuristic estimate of the remaining
cost. As long as `h` never overestimates the true remaining cost
("admissible"), A\* is still guaranteed optimal, but explores dramatically
fewer cells because the heuristic steers it toward the goal instead of
outward in every direction.

Full mechanics, the admissibility argument, the replanning policy, and the
heuristic-breaking experiments are written up in `WRITEUPS.md`.

## Dijkstra vs A\*: benchmark results

20 random 25x25 grids, 10-35% obstacle density, both algorithms run on the
identical grid/start/goal. Full data in `benchmark_results/results.csv`,
plot in `benchmark_results/comparison.png`, analysis in
`benchmark_results/writeup.md`.

| Trial | Obstacle density | Path length | Dijkstra cells | A\* cells | Dijkstra ms | A\* ms |
|---:|---:|---:|---:|---:|---:|---:|
| 1  | 0.260 | 11 | 79  | 20  | 0.24 | 0.07 |
| 2  | 0.154 | 33 | 455 | 163 | 1.34 | 0.50 |
| 3  | 0.327 | 27 | 292 | 76  | 0.74 | 0.21 |
| 4  | 0.178 | 27 | 402 | 81  | 1.10 | 0.25 |
| 5  | 0.214 | 33 | 450 | 152 | 1.26 | 0.57 |
| 6  | 0.193 | 24 | 390 | 115 | 1.12 | 0.35 |
| 7  | 0.234 | 5  | 30  | 11  | 0.08 | 0.03 |
| 8  | 0.171 | 7  | 85  | 12  | 0.23 | 0.04 |
| 9  | 0.298 | 19 | 108 | 40  | 0.27 | 0.15 |
| 10 | 0.111 | 21 | 212 | 62  | 0.76 | 0.23 |
| 11 | 0.172 | 12 | 153 | 29  | 0.42 | 0.09 |
| 12 | 0.158 | 29 | 350 | 97  | 1.23 | 0.58 |
| 13 | 0.137 | 18 | 244 | 25  | 0.64 | 0.08 |
| 14 | 0.177 | 15 | 286 | 52  | 0.93 | 0.16 |
| 15 | 0.180 | 25 | 310 | 91  | 0.80 | 0.27 |
| 16 | 0.330 | 7  | 34  | 15  | 0.17 | 0.04 |
| 17 | 0.231 | 14 | 257 | 33  | 0.68 | 0.10 |
| 18 | 0.306 | 43 | 399 | 321 | 1.04 | 0.90 |
| 19 | 0.151 | 8  | 88  | 19  | 0.24 | 0.06 |
| 20 | 0.203 | 22 | 187 | 84  | 0.47 | 0.24 |

A\* explored 68.9% fewer cells than Dijkstra on average. The gap collapses
on trial 18 -- the longest, most obstacle-dense route in the set -- because
a forced detour makes Manhattan distance a much weaker predictor of true
travel cost. See `benchmark_results/writeup.md` for the full breakdown.

## Repo layout

```
nav/
  grid.py          Grid model: cells, obstacles, start/goal, neighbors
  algorithms.py     Dijkstra, A*, edge-case handling, path cost
  heuristics.py     Manhattan / Euclidean / Chebyshev / Octile / scaled
  obstacles.py      Moving obstacles + the replanning policy
  maze.py           Recursive-backtracking maze generator
  visualizer.py     The pygame app
  benchmark.py      20-trial Dijkstra vs A* benchmark -> CSV + plot
  scratch/          Standalone throwaway scripts used to prove each piece
                     works before it was wired into the visualizer
benchmark_results/  Generated CSV, plot, and writeup from benchmark.py
WRITEUPS.md         Algorithm explanations, replanning policy, and the
                     heuristic experiments' findings
```
