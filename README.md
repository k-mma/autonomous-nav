# autonomous-nav

### Less is More: Feature Selection for Autonomous Navigation

[![tests](https://github.com/k-mma/autonomous-nav/actions/workflows/tests.yml/badge.svg)](https://github.com/k-mma/autonomous-nav/actions/workflows/tests.yml)

**A pathfinding and sensor-strategy testbed for *FIRST* Tech Challenge (FTC)
robotics.** Five path-planning algorithms (Dijkstra, A\*, RRT, RRT\*, D\* Lite)
sit under a belief-planning and sensor-fusion layer that runs thousands of
controlled, repeatable autonomous-period trials — the statistical power a real
FTC season's ~10 matches can never supply — to answer one question: **which
sensors are actually worth the money, and when?**

![Four sensor suites replaying the identical seeded FTC match side by side. Dead reckoning has drifted 18.8 inches from its true position and is stuck; AprilTag, its camera field-of-view drawn as a blue cone, has corrected to 0.8 inches and reached the goal; odometry pods is stuck at 13.0 inches; the distance-sensor suite's three narrow ToF cones are drawn in tan, visibly covering only a fraction of the robot's surroundings](screenshots/pygame/ftc_suite_replay.png)

Four sensor suites driving the *identical* seeded 30-second match, in real
time (`pygame_app/scenarios/scenario_ftc_suites.py`). The gray outline behind
each robot is where it *believes* it is; the gap is accumulated pose error.
AprilTag's camera FOV is the blue cone; the distance suite's three ToF cones
are the tan wedges, and the unshaded space between them is what it cannot see.
This is one scenario, not a ranking — the aggregate over 5,775 matches is in
[Results](#results).

## Headline results

- **Best value is not best performance.** A $25 AprilTag webcam returns 32x
  the success-rate gain per dollar of the $314 "buy everything" bundle
  (+16.0 vs. +0.5 points per $100), even though odometry pods alone post the
  highest raw success rate (30%). Stacking sensors hits diminishing returns
  fast. *(This ranking is tier-dependent — see the caveat under
  [Results](#results).)*
- **The "obviously better" fix backfires.** Naively merging AprilTag +
  odometry pods looks good (35% success), but a real confidence-weighted
  fusion of those same two sensors *drops* it to 22% — below odometry alone
  (25%). More sensors, fused more carefully, made the robot less reliable.
- **A "safe-looking" sensor can be the worst buy.** The 3-sensor distance
  suite is the only paid upgrade to land *below* the free baseline (10% vs.
  14%, p=0.003). Its cones see ~75° of 360°, and reacting to that fragment
  routes the robot into the 285° it cannot see.
- **Underneath it all, A\* explores 68.9% fewer cells than Dijkstra** for an
  identical-cost path — 241 vs. 75 cells across 20 random 25x25 grids. The
  planner is infrastructure here, not the finding; picking it was a
  completeness argument, not a speed one.

Numbers and 95% bootstrap CIs behind each claim: [Results](#results) below,
`benchmark_results/ftc_suite_writeup.md`, and [VALIDITY.md](VALIDITY.md).

## Poster

The one-page version of everything here, built for a research symposium at
UC Berkeley.

[![Conference poster: Less is More — Feature Selection for Autonomous Navigation](docs/poster-preview.png)](docs/poster.pdf)

**[Full-size PDF →](docs/poster.pdf)**

## Quickstart

Python 3.11+.

```bash
python3 -m venv nav-env
source nav-env/bin/activate
pip install -r requirements.txt
```

```bash
python3 pygame_app/main.py       # interactive visualizer (screenshot above)
python3 -m ftc.suite_benchmark   # regenerate the headline study (~1 min)
python3 -m ftc.recommend         # "which sensors should I buy" decision CLI
python3 -m ftc.optimizer         # which COMBINATION of sensors to buy
```

```bash
pip install -r requirements-dev.txt   # adds pytest
python3 -m pytest                     # CI runs this on every push
```

Preset scenarios, the PyBullet 3D demos, and every side-study benchmark are in
[WRITEUPS.md's command reference](WRITEUPS.md#command-reference). If `pip
install` fails building `pybullet` from source (common on very new
macOS/Xcode), the one-line fix is in WRITEUPS.md's "A build problem worth
documenting".

### Visualizer controls

| Key / click | Action |
|---|---|
| Left-click | Toggle obstacle |
| Shift + Left-click | Place/remove a moving obstacle |
| Right-click / Shift + Right-click | Place start / goal |
| `D` / `A` / `R` | Switch algorithm (Dijkstra / A\* / RRT) |
| Space | Run the active algorithm |
| `W` | Start/stop the robot walking the path |
| `H` | Cycle A\*'s heuristic |
| `X` | Toggle 8-directional movement |
| `M` / `K` / `S` / `C` | New maze / cost map / lidar sensor / clear |

## The research question

FTC autonomous is a 30-second dash: drive from a known start to a scoring
position on a pre-programmed route, with no driver input. Every team already
senses *something* — at minimum, motor encoders — and can buy more: odometry
pods, distance sensors, AprilTag vision, or all three. Each costs real money
and integration time.

Which one is worth it isn't obvious from specs, because the answer depends on
*which kind* of deviation shows up on a given field. A robot that starts a few
inches off its mark needs pose correction, not obstacle sensing; a field whose
elements don't match the CAD, or an opponent parked somewhere unplanned, needs
the opposite. These are two different failure modes with two different fixes,
not one generic "uncertainty" axis.

![Two field diagrams side by side. Left, pose error: the robot's true position has diverged from where it believes it is, its gold true track peeling away from the blue planned route. Right, obstacle error: the map said one thing and the field says another, with a red obstacle blocking the planned route that the map never had](screenshots/poster%20figures/figure1_pose_vs_obstacle_error.png)

A team gets roughly 10 matches a season — one noisy, unrepeatable trial each,
no ground truth, no control over how far reality deviates from what the
routine assumed. That is not enough data to answer this empirically, no matter
how many matches they play. That's the actual thesis here: simulation is the
only viable instrument for this question, not a stand-in for hardware you'd
use if you had more of it.

`ftc/suite_benchmark.py` sweeps 7 sensor suites (`ftc/sensors.py`) against 3
independently-scaled deviation types (`nav/field_variance.py`) at 11 deviation
levels on a real 30-second budget (`ftc/match.py`) — 5,775 matches, in about a
minute. One match runs like this:

![Flow diagram of a single simulated match: PLAN an A* route from the believed start over the assumed map, DRIVE one step while pose error grows with distance driven, SENSE to correct pose and reveal real obstacles, RE-PLAN if the map changed, repeating every step; after each step the match checks for success when the true position reaches the goal, or failure by collision or by the 30-second period expiring](screenshots/poster%20figures/match_lifecycle.png)

The split that makes the whole study work: the robot only ever acts on what it
**believes**, and the match is scored on what is **true**.

## Results

25 trials x 7 suites x 3 deviation types x 11 levels, on the "cluttered" field
layout. Full breakdown in `benchmark_results/ftc_suite_writeup.md`.

![Three side-by-side line charts, one per deviation type -- start drift (pose error), obstacle drift (map error), and unplanned blocker (opponent robot) -- tracking success rate against deviation severity for dead reckoning, AprilTag, and odometry pods. Each suite degrades on a different curve, and which one leads changes between panels](screenshots/poster%20figures/severity_curves.png)

Three suites shown for legibility; the all-7 version with 95% bootstrap CI
bands is `benchmark_results/ftc_suite_comparison.png`.

| Suite | Cost | Success rate (variance_level >= 0.3) |
|---|---:|---:|
| Odometry pods | $195 | 30% |
| AprilTag (front camera) | $25 | 18% |
| Rear camera | $50 | 18% |
| Full suite | $314 | 16% |
| Dead reckoning (baseline) | $0 | 14% |
| IMU | $0 | 14% |
| Distance sensors | $95 | 10% |

![Horizontal bar chart of success-rate gain per $100 spent, highest value first: AprilTag front camera at $25 returns +16.0 points per $100, front plus rear cameras +8.0, odometry pods +7.8, the $314 full suite only +0.5, and distance sensors a negative -4.2](screenshots/poster%20figures/figure3_reliability_per_dollar.png)

Odometry pods wins on raw success rate; AprilTag wins on value by a wide
margin (+16.0 points per $100 over the free baseline, vs. the full suite's
+0.5 — a 32x gap). The suites the full bundle stacks on top run into
diminishing returns rather than each adding its standalone value again. Every
cost is a real, currently-listed vendor price (REV Robotics, Optii, Logitech);
see `ftc/config.py`'s per-constant source comments.

**The value ranking is tier-dependent, and this is the honest caveat.** Every
number in this section assumes `ftc/config.py`'s `"optimistic"` fidelity tier —
an omnidirectional camera and perfect heading knowledge, both real unmodeled
optimisms. `ftc/fidelity_benchmark.py` reruns the identical sweep one step
down, and best value flips from AprilTag (+16.0/$100) to **Odometry pods**
(+8.8/$100) at `"realistic"`, staying there at `"pessimistic"`. AprilTag's
raw-success and cost figures hold; its *best-value crown* does not survive
leaving the optimistic tier. [Detail](VALIDITY.md#camera-fov-heading-error)

### The most useful result is negative

The distance-sensor suite is the only paid upgrade that lands *below* the free
baseline — 10% vs. 14%, a paired difference of -4.0% (95% CI [-6.7%, -1.3%],
p=0.003). Adding a sensor measurably hurt.

![Polar diagram of the robot's surroundings: three narrow green wedges, one forward and one to each side, cover about 75 degrees in total, while the remaining 285 degrees is shaded as unsensed, with an obstacle marked in the blind region annotated as invisible until the robot drives into it](screenshots/poster%20figures/sensor_coverage.png)

The geometry is manufacturer spec, not an estimate: the REV-31-1505 has a 25°
field of view and 2m range, so three of them mounted front/left/right — the
usual FTC pattern — see ~75° of 360°. But the coverage gap alone would predict
*no benefit*, not active harm. The mechanism behind the harm shows up in the
per-trial data: these robots collide barely more often than dead reckoning
(79.5% vs. 77.3%), but they do it after 3.1 steps instead of 6.6. **Acting on a
fragment of obstacle truth diverts them into the 285° they cannot see**, where
dead reckoning at least follows a route planned against the whole assumed map.
More sensors don't rescue it — `ftc/coverage_benchmark.py` sweeps up to 8 ToF
sensors ($252) and success only moves 14% → 16%, because a 160° blind arc
survives any FTC-legal count.

**Scope this one carefully.** It is a finding about using ToF sensors as
general obstacle detectors, which is how this model deploys them. Real teams
more often use them for wall alignment and short-range squaring against a
*known* target — a use this study doesn't model at all, and one where a narrow
cone pointed where you already expect something is exactly the right tool. Read
it as "narrow-cone sensing plus reactive replanning is a bad general obstacle
strategy," not as "don't buy distance sensors."

### Which combination to buy

The table above compares fixed suites; a team's real question is a shopping
question. `ftc/bundle.py` composes suites into one working suite — costed over
the **union of their parts**, which is what reduces 63 raw combinations to 23
genuinely distinct robots — and `ftc/optimizer.py` searches that space.

![Grouped bar chart across five scenarios, comparing the best single sensor against the best 2-or-more-part bundle in each. The bundle wins or ties everywhere: the $245 odometry-plus-two-cameras build takes four scenarios, and in the tight-corridor case the $220 odometry-plus-front-camera build reaches 62 percent against the best single sensor's 39 percent](screenshots/poster%20figures/figure5_scenario_deepdive.png)

This ran over two scenario catalogs, and they name different robots — worth
being explicit about, because the difference is the point rather than a
discrepancy:

| | Baseline catalog | Match-realistic catalog |
|---|---|---|
| Profiles / trials | 5 x 25 (2,875 matches) | 5 x 90 (10,350 matches) |
| Fidelity | optimistic | realistic |
| **Best average** | odometry + front camera ($220, 34%) | odometry + front + rear camera ($245, 26%) |
| **Best worst-case** | *everything ties at 0%* — so $0 wins | odometry + front + rear camera ($245, 12%) |
| Writeup | `ftc_optimizer_writeup.md` | `ftc_optimizer_match_writeup.md` |

Read that worst-case row carefully: in the baseline catalog *every* robot has
some scenario it never finishes, so "most robust" is a tie the free option wins
on price, not evidence that encoders are sturdy. The match-realistic catalog
separates them properly.

The match-realistic catalog is the one to trust for a purchasing decision: it
runs at realistic fidelity with mixed deviation axes per scenario, and there
best-average and most-robust name the *same* robot, so there's no tradeoff to
weigh. The optimistic catalog's headline $220 robot is the cheaper answer to an
easier question.

Per scenario (chart above), a bundle wins or ties everywhere — the $245 build
takes four of the five, and the $220 build takes the tight-corridor case, where
the rear camera has no room to earn its $25. The cost-vs-success Pareto
frontier over all 23 buildable robots is
`benchmark_results/ftc_optimizer_match_frontier.png`.

Either way the cheapest *significant* upgrade is the same: adding AprilTag to
odometry pods, +8.8% (95% CI [+4.0%, +14.4%], p<0.001) for $25.

Because every candidate runs the *identical* seeded scenarios, "is this bundle
better?" is answered with a **paired** bootstrap, not by checking whether two
independent CIs overlap. That distinction is not cosmetic: two candidates whose
independent CIs overlap heavily (20-50% vs. 35-65%) can have a paired
difference of [+5.0%, +27.5%], p=0.004.

Two findings worth stating plainly:

- **Bundling works, but only across capability categories.** Every bundle that
  significantly beat its own best single component spans more than one
  category (pose fixing / obstacle sensing / drift reduction / heading
  holding) *and* adds one that component lacked. Two sensors fixing the same
  failure mode don't stack. Buy across failure modes, not the two best sensors.
- **Greedy reasoning happens to work here.** On the baseline catalog, forward
  selection lands on the same robot as exhaustive search: from odometry pods
  (26%, $195), its one addition — AprilTag — is itself significant (p<0.001),
  and nothing further is worth adding.

## Threats to validity

Naming these plainly is what separates a research testbed from a demo. Full
mechanism, numbers, and CIs for all 12 are in [VALIDITY.md](VALIDITY.md);
the three that most constrain the headline result:

- **Synthetic ground truth.** Every trial's "ground truth" is a procedurally
  perturbed copy of the assumed map, not a measurement of a real field — a
  hypothesis about what deviation matters, not a validated one.
  [Detail](VALIDITY.md#synthetic-ground-truth)
- **Camera FOV and heading error.** Every number above assumes the
  `"optimistic"` fidelity tier (omnidirectional camera, perfect heading). The
  best-value suite does *not* survive the first step off it — it flips from
  AprilTag to Odometry pods at "realistic".
  [Detail](VALIDITY.md#camera-fov-heading-error)
- **No sensor-fusion conflict.** Bounded for AprilTag vs. odometry pods, open
  for every other pairing — this is the fusion reversal in the headline
  results above. [Detail](VALIDITY.md#sensor-fusion-conflict)

## How it fits together

`nav/` is a domain-neutral belief-planning library — grid, planners, sensor
and occupancy models, benchmarks — with no FTC-specific identifiers anywhere
in it. `ftc/` is a separate package holding the domain specialization (an 18in
robot, a 144in field, a 30-second clock, AprilTags). Keeping the boundary at
the package level, rather than scattering FTC `if` branches through `nav/`, is
what lets the core stay reusable for a different robot, field, or competition.

```
nav/           Framework-agnostic core: grid, planners, sensor/occupancy
               models, field variance, policies, stats, benchmarks
ftc/           FTC domain layer: field, sensor suites, match model, the
               headline study, bundle optimizer, decision CLIs
pygame_app/    2D interactive visualizer + preset scenarios
pybullet_app/  3D port: single-robot, multi-robot, and CBS demos
benchmark_results/  Generated CSVs, plots, and writeups
docs/          Symposium poster
```

Neither app directory is named literally `pygame` or `pybullet` — a directory
with that exact name on `sys.path` would shadow the real installed library.
The file-by-file tree is in
[WRITEUPS.md](WRITEUPS.md#repo-layout-the-full-annotated-tree).

### Planners

Every planner is a backend `nav/algorithms.py`'s `find_path` can swap in, but
`ftc/` uses A\* exclusively — and the reason is a completeness argument, not a
speed one. Dijkstra and A\* are resolution-complete (guaranteed to find a path
at the grid's resolution if one exists); RRT is only probabilistically
complete (guaranteed as sample count → ∞, not at any fixed budget).
`nav/scale_benchmark.py` shows that gap as measured incompleteness as grids
grow:

![Horizontal bar chart of planning time on a log scale for five algorithms: A* selected at 0.25ms exploring 75 cells, 69 percent fewer than Dijkstra's 241 at 0.74ms; RRT at 0.99ms taking a 9.2 percent longer route; RRT* at 67.72ms for a 25 percent shorter route but 268 times longer to compute; and D* Lite noted as a route repairer that is no faster than A* at FTC field scale](screenshots/poster%20figures/figure2_ftc_field.png)

![The same grid, start, and goal run through all three planners: Dijkstra floods nearly every reachable cell before finding a path, A* reaches an identical-cost path having explored a small fraction of that area, RRT sketches a rougher, longer tree-based route](screenshots/pygame/forest_corridor.png)

| Size | Dijkstra | A\* | RRT (k-d tree) | RRT found path |
|---:|---:|---:|---:|---:|
| 20x20 | 0.465ms | 0.227ms | 0.443ms | 8/8 |
| 50x50 | 2.679ms | 0.762ms | 1.520ms | 8/8 |
| 100x100 | 9.732ms | 1.156ms | 15.162ms | 7/8 |
| 200x200 | 56.511ms | 9.968ms | 58.548ms | **5/8** |

A 100x increase in cells grows Dijkstra's runtime ~120x but A\*'s only ~44x.
`nav/kdtree.py`'s spatial index made RRT faster than Dijkstra at every size
above (it used to be 620x slower) — but the completeness gap is untouched by
that, which is the point. RRT/RRT\* stay in the repo as measured baselines for
*why* an exhaustive heuristic search wins here, not as something `ftc/` should
reach for. `nav/rrt_star.py` and `nav/dstar_lite.py` (measured against fresh
A\* in `nav/replan_benchmark.py`) round out the set.

`pybullet_app/` ports the identical `nav.grid.Grid` into 3D and plans with the
same unmodified `find_path`: a Husky driven by velocity control along a
spline-smoothed route, a real raycast lidar (`pybullet.rayTestBatch`, 48 rays)
that replans on discovery, a two-robot corridor conflict, and an N-robot
intersection coordinated by `nav/cbs.py`'s Conflict-Based Search. Mechanics,
build stories, and the bugs found along the way are in
[WRITEUPS.md](WRITEUPS.md).

## Further reading

- **[WRITEUPS.md](WRITEUPS.md)** — algorithm mechanics, the admissibility
  argument, cost map and sensor model design, the PyBullet port, sweep
  parallelization, the full repo tree, and the command reference.
- **[VALIDITY.md](VALIDITY.md)** — all 12 threats to validity, each with its
  mechanism, numbers, and current status.
- **`benchmark_results/`** — every study's raw CSV, plot, and writeup.

---

Independent project, inspired by my time on an FTC team in high school.
