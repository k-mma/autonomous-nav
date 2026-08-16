# Does planning-time tail latency ever change a match outcome?

Measured on Darwin 25.5.0, arm64, Python 3.11.9 -- see "Honest findings" below for why this number does not transfer to different hardware, including the actual FTC-legal onboard compute (a REV Control Hub) this project's PLANNING_OVERHEAD_S constant is meant to represent.

12 trials x 5 grid sizes (24, 48, 96, 192, 384 cells/side) x 3 layouts x 7 suites = 1260 matches, 1843 individual planning calls. Raw data (one row per call) in `planning_latency.csv`, chart in `planning_latency_comparison.png`.

## What ftc/match.py actually does with planning time

There is no per-tick wall-clock deadline anywhere in this codebase. `elapsed_s` is a purely SIMULATED time accumulator; the only wall-clock measurement `run_match` ever takes is around each `astar()` call, and that measured value (`MatchResult.planning_time_s`) is never added to `elapsed_s`. What IS charged is a flat `PLANNING_OVERHEAD_S` (50ms, ftc/config.py) on every replan AFTER the first -- the very first planning call in a match costs 0 simulated seconds, charged or measured, no matter how long it actually took. This is a deliberate, documented choice (PLANNING_OVERHEAD_S's own comment: raw Python `astar()` "is sub-millisecond... and would understate what a real re-plan actually costs"), not an oversight -- but it has a structural consequence worth stating plainly: under the model exactly as implemented, no measured planning latency, however large, can change `over_budget` or `success`. Tail latency is measured and reported (`planning_time_s`, `avg_planning_ms` in other benchmarks' aggregates) but never charged.

## The distribution

| Grid size | n calls | median | p99 | max |
|---:|---:|---:|---:|---:|
| 24 | 714 | 0.35ms | 3.95ms | 4.26ms |
| 48 | 349 | 0.44ms | 3.83ms | 8.08ms |
| 96 | 269 | 0.60ms | 2.32ms | 3.84ms |
| 192 | 256 | 0.71ms | 2.28ms | 309.99ms |
| 384 | 255 | 0.73ms | 2.25ms | 1351.35ms |

The median stays well under PLANNING_OVERHEAD_S (50ms) at every size tested -- planning is not the bottleneck on average, at any size here. The MAX does not: at grid size 192, the single slowest observed planning call (310.0ms) already exceeds the flat constant this project charges for an entire replan. That is exactly the average-fine/tail-not gap this study set out to check for -- on THIS machine, at THIS grid size, for THIS Python implementation of A*.

## Does the tail change a match outcome?

For every match the flat-cost model reports as under budget, the counterfactual: what if `elapsed_s` had used this exact match's REAL measured total planning time instead of `replans * PLANNING_OVERHEAD_S`? (This also charges the first plan, which the flat model never charges at all.)

| Grid size | Under-budget matches | Flipped to over-budget | Rate | 95% CI |
|---:|---:|---:|---:|---|
| 24 | 252 | 0 | 0.0% | [0.0%, 0.0%] |
| 48 | 252 | 0 | 0.0% | [0.0%, 0.0%] |
| 96 | 252 | 0 | 0.0% | [0.0%, 0.0%] |
| 192 | 252 | 0 | 0.0% | [0.0%, 0.0%] |
| 384 | 252 | 0 | 0.0% | [0.0%, 0.0%] |

At this project's actual published grid scale (24 cells/side, resolution multiplier 1x -- what every existing benchmark_results/ CSV in this repo was measured at), no match flips outcome under this counterfactual. The tail is real (see the distribution table above) but never large enough, at this grid size, to move `elapsed_s` past `AUTONOMOUS_PERIOD_S` on top of everything else already charged in a 30-second match -- planning latency's absolute scale (single-digit milliseconds even at p99) is just too small relative to the budget for this to matter here. This BOUNDS the question this project's other benchmarks leave unstated: not "could tail latency ever matter" (yes, at large enough scale -- see the rows above) but "does it matter at the scale every published number in this repo was actually measured at" (no).

No synthetic grid size tested flips any match either -- the gap between measured tail latency and the 30-second budget stays wide even at 16x this project's native grid size.

## Replan frequency by suite

| Suite | Mean replans / match |
|---|---:|
| Dead reckoning | 0.00 |
| Odometry pods | 0.00 |
| Distance sensors | 0.13 |
| AprilTag (front camera) | 1.25 |
| IMU | 0.00 |
| Rear camera | 0.96 |
| Full suite | 0.90 |

AprilTag replans more often than DistanceSensorSuite in this sweep too (1.25 vs. 0.13 per match) -- consistent with ftc/budget_benchmark.py's own docstring claim (AprilTag replans on every pose correction, not just on newly-sensed obstacles) rather than contradicting it.

## Honest findings

- **Hardware.** Every latency figure above was measured on the machine named at the top of this file -- a developer laptop, not FTC-legal competition hardware (a REV Control Hub). PLANNING_OVERHEAD_S is explicitly documented as an estimate of REAL onboard-compute cost, acknowledged in that same comment to be well above raw Python astar() time on a machine like this one. These results say nothing about whether PLANNING_OVERHEAD_S is calibrated correctly for real hardware -- only ftc/calibration.py run against real onboard measurements could do that. A slower target would show this same tail-latency effect at a smaller grid size than reported here, not a different effect.
- **`cell_size_in` was deliberately not swept.** Every `*_CELLS` sensor-range constant in ftc/config.py (APRILTAG_RANGE_CELLS, DISTANCE_SENSOR_RANGE_CELLS, ...) is computed once at import time from the native CELL_SIZE_IN and does not read whatever `cell_size_in` a particular `build_grid()` call used. This study only ever varies `size` (a bigger physical field, same 6in cells), which keeps every sensor's real-world range correct -- but it means `ftc/field.py`'s `cell_size_in` parameter is not currently safe to vary independently of the constants in ftc/config.py, a limitation this study surfaced while designing its own scenario generation rather than something already documented elsewhere in this repo.
- **Grid sizes beyond 24 cells/side are a synthetic computational stress test, not a claim about a physically larger competition field.** `ftc/field.py`'s `DEFAULT_TAG_SITES` (and every named layout's own obstacle placement) are fixed in real inches and do not reposition as `size` grows -- at 16x scale, AprilTag's tag sites sit in a small corner of a much bigger grid rather than spread across its actual perimeter. The replan-frequency table above pools across sizes, so this doesn't distort it, but any AprilTag-specific number broken out BY size at the larger end should be read with that in mind.
- Start/goal pairs are bounded to a comparable path-length range at every grid size (see module docstring) specifically so drive time doesn't confound this study's grid-size axis -- a genuinely unconstrained scenario sampler would make matches at large sizes fail from drive time alone long before planning latency became relevant, which would answer a different question than the one asked.
