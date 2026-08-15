# FTC sensor suite comparison under field/reality deviation

25 trials per (suite, deviation_type, variance_level) point, 11 variance_level steps from 0.0 to 1.0, 3 deviation types swept independently (nav/field_variance.py's *_scale kwargs -- Phase 2), on the 'cluttered' field layout (ftc/field.py). Every suite at a given (deviation_type, variance_level, trial index) runs against the identical ground truth, so a gap between suites reflects the suite, not which random scenario it happened to get. Raw data in `ftc_suite_results.csv`, charts in `ftc_suite_comparison.png` and `ftc_reliability_per_dollar.png`.

## Which suite wins

Overall success rate, variance_level >= 0.3 across all three deviation types (excludes the near-zero-deviation points every suite trivially clears):

| Suite | Cost | Overall success rate |
|---|---:|---:|
| Full suite | $399 | 58% |
| Odometry pods | $280 | 45% |
| AprilTag (front camera) | $25 | 44% |
| Rear camera | $50 | 44% |
| Distance sensors | $94 | 21% |
| Dead reckoning | $0 | 19% |
| IMU | $0 | 19% |

Full suite has the highest overall success rate (58%) at $399. See `ftc_reliability_per_dollar.png` and the value section below for whether that's actually the best *spend*, not just the best raw number.

## Which deviation type dominates real failure

For each suite, the deviation type with the lowest mean success rate at variance_level >= 0.5 -- the one that actually hurts that suite the most in a match:

| Suite | Dominant deviation type | Success rate on that type | start_drift | obstacle_drift | unplanned_blocker |
|---|---|---:|---:|---:|---:|
| Dead reckoning | Start drift (pose error) | 5% | 5% | 41% | 11% |
| Odometry pods | Start drift (pose error) | 9% | 9% | 79% | 27% |
| Distance sensors | Start drift (pose error) | 9% | 9% | 26% | 28% |
| AprilTag (front camera) | Unplanned blocker (opponent robot) | 21% | 38% | 64% | 21% |
| IMU | Start drift (pose error) | 5% | 5% | 41% | 11% |
| Rear camera | Unplanned blocker (opponent robot) | 21% | 38% | 64% | 21% |
| Full suite | Start drift (pose error) | 30% | 30% | 73% | 65% |

With no sensing at all (DeadReckoningSuite, the baseline every FTC team already has for free), Start drift (pose error) is what actually breaks a run. FullSuite's worst deviation type is the *same* one (Start drift (pose error)) -- spending on every suite at once didn't change which failure mode dominates, only how often it happens.

## Do the expensive suites earn their cost?

- AprilTag (front camera) ($25): +25% success rate over the free baseline -- +99.3pp/$100.
- Rear camera ($50): +25% success rate over the free baseline -- +49.7pp/$100.
- Full suite ($399): +39% success rate over the free baseline -- +9.7pp/$100.
- Odometry pods ($280): +26% success rate over the free baseline -- +9.3pp/$100.
- Distance sensors ($94): +2% success rate over the free baseline -- +2.3pp/$100.
- IMU ($0): +0% success rate over the free baseline -- undefined (cost_usd == 0).

AprilTag (front camera) is the best value by success-rate-gained-per-dollar. FullSuite -- the most expensive option -- is also the best raw performer, but its per-dollar return (+9.7pp/$100) is lower than AprilTag (front camera)'s: the extra suites it stacks on top run into diminishing returns rather than each adding its standalone value again.

IMU gained +0% success rate over dead reckoning for $0 -- a real gain (or loss) with no dollar figure to divide it by (every REV Control Hub already ships one; the only real cost is the integration effort of reading and fusing it, ftc/sensors.py's ImuSuite, which this project's dollar-based cost model has no way to price). "Is the free hardware worth the code" has to be answered by the gain itself, not a per-dollar ranking.

## Honest findings

Two of this sweep's own numbers above are easy to misread as a bug rather than what they are -- an explicit consequence of `MODEL_FIDELITY = "optimistic"` (`ftc/config.py`), the tier this whole headline sweep runs at by default. IMU's overall success rate is IDENTICAL to DeadReckoningSuite's, cell for cell across every deviation type, because the optimistic tier's heading drift is 0.0 by construction -- there is no heading error anywhere for an IMU's continuous heading correction to fix, so ImuSuite behaves byte-for-byte like DeadReckoningSuite at this tier. Rear camera's overall success rate is IDENTICAL to AprilTag (front camera)'s, because the optimistic tier's camera FOV is 360 degrees (omnidirectional) -- a second, rear-facing camera adds no coverage a camera that already sees everything didn't already have. Both are real findings about this tier's own stated assumptions, not measurement noise -- `ftc/fidelity_benchmark.py` reruns this sweep at the `realistic`/`pessimistic` tiers, where heading drift and real camera FOV are both live, and both suites separate measurably from their baselines there (see `ftc_fidelity_writeup.md`).

DistanceSensorSuite collides in 48% of trials even at variance_level=0.0 (ground truth cell-for-cell identical to the assumed map, every deviation type at 0) -- so field deviation isn't causing these collisions at all. A controlled check (same trials, drift_per_cell forced to 0) shows roughly two-thirds of these collisions persist with pose error completely disabled, so the dominant cause isn't pose drift -- it's the sensor geometry itself. 3 narrow ToF cones (12.5 deg half-angle each) mounted front/left/right cover only about 75 of the 360 degrees around the robot; anything in the roughly 285-degree gap between cones -- a very plausible place for an obstacle to sit relative to a robot that's mid-turn on a diagonal grid -- is simply never seen until the robot's next planned step walks straight into it. Pose drift is a real, secondary compounding factor on top of that (the same check found collisions drop by roughly a third once drift is disabled, since a correctly-remembered obstacle position still isn't the same as never having missed one), but the primary lesson is blunter than a SLAM-consistency story: a sparse fixed-cone sensor suite has real, geometry-driven blind spots, and this project's own headline nav/ result (reactive beats belief, measured against nav/sensor.py's full disc-scan sensor model, which has no blind spot at all) doesn't transfer to a suite whose sensing coverage is this incomplete. `ftc/coverage_benchmark.py` sweeps more ToF sensors against this exact gap and finds it's worse than 'currently unmet': even at the largest count tested, a structural blind arc survives no FTC-legal ToF sensor count can close (see `ftc_coverage_writeup.md`'s 'Is full coverage even reachable?'). Buying distance sensors without covering enough of the robot's perimeter can be worse than not sensing at all, purely from what the hardware physically cannot see.

No suite ever ran out of the 30-second budget in this sweep -- on the 'cluttered' layout at this grid scale, drive time and replan overhead never came close to 30s even under heavy deviation. A larger/more cluttered layout or a tighter budget would be needed to make AUTONOMOUS_PERIOD_S itself the binding constraint rather than success/collision.
