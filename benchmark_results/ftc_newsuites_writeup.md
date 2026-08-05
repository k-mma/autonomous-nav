# New suites enabled by the fidelity-tier model: IMU and dual-camera AprilTag

ImuSuite ($0 -- every REV Control Hub already ships one) corrects HEADING error only, continuously, with no replan cost; AprilTagImuSuite stacks it on AprilTagSuite; DualCameraAprilTagSuite adds a second (rear) camera to AprilTag's detection pipeline. Crossed with fidelity tier (optimistic, realistic) x 3 deviation types x levels [0.3, 0.5, 0.7, 0.9] x 15 trials/point on the 'cluttered' layout -- reduced relative to the headline sweep (see module docstring). Raw data in `ftc_newsuites_results.csv`, chart in `ftc_newsuites_comparison.png`.

## Headline table

| Suite | Cost | Optimistic | Realistic |
|---|---:|---:|---:|
| Dead reckoning | $0 | 21% | 18% |
| AprilTag | $40 | 31% | 24% |
| IMU | $0 | 21% | 19% |
| AprilTag + IMU | $40 | 31% | 25% |
| Dual-camera AprilTag | $80 | 31% | 28% |

## Value ranking (success-rate gain over dead reckoning, per $100)

### optimistic

Dead-reckoning baseline: 21%

| Suite | Cost | Gain | pp/$100 |
|---|---:|---:|---:|
| IMU | $0 | +0% | undefined (cost_usd == 0) |
| AprilTag | $40 | +10% | +25.0 *best priced value* |
| AprilTag + IMU | $40 | +10% | +25.0 |
| Dual-camera AprilTag | $80 | +10% | +12.5 |

ImuSuite gained +0% success rate over dead reckoning for $0 -- a real gain (or loss) with no dollar figure to divide it by. "Is the free hardware worth the code" has to be answered by the gain itself, not a per-dollar ranking: this project's pp/$100 metric is silent on a $0 suite by construction, and reporting it as "infinite value" would be a more misleading claim than reporting it as undefined.

### realistic

Dead-reckoning baseline: 18%

| Suite | Cost | Gain | pp/$100 |
|---|---:|---:|---:|
| IMU | $0 | +2% | undefined (cost_usd == 0) |
| AprilTag + IMU | $40 | +7% | +18.1 *best priced value* |
| AprilTag | $40 | +7% | +16.7 |
| Dual-camera AprilTag | $80 | +10% | +12.5 |

ImuSuite gained +2% success rate over dead reckoning for $0 -- a real gain (or loss) with no dollar figure to divide it by. "Is the free hardware worth the code" has to be answered by the gain itself, not a per-dollar ranking: this project's pp/$100 metric is silent on a $0 suite by construction, and reporting it as "infinite value" would be a more misleading claim than reporting it as undefined.

## Does the dual camera actually matter -- and only where expected?

| Fidelity | AprilTag (1 camera) | Dual-camera AprilTag (2 cameras) | Difference |
|---|---:|---:|---:|
| optimistic | 31% | 31% | +0% |
| realistic | 24% | 28% | +3% |

Exactly as expected: a second camera makes essentially no difference under the optimistic tier's omnidirectional-camera assumption (an omnidirectional camera already sees everything a second one could add), but measurably helps under the realistic tier's real camera-FOV gating -- a clean demonstration of why the Priority 1 fidelity fix mattered. Under the OLD (pre-fidelity-tier) model, DualCameraAprilTagSuite would have been indistinguishable from AprilTagSuite; it isn't, once heading_deg_now actually gates detection.

## What this does and does not prove

This is a reduced-rigor sweep (see module docstring), enough to see the shape of both effects, not a publication-grade confidence interval on the exact magnitude. ImuSuite's IMU_HEADING_CORRECTION_FACTOR and DualCameraAprilTagSuite's second-camera placement (front + rear) are both documented ballpark engineering choices, same status as every other estimated constant in this project.
