# New suites enabled by the fidelity-tier model: IMU and dual-camera AprilTag

ImuSuite ($0 -- every REV Control Hub already ships one) corrects HEADING error only, continuously, with no replan cost; AprilTagImuSuite stacks it on AprilTagSuite; DualCameraAprilTagSuite adds a second (rear) camera to AprilTag's detection pipeline. Crossed with fidelity tier (optimistic, realistic) x 3 deviation types x levels [0.3, 0.5, 0.7, 0.9] x 15 trials/point on the 'cluttered' layout -- reduced relative to the headline sweep (see module docstring). Raw data in `ftc_newsuites_results.csv`, chart in `ftc_newsuites_comparison.png`.

## Headline table

| Suite | Cost | Optimistic | Realistic |
|---|---:|---:|---:|
| Dead reckoning | $0 | 21% | 18% |
| AprilTag (front camera) | $25 | 43% | 27% |
| IMU | $0 | 21% | 19% |
| AprilTag + IMU | $25 | 43% | 27% |
| Rear camera | $50 | 43% | 30% |

## Value ranking (success-rate gain over dead reckoning, per $100)

### optimistic

Dead-reckoning baseline: 21%

| Suite | Cost | Gain | pp/$100 |
|---|---:|---:|---:|
| IMU | $0 | +0% | undefined (cost_usd == 0) |
| AprilTag (front camera) | $25 | +22% | +88.9 *best priced value* |
| AprilTag + IMU | $25 | +22% | +88.9 |
| Rear camera | $50 | +22% | +44.4 |

ImuSuite gained +0% success rate over dead reckoning for $0 -- a real gain (or loss) with no dollar figure to divide it by. "Is the free hardware worth the code" has to be answered by the gain itself, not a per-dollar ranking: this project's pp/$100 metric is silent on a $0 suite by construction, and reporting it as "infinite value" would be a more misleading claim than reporting it as undefined.

### realistic

Dead-reckoning baseline: 18%

| Suite | Cost | Gain | pp/$100 |
|---|---:|---:|---:|
| IMU | $0 | +2% | undefined (cost_usd == 0) |
| AprilTag + IMU | $25 | +9% | +37.8 *best priced value* |
| AprilTag (front camera) | $25 | +9% | +35.6 |
| Rear camera | $50 | +12% | +24.4 |

ImuSuite gained +2% success rate over dead reckoning for $0 -- a real gain (or loss) with no dollar figure to divide it by. "Is the free hardware worth the code" has to be answered by the gain itself, not a per-dollar ranking: this project's pp/$100 metric is silent on a $0 suite by construction, and reporting it as "infinite value" would be a more misleading claim than reporting it as undefined.

## Does the dual camera actually matter -- and only where expected?

| Fidelity | AprilTag (1 camera) | Dual-camera AprilTag (2 cameras) | Difference |
|---|---:|---:|---:|
| optimistic | 43% | 43% | +0% |
| realistic | 27% | 30% | +3% |

Exactly as expected: a second camera makes essentially no difference under the optimistic tier's omnidirectional-camera assumption (an omnidirectional camera already sees everything a second one could add), but measurably helps under the realistic tier's real camera-FOV gating -- a clean demonstration of why the Priority 1 fidelity fix mattered. Under the OLD (pre-fidelity-tier) model, DualCameraAprilTagSuite would have been indistinguishable from AprilTagSuite; it isn't, once heading_deg_now actually gates detection.

## What this does and does not prove

This is a reduced-rigor sweep (see module docstring), enough to see the shape of both effects, not a publication-grade confidence interval on the exact magnitude. ImuSuite's IMU_HEADING_CORRECTION_FACTOR and DualCameraAprilTagSuite's second-camera placement (front + rear) are both documented ballpark engineering choices, same status as every other estimated constant in this project.
