# Does the headline finding hold across field layouts?

`ftc_suite_writeup.md`'s headline sweep runs on `ftc/field.py`'s `'cluttered'` layout only. This reruns the identical full-rigor sweep -- same 11 variance_level steps, same 3 deviation types, same 25 trials/point, nothing reduced -- on all three layouts `ftc/field.py` ships, and checks whether the best-value suite (and the overall success-rate ranking) changes. Raw data (with an added `layout` column) in `ftc_layout_results.csv`, chart in `ftc_layout_comparison.png`.

## Per-layout results

### Sparse (near-open field)

DeadReckoningSuite baseline (variance_level >= 0.3): 28%

| Suite | Cost | Overall success rate | Value (pp/$100) |
|---|---:|---:|---:|
| Full suite | $314 | 75% | +14.8 |
| AprilTag (front camera) | $25 | 67% | +156.7 |
| Rear camera | $50 | 67% | +78.3 |
| Odometry pods | $195 | 49% | +10.7 |
| Distance sensors | $95 | 30% | +1.4 |
| Dead reckoning | $0 | 28% | n/a (free) |
| IMU | $0 | 28% | undefined (cost_usd == 0) |

Best value on this layout: AprilTag (front camera).

### Cluttered (headline layout)

DeadReckoningSuite baseline (variance_level >= 0.3): 14%

| Suite | Cost | Overall success rate | Value (pp/$100) |
|---|---:|---:|---:|
| Odometry pods | $195 | 30% | +7.8 |
| AprilTag (front camera) | $25 | 18% | +16.0 |
| Rear camera | $50 | 18% | +8.0 |
| Full suite | $314 | 16% | +0.5 |
| Dead reckoning | $0 | 14% | n/a (free) |
| IMU | $0 | 14% | undefined (cost_usd == 0) |
| Distance sensors | $95 | 10% | -4.2 |

Best value on this layout: AprilTag (front camera).

### Corridor (single narrow gap)

DeadReckoningSuite baseline (variance_level >= 0.3): 20%

| Suite | Cost | Overall success rate | Value (pp/$100) |
|---|---:|---:|---:|
| AprilTag (front camera) | $25 | 51% | +126.7 |
| Rear camera | $50 | 51% | +63.3 |
| Full suite | $314 | 51% | +10.0 |
| Odometry pods | $195 | 45% | +13.3 |
| Distance sensors | $95 | 20% | +0.5 |
| Dead reckoning | $0 | 20% | n/a (free) |
| IMU | $0 | 20% | undefined (cost_usd == 0) |

Best value on this layout: AprilTag (front camera).

## Does the conclusion hold?

AprilTag (front camera) is the best-value suite on all three layouts -- sparse, cluttered, and corridor. The headline recommendation is not an artifact of testing on the one layout with the most obstacles to sense; it holds on a near-open field and a single-forced-corridor field too.

## Consistency check

The 'cluttered' pass in this module uses the exact same trial_seed formula as `ftc/suite_benchmark.py`'s own `__main__`, so it reruns the identical scenarios. Overall success rate here: Dead reckoning 14%, Odometry pods 30%, Distance sensors 10%, AprilTag (front camera) 18%, IMU 14%, Rear camera 18%, Full suite 16% -- compare against `ftc_suite_writeup.md`'s table; any mismatch would mean this module accidentally changed what 'cluttered' means rather than just adding two more layouts.
