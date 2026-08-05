# Does the headline finding hold across field layouts?

`ftc_suite_writeup.md`'s headline sweep runs on `ftc/field.py`'s `'cluttered'` layout only. This reruns the identical full-rigor sweep -- same 11 variance_level steps, same 3 deviation types, same 25 trials/point, nothing reduced -- on all three layouts `ftc/field.py` ships, and checks whether the best-value suite (and the overall success-rate ranking) changes. Raw data (with an added `layout` column) in `ftc_layout_results.csv`, chart in `ftc_layout_comparison.png`.

## Per-layout results

### Sparse (near-open field)

DeadReckoningSuite baseline (variance_level >= 0.3): 27%

| Suite | Cost | Overall success rate | Value (pp/$100) |
|---|---:|---:|---:|
| Full suite | $399 | 86% | +14.8 |
| AprilTag | $25 | 70% | +170.7 |
| Odometry pods | $280 | 51% | +8.4 |
| Distance sensors | $94 | 30% | +2.8 |
| Dead reckoning | $0 | 27% | n/a (free) |

Best value on this layout: AprilTag.

### Cluttered (headline layout)

DeadReckoningSuite baseline (variance_level >= 0.3): 19%

| Suite | Cost | Overall success rate | Value (pp/$100) |
|---|---:|---:|---:|
| Full suite | $399 | 58% | +9.7 |
| Odometry pods | $280 | 45% | +9.3 |
| AprilTag | $25 | 44% | +99.3 |
| Distance sensors | $94 | 21% | +2.3 |
| Dead reckoning | $0 | 19% | n/a (free) |

Best value on this layout: AprilTag.

### Corridor (single narrow gap)

DeadReckoningSuite baseline (variance_level >= 0.3): 22%

| Suite | Cost | Overall success rate | Value (pp/$100) |
|---|---:|---:|---:|
| Full suite | $399 | 67% | +11.2 |
| AprilTag | $25 | 61% | +155.3 |
| Odometry pods | $280 | 47% | +8.9 |
| Distance sensors | $94 | 24% | +1.4 |
| Dead reckoning | $0 | 22% | n/a (free) |

Best value on this layout: AprilTag.

## Does the conclusion hold?

AprilTag is the best-value suite on all three layouts -- sparse, cluttered, and corridor. The headline recommendation is not an artifact of testing on the one layout with the most obstacles to sense; it holds on a near-open field and a single-forced-corridor field too.

## Consistency check

The 'cluttered' pass in this module uses the exact same trial_seed formula as `ftc/suite_benchmark.py`'s own `__main__`, so it reruns the identical scenarios. Overall success rate here: Dead reckoning 19%, Odometry pods 45%, Distance sensors 21%, AprilTag 44%, Full suite 58% -- compare against `ftc_suite_writeup.md`'s table; any mismatch would mean this module accidentally changed what 'cluttered' means rather than just adding two more layouts.
