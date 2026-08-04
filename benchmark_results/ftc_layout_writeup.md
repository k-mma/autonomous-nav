# Does the headline finding hold across field layouts?

`ftc_suite_writeup.md`'s headline sweep runs on `ftc/field.py`'s `'cluttered'` layout only. This reruns the identical full-rigor sweep -- same 11 variance_level steps, same 3 deviation types, same 25 trials/point, nothing reduced -- on all three layouts `ftc/field.py` ships, and checks whether the best-value suite (and the overall success-rate ranking) changes. Raw data (with an added `layout` column) in `ftc_layout_results.csv`, chart in `ftc_layout_comparison.png`.

## Per-layout results

### Sparse (near-open field)

DeadReckoningSuite baseline (variance_level >= 0.3): 27%

| Suite | Cost | Overall success rate | Value (pp/$100) |
|---|---:|---:|---:|
| Full suite | $230 | 87% | +25.8 |
| AprilTag | $40 | 66% | +96.2 |
| Odometry pods | $100 | 51% | +23.5 |
| Distance sensors | $90 | 30% | +3.0 |
| Dead reckoning | $0 | 27% | n/a (free) |

Best value on this layout: AprilTag.

### Cluttered (headline layout)

DeadReckoningSuite baseline (variance_level >= 0.3): 19%

| Suite | Cost | Overall success rate | Value (pp/$100) |
|---|---:|---:|---:|
| Full suite | $230 | 56% | +16.2 |
| Odometry pods | $100 | 45% | +26.2 |
| AprilTag | $40 | 35% | +40.0 |
| Distance sensors | $90 | 21% | +2.4 |
| Dead reckoning | $0 | 19% | n/a (free) |

Best value on this layout: AprilTag.

### Corridor (single narrow gap)

DeadReckoningSuite baseline (variance_level >= 0.3): 22%

| Suite | Cost | Overall success rate | Value (pp/$100) |
|---|---:|---:|---:|
| Full suite | $230 | 67% | +19.4 |
| AprilTag | $40 | 60% | +94.6 |
| Odometry pods | $100 | 47% | +25.0 |
| Distance sensors | $90 | 24% | +1.5 |
| Dead reckoning | $0 | 22% | n/a (free) |

Best value on this layout: AprilTag.

## Does the conclusion hold?

AprilTag is the best-value suite on all three layouts -- sparse, cluttered, and corridor. The headline recommendation is not an artifact of testing on the one layout with the most obstacles to sense; it holds on a near-open field and a single-forced-corridor field too.

## Consistency check

The 'cluttered' pass in this module uses the exact same trial_seed formula as `ftc/suite_benchmark.py`'s own `__main__`, so it reruns the identical scenarios. Overall success rate here: Dead reckoning 19%, Odometry pods 45%, Distance sensors 21%, AprilTag 35%, Full suite 56% -- compare against `ftc_suite_writeup.md`'s table; any mismatch would mean this module accidentally changed what 'cluttered' means rather than just adding two more layouts.
