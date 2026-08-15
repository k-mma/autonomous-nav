# Does the headline finding hold under mecanum, not just tank?

`ftc_suite_writeup.md`'s headline sweep runs under `ftc/match.py`'s legacy no-drivetrain default, which is byte-for-byte the same model as an explicit TANK drivetrain (`ftc/drivetrain.py`). This reruns the identical full-rigor sweep -- same 11 variance_level steps, same 3 deviation types, same 25 trials/point, nothing reduced -- once per drivetrain in `ftc/drivetrain.py`'s `DRIVETRAIN_ORDER` (tank, mecanum), for all 7 headline suites, and checks whether the best-value suite (and the overall success-rate ranking) changes on mecanum. Cost per suite includes that drivetrain's own premium (`MECANUM_WHEEL_COST_USD`/`TANK_WHEEL_COST_USD`, `ftc/config.py`) on top of the sensor cost, since a team buying a suite also has to buy wheels. Raw data (with an added `drivetrain` column) in `ftc_drivetrain_suite_results.csv`, chart in `ftc_drivetrain_suite_comparison.png`.

This is a DIFFERENT, separately-scoped comparison from `ftc_drivetrain_writeup.md` (`ftc/drivetrain_benchmark.py`), which crosses the headline suites against {tank, mecanum} x {optimistic, realistic} fidelity at reduced rigor to isolate one specific mechanism (does holding a fixed heading toward a tag wall keep AprilTag's camera aimed at tags long enough to pay for mecanum's premium). That study's own numbers are frozen and untouched by this one -- this module asks the broader question at full statistical rigor: for EVERY headline suite, not just AprilTag, does success rate -- and the best-value recommendation -- change under mecanum?

## Success rate by suite x drivetrain

| Suite | Tank | Mecanum | Difference |
|---|---:|---:|---:|
| Dead reckoning | 19% | 16% | -3% |
| Odometry pods | 45% | 45% | -0% |
| Distance sensors | 21% | 7% | -14% |
| AprilTag (front camera) | 44% | 36% | -8% |
| IMU | 19% | 16% | -3% |
| Rear camera | 44% | 36% | -8% |
| Full suite | 58% | 24% | -34% |

## Per-drivetrain value ranking

### Tank (headline default)

DeadReckoningSuite baseline (variance_level >= 0.3): 19%

| Suite | Cost (incl. drivetrain) | Overall success rate | Value (pp/$100) |
|---|---:|---:|---:|
| Full suite | $439 | 58% | +8.8 |
| Odometry pods | $320 | 45% | +8.2 |
| AprilTag (front camera) | $65 | 44% | +38.2 |
| Rear camera | $90 | 44% | +27.6 |
| Distance sensors | $134 | 21% | +1.6 |
| Dead reckoning | $40 | 19% | n/a (free) |
| IMU | $40 | 19% | +0.0 |

Best value under Tank (headline default): AprilTag (front camera).

### Mecanum

DeadReckoningSuite baseline (variance_level >= 0.3): 16%

| Suite | Cost (incl. drivetrain) | Overall success rate | Value (pp/$100) |
|---|---:|---:|---:|
| Odometry pods | $450 | 45% | +6.4 |
| AprilTag (front camera) | $195 | 36% | +10.3 |
| Rear camera | $220 | 36% | +9.1 |
| Full suite | $569 | 24% | +1.3 |
| Dead reckoning | $170 | 16% | n/a (free) |
| IMU | $170 | 16% | +0.0 |
| Distance sensors | $264 | 7% | -3.3 |

Best value under Mecanum: AprilTag (front camera).

## Does the conclusion hold?

AprilTag (front camera) is the best-value suite under both tank and mecanum. The headline recommendation is not tank-specific -- teams running mecanum, a large fraction of the FTC population, should reach the same sensing decision.

## Consistency check

The 'tank' pass in this module uses the exact same trial_seed formula as `ftc/suite_benchmark.py`'s own `__main__` (tank and 'no drivetrain' are byte-for-byte the same model, `ftc/drivetrain.py`'s own module docstring), so it reruns the identical scenarios. Overall success rate here: Dead reckoning 19%, Odometry pods 45%, Distance sensors 21%, AprilTag (front camera) 44%, IMU 19%, Rear camera 44%, Full suite 58% -- compare against `ftc_suite_writeup.md`'s table; any mismatch would mean this module accidentally changed what the tank pass measures rather than just adding a mecanum one.
