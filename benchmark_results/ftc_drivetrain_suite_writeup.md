# Does the headline finding hold under mecanum, not just tank?

`ftc_suite_writeup.md`'s headline sweep runs under `ftc/match.py`'s legacy no-drivetrain default, which is byte-for-byte the same model as an explicit TANK drivetrain (`ftc/drivetrain.py`). This reruns the identical full-rigor sweep -- same 11 variance_level steps, same 3 deviation types, same 25 trials/point, nothing reduced -- once per drivetrain in `ftc/drivetrain.py`'s `DRIVETRAIN_ORDER` (tank, mecanum), for all 7 headline suites, and checks whether the best-value suite (and the overall success-rate ranking) changes on mecanum. Cost per suite includes that drivetrain's own premium (`MECANUM_WHEEL_COST_USD`/`TANK_WHEEL_COST_USD`, `ftc/config.py`) on top of the sensor cost, since a team buying a suite also has to buy wheels. Raw data (with an added `drivetrain` column) in `ftc_drivetrain_suite_results.csv`, chart in `ftc_drivetrain_suite_comparison.png`.

This is a DIFFERENT, separately-scoped comparison from `ftc_drivetrain_writeup.md` (`ftc/drivetrain_benchmark.py`), which crosses the headline suites against {tank, mecanum} x {optimistic, realistic} fidelity at reduced rigor to isolate one specific mechanism (does holding a fixed heading toward a tag wall keep AprilTag's camera aimed at tags long enough to pay for mecanum's premium). That study's own numbers are frozen and untouched by this one -- this module asks the broader question at full statistical rigor: for EVERY headline suite, not just AprilTag, does success rate -- and the best-value recommendation -- change under mecanum?

## Success rate by suite x drivetrain

| Suite | Tank | Mecanum | Difference |
|---|---:|---:|---:|
| Dead reckoning | 14% | 3% | -11% |
| Odometry pods | 30% | 17% | -13% |
| Distance sensors | 10% | 2% | -8% |
| AprilTag (front camera) | 18% | 3% | -15% |
| IMU | 14% | 3% | -11% |
| Rear camera | 18% | 3% | -15% |
| Full suite | 16% | 4% | -13% |

## Per-drivetrain value ranking

### Tank (headline default)

DeadReckoningSuite baseline (variance_level >= 0.3): 14%

| Suite | Cost (incl. drivetrain) | Overall success rate | Value (pp/$100) |
|---|---:|---:|---:|
| Odometry pods | $235 | 30% | +6.5 |
| AprilTag (front camera) | $65 | 18% | +6.2 |
| Rear camera | $90 | 18% | +4.4 |
| Full suite | $354 | 16% | +0.5 |
| Dead reckoning | $40 | 14% | n/a (free) |
| IMU | $40 | 14% | +0.0 |
| Distance sensors | $134 | 10% | -3.0 |

Best value under Tank (headline default): Odometry pods.

### Mecanum

DeadReckoningSuite baseline (variance_level >= 0.3): 3%

| Suite | Cost (incl. drivetrain) | Overall success rate | Value (pp/$100) |
|---|---:|---:|---:|
| Odometry pods | $365 | 17% | +3.7 |
| Full suite | $484 | 4% | +0.1 |
| AprilTag (front camera) | $195 | 3% | +0.1 |
| Rear camera | $220 | 3% | +0.1 |
| Dead reckoning | $170 | 3% | n/a (free) |
| IMU | $170 | 3% | +0.0 |
| Distance sensors | $264 | 2% | -0.3 |

Best value under Mecanum: Odometry pods.

## Does the conclusion hold?

Odometry pods is the best-value suite under both tank and mecanum. The headline recommendation is not tank-specific -- teams running mecanum, a large fraction of the FTC population, should reach the same sensing decision.

## Consistency check

The 'tank' pass in this module uses the exact same trial_seed formula as `ftc/suite_benchmark.py`'s own `__main__` (tank and 'no drivetrain' are byte-for-byte the same model, `ftc/drivetrain.py`'s own module docstring), so it reruns the identical scenarios. Overall success rate here: Dead reckoning 14%, Odometry pods 30%, Distance sensors 10%, AprilTag (front camera) 18%, IMU 14%, Rear camera 18%, Full suite 16% -- compare against `ftc_suite_writeup.md`'s table; any mismatch would mean this module accidentally changed what the tank pass measures rather than just adding a mecanum one.
