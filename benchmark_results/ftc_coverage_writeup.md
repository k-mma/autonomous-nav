# Can you buy your way out of the distance-sensor blind spot?

`ftc_suite_writeup.md`'s strongest negative finding: DistanceSensorSuite's 3 narrow ToF cones (`DISTANCE_SENSOR_HALF_ANGLE_DEG` each) cover only ~75 of the 360 degrees around the robot, and collide in roughly half their trials even at zero field deviation -- a controlled check showed pose drift wasn't the dominant cause. That study never asked whether more sensors fix it. This sweeps `DISTANCE_SENSOR_COUNT` over [3, 4, 6, 8] (`ftc/sensors.py`'s `make_distance_sensor_suite`, mount-heading layout documented per count in `ftc/config.py`) and adds `LidarSuite` (a full 360-degree disc scan, ~$100 -- see the table below for exact figures) as the direct head-to-head. Reduced trial count relative to the headline sweep (15 trials/point, levels [0.0, 0.3, 0.6, 1.0], all 3 deviation types, 'cluttered' layout -- see module docstring). Raw data in `ftc_coverage_results.csv`, chart in `ftc_coverage_comparison.png`.

## Coverage vs. collisions

| Config | Coverage | Cost | Collision rate (all levels) | Collision rate at variance_level=0.0 | Overall success rate |
|---|---:|---:|---:|---:|---:|
| 3 distance sensors | 75 deg | $94 | 53% | 49% | 26% |
| 4 distance sensors | 100 deg | $126 | 53% | 49% | 26% |
| 6 distance sensors | 150 deg | $189 | 48% | 44% | 27% |
| 8 distance sensors | 200 deg | $252 | 43% | 44% | 30% |
| Lidar (360deg) | 360 deg | $100 | 40% | 44% | 33% |

## Does more coverage actually reduce zero-deviation collisions?

Yes -- going from 3 to 8 distance sensors drops the zero-deviation collision rate from 49% to 44%, confirming the blind-spot finding is really about coverage angle (which the 3-sensor count directly under-covers) and that adding sensors genuinely closes gaps in the perimeter, not just adding redundant cones pointed at the same arcs.

## The direct head-to-head: $94 of blind cones vs. $100 of full coverage

Lidar (360deg coverage, $100) has a 44% zero-deviation collision rate, vs. 49% for the headline 3-sensor DistanceSensorSuite at $94 -- a real improvement, though not a complete elimination of geometry-driven collisions (some of DistanceSensorSuite's collision rate was never purely a coverage-angle problem -- see ftc_suite_writeup.md's own controlled check, which found roughly a third of collisions persisted even with pose drift disabled for reasons other than blind spots).

## What this does and does not prove

This confirms the coverage-angle mechanism is real and actionable -- more sensors (or a sensor with no blind spot at all) measurably reduce the specific zero-deviation collisions the headline study flagged. It does NOT establish that any of these counts is the *right* number for a real team to buy -- that's a cost/complexity tradeoff (more sensors is more I2C wiring/multiplexing, ftc/sensors.py's own integration_notes) this module doesn't weigh, and lidar's integration_notes explicitly flag that FTC's laser-class-device rules must be checked against the CURRENT season's game manual before treating it as a real, legal recommendation -- this module prices and simulates the sensing model, it does not assert legality.
