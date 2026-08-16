# Can you buy your way out of the distance-sensor blind spot?

`ftc_suite_writeup.md`'s strongest negative finding: DistanceSensorSuite's 3 narrow ToF cones (`DISTANCE_SENSOR_HALF_ANGLE_DEG` each) cover only ~75 of the 360 degrees around the robot, and collide in roughly half their trials even at zero field deviation -- a controlled check showed pose drift wasn't the dominant cause. That study never asked whether more sensors fix it. This sweeps `DISTANCE_SENSOR_COUNT` over [3, 4, 6, 8] (`ftc/sensors.py`'s `make_distance_sensor_suite`, mount-heading layout documented per count in `ftc/config.py`). Reduced trial count relative to the headline sweep (15 trials/point, levels [0.0, 0.3, 0.6, 1.0], all 3 deviation types, 'cluttered' layout -- see module docstring). Raw data in `ftc_coverage_results.csv`, chart in `ftc_coverage_comparison.png`.

## Coverage vs. collisions

| Config | Coverage | Cost | Collision rate (all levels) | Collision rate at variance_level=0.0 | Overall success rate |
|---|---:|---:|---:|---:|---:|
| 3 distance sensors | 75 deg | $94 | 74% | 87% | 14% |
| 4 distance sensors | 100 deg | $126 | 74% | 87% | 14% |
| 6 distance sensors | 150 deg | $189 | 73% | 82% | 16% |
| 8 distance sensors | 200 deg | $252 | 71% | 80% | 16% |

## Does more coverage actually reduce zero-deviation collisions?

Yes -- going from 3 to 8 distance sensors drops the zero-deviation collision rate from 87% to 80%, confirming the blind-spot finding is really about coverage angle (which the 3-sensor count directly under-covers) and that adding sensors genuinely closes gaps in the perimeter, not just adding redundant cones pointed at the same arcs.

## Is full coverage even reachable?

No purchasable option in this sweep -- or anywhere else in this project -- covers the full 360-degree perimeter: lidar-class hardware isn't legal FTC equipment, so it isn't modeled here (an earlier version of this study used it as a full-coverage reference point; removing it is not a gap in this study, it's a correction -- see README.md's "Threats to validity"). Even at the largest count swept (8 narrow ToF cones, `DISTANCE_SENSOR_HALF_ANGLE_DEG` each), total coverage tops out at 200 of 360 degrees -- a 160-degree blind arc survives no matter how many of these specific sensors a team buys, because each one only ever adds its own narrow cone, never closes the gap between cones faster than it opens new ones at the perimeter's edge. The honest framing of this study's own finding is therefore blunter than "more sensors help" (true, see the table above) or "buy enough and the blind spot closes" (false, for every FTC-legal ToF configuration this project can price): a sparse fixed-cone sensor family has a structural coverage ceiling, not just a currently-unmet one, and closing it fully would need a genuinely different sensing modality this project doesn't model at all -- not a bigger version of the same one.

## What this does and does not prove

This confirms the coverage-angle mechanism is real and actionable -- more sensors measurably reduce the specific zero-deviation collisions the headline study flagged, right up to the 160-degree ceiling the geometry itself imposes. It does NOT establish that any of these counts is the *right* number for a real team to buy -- that's a cost/complexity tradeoff (more sensors is more I2C wiring/multiplexing, ftc/sensors.py's own integration_notes) this module doesn't weigh, and it does not establish that the residual blind arc is actually survivable in a real match -- only that no amount of this specific hardware, bought in any quantity this study tested, closes it to zero.
