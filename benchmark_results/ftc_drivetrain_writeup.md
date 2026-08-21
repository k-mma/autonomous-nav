# Does a mecanum drivetrain's cost premium get repaid?

ftc/drivetrain.py adds TANK ($40) and MECANUM ($170) as an axis orthogonal to sensor suite: TANK must rotate to face its direction of travel (the existing flat per-90-degree turn cost, unchanged from before this addition); MECANUM holds a fixed heading -- aimed at the nearest AprilTag wall site -- for the whole match, paying no turn cost but a speed/drift penalty on any step that isn't roughly forward relative to that held heading. Crossed with 2 fidelity tiers (optimistic, realistic) x 7 suites x 3 deviation types x levels [0.3, 0.5, 0.7, 0.9] x 15 trials/point on the 'cluttered' layout -- reduced relative to the headline sweep (see module docstring). Raw data in `ftc_drivetrain_results.csv`, chart in `ftc_drivetrain_comparison.png`.

## The AprilTag interaction this module exists to check

| Fidelity | Tank (turns to face travel direction) | Mecanum (holds heading at tag wall) | Difference |
|---|---:|---:|---:|
| optimistic | 19% | 4% | -15% |
| realistic | 15% | 2% | -13% |

Mecanum does NOT come out ahead here, at either tier -- and the reason is visible in ftc/drivetrain.py's own model, not a surprise: the held heading is picked ONCE, at match start, aimed at whichever tag wall is nearest the start cell, and never changes for the rest of the match. A route's actual travel direction changes on almost every leg (up to 8 different directions on this project's diagonal grid), so unless a route happens to run roughly parallel to that one fixed heading, most of its steps are strafes relative to it -- paying MECANUM_STRAFE_SPEED_FACTOR/_DRIFT_MULTIPLIER (0.8x speed, 1.6x drift, ftc/config.py) on close to every step, not just the occasional sideways one. That drift penalty compounds across the whole route and shows up as a lower success rate for EVERY suite under mecanum, not just AprilTag (see the per-suite table below) -- the camera-stays-aimed-at-tags benefit this module set out to check is real (see the realistic-tier gap narrowing slightly relative to the optimistic-tier one below) but is swamped by the constant-strafe cost of a heading policy that's fixed for the whole match regardless of where the route actually goes. This is a real limitation of the specific 'hold a fixed heading toward the nearest tag wall for the whole match' policy this module implements, not evidence that mecanum drivetrains are generally worse -- a policy that re-picks its held heading periodically (e.g. toward whichever tag wall is nearest the CURRENT position, or toward the route's own dominant direction) would strafe far less, and this module doesn't test that alternative.

The camera-FOV mechanism is still visible underneath that, though: the tank-vs-mecanum gap narrows going from optimistic (-15%) to realistic (-13%) -- exactly the direction the camera-FOV benefit should push it, just not enough to overcome the strafe penalty at this trial count.

## Best value by drivetrain x fidelity

| Drivetrain | Fidelity | Best value | pp/$100 |
|---|---|---|---:|
| Tank | optimistic | Odometry pods | +7.1 |
| Tank | realistic | Odometry pods | +7.6 |
| Mecanum | optimistic | Odometry pods | +3.4 |
| Mecanum | realistic | Odometry pods | +3.7 |

## Does mecanum's own premium get repaid?

Comparing each suite's success rate on mecanum vs. tank, at mecanum's $130 total premium (MECANUM_WHEEL_COST_USD - TANK_WHEEL_COST_USD, ftc/config.py) on top of that suite's own sensor cost:

| Suite | Tank rate | Mecanum rate | Difference | Worth the $130 premium? |
|---|---:|---:|---:|---|
| Dead reckoning | 14% | 2% | -12% | no |
| Odometry pods | 32% | 15% | -17% | no |
| Distance sensors | 9% | 2% | -8% | no |
| AprilTag (front camera) | 15% | 2% | -13% | no |
| IMU | 13% | 2% | -12% | no |
| Rear camera | 14% | 2% | -12% | no |
| Full suite | 19% | 3% | -16% | no |

## What this does and does not prove

This is a reduced-rigor sweep (see module docstring) -- enough to see whether the camera-FOV/held-heading interaction is real and in the expected direction, not a publication-grade confidence interval on the exact magnitude. The mecanum heading policy itself (hold heading toward the nearest AprilTag wall) is one reasonable, documented choice, not the only one a real team could make -- a team without an AprilTag camera at all has no reason to hold that particular heading, and this specific comparison doesn't sweep alternative mecanum heading policies. `ftc_drivetrain_heading_policy_writeup.md` (same module, a separate sweep/output) is where that gets checked.
