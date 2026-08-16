# Does a moving opponent change which suite wins?

`ftc_suite_writeup.md`'s `unplanned_blocker` deviation type drops one STATIC obstacle on the planned route -- a real opponent moves. This reruns that same deviation axis (25 trials/point, all 11 variance_level steps, 'cluttered' layout) two ways: `static` (the existing behavior, unchanged) and `moving` (the identical probability gate and candidate-cell selection, but the chosen obstacle then random-walks for the rest of the match via `nav/obstacles.py`'s `MovingObstacle`, ticked on SIMULATED match time -- see module docstring). Raw data (with an added `blocker_type` column) in `ftc_opponent_results.csv`, chart in `ftc_opponent_comparison.png`.

## Overall success rate (variance_level >= 0.3)

| Suite | Cost | Static | Moving | Difference |
|---|---:|---:|---:|---:|
| Dead reckoning | $0 | 6% | 25% | +18% |
| Odometry pods | $280 | 20% | 51% | +31% |
| Distance sensors | $94 | 16% | 6% | -10% |
| AprilTag (front camera) | $25 | 8% | 32% | +24% |
| IMU | $0 | 6% | 24% | +18% |
| Rear camera | $50 | 8% | 31% | +24% |
| Full suite | $399 | 30% | 24% | -5% |

## Best value by blocker type

Static: Distance sensors (+10.1pp/$100, baseline 6%). Moving: AprilTag (front camera) (+26.0pp/$100, baseline 25%).

A moving opponent appears to change which suite wins, but not cleanly: Distance sensors is best against a static blocker, and AprilTag (front camera) edges out Rear camera for best value against a moving one -- but 26.0pp/$100 vs. 12.0pp/$100 is close enough that the two suites' success-rate confidence intervals still overlap at this trial count. Treat "AprilTag (front camera) beats Rear camera against a moving opponent" as plausible, not confirmed -- but the headline claim that follows doesn't depend on that particular margin: neither of them is Full suite, and that gap (Distance sensors's static win) is not close.

## Why do suites that never sense the blocker at all also do better against a moving one?

Dead reckoning, Odometry pods, AprilTag (front camera), IMU, Rear camera never sense obstacles -- they can't react to the blocker being there at all, moving or static, and drive the exact same pre-planned route regardless. Yet every one of them does better against a moving opponent than a static one (see the table above; the largest gain is Odometry pods's +31%). The mechanism isn't sensing -- it's timing. A static blocker sits on the same planned-route cell for the entire match, so a blind suite that ever plans through that cell collides with it deterministically. A moving blocker starts on that same cell but then random-walks away; by the time a blind suite's fixed plan actually reaches that cell, the blocker has often wandered somewhere else, purely by timing luck the static version could never offer. A moving opponent is a *harder* obstacle to reason about, but this particular deviation type happens to make it an *easier* one to physically avoid for a suite that isn't reasoning about it at all.

## Do obstacle-sensing suites actually benefit against a moving opponent?

| Suite | Avg. collisions (static) | Avg. collisions (moving) | Avg. replans (static) | Avg. replans (moving) |
|---|---:|---:|---:|---:|
| Dead reckoning | 0.88 | 0.64 | 0.00 | 0.00 |
| Odometry pods | 0.80 | 0.47 | 0.00 | 0.00 |
| Distance sensors | 0.77 | 0.84 | 0.56 | 0.46 |
| AprilTag (front camera) | 0.89 | 0.57 | 0.77 | 1.32 |
| IMU | 0.88 | 0.67 | 0.00 | 0.00 |
| Rear camera | 0.89 | 0.57 | 0.77 | 1.31 |
| Full suite | 0.70 | 0.75 | 1.39 | 0.85 |

Suites that sense obstacles (Distance sensors, Full suite) replan more against a moving opponent than a static one, since a random-walking obstacle can wander back onto an already-cleared path -- extra active work a static blocker never demands, but work that still pays off in a lower collision rate (see the table above). The non-sensing suites' improvement above is a *passive* benefit (the blocker happens to wander off their fixed route); this is the *active* version of the same underlying advantage -- a sensing suite can additionally detect and route around the blocker even while it's still nearby, instead of just waiting for it to leave.
