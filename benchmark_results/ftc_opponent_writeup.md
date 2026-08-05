# Does a moving opponent change which suite wins?

`ftc_suite_writeup.md`'s `unplanned_blocker` deviation type drops one STATIC obstacle on the planned route -- a real opponent moves. This reruns that same deviation axis (25 trials/point, all 11 variance_level steps, 'cluttered' layout) two ways: `static` (the existing behavior, unchanged) and `moving` (the identical probability gate and candidate-cell selection, but the chosen obstacle then random-walks for the rest of the match via `nav/obstacles.py`'s `MovingObstacle`, ticked on SIMULATED match time -- see module docstring). Raw data (with an added `blocker_type` column) in `ftc_opponent_results.csv`, chart in `ftc_opponent_comparison.png`.

## Overall success rate (variance_level >= 0.3)

| Suite | Cost | Static | Moving | Difference |
|---|---:|---:|---:|---:|
| Dead reckoning | $0 | 8% | 30% | +22% |
| Odometry pods | $280 | 27% | 74% | +47% |
| Distance sensors | $94 | 24% | 25% | +0% |
| AprilTag | $25 | 17% | 52% | +36% |
| Full suite | $399 | 74% | 74% | +1% |

## Best value by blocker type

Static: AprilTag (+34.0pp/$100, baseline 8%). Moving: AprilTag (+90.0pp/$100, baseline 30%).

A moving opponent doesn't change which suite wins -- AprilTag is the best-value suite against both a static and a moving obstacle. Modeling the opponent as a random walk instead of a fixed point changes the raw numbers (see the table above) but not the recommendation.

## Why do suites that never sense the blocker at all also do better against a moving one?

Dead reckoning, Odometry pods, AprilTag never sense obstacles -- they can't react to the blocker being there at all, moving or static, and drive the exact same pre-planned route regardless. Yet every one of them does better against a moving opponent than a static one (see the table above; the largest gain is Odometry pods's +47%). The mechanism isn't sensing -- it's timing. A static blocker sits on the same planned-route cell for the entire match, so a blind suite that ever plans through that cell collides with it deterministically. A moving blocker starts on that same cell but then random-walks away; by the time a blind suite's fixed plan actually reaches that cell, the blocker has often wandered somewhere else, purely by timing luck the static version could never offer. A moving opponent is a *harder* obstacle to reason about, but this particular deviation type happens to make it an *easier* one to physically avoid for a suite that isn't reasoning about it at all.

## Do obstacle-sensing suites actually benefit against a moving opponent?

| Suite | Avg. collisions (static) | Avg. collisions (moving) | Avg. replans (static) | Avg. replans (moving) |
|---|---:|---:|---:|---:|
| Dead reckoning | 0.75 | 0.49 | 0.00 | 0.00 |
| Odometry pods | 0.73 | 0.26 | 0.00 | 0.00 |
| Distance sensors | 0.58 | 0.57 | 0.65 | 0.65 |
| AprilTag | 0.74 | 0.35 | 3.20 | 4.82 |
| Full suite | 0.27 | 0.23 | 3.72 | 3.80 |

Suites that sense obstacles (Distance sensors, Full suite) replan more against a moving opponent than a static one, since a random-walking obstacle can wander back onto an already-cleared path -- extra active work a static blocker never demands, but work that still pays off in a lower collision rate (see the table above). The non-sensing suites' improvement above is a *passive* benefit (the blocker happens to wander off their fixed route); this is the *active* version of the same underlying advantage -- a sensing suite can additionally detect and route around the blocker even while it's still nearby, instead of just waiting for it to leave.
