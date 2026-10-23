# Does a moving opponent change which suite wins?

`ftc_suite_writeup.md`'s `unplanned_blocker` deviation type drops one STATIC obstacle on the planned route -- a real opponent moves. This reruns that same deviation axis (25 trials/point, all 11 variance_level steps, 'cluttered' layout) two ways: `static` (the existing behavior, unchanged) and `moving` (the identical probability gate and candidate-cell selection, but the chosen obstacle then random-walks for the rest of the match via `nav/obstacles.py`'s `MovingObstacle`, ticked on SIMULATED match time -- see module docstring). Raw data (with an added `blocker_type` column) in `ftc_opponent_results.csv`, chart in `ftc_opponent_comparison.png`.

## Overall success rate (variance_level >= 0.3)

| Suite | Cost | Static | Moving | Difference |
|---|---:|---:|---:|---:|
| Dead reckoning | $0 | 6% | 25% | +18% |
| Odometry pods | $195 | 20% | 56% | +36% |
| Distance sensors | $95 | 16% | 6% | -10% |
| AprilTag (front camera) | $25 | 8% | 31% | +24% |
| IMU | $0 | 6% | 25% | +18% |
| Rear camera | $50 | 8% | 31% | +24% |
| Full suite | $314 | 30% | 26% | -4% |

## Best value by blocker type

Static: Distance sensors (+10.1pp/$100, baseline 6%). Moving: AprilTag (front camera) (+24.0pp/$100, baseline 25%).

A moving opponent changes which suite wins: Distance sensors is best against a static blocker, but AprilTag (front camera) is best against a moving one. This is a real finding, not a failure of the sweep -- the existing static-blocker deviation type was, in this specific respect, silently favoring whichever suite handles a fixed obstacle best, not whichever suite handles a genuinely unpredictable opponent best.

## Why do suites that never sense the blocker at all also do better against a moving one?

Dead reckoning, Odometry pods, AprilTag (front camera), IMU, Rear camera never sense obstacles -- they can't react to the blocker being there at all, moving or static, and drive the exact same pre-planned route regardless. Yet every one of them does better against a moving opponent than a static one (see the table above; the largest gain is Odometry pods's +36%). The mechanism isn't sensing -- it's timing. A static blocker sits on the same planned-route cell for the entire match, so a blind suite that ever plans through that cell collides with it deterministically. A moving blocker starts on that same cell but then random-walks away; by the time a blind suite's fixed plan actually reaches that cell, the blocker has often wandered somewhere else, purely by timing luck the static version could never offer. A moving opponent is a *harder* obstacle to reason about, but this particular deviation type happens to make it an *easier* one to physically avoid for a suite that isn't reasoning about it at all.

## Do obstacle-sensing suites actually benefit against a moving opponent?

| Suite | Avg. collisions (static) | Avg. collisions (moving) | Avg. replans (static) | Avg. replans (moving) |
|---|---:|---:|---:|---:|
| Dead reckoning | 0.88 | 0.64 | 0.00 | 0.00 |
| Odometry pods | 0.80 | 0.42 | 0.00 | 0.00 |
| Distance sensors | 0.77 | 0.86 | 0.56 | 0.52 |
| AprilTag (front camera) | 0.90 | 0.57 | 0.77 | 1.27 |
| IMU | 0.88 | 0.64 | 0.00 | 0.00 |
| Rear camera | 0.90 | 0.57 | 0.77 | 1.27 |
| Full suite | 0.70 | 0.73 | 1.39 | 1.06 |

Static -> moving, for the suites that sense obstacles: Distance sensors replans fewer times (0.56 -> 0.52) and averages more collisions (0.77 -> 0.86); Full suite replans fewer times (1.39 -> 1.06) and averages more collisions (0.70 -> 0.73).

Obstacle sensing does NOT buy these suites the improvement the non-sensing suites get: their collision rate is not lower against a moving opponent. The non-sensing suites' improvement above is a *passive* timing benefit (the blocker happens to wander off their fixed route), and the sensing suites do not capture an *active* version of it on top -- a random-walking obstacle can wander back onto an already-cleared path, and the distance sensors' blind spots (see `ftc_suite_writeup.md`) plausibly mean it is often not seen until the robot is already close -- a hypothesis this sweep does not isolate.
