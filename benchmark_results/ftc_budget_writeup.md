# Does the 30-second autonomous budget ever actually bind?

`ftc_suite_writeup.md`'s own "Honest findings" section flags this as open: no suite ran out of budget anywhere in the headline sweep. This sweeps `AUTONOMOUS_PERIOD_S` (ftc/config.py, real value 30.0s) downward -- 30s, 20s, 15s, 10s, 7s, 5s, 4s, 3s, 2s, 1.5s -- rerunning the identical full-rigor sweep (25 trials/point, all 11 variance_level steps, all 3 deviation types) at each budget via `ftc.match.AUTONOMOUS_PERIOD_S` patching (the pattern `ftc/scratch/match_test.py` already established). Raw data (with an added `budget_s` column) in `ftc_budget_results.csv`, chart in `ftc_budget_comparison.png`.

The example budgets in the original ask (30s, 20s, 15s, 10s, 7s) do bind, starting at 10s (worst-suite over-budget rate 9.17% there -- small enough to round to 0% in the per-budget table below, but genuinely nonzero) -- ftc/match.py's trapezoidal drive-time model (added in this same sweep of priorities, see ftc/config.py's MAX_ACCEL_MPS2) makes every step take noticeably longer than the old naive distance/speed formula assumed, which erodes the margin this budget used to have. This sweep continues down to 1.5s so the full shape of the bind, not just where it starts, is visible.

## Per-budget results (variance_level >= 0.3, all deviation types)

| Budget (s) | Ranking (best to worst) | Worst over-budget rate |
|---:|---|---:|
| 30 | Odometry pods > AprilTag (front camera) > Rear camera > Full suite > Dead reckoning > IMU > Distance sensors | 0% |
| 20 | Odometry pods > AprilTag (front camera) > Rear camera > Full suite > Dead reckoning > IMU > Distance sensors | 0% |
| 15 | Odometry pods > AprilTag (front camera) > Rear camera > Full suite > Dead reckoning > IMU > Distance sensors | 0% |
| 10 | Odometry pods > AprilTag (front camera) > Rear camera > Full suite > Dead reckoning > IMU > Distance sensors | 9% |
| 7 | Odometry pods > AprilTag (front camera) > Rear camera > Full suite > Dead reckoning > IMU > Distance sensors | 23% |
| 5 | Odometry pods > Full suite > AprilTag (front camera) > Rear camera > Dead reckoning > IMU > Distance sensors | 36% |
| 4 | Full suite > Odometry pods > AprilTag (front camera) > Rear camera > Dead reckoning > IMU > Distance sensors | 46% |
| 3 | Dead reckoning > Odometry pods > IMU > AprilTag (front camera) > Rear camera > Full suite > Distance sensors | 54% |
| 2 | Odometry pods > Dead reckoning > AprilTag (front camera) > IMU > Rear camera > Distance sensors > Full suite | 68% |
| 1.5 | Distance sensors > Full suite > Dead reckoning > Odometry pods > AprilTag (front camera) > IMU > Rear camera | 77% |

## Replanning frequency (real 30s budget, unaffected by tightening)

| Suite | Avg. replans/trial |
|---|---:|
| AprilTag (front camera) | 0.69 |
| Rear camera | 0.69 |
| Full suite | 0.58 |
| Distance sensors | 0.32 |
| Dead reckoning | 0.00 |
| Odometry pods | 0.00 |
| IMU | 0.00 |

AprilTag replans on every successful tag correction, not just DistanceSensorSuite/FullSuite's every-newly-sensed-obstacle trigger -- it ends up replanning more often than DistanceSensorSuite despite never sensing obstacles at all.

## Where it starts to bind

The budget starts binding at 10s -- the tightest budget at which every suite still has a 0% over-budget rate is the next one up in this sweep. At the tightest budget tested (1.5s), Odometry pods has the highest over-budget rate (77%).

This does not match the replan-heavy-suites-degrade-first hypothesis -- Odometry pods replans only 0.00 times/trial on average, at or below the 0.32/trial median. PLANNING_OVERHEAD_S isn't the dominant cost near the budget edge for this suite; total elapsed_s (drive + turn time over whatever path length that suite's pose/obstacle error forces) matters at least as much as replan count.

## Does tightening the budget change which suite wins?

Yes -- at 4s, Full suite overtakes Odometry pods as the #1 suite by raw success rate. This is a statistically clean change (CIs don't overlap), so it is a genuine effect of the tighter budget, not noise.
