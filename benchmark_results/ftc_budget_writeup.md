# Does the 30-second autonomous budget ever actually bind?

`ftc_suite_writeup.md`'s own "Honest findings" section flags this as open: no suite ran out of budget anywhere in the headline sweep. This sweeps `AUTONOMOUS_PERIOD_S` (ftc/config.py, real value 30.0s) downward -- 30s, 20s, 15s, 10s, 7s, 5s, 4s, 3s, 2s, 1.5s -- rerunning the identical full-rigor sweep (25 trials/point, all 11 variance_level steps, all 3 deviation types) at each budget via `ftc.match.AUTONOMOUS_PERIOD_S` patching (the pattern `ftc/scratch/match_test.py` already established). Raw data (with an added `budget_s` column) in `ftc_budget_results.csv`, chart in `ftc_budget_comparison.png`.

**The example budgets in the original ask (30s, 20s, 15s, 10s, 7s) do bind**, starting at 20s (worst-suite over-budget rate 0.50% there -- small enough to round to 0% in the per-budget table below, but genuinely nonzero) -- ftc/match.py's trapezoidal drive-time model (added in this same sweep of priorities, see ftc/config.py's MAX_ACCEL_MPS2) makes every step take noticeably longer than the old naive distance/speed formula assumed, which erodes the margin this budget used to have. This sweep continues down to 1.5s so the *full* shape of the bind, not just where it starts, is visible.

## Per-budget results (variance_level >= 0.3, all deviation types)

| Budget (s) | Ranking (best to worst) | Worst over-budget rate |
|---:|---|---:|
| 30 | Full suite > Odometry pods > AprilTag > Distance sensors > Dead reckoning | 0% |
| 20 | Full suite > Odometry pods > AprilTag > Distance sensors > Dead reckoning | 0% |
| 15 | Full suite > Odometry pods > AprilTag > Distance sensors > Dead reckoning | 7% |
| 10 | Full suite > Odometry pods > AprilTag > Distance sensors > Dead reckoning | 22% |
| 7 | Full suite > Odometry pods > Distance sensors > AprilTag > Dead reckoning | 40% |
| 5 | Full suite > Distance sensors > Odometry pods > AprilTag > Dead reckoning | 55% |
| 4 | Full suite > Distance sensors > Odometry pods > AprilTag > Dead reckoning | 64% |
| 3 | Full suite > Distance sensors > Odometry pods > Dead reckoning > AprilTag | 74% |
| 2 | Distance sensors > Full suite > Dead reckoning > Odometry pods > AprilTag | 82% |
| 1.5 | Distance sensors > Full suite > Dead reckoning > Odometry pods > AprilTag | 84% |

## Replanning frequency (real 30s budget, unaffected by tightening)

| Suite | Avg. replans/trial |
|---|---:|
| Full suite | 2.15 |
| AprilTag | 1.80 |
| Distance sensors | 0.63 |
| Dead reckoning | 0.00 |
| Odometry pods | 0.00 |

AprilTag replans on every successful tag correction, not just DistanceSensorSuite/FullSuite's every-newly-sensed-obstacle trigger -- it ends up replanning more often than DistanceSensorSuite despite never sensing obstacles at all.

## Where it starts to bind

**The budget starts binding at 20s** -- the tightest budget at which every suite still has a 0% over-budget rate is the next one up in this sweep. At the tightest budget tested (1.5s), **AprilTag** has the highest over-budget rate (84%).

This matches the replan-heavy-suites-degrade-first hypothesis: AprilTag replans 1.80 times/trial on average (above the 0.63/trial median across all 5 suites, see the table above), each replan charged PLANNING_OVERHEAD_S on top of drive time -- exactly the kind of suite expected to feel a tight budget first, even though the *specific* suite (AprilTag, corrections-driven) isn't the one the original obstacle-sensing-suites hypothesis named.

## Does tightening the budget change which suite wins?

**Yes -- at 2s, Distance sensors overtakes Full suite as the #1 suite by raw success rate.** This is a statistically clean change (CIs don't overlap), so it is a genuine effect of the tighter budget, not noise.
