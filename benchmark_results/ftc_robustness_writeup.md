# Robustness: how wrong would the estimated constants have to be?

The headline finding (`ftc_suite_writeup.md`) is that AprilTag ($40) is the best-value sensor suite by success-rate-gained-per-dollar, more than double FullSuite's ($230) return. That rests on estimated constants in ftc/config.py -- documented ballpark engineering figures, not measurements. This sweeps each one from 0.25x to 4x its estimated value and asks one question: does the best-value suite actually change? 15 trials/point at levels [0.3, 0.5, 0.7, 0.9] (reduced from the headline sweep's 25 trials/11 levels -- a tipping-point search needs "did the ranking flip," not a publication-grade curve at every multiplier), same 'cluttered' layout and identical scenario sequence across every point compared. Raw data in `ftc_robustness.csv`, chart in `ftc_robustness.png`.

What this does and does not prove: a tipping point bounds the estimate error the conclusion can tolerate. It does not tell you whether the *real* value is inside or outside that bound -- only measured field/robot data through ftc/calibration.py can do that. A parameter that never tips across 0.25x-4x means the recommendation is insensitive to that estimate being wrong by up to 4x in either direction; it does not mean the estimate is correct.

## Tipping points

| Parameter | Baseline best value | Tips at | New best value | Statistically clean? |
|---|---|---:|---|---|
| Dead-reckoning drift rate (DEAD_RECKONING_DRIFT_PER_CELL) | AprilTag | 2.0x | Odometry pods | yes |
| Odometry-pod drift rate (ODOMETRY_DRIFT_PER_CELL) | AprilTag | never (0.25x-4x) | -- | -- |
| AprilTag max correction | AprilTag | 0.25x | Odometry pods | yes |
| AprilTag range+angle degradation | AprilTag | 2.0x | Odometry pods | yes |
| Odometry pods cost | AprilTag | 0.25x | Odometry pods | no -- CIs still overlap |
| Distance sensors cost | AprilTag | never (0.25x-4x) | -- | -- |
| AprilTag cost | AprilTag | 2.0x | Odometry pods | no -- CIs still overlap |
| Full suite cost | AprilTag | 0.25x | Full suite | yes |

## What this means in plain language

The recommendation is not universally robust: Dead-reckoning drift rate (DEAD_RECKONING_DRIFT_PER_CELL) tips the best-value suite at only 2.0x its estimated value -- the least forgiving parameter this sweep found. See the table above for the rest; any row with a real (non-noise) tip is a specific, named number a reviewer can push back on, which is the point of running this at all.

Some apparent flips did not survive a statistical check: Odometry pods cost at 0.25x; AprilTag cost at 2.0x -- the new "winner"'s success-rate confidence interval still overlaps the old one's at that multiplier, so this could be sampling noise from only 15 trials/point rather than a genuine ranking change. Treat these as "maybe, not confirmed" rather than real tipping points.

Parameters that never tipped the ranking anywhere in [0.25x, 4x]: Odometry-pod drift rate (ODOMETRY_DRIFT_PER_CELL), Distance sensors cost.
