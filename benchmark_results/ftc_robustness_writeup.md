# Robustness: how wrong would the estimated constants have to be?

The headline finding (`ftc_suite_writeup.md`) is that AprilTag ($25) is the best-value sensor suite by success-rate-gained-per-dollar, well ahead of FullSuite's ($399) return. That rests on estimated constants in ftc/config.py -- documented ballpark engineering figures, not measurements. This sweeps each one from 0.25x to 4x its estimated value and asks one question: does the best-value suite actually change? 15 trials/point at levels [0.3, 0.5, 0.7, 0.9] (reduced from the headline sweep's 25 trials/11 levels -- a tipping-point search needs "did the ranking flip," not a publication-grade curve at every multiplier), same 'cluttered' layout and identical scenario sequence across every point compared. Raw data in `ftc_robustness.csv`, chart in `ftc_robustness.png`.

What this does and does not prove: a tipping point bounds the estimate error the conclusion can tolerate. It does not tell you whether the *real* value is inside or outside that bound -- only measured field/robot data through ftc/calibration.py can do that. A parameter that never tips across 0.25x-4x means the recommendation is insensitive to that estimate being wrong by up to 4x in either direction; it does not mean the estimate is correct.

## Tipping points

| Parameter | Baseline best value | Tips at | New best value | Statistically clean? |
|---|---|---:|---|---|
| Dead-reckoning drift rate (DEAD_RECKONING_DRIFT_PER_CELL) | AprilTag | never (0.25x-4x) | -- | -- |
| Odometry-pod drift rate (ODOMETRY_DRIFT_PER_CELL) | AprilTag | never (0.25x-4x) | -- | -- |
| AprilTag max correction | AprilTag | never (0.25x-4x) | -- | -- |
| AprilTag range+angle degradation | AprilTag | never (0.25x-4x) | -- | -- |
| Odometry pods cost | AprilTag | never (0.25x-4x) | -- | -- |
| Distance sensors cost | AprilTag | never (0.25x-4x) | -- | -- |
| AprilTag cost | AprilTag | never (0.25x-4x) | -- | -- |
| Full suite cost | AprilTag | never (0.25x-4x) | -- | -- |

## What this means in plain language

AprilTag stayed the best-value suite across every parameter, at every multiplier tested (0.25x to 4x). None of the estimated constants this sweep touched -- drift rates, AprilTag's correction quality, or any suite's price -- would have to be *exactly right* for the recommendation to hold; they'd all have to be off by more than 4x, in the specific direction that hurts AprilTag, before a different suite would actually be the better buy.

Parameters that never tipped the ranking anywhere in [0.25x, 4x]: Dead-reckoning drift rate (DEAD_RECKONING_DRIFT_PER_CELL), Odometry-pod drift rate (ODOMETRY_DRIFT_PER_CELL), AprilTag max correction, AprilTag range+angle degradation, Odometry pods cost, Distance sensors cost, AprilTag cost, Full suite cost.
