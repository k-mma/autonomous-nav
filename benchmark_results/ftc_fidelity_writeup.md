# Headline results at all three model fidelity tiers

ftc/config.py's MODEL_FIDELITY picks which tier's camera-FOV-gating, heading-drift, and AprilTag-heading-correction/dropout assumptions ftc/sensors.py and ftc/match.py use. This reruns ftc/suite_benchmark.py's exact full-rigor headline sweep (same layout, same seed formula, same 25 trials/point, all 11 variance_level steps, all 3 deviation types) once per tier. The 'optimistic' row below is byte-for-byte the same sweep `ftc_suite_writeup.md` already reports -- see ftc/scratch/fidelity_test.py's regression check -- included here for direct comparison, not as a separate estimate. Raw data (with an added `fidelity` column) in `ftc_fidelity_results.csv`, chart in `ftc_fidelity_comparison.png`.

## Headline table, all three tiers

Overall success rate, variance_level >= 0.3 across all three deviation types:

| Suite | Cost | Optimistic | Realistic | Pessimistic |
|---|---:|---:|---:|---:|
| Full suite | $230 | 56% | 53% | 52% |
| Odometry pods | $100 | 45% | 45% | 45% |
| AprilTag | $40 | 35% | 26% | 23% |
| Distance sensors | $90 | 21% | 19% | 19% |
| Dead reckoning | $0 | 19% | 23% | 22% |

## Best-value suite at each tier (success-rate gain over dead reckoning, per $100)

| Tier | Best value | pp/$100 | Full suite's pp/$100 |
|---|---|---:|---:|
| optimistic | AprilTag | +40.0 | +16.2 |
| realistic | Odometry pods | +22.3 | +13.0 |
| pessimistic | Odometry pods | +22.7 | +13.0 |

## Does the best-value recommendation survive tightening the model?

The best-value suite changes across tiers: optimistic -> AprilTag; realistic -> Odometry pods; pessimistic -> Odometry pods. This is a real finding, not a failure of the sweep -- it means the published optimistic-tier recommendation is conditional on the optimistic tier's assumptions (omnidirectional camera, perfect heading knowledge), not universal. Fidelity tiers BOUND the camera-FOV/heading-error gap in the model (README.md's "Threats to validity"); they do not CALIBRATE it -- only ftc/calibration.py run against real measured data does that.

## What this does and does not prove

This shows the RANGE the published headline numbers sit in as the camera-FOV and heading-error assumptions tighten -- it does not tell you which tier is closer to any particular real robot/field. All three tiers' non-optimistic constants (ftc/config.py's CAMERA_FOV_DEG_BY_TIER, HEADING_DRIFT_DEG_PER_CELL_BY_TIER, etc.) are documented ballpark engineering estimates, the same status as every other estimated constant in this project -- see README.md's "Threats to validity" and ftc/robustness.py for how sensitive the optimistic-tier recommendation is to ITS OWN estimated constants being wrong.
