# Optimistic merge vs. confidence-weighted vs. Kalman fusion

**nav/kalman.py's fusion path runs on ftc/calibration.py's labeled SYNTHETIC PLACEHOLDER AprilTag detection scatter, not real measured hardware data -- see this module's own docstring (calibration source = `synthetic_placeholder`). Every 'kalman' number below describes how this project's own placeholder behaves under a proper Kalman update -- it is not a claim about real AprilTag hardware.**

40 trials x 5 scenario profiles (ftc/optimizer.py's DEFAULT_PROFILES, identical to ftc/fusion_benchmark.py's own scenario generation and seeds) x 4 conditions -- the AprilTag+odometry bundle's best single component as a floor, then the bundle itself under optimistic merging (`fusion=None`), confidence-weighted fusion (`fusion=True`, ftc/fusion.py), and Kalman fusion (`fusion="kalman"`, nav/kalman.py). Every condition runs against identical seeded scenarios (paired), so nav/stats.py's `bootstrap_paired_diff_ci` is used for every headline comparison below rather than eyeballing whether independent CIs overlap. Raw data in `ftc_fusion_kalman_results.csv`, chart in `ftc_fusion_kalman_comparison.png`.

## Headline pairwise comparisons

| Comparison | Rate A | Rate B | B - A [95% CI] | Verdict |
|---|---:|---:|---:|---|
| Optimistic merge vs. Confidence-weighted | 35% | 22% | -13.5% [-18.5%, -8.5%] | **Confidence-weighted is significantly worse** |
| Confidence-weighted vs. Kalman (synthetic variance) | 22% | 26% | +5.0% [+2.0%, +8.5%] | **Kalman (synthetic variance) is significantly better** |
| Optimistic merge vs. Kalman (synthetic variance) | 35% | 26% | -8.5% [-12.5%, -4.5%] | **Kalman (synthetic variance) is significantly worse** |

At this project's synthetic placeholder variance, Kalman fusion (26%) does better than confidence-weighted fusion (22%) -- see the pairwise row above for whether that gap is statistically real or noise at this trial count. Even if real, this says the MATH is doing what it should (a properly gated, variance-aware update recovers more of the bundle's advantage than a fixed confidence weight) -- it does not mean a real AprilTag+odometry bundle would perform this well, since nav/kalman.py's fusion path runs on ftc/calibration.py's labeled SYNTHETIC PLACEHOLDER AprilTag detection scatter, not real measured hardware data -- see this module's own docstring.

## Does any fusion strategy beat the bundle's best single component?

| Condition | Success rate | Advantage over best single |
|---|---:|---:|
| Best single component (no fusion) | 25% | -- |
| AprilTag+odometry bundle (optimistic merge) | 35% | +10% |
| AprilTag+odometry bundle (confidence-weighted fusion) | 22% | -4% |
| AprilTag+odometry bundle (Kalman fusion, SYNTHETIC variance) | 26% | +2% |

## Per-profile breakdown

| Profile | Best single | Optimistic merge | Confidence-weighted | Kalman (synthetic) |
|---|---:|---:|---:|---:|
| Heavy pose drift | 8% | 15% | 8% | 12% |
| Field doesn't match the map | 60% | 60% | 45% | 50% |
| Opponent parks in the route | 32% | 32% | 20% | 28% |
| Everything at once (realistic fidelity) | 15% | 15% | 15% | 18% |
| Tight corridor, mixed deviation | 10% | 52% | 20% | 25% |

## Honest findings / limitations

- nav/kalman.py's fusion path runs on ftc/calibration.py's labeled SYNTHETIC PLACEHOLDER AprilTag detection scatter, not real measured hardware data -- see this module's own docstring. Pass real detection scatter to `ftc.calibration.load_calibration(apriltag_csv_path=...)` and its `.apriltag_variance.model` to `ftc.fusion.fused_tag_correction_kalman` (or `ftc.match.run_match`'s Kalman path, once a caller threads a model through) to replace this with a real one.
- `_apriltag_observation_variance_cells2` (ftc/fusion.py) derives the Kalman path's observation variance from `frac` rather than from raw range/incidence -- see that function's own docstring for exactly what this approximates away.
- Both fusion paths (confidence-weighted and Kalman) share the identical generative observation model (`_apriltag_observation_value`, ftc/fusion.py) -- this comparison is entirely about the fusion MATH, not about two different noise assumptions.
- Every constant this comparison inherits from `bundle_confidence` (ODOMETRY_FUSION_CONFIDENCE, APRILTAG_FUSION_CONFIDENCE, APRILTAG_SYSTEMATIC_BIAS_CELLS, APRILTAG_BAD_DETECTION_PROBABILITY, APRILTAG_BAD_DETECTION_SIGMA_CELLS, all in ftc/config.py) is the same uncalibrated engineering estimate `ftc_fusion_writeup.md` already flags -- unaffected by this module's own additions.
