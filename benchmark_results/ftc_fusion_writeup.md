# Does the AprilTag+odometry bundle's advantage survive sensor disagreement?

40 trials x 5 scenario profiles (ftc/optimizer.py's DEFAULT_PROFILES) x 3 conditions -- the AprilTag+odometry bundle under ftc/bundle.py's existing optimistic merge (`fusion=None`) and under ftc/fusion.py's confidence-weighted fusion (`fusion=True`), plus its best single component (Odometry pods) at `fusion=None` as the comparison floor. Every condition runs against identical seeded scenarios (paired), so nav/stats.py's `bootstrap_paired_diff_ci` is used for the headline comparison rather than eyeballing whether independent CIs overlap -- the same tool ftc/optimizer_benchmark.py's own synergy claims use. Raw data in `ftc_fusion_results.csv`, chart in `ftc_fusion_comparison.png`.

## The headline number

Pooled across all 5 profiles: the bundle succeeds in 35% of trials under optimistic merging, 22% under confidence-weighted fusion -- a change of -13.5% [95% CI -18.5%, -9.0%] (paired bootstrap). One-sided p for "fusion performs better than optimistic merging": 1.000 -- a value near 1.0 supports a drop, near 0.0 supports an increase, and anywhere in between is noise.

That drop is **statistically significant**: modeling AprilTag-vs-odometry disagreement costs the bundle a real, measurable amount of success rate, not noise. The optimistic-merge model was overstating this bundle's reliability by roughly 13% (95% CI [9.0%, 18.5%]) at this project's estimated fusion constants (ftc/config.py's `APRILTAG_BAD_DETECTION_PROBABILITY`/`APRILTAG_SYSTEMATIC_BIAS_CELLS`, uncalibrated -- see "Honest findings" below).

## Does the bundle still beat its best single component?

| Condition | Success rate | Advantage over best single |
|---|---:|---:|
| Best single component (no fusion) | 25% | -- |
| AprilTag+odometry bundle (optimistic merge) | 35% | +10% |
| AprilTag+odometry bundle (confidence-weighted fusion) | 22% | -4% |

The bundle's advantage over its best single component **changes sign** once fusion is modeled -- this is the kind of finding this study exists to catch: an optimizer recommendation that depended on optimistic merging, not on the bundle actually being the better buy.

## Per-profile breakdown

| Profile | Best single | Bundle (no fusion) | Bundle (fusion) |
|---|---:|---:|---:|
| Heavy pose drift | 8% | 15% | 8% |
| Field doesn't match the map | 60% | 60% | 45% |
| Opponent parks in the route | 32% | 32% | 20% |
| Everything at once (realistic fidelity) | 15% | 15% | 15% |
| Tight corridor, mixed deviation | 10% | 52% | 20% |

## Honest findings

- Every fusion constant (`ODOMETRY_FUSION_CONFIDENCE`, `APRILTAG_FUSION_CONFIDENCE`, `APRILTAG_SYSTEMATIC_BIAS_CELLS`, `APRILTAG_BAD_DETECTION_PROBABILITY`, `APRILTAG_BAD_DETECTION_SIGMA_CELLS`, `FUSION_DISAGREEMENT_THRESHOLD_CELLS`, `FUSION_DISTRUST_FACTOR` -- all in ftc/config.py) is an engineering estimate with no real AprilTag-vs-odometry disagreement measurement behind it, the same status as this project's other unmeasured constants until real data goes through ftc/calibration.py. This study's answer is conditional on those estimates, not a calibrated number.
- The disagreement check (nav/estimation.py's `fuse()`) uses a fixed distance threshold rather than one scaled by how much the prior itself has already drifted -- a documented simplification (see ftc/fusion.py's module docstring). A sanity check across this project's typical error-magnitude range found roughly 6.5% of ORDINARY (non-bad) detections still cross the threshold purely from correction magnitude, against roughly 87% of genuinely bad ones -- real separation, not perfect separation.
- Fusion in this project only ever applies to a bundle's tag-detection events -- it does not touch obstacle sensing, heading correction, or any suite that isn't `fixes_pose`. A bundle that senses obstacles (e.g. FullSuite) is unaffected by anything in this study beyond its own AprilTag component's position correction.
