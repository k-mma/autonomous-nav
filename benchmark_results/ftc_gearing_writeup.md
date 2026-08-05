# Is a faster-motor purchase worth it now that the budget binds?

`ftc/budget_benchmark.py` found `AUTONOMOUS_PERIOD_S` starts binding around 15-20s under the trapezoidal kinematics model -- the first point at which a faster drivetrain has anything to win. `ftc/config.py`'s `GEARING_OPTIONS` trade drive speed for wheel slip: `slip_factor` scales `drift_per_cell` up along with speed, so more speed isn't a free win. Crossed with budget at [30.0, 15.0, 10.0] (30s = the real budget where it never binds; 15s = right at the binding point; 10s = binds hard), averaged across all 5 headline suites, 15 trials/point, levels [0.3, 0.5, 0.7, 0.9], all 3 deviation types, 'cluttered' layout -- reduced relative to the headline sweep (see module docstring). Raw data in `ftc_gearing_results.csv`, chart in `ftc_gearing_comparison.png`.

## Success rate by gearing x budget

| Budget | Stock | Fast | Faster |
|---:|---:|---:|---:|
| 30s | 37% | 32% | 28% |
| 15s | 36% | 31% | 28% |
| 10s | 27% | 25% | 24% |

## Accumulated pose error by gearing (the slip cost)

| Budget | Stock | Fast | Faster |
|---:|---:|---:|---:|
| 30s | 4.85in | 5.75in | 6.67in |
| 15s | 4.85in | 5.73in | 6.66in |
| 10s | 4.79in | 5.65in | 6.61in |

## Does it pay off?

At 30s, faster gearing is ACTIVELY WORSE, not just unhelpful: Faster gearing lands at 28% vs. stock's 37% (-9%), and the two suites' success-rate confidence intervals don't overlap at this trial count -- at a 30s budget, the wheel-slip drift cost outweighs whatever time savings the extra speed bought. Buying speed without also buying something that corrects pose (odometry pods, AprilTag) makes the average suite's overall reliability worse here, not better.
At 15s, faster gearing is ACTIVELY WORSE, not just unhelpful: Faster gearing lands at 28% vs. stock's 36% (-8%), and the two suites' success-rate confidence intervals don't overlap at this trial count -- at a 15s budget, the wheel-slip drift cost outweighs whatever time savings the extra speed bought. Buying speed without also buying something that corrects pose (odometry pods, AprilTag) makes the average suite's overall reliability worse here, not better.
At 10s, no gearing option measurably beats stock (27%) at this trial count (fast: 25%, faster: 24%) -- the apparent drop doesn't clear the noise bar at this trial count; worth rechecking with more trials before calling it either a real cost or a real non-effect.

## What this does and does not prove

This is a reduced-rigor sweep (see module docstring), averaged across all 5 suites rather than reported per suite -- a real team would want to check this against the SPECIFIC suite it's actually running, since a suite that already fixes pose (AprilTag, odometry pods) can absorb the extra slip-driven drift better than one that can't (dead reckoning). GEARING_OPTIONS' speed/accel multipliers and slip_factor values are documented ballpark engineering estimates, the same status as every other estimated constant in this project.
