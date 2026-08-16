# Is a faster-motor purchase worth it now that the budget binds?

`ftc/budget_benchmark.py` found `AUTONOMOUS_PERIOD_S` starts binding around 15-20s under the trapezoidal kinematics model -- the first point at which a faster drivetrain would have anything to win, IF it actually bought more speed where it matters. `ftc/config.py`'s `GEARING_OPTIONS` is built from goBILDA's published 5203-series RPM/torque table, and that data says it doesn't: torque falls as RPM rises, and every option's accel-to-cruise distance is far larger than one 6in grid cell, so a single step never reaches cruise speed regardless of gearing -- only acceleration governs per-cell drive time, and real acceleration is LOWER at every faster ratio (see ftc/scratch/gearing_test.py). `slip_factor` then scales `drift_per_cell` up on top of that. This is not a speed-vs-slip tradeoff; it's a lose-lose at this grid's cell scale. Crossed with budget at [30.0, 15.0, 10.0] (30s = the real budget where it never binds; 15s = right at the binding point; 10s = binds hard), averaged across all 7 headline suites, 15 trials/point, levels [0.3, 0.5, 0.7, 0.9], all 3 deviation types, 'cluttered' layout -- reduced relative to the headline sweep (see module docstring). Raw data in `ftc_gearing_results.csv`, chart in `ftc_gearing_comparison.png`.

## Success rate by gearing x budget

| Budget | Stock | Fast | Faster |
|---:|---:|---:|---:|
| 30s | 15% | 12% | 10% |
| 15s | 15% | 12% | 8% |
| 10s | 13% | 10% | 6% |

## Accumulated pose error by gearing (the slip cost)

| Budget | Stock | Fast | Faster |
|---:|---:|---:|---:|
| 30s | 4.88in | 5.43in | 6.14in |
| 15s | 4.89in | 5.43in | 6.11in |
| 10s | 4.87in | 5.40in | 5.81in |

## Does it pay off?

At 30s, faster gearing is worse, exactly as the per-cell kinematics predict: Faster gearing lands at 10% vs. stock's 15% (-5%), and the two suites' success-rate confidence intervals don't overlap at this trial count. This isn't a tradeoff that failed to pay off -- 'faster' gearing is strictly slower per cell AND drifts more (see the table above and ftc/scratch/gearing_test.py); there was never a time saving here for the drift cost to be weighed against. Buying speed without also buying something that corrects pose (odometry pods, AprilTag) makes the average suite's overall reliability worse, not better.
At 15s, faster gearing is worse, exactly as the per-cell kinematics predict: Faster gearing lands at 8% vs. stock's 15% (-7%), and the two suites' success-rate confidence intervals don't overlap at this trial count. This isn't a tradeoff that failed to pay off -- 'faster' gearing is strictly slower per cell AND drifts more (see the table above and ftc/scratch/gearing_test.py); there was never a time saving here for the drift cost to be weighed against. Buying speed without also buying something that corrects pose (odometry pods, AprilTag) makes the average suite's overall reliability worse, not better.
At 10s, faster gearing is worse, exactly as the per-cell kinematics predict: Faster gearing lands at 6% vs. stock's 13% (-7%), and the two suites' success-rate confidence intervals don't overlap at this trial count. This isn't a tradeoff that failed to pay off -- 'faster' gearing is strictly slower per cell AND drifts more (see the table above and ftc/scratch/gearing_test.py); there was never a time saving here for the drift cost to be weighed against. Buying speed without also buying something that corrects pose (odometry pods, AprilTag) makes the average suite's overall reliability worse, not better.

## What this does and does not prove

This is a reduced-rigor sweep (see module docstring), averaged across all 7 suites rather than reported per suite -- a real team would want to check this against the SPECIFIC suite it's actually running, since a suite that already fixes pose (AprilTag, odometry pods) can absorb the extra slip-driven drift better than one that can't (dead reckoning). The per-cell-slower finding itself is not a ballpark estimate -- it follows directly from goBILDA's own published RPM/torque table for the 5203 motor and this project's own grid cell size, both fixed facts, not tuned constants. `slip_factor` is the one number in GEARING_OPTIONS that remains an explicit ballpark engineering estimate: no vendor publishes slip-vs-gearing data, so it's a documented guess, same status as every other estimated constant in this project.
