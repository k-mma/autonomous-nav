# Does re-aiming mecanum's held heading recover its unpaid premium?

`ftc_drivetrain_writeup.md` found MECANUM's $130 premium over TANK NOT repaid under the one heading policy that study tested (hold a heading fixed at match start, aimed at the nearest AprilTag wall) -- and named the untested alternative explicitly: a policy that re-picks its held heading as the match progresses. This module checks that alternative directly, crossing MECANUM_HEADING_POLICY_ORDER (tank, plus MECANUM under `fixed_at_start` / `nearest_tag_current` / `route_dominant` / `match_travel`, see ftc/drivetrain.py) with the identical fidelity tiers x suites x deviation types x levels [0.3, 0.5, 0.7, 0.9] x 15 trials/point this module's original sweep already uses, on a DIFFERENT base seed (HEADING_POLICY_BASE_SEED) so this study's trials never overlap with `ftc_drivetrain_writeup.md`'s. Raw data in `ftc_drivetrain_heading_policy_results.csv`, chart in `ftc_drivetrain_heading_policy_comparison.png`.

## Does any alternative policy beat fixed_at_start?

Pooled across every suite, paired on identical scenarios, at the `realistic` fidelity tier (where the camera-FOV mechanism this whole investigation is about actually applies -- see `ftc_drivetrain_writeup.md`'s own optimistic-tier control):

| Policy | Success rate | vs. fixed_at_start [95% CI] | Verdict |
|---|---:|---:|---|
| fixed_at_start (baseline) | 7% | -- | -- |
| Mecanum (re-aim to nearest tag) | 7% | +0.0% [-1.0%, +1.0%] | not distinguishable from noise |
| Mecanum (aim along route) | 9% | +1.8% [+0.7%, +3.0%] | **significantly better** |
| Mecanum (match next leg) | 13% | +6.7% [+5.2%, +8.4%] | **significantly better** |

Tank (no held heading at all, for reference): 18%.

At least one alternative heading policy (Mecanum (aim along route), Mecanum (match next leg)) is a statistically real improvement over fixed_at_start -- re-aiming genuinely helps, exactly the mechanism `ftc_drivetrain_writeup.md` predicted but didn't have a policy to demonstrate it with. Whether that improvement is enough to catch up to tank (see the per-suite table below) is a separate question from whether it helps at all.

## Does any policy actually catch up to tank?

The comparison above is against fixed_at_start (MECANUM's own baseline policy), not against tank -- closing part of MECANUM's internal gap is a different question from closing the gap with tank itself, which is the one README.md's "no mecanum-specific strafing advantage" threat-to-validity entry actually asks. Same pooled-across-every-suite, paired-on-identical-scenarios comparison, against TANK directly this time:

| Policy | Success rate | vs. tank [95% CI] | Verdict |
|---|---:|---:|---|
| Mecanum | 7% | -10.8% [-12.8%, -9.0%] | **significantly worse than tank** |
| Mecanum (re-aim to nearest tag) | 7% | -10.8% [-12.8%, -8.9%] | **significantly worse than tank** |
| Mecanum (aim along route) | 9% | -9.0% [-10.8%, -7.1%] | **significantly worse than tank** |
| Mecanum (match next leg) | 13% | -4.1% [-5.9%, -2.5%] | **significantly worse than tank** |

## Best value by policy (realistic fidelity)

| Policy | Best value | pp/$100 |
|---|---|---:|
| Tank | Odometry pods | +9.7 |
| Mecanum | Odometry pods | +3.7 |
| Mecanum (re-aim to nearest tag) | Odometry pods | +4.0 |
| Mecanum (aim along route) | Odometry pods | +3.8 |
| Mecanum (match next leg) | Odometry pods | +4.7 |

## Per-suite success rate (realistic fidelity)

| Suite | Tank | Mecanum | Mecanum (re-aim to nearest tag) | Mecanum (aim along route) | Mecanum (match next leg) |
|---|---:|---:|---:|---:|---:|
| Dead reckoning | 14% | 6% | 5% | 7% | 11% |
| Odometry pods | 37% | 19% | 19% | 21% | 28% |
| Distance sensors | 9% | 3% | 2% | 4% | 7% |
| AprilTag (front camera) | 15% | 6% | 6% | 8% | 11% |
| IMU | 14% | 6% | 5% | 7% | 11% |
| Rear camera | 18% | 6% | 6% | 7% | 12% |
| Full suite | 16% | 4% | 4% | 6% | 14% |

## What this does and does not prove

Same reduced-rigor scope as `ftc_drivetrain_writeup.md` (see that module's own docstring) -- enough to see whether re-aiming helps at all, not a publication-grade estimate of the exact magnitude. All three alternative policies are single, specific, documented choices (ftc/drivetrain.py's resolve_held_heading_deg) -- `nearest_tag_current` optimizes for keeping a tag in view, `route_dominant` optimizes for the route's average direction, and `match_travel` (the one policy that directly targets the route's own IMMEDIATE next leg -- the gap both `ftc_drivetrain_writeup.md` and an earlier version of this writeup named as untested) holds the identical heading TANK would already be facing on that leg -- avoiding the strafe penalty on most steps, at NO turn-cost charge (Drivetrain.turn_cost_s is 0 for any holonomic drivetrain, ftc/drivetrain.py, regardless of how often its held heading changes -- a real mecanum chassis blends re-aiming into the same wheel commands still driving it forward, unlike TANK's forced stop-pivot-accelerate). What `match_travel` does NOT avoid is residual strafe: it's re-aimed once per leg from the PLANNED path's nominal direction, not the tick's true post-error travel heading, so position noise still produces some mismatch between chassis heading and actual travel direction -- a genuine trade-off to measure, not a strictly-better-by-construction policy.
