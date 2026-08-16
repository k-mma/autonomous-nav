# Does re-aiming mecanum's held heading recover its unpaid premium?

`ftc_drivetrain_writeup.md` found MECANUM's $130 premium over TANK NOT repaid under the one heading policy that study tested (hold a heading fixed at match start, aimed at the nearest AprilTag wall) -- and named the untested alternative explicitly: a policy that re-picks its held heading as the match progresses. This module checks that alternative directly, crossing MECANUM_HEADING_POLICY_ORDER (tank, plus MECANUM under `fixed_at_start` / `nearest_tag_current` / `route_dominant`, see ftc/drivetrain.py) with the identical fidelity tiers x suites x deviation types x levels [0.3, 0.5, 0.7, 0.9] x 15 trials/point this module's original sweep already uses, on a DIFFERENT base seed (HEADING_POLICY_BASE_SEED) so this study's trials never overlap with `ftc_drivetrain_writeup.md`'s. Raw data in `ftc_drivetrain_heading_policy_results.csv`, chart in `ftc_drivetrain_heading_policy_comparison.png`.

## Does either alternative policy beat fixed_at_start?

Pooled across every suite, paired on identical scenarios, at the `realistic` fidelity tier (where the camera-FOV mechanism this whole investigation is about actually applies -- see `ftc_drivetrain_writeup.md`'s own optimistic-tier control):

| Policy | Success rate | vs. fixed_at_start [95% CI] | Verdict |
|---|---:|---:|---|
| fixed_at_start (baseline) | 7% | -- | -- |
| Mecanum (re-aim to nearest tag) | 7% | +0.0% [-1.0%, +1.0%] | not distinguishable from noise |
| Mecanum (aim along route) | 9% | +1.8% [+0.8%, +2.9%] | **significantly better** |

Tank (no held heading at all, for reference): 18%.

At least one alternative heading policy is a statistically real improvement over fixed_at_start -- re-aiming genuinely helps, exactly the mechanism `ftc_drivetrain_writeup.md` predicted but didn't have a policy to demonstrate it with. Whether that improvement is enough to catch up to tank (see the per-suite table below) is a separate question from whether it helps at all.

## Best value by policy (realistic fidelity)

| Policy | Best value | pp/$100 |
|---|---|---:|
| Tank | Odometry pods | +7.1 |
| Mecanum | Odometry pods | +3.0 |
| Mecanum (re-aim to nearest tag) | Odometry pods | +3.2 |
| Mecanum (aim along route) | Odometry pods | +3.1 |

## Per-suite success rate (realistic fidelity)

| Suite | Tank | fixed_at_start | nearest_tag_current | route_dominant |
|---|---:|---:|---:|---:|
| Dead reckoning | 14% | 6% | 5% | 7% |
| Odometry pods | 37% | 19% | 19% | 21% |
| Distance sensors | 9% | 3% | 2% | 4% |
| AprilTag (front camera) | 15% | 6% | 6% | 8% |
| IMU | 14% | 6% | 5% | 7% |
| Rear camera | 18% | 6% | 6% | 7% |
| Full suite | 16% | 4% | 4% | 6% |

## What this does and does not prove

Same reduced-rigor scope as `ftc_drivetrain_writeup.md` (see that module's own docstring) -- enough to see whether re-aiming helps at all, not a publication-grade estimate of the exact magnitude. Both alternative policies are single, specific, documented choices (ftc/drivetrain.py's resolve_held_heading_deg) -- `nearest_tag_current` optimizes for keeping a tag in view, `route_dominant` optimizes for the route's average direction, and NEITHER directly minimizes per-step strafe against the route's own IMMEDIATE next leg, which a real team implementing this on hardware might reasonably try instead. This module does not test that policy.
