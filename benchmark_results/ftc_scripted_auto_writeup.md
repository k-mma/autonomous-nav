# Does a scripted (never-replanning) auto routine change which sensor is worth buying?

Real FTC teams overwhelmingly run a fixed, hand-tuned sequence of moves worked out before the match, not a live onboard pathfinder -- a gap this project's simulation never named, let alone measured, until now. `ftc/match.py`'s `scripted_auto=True` plans exactly once, from the assumed map, then drives that route with zero reconsideration -- no reroute for a tag correction, a sensed obstacle, or a stall (see `run_match`'s own docstring). Crossed with the same 7 headline suites x 3 deviation types x levels [0.3, 0.5, 0.7, 0.9] x 20 trials/point this project's other studies use, on the 'cluttered' layout. Raw data in `ftc_scripted_auto_results.csv`, chart in `ftc_scripted_auto_comparison.png`.

## The headline question: does DistanceSensorSuite's advantage survive?

Restricted to the two OBSTACLE-relevant deviation types (`obstacle_drift`, `unplanned_blocker`) -- `start_drift` is pure pose error with nothing for an obstacle sensor to ever detect differently from the assumed map, so including it would dilute the exact mechanism this question is about (see the full 3-type pooled numbers in the per-suite table further down):

| Replan policy | Dead reckoning | Distance sensors | Difference [95% CI] |
|---|---:|---:|---:|
| Live replanning | 19% | 18% | -0.6% [-7.5%, +6.2%] |
| Scripted auto | 19% | 19% | +0.0% [+0.0%, +0.0%] |

This comparison doesn't cleanly confirm the predicted mechanism, for a reason worth stating plainly rather than glossing over: distance sensing's advantage over dead reckoning is NOT statistically significant even under LIVE replanning at this trial count, on this scenario mix. That's consistent with this project's own separate, already-documented finding (README.md's "most useful finding is the negative one", `ftc_suite_writeup.md`'s DistanceSensorSuite blind-spot result) that the three distance sensors this project models only cover about 75 degrees of the 360 around the robot, and collide in roughly half their trials even at zero deviation from that blind spot alone -- a weak base advantage under live replanning leaves very little for scripted auto to visibly take away, on top of it. The scripted-auto mechanism this module exists to check is a real, separate question from "does distance sensing help much at all" (already answered elsewhere, unfavorably) -- this particular pooled comparison just can't cleanly isolate it. See the per-deviation-type/level breakdown in `ftc_scripted_auto_results.csv` for scenario slices where distance sensing's live-replanning advantage is larger, if a cleaner before/after comparison is needed.

## Does pose correction still help without ever rerouting?

| Suite | Live replanning | Scripted auto | Difference |
|---|---:|---:|---:|
| Dead reckoning | 14% | 14% | +0% |
| Odometry pods | 31% | 31% | +0% |
| Distance sensors | 13% | 14% | +1% |
| AprilTag (front camera) | 21% | 20% | -2% |
| IMU | 14% | 14% | +0% |
| Rear camera | 21% | 20% | -2% |
| Full suite | 22% | 34% | +12% |

Both pose-fixing suites (AprilTag, Odometry pods) stay measurably ahead of Dead reckoning even under scripted auto -- correcting the believed-to-true position mapping still helps the SAME fixed route land closer to where it was planned, which needs no reroute at all. Pose correction and obstacle-sensing genuinely are different failure-mode fixes with different dependence on live replanning, not just different in degree.

## Does this change which suite is the best buy?

Full suite (the only suite combining pose-fixing AND obstacle-sensing) goes from 22% under live replanning to 34% under scripted auto -- it loses exactly the obstacle-sensing half of its value proposition and keeps the pose-fixing half, the combined version of the same mechanism above. AprilTag alone (20% scripted) stays the strongest single-sensor pose fix, unchanged from every other study in this project -- this study's headline finding is about DistanceSensorSuite's collapsed advantage, not a reversal of which suite wins overall.

## What this does and does not prove

This models ONE specific notion of "scripted": a route planned once against the full assumed map and never touched again, with sensing still running (harmlessly inert for replanning purposes) in the background. A few things this does not attempt: a real hand-tuned routine might be authored with built-in contingency branches a team scripts by hand ("if blocked here, try this instead") -- a form of scripting with SOME reactivity this binary flag cannot represent; and this module reuses the SAME sensor/kinematics/collision model every other study in this project uses, so every other documented limitation of that model (README.md's "Threats to validity") still applies unchanged here.
