# Threats to validity / limitations

Full detail behind the condensed list in [README.md](README.md#threats-to-validity--limitations).
Naming these plainly is what separates a research testbed from a demo
-- none of them are secret, and none of them are fixed by this repo
alone.

<a id="synthetic-ground-truth"></a>
- Synthetic ground truth. Every trial's "ground truth" grid
  (`nav/field_variance.py`'s `generate_ground_truth`) is a
  procedurally-perturbed copy of the assumed map, not a measurement of
  a real field. The perturbation model (start drift, obstacle drift,
  an unplanned blocker) is a hypothesis about what kinds of deviation
  matter, not a validated model of what FTC fields actually do.
<a id="sensor-fusion-conflict"></a>
- No sensor-fusion conflict -- BOUNDED for one pairing (AprilTag vs.
  odometry pods), still open for every other. `ftc/bundle.py` merges a
  bundle's capabilities optimistically: obstacle sensors union their
  detections, the best localization hardware sets the drift rate, and
  multiple cameras feed one detection pipeline. Two sensors
  *disagreeing* about where the robot is -- and the filter/fusion work
  of resolving that -- was not modeled at all, so the optimizer's
  bundle results were an upper bound on what combining buys, size
  unknown. `nav/estimation.py` (domain-neutral confidence-weighted
  fusion of two position estimates) and `ftc/fusion.py` (the wiring
  that applies it to a tag detection disagreeing with odometry's own
  tracked position, opt-in via `run_match`'s `fusion=None` default --
  see `ftc/scratch/fusion_test.py` for the byte-for-byte no-op
  guarantee) measure that size for the one bundle this project's own
  research question centers on. `benchmark_results/
  ftc_fusion_writeup.md`'s finding: at this project's estimated fusion
  constants, the AprilTag+odometry bundle's advantage over its best
  single component doesn't just shrink, it inverts -- 35% success under
  optimistic merging vs. 22% under confidence-weighted fusion (paired
  95% CI [-18.5%, -9.0%], a statistically significant drop), which
  puts the fused bundle *below* its own best single component, Odometry
  pods (25%). The mechanism:
  AprilTag corrects pose often enough in this project's matches that a
  5%-per-detection chance of an outright bad reading (a misidentified
  or occluded tag, `APRILTAG_BAD_DETECTION_PROBABILITY`) compounds to a
  real chance of at least one happening per match, and this project's
  match model has no recovery from a single badly wrong correction by
  default (`on_collision="halt"`). Every fusion constant is an
  engineering estimate with no real AprilTag-vs-odometry disagreement
  measurement behind it -- the same uncalibrated status as every other
  constant in this file -- so this bounds how loose the optimistic-
  merge upper bound *could* be at a plausible bad-detection rate, it
  does not calibrate how loose it *actually* is, and it says nothing
  about any other bundle pairing (a bundle with obstacle-sensing
  suites, or two pose-fixing suites other than this one, is untouched
  by any of this). Real fusion also costs integration effort this
  project's dollar model can't price; the `parts` count is the only
  proxy for it. A THIRD fusion strategy -- `nav/kalman.py`'s
  variance-aware Kalman update, `run_match(fusion="kalman")` -- narrows
  this further; see "Uncalibrated variance" below and
  `benchmark_results/ftc_fusion_kalman_writeup.md` for what it changes
  and, just as importantly, what it still doesn't.
<a id="uncalibrated-variance"></a>
- Uncalibrated variance -- BOUNDED, not closed, until real
  measurements are supplied. `variance_level` and `ftc/sensors.py`'s
  drift-rate constants are order-of-magnitude engineering estimates
  (see ftc/config.py's per-constant source comments) until `ftc/
  calibration.py` is run against real CSVs. `ftc/robustness.py` sweeps
  every estimated constant from 0.25x-4x its documented value and
  checks whether the best-value recommendation (AprilTag) survives
  being wrong by that much -- see `benchmark_results/
  ftc_robustness_writeup.md` for exactly which parameters tip the
  ranking and at what multiplier, and which never do across the swept
  range. A tipping point bounds how wrong an estimate can be before the
  conclusion changes; it does not tell you whether the *real* value is
  inside or outside that bound. Only `ftc/calibration.py` run against
  real measurements closes this -- right now, none of it has been
  checked against a real field or robot; the synthetic placeholder
  dataset exists to make the pipeline runnable, not to make its output
  trustworthy.

  What changed: this project deliberately never used a Kalman filter
  for AprilTag/odometry fusion, for exactly this reason -- a Kalman
  filter's whole mechanism compares the VARIANCE of a prior estimate
  against the variance of a new observation, and there was no real
  variance anywhere in this project to compare, only a single scalar
  drift value (see `ftc/fusion.py`'s original module docstring, still
  true of the confidence-weighted default). `ftc/calibration.py` now
  fits BOTH halves a real Kalman filter needs: `fit_apriltag_
  measurement_variance` (an OLS fit of position-error variance against
  real detection range/incidence scatter, replacing `APRILTAG_RANGE_
  DEGRADATION`/`APRILTAG_ANGLE_DEGRADATION`'s ASSUMED linear shape) and
  `fit_process_variance_per_cell` (the odometry drift-rate fit already
  in this file, squared into the variance a Kalman predict step
  actually consumes). `nav/kalman.py` is the estimator itself (proven
  standalone in `nav/scratch/kalman_test.py`); `ftc/fusion.py`'s
  `fused_tag_correction_kalman` wires it into `ftc/match.py` as a THIRD
  opt-in fusion mode (`run_match(fusion="kalman")`, alongside the
  existing `None`/confidence-weighted paths, which are unaffected --
  see `ftc/scratch/fusion_kalman_test.py`'s cross-call-interference
  check). `benchmark_results/ftc_fusion_kalman_writeup.md` is the
  fusion-strategy comparison this unlocks: at this project's SYNTHETIC
  placeholder variance (the machinery is built, the real measurement
  still is not), Kalman fusion (26%) beats confidence-weighted fusion
  (22%) by a statistically real margin (+5.0%, 95% CI [+2.0%, +8.5%]),
  and edges narrowly back above its own best single component (Odometry
  pods, 25%) -- the inversion doesn't just shrink under Kalman
  specifically, it reverses, if only barely (+2%, not separately tested
  for significance against the single-component floor). Optimistic
  merging still wins outright (35% vs. 26% Kalman), just by a much
  smaller margin than confidence-weighted fusion (22%) leaves on the
  table. This BOUNDS the "would a real Kalman filter have
  fixed the inversion" question -- the math genuinely helps, but not
  enough to flip the verdict at this project's own estimated fusion
  constants -- it does not CLOSE it, since "this project's own
  estimated fusion constants" is still the load-bearing uncertainty:
  real AprilTag detection scatter run through `ftc/calibration.py`
  (`--apriltag`, `fit_apriltag_measurement_variance`) is the only thing
  that would.
<a id="simplified-kinematics"></a>
- Simplified kinematics -- BOUNDED. `ftc/match.py` now charges
  drive time via a trapezoidal (accelerate/cruise/decelerate) velocity
  profile bounded by `MAX_ACCEL_MPS2` (`ftc/match.py`'s
  `_trapezoidal_drive_time_s`) rather than assuming instantaneous
  acceleration to `MAX_DRIVE_SPEED_MPS` -- a real, if still simplified,
  improvement (a single 6in cell step is almost always too short to
  reach cruise speed at all, so drive time per step is now ~4-5x the
  old naive distance/speed figure). `ftc/config.py`'s optional
  `GEARING_OPTIONS` (`ftc/gearing_benchmark.py`), now built directly
  from goBILDA's own published 5203-series RPM/torque table rather than
  invented multipliers, surfaces a sharper and more surprising finding
  than "faster costs drift": because motor torque falls as RPM rises,
  and every gearing option's accel-to-cruise distance is far larger
  than one 6in grid cell, a faster ratio is strictly SLOWER per cell in
  this model, not faster, on top of drifting more (`slip_factor`) --
  buying speed doesn't even buy the thing it promises at this project's
  short-hop grid scale. Measured: at every budget tested (30s/15s/10s),
  faster gearing options measurably lose to stock, averaged across all
  7 headline suites (a single suite that already fixes pose, e.g.
  AprilTag or odometry pods, would likely absorb the extra drift better
  -- not checked per-suite here). What's still not modeled:
  velocity isn't carried across consecutive collinear steps (each step
  starts and ends at rest, the same assumption the existing
  per-90-degree turn cost already makes), and wheel slip beyond what
  speed alone predicts (cornering, robot mass, floor traction). A real
  robot's actual time-to-goal will still differ from this model's
  prediction by some amount this repo doesn't measure.
<a id="planning-time-model"></a>
- Planning-time model validated at the scale this repo actually
  publishes at, not beyond it, and not on real hardware -- BOUNDED.
  `ftc/match.py`'s `elapsed_s`/`over_budget` accounting never reads the
  wall-clock time it measures around each `astar()` call
  (`MatchResult.planning_time_s`) -- only a flat `PLANNING_OVERHEAD_S`
  (50ms, a documented estimate of real onboard-compute cost, not raw
  Python search time) is charged, on every replan after the first.
  Structurally, this means no measured planning latency, however
  large, can change a match outcome under the model as implemented --
  confirmed directly in `ftc/scratch/planning_latency_test.py` by
  artificially delaying every `astar()` call and checking `elapsed_s`
  comes out byte-for-byte identical either way. `ftc/planning_
  latency_benchmark.py` measures the real per-call latency distribution
  (median/p99/max) this repo had never looked at, across 5 grid sizes
  (24 through 384 cells/side) x 3 layouts x 7 suites, and asks the
  question that actually matters: would a match currently reported as
  under budget flip to over budget if `elapsed_s` used each match's own
  REAL measured planning time instead of the flat constant? At this
  project's native, published grid scale, no -- 0 of 1260 matches tested
  flip under that counterfactual (`benchmark_results/planning_
  latency_writeup.md`), and none do at any synthetic size tested either,
  because a bounded-path-length scenario design (deliberate, to keep
  drive time from confounding the grid-size axis) keeps A*'s own search
  cost small regardless of total grid size -- a supplementary,
  unbounded-path check in that same writeup shows latency growing to
  175ms (>3x PLANNING_OVERHEAD_S) once path length itself is allowed to
  grow, confirming path length, not raw cell count, is what actually
  drives this cost. This BOUNDS the question, it does not CLOSE it: it
  says tail latency doesn't matter at the scale and path lengths this
  repo's own published numbers were measured at, on one developer
  laptop (Apple M1, macOS -- see the writeup for exact versions), not
  that it could never matter on slower, FTC-legal onboard hardware (a
  REV Control Hub) or at longer path lengths -- only real onboard
  timing data run through `ftc/calibration.py` could calibrate
  `PLANNING_OVERHEAD_S` itself, which this study does not attempt.
<a id="mecanum-strafing"></a>
- No mecanum-specific strafing advantage -- BOUNDED across three
  documented heading policies now, still not a universally closed
  question. `ftc/field.py` defaulted to 8-directional movement "on the
  assumption of a holonomic drivetrain" without ever actually modeling
  one; `ftc/drivetrain.py` adds TANK/MECANUM as an axis orthogonal to
  sensor suite (TANK pays the existing flat per-90-degree turn cost on
  every direction change; MECANUM pays none, but a speed/drift penalty
  on any step that isn't roughly forward relative to whatever heading
  it currently holds). `ftc/drivetrain_benchmark.py`'s original
  finding: mecanum's $130 premium (`MECANUM_WHEEL_COST_USD -
  TANK_WHEEL_COST_USD`) is NOT repaid under the one heading policy that
  study tested (hold heading toward the nearest AprilTag wall FIXED
  from match start) -- a route's travel direction changes far more
  often than that one fixed heading does, so most steps end up
  strafing, and the resulting drift penalty swamps the camera-stays-
  aimed-at-tags benefit the policy was chosen to demonstrate. See
  `benchmark_results/ftc_drivetrain_writeup.md` for the full diagnosis
  -- unchanged by everything below, since that study and its numbers
  are already cited elsewhere in this repo.

  What changed: `heading_policy` (`ftc/drivetrain.py`) is now a
  swappable field on `Drivetrain`, not a single hardcoded shape --
  `fixed_at_start` (the original, unchanged default), `nearest_tag_
  current` (re-aim toward whichever tag is nearest wherever the robot
  actually is right now), and `route_dominant` (aim along the
  currently-planned route's own circular-mean travel direction, which
  minimizes total strafe against that specific route rather than
  against any tag at all). `ftc/drivetrain_benchmark.py`'s SECOND,
  separate sweep (`benchmark_results/
  ftc_drivetrain_heading_policy_writeup.md`, own output files, own base
  seed, doesn't touch the original study's numbers) measures what
  re-aiming actually buys: `route_dominant` is a real, paired-
  bootstrap-significant improvement over `fixed_at_start` (7% -> 9%
  pooled success rate at the `realistic` fidelity tier), while
  `nearest_tag_current` is not distinguishable from the fixed baseline
  at this trial count. Neither alternative closes the gap to tank
  (18%), and neither changes which sensor suite is the best value on
  mecanum (still Odometry pods, not AprilTag or Rear camera, under
  every heading policy tested) -- re-aiming genuinely helps, exactly the mechanism
  the original study predicted but had no policy to demonstrate with,
  it just doesn't help enough to flip either headline verdict. A
  policy that minimizes strafe against the route's own IMMEDIATE next
  leg (rather than the route's average direction, or a fixed tag)
  remains untested -- see `ftc_drivetrain_heading_policy_writeup.md`'s
  own closing section.

  A separate, full-rigor question neither sweep above answers: does
  the *headline* best-value recommendation itself (AprilTag, measured
  under the default tank-equivalent drivetrain) hold for the large
  fraction of FTC teams that run mecanum? `ftc/
  drivetrain_suite_benchmark.py` reruns `ftc_suite_writeup.md`'s exact
  full-rigor sweep -- all 11 variance_level steps, all 3 deviation
  types, 25 trials/point, nothing reduced -- once per drivetrain, for
  all 7 headline suites (own output files, own labeled writeup,
  doesn't touch either sweep above). The answer: no -- the best-value
  suite is drivetrain-dependent, AprilTag under tank (the headline
  default) but Odometry pods under mecanum -- and every suite's raw
  success rate drops substantially on mecanum regardless, from -8
  points (Distance sensors) up to -15 (AprilTag (front camera) and Rear
  camera, the largest drops) -- see `benchmark_results/
  ftc_drivetrain_suite_writeup.md` for the full per-suite table.
<a id="camera-fov-heading-error"></a>
- Camera field of view and heading error -- previously UNSTATED,
  now BOUNDED via `ftc/config.py`'s `MODEL_FIDELITY` tiers, not
  calibrated. Every number in this README before this addition assumed
  an omnidirectional camera (`ftc/sensors.py`'s `AprilTagSuite.
  tag_correction` accepted `heading_deg_now` and never used it -- a tag
  was detected regardless of which way the robot's camera actually
  faced) and perfect heading knowledge (`ftc/match.py` tracked pose
  error as a translation vector only, with no heading error anywhere,
  even though small angular error compounding into large lateral error
  over distance is the dominant real dead-reckoning failure mode). The
  "optimistic" tier reproduces that assumption exactly, by construction
  (see `ftc/scratch/fidelity_test.py`'s regression check) -- it is not
  a new, more honest default, it's the OLD default now given a name so
  it can be compared against something. "Realistic" and "pessimistic"
  add real camera-FOV gating, heading drift that actually rotates
  executed motion (not just a reported number), and, at the pessimistic
  tier, AprilTag detection dropout. `ftc/fidelity_benchmark.py`'s
  finding: the best-value suite does not survive even the first step
  off "optimistic" -- AprilTag (+16.0pp/$100) flips to Odometry pods at
  "realistic" (+6.1pp/$100) and stays there at "pessimistic"
  (+5.8pp/$100). Fidelity tiers BOUND this
  gap -- they
  make its size visible and swept -- they do NOT CALIBRATE it: every
  non-optimistic tier's constants (`CAMERA_FOV_DEG_BY_TIER`,
  `HEADING_DRIFT_DEG_PER_CELL_BY_TIER`, etc.) are documented ballpark
  engineering estimates, the same status as every other estimated
  constant in this file, until real measurements are run through `ftc/
  calibration.py`.
<a id="no-opponent-modeling"></a>
- No opponent modeling -- BOUNDED. The existing `unplanned_blocker`
  deviation type (one static obstacle, dropped once and left in place)
  is now joined by a `moving_blocker` variant in `ftc/
  opponent_benchmark.py`, which reuses `nav/obstacles.py`'s
  `MovingObstacle` (a seeded random walk, ticked on simulated match
  time) for a genuinely moving opponent, added alongside the static
  version rather than replacing it. The finding: a moving opponent
  DOES appear to change which suite is the best value, though not
  cleanly -- Distance sensors is best against a static blocker
  (+10.1pp/$100), while AprilTag (front camera) edges out Rear camera
  against a moving one (+26.0pp/$100 vs. +12.0pp/$100, close enough
  that the two suites' success-rate confidence intervals still overlap
  at this trial count -- treat that particular ranking as plausible,
  not confirmed). What IS confirmed: neither winner is Full suite, and
  Distance sensors's static-blocker win isn't close -- see
  `benchmark_results/ftc_opponent_writeup.md`.
  Suites that never sense obstacles at all still do substantially
  better against a moving opponent than a static one, purely from
  timing luck (a parked obstacle blocks a fixed plan deterministically;
  a wandering one often isn't there anymore by the time a blind suite's
  plan reaches that cell). Still not modeled: the opponent has no goals
  of its own and doesn't react to this robot's presence -- a random
  walk is a step up from a fixed point, not a full multi-agent model.
<a id="one-field-layout"></a>
- One field layout for the headline study -- CLOSED. `ftc/
  suite_benchmark.py` still runs on the `'cluttered'` layout only, but
  `ftc/layout_benchmark.py` reruns the identical full-rigor sweep on
  all three layouts `ftc/field.py` ships (sparse, cluttered, corridor)
  and checks whether the best-value suite changes: it doesn't --
  AprilTag is the best-value suite on every layout tested (see
  `benchmark_results/ftc_layout_writeup.md`). This closes the "would it
  shift on a different layout" question for the three layouts this repo
  actually ships; a season-specific surveyed layout dropped in later
  (see `ftc/field.py`'s module docstring) is still unchecked.
<a id="budget-rarely-binds"></a>
- Small, fast trials mean the 30-second budget rarely binds --
  CLOSED. `ftc/budget_benchmark.py` sweeps `AUTONOMOUS_PERIOD_S`
  downward (30s to 1.5s) and finds it now binds starting at 10s
  under the trapezoidal kinematics model above (it barely bound at all
  under the old naive drive-time formula) -- see `benchmark_results/
  ftc_budget_writeup.md`. Tightening the budget far enough does change
  which suite wins by raw success rate (Full suite overtakes Odometry
  pods as the #1 suite at 4s, a statistically clean change, CIs don't
  overlap), though the headline 30s budget itself still never binds in
  the actual headline sweep.
<a id="live-replanner-assumption"></a>
- Every match in this repo assumes a live, onboard A* replanner --
  BOUNDED. Real FTC teams overwhelmingly run a fixed, hand-tuned
  sequence of moves worked out before the match, not a pathfinder
  running live during it; this repo's entire match model assumed the
  opposite, unstated, until now. `ftc/match.py`'s `run_match(
  scripted_auto=True)` plans exactly once, from the assumed map, then
  drives that route with zero reconsideration -- no reroute for a
  sensed obstacle, a tag correction, or a stall (nav/policies.py's
  `OpenLoopPolicy` already names this exact concept -- "what a
  standard FTC autonomous routine does today" -- but its minimal
  grid-only interface couldn't carry this project's kinematics/sensor/
  fusion machinery, so the concept was reused and the code wasn't).
  `ftc/scripted_auto_benchmark.py`'s finding, restricted to the two
  deviation types an obstacle sensor has any mechanism to react to
  (`obstacle_drift`, `unplanned_blocker` -- pooling in `start_drift`,
  pure pose error, would dilute the exact question being asked):
  DistanceSensorSuite's live-replanning advantage over dead reckoning
  was already too small to separate from noise at this trial count
  (consistent with the blind-spot finding two bullets up), so this
  particular comparison can't cleanly show the predicted collapse --
  but the STRUCTURAL argument is confirmed directly regardless: a
  pose-fixing suite (AprilTag, Odometry pods) stays measurably ahead of
  dead reckoning even under scripted auto (correcting the believed-to-
  true position mapping still helps the SAME fixed route land closer
  to plan, no reroute required), while an obstacle-sensing suite's own
  `senses_obstacles` flag keeps sensing (harmlessly) with no avenue
  left to ever act on what it finds -- pose error and obstacle error
  really are different failure modes with different dependence on live
  replanning, not just different in degree. See `benchmark_results/
  ftc_scripted_auto_writeup.md` for the full breakdown, including the
  one modeling choice actually load-bearing here: the single plan
  `scripted_auto` makes is computed against the full assumed map
  regardless of sensor suite (a stand-in for a team planning by hand
  against the field's known layout), not against whatever a live
  sensor cone would have seen in the match's first instant -- getting
  that backwards would have penalized an obstacle-sensing suite's one
  plan for a reason unrelated to the actual question. Still open: a
  real hand-tuned routine can include contingency branches a team
  scripts by hand ("if blocked here, try this instead") -- a form of
  scripting with SOME reactivity this binary flag can't represent.

## Changelog

<a id="odometry-pod-pricing-correction"></a>
**Odometry pod pricing correction.** This project used to price
odometry pods at goBILDA's $279.99 2-pod-plus-Pinpoint-computer
bundle, on the assumption that dead-wheel odometry needs a separate
fusion computer to read. It doesn't -- Optii's wiring docs confirm
their pods plug directly into a hub encoder port, and goBILDA's own
pods are readable the same way. Fixing it changes this project's own
reliability-per-dollar headline; see the results table in
[README.md](README.md#ftc-sensor-suite-study-results) and
`benchmark_results/ftc_suite_writeup.md`.
