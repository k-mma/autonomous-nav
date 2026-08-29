# Which BUNDLE of sensors should an FTC team buy?

Every buildable combination of 7 sensor suites (ftc/sensors.py), up to 3 suites per bundle, composed by ftc/bundle.py and evaluated by ftc/optimizer.py over 5 scenario profiles x 25 seeded trials each. 63 raw combinations collapse to 23 distinct robots once bundles that buy identical hardware are recognized as the same purchase (2875 matches). Raw data in `ftc_optimizer_match_results.csv`, charts in `ftc_optimizer_match_frontier.png` and `ftc_optimizer_match_synergy.png`.

Two things make this different from `ftc_suite_writeup.md`'s seven-suite comparison:

- Bundles are costed over the UNION of their parts, so shared hardware is counted once. AprilTag + AprilTag-with-IMU is a $25 robot with one camera, not a $50 robot with two, and the two descriptions collapse to the same candidate before anything is simulated.
- Every candidate runs the identical seeded scenarios, so comparisons use a PAIRED bootstrap (nav/stats.py's `bootstrap_paired_diff_ci`) rather than checking whether two independent CIs overlap. Every "significant" below means a paired 95% CI that excludes zero AND a bootstrap p < 0.05, not a taller bar.

## The best robot and the most robust robot are not the same robot

| Rank | Robot | Cost | Weighted success | Worst scenario | pp/$100 |
|---:|---|---:|---:|---:|---:|
| 1 | odometry pods + front camera | $219.85 | 25% | 8% | +7.6 |
| 2 | odometry pods + IMU + front camera | $219.85 | 25% | 8% | +7.6 |
| 3 | odometry pods + front camera + rear camera | $244.85 | 25% | 8% | +6.9 |
| 4 | odometry pods + IMU + front camera + rear camera | $244.85 | 25% | 8% | +6.9 |
| 5 | front camera | $25.00 | 15% | 8% | +28.8 |
| 6 | IMU + front camera | $25.00 | 15% | 8% | +28.8 |
| 7 | front camera + rear camera | $50.00 | 15% | 8% | +14.4 |
| 8 | IMU + front camera + rear camera | $50.00 | 15% | 8% | +14.4 |
| 9 | odometry pods + front camera + 3 ToF sensors | $314.35 | 14% | 4% | +1.8 |
| 10 | odometry pods + IMU + front camera + 3 ToF sensors | $314.35 | 14% | 4% | +1.8 |

Best average: odometry pods + front camera ($219.85, 25%). Most robust -- highest success rate on its OWN WORST scenario, the right objective if you can't predict your division: front camera ($25.00, 8% worst-case vs. 8% for the best-average robot). Those worst-case rates are the same, so no robustness is being given up by taking the best-average robot here -- the two objectives happen to agree on worst-case performance, but still name different robots at different prices: $25.00 vs. $219.85 for +10% average success, which is the real choice on offer.

## Does bundling actually beat buying one sensor?

For each of the top bundles, a paired comparison against the best SINGLE suite that bundle itself contains -- the honest form of the question, since a bundle containing one strong sensor should not be credited with that sensor's performance:

| Bundle | Best single component | Bundle | Single | Gain | 95% CI (paired) | p | Extra cost | Verdict |
|---|---|---:|---:|---:|---|---:|---:|---|
| Odometry pods + AprilTag (front camera) | AprilTag (front camera) | 25% | 15% | +9.6% | [+4.0%, +16.0%] | 0.003 | $+194.85 | **significant** |
| Odometry pods + AprilTag + IMU | AprilTag + IMU | 25% | 15% | +9.6% | [+4.0%, +16.0%] | 0.001 | $+194.85 | **significant** |
| Odometry pods + Rear camera | Rear camera | 25% | 15% | +9.6% | [+4.0%, +16.0%] | 0.002 | $+194.85 | **significant** |
| Odometry pods + IMU + Rear camera | Rear camera | 25% | 15% | +9.6% | [+4.0%, +16.0%] | 0.000 | $+194.85 | **significant** |
| IMU + Rear camera | Rear camera | 15% | 15% | +0.0% | [+0.0%, +0.0%] | 1.000 | $+0.00 | no gain |
| Odometry pods + Distance sensors + AprilTag (front camera) | AprilTag (front camera) | 14% | 15% | -1.6% | [-8.8%, +5.6%] | 0.718 | $+289.35 | no gain |
| Odometry pods + Distance sensors + AprilTag + IMU | AprilTag + IMU | 14% | 15% | -1.6% | [-8.0%, +5.6%] | 0.712 | $+289.35 | no gain |
| Odometry pods + Distance sensors + Rear camera | Rear camera | 14% | 15% | -1.6% | [-8.8%, +5.6%] | 0.718 | $+289.35 | no gain |

4 of the 8 bundles tested beat their own best single component by a statistically significant margin. The cheapest of those upgrades is Odometry pods + AprilTag (front camera): +$194.85 over AprilTag (front camera) for +9.6% success rate [95% CI +4.0%, +16.0%].

By marginal value -- gain per dollar of the UPGRADE, not of the whole robot -- the best combination to buy is Odometry pods + AprilTag (front camera) at +4.9pp per extra $100 over AprilTag (front camera) alone.

The mechanism is the one this project's own deviation-type analysis predicts, and it holds for every significant bundle above without exception: each spans more than one capability category (pose fixing, obstacle sensing, drift reduction, heading holding) AND adds a category its best single component did not have. Two sensors that fix the SAME failure mode mostly don't stack -- the second is correcting an error the first already removed -- while two that fix DIFFERENT failure modes do, because a match is lost to whichever deviation the robot has no answer for. That is the part of this result that should generalize past this specific catalog: buy across categories, not the two best sensors.

## The Pareto frontier: what's worth buying at each price

Every robot NOT on this list is one you should never buy -- something else is both cheaper and better.

| Robot | Cost | Weighted success | Worst scenario | Parts |
|---|---:|---:|---:|---:|
| encoders only | $0.00 | 8% | 0% | 0 |
| front camera | $25.00 | 15% | 8% | 1 |
| odometry pods + front camera | $219.85 | 25% | 8% | 2 |

### Best robot at each budget

| Budget | Best robot | Cost | Money left over | Weighted success |
|---:|---|---:|---:|---:|
| $50 | front camera | $25.00 | $25.00 | 15% |
| $150 | front camera | $25.00 | $125.00 | 15% |
| $300 | odometry pods + front camera | $219.85 | $80.15 | 25% |
| $500 | odometry pods + front camera | $219.85 | $280.15 | 25% |

Note the unspent columns: at $50, $150 and $500, the best available robot is still front camera at $25.00. Nothing purchasable in between improves on it -- the next rung of the frontier is out of reach, and the intermediate options are worse buys than something cheaper. A team at those budgets should bank the difference (or spend it on drivetrain/gearing, which ftc/drivetrain_benchmark.py and ftc/gearing_benchmark.py price separately) rather than stretch to a mid-priced sensor.

## Different scenarios, different answers

The single strongest argument against a one-number ranking: the winning robot changes with the kind of match you expect.

| Scenario | Best robot | Its success rate | Cost |
|---|---|---:|---:|
| Heavy pose drift (map also off) | odometry pods + front camera | 20% | $219.85 |
| Field doesn't match the map (also drifting) | odometry pods + front camera | 20% | $219.85 |
| Opponent parks in the route (also drifting) | odometry pods + front camera | 16% | $219.85 |
| Everything at once (match-realistic mix) | encoders only | 8% | $0.00 |
| Tight corridor, mixed deviation | odometry pods + front camera | 60% | $219.85 |

2 different robots win at least one scenario out of 5. A team that knows its own dominant failure mode -- which is exactly what ftc/calibration.py's measured field/odometry CSVs are for -- can buy a cheaper robot than the overall ranking suggests and do better in the matches it actually plays.

## Exhaustive vs. greedy search

Greedy forward selection (start from the best single sensor, keep adding whichever one improves the objective most) is O(N^2) evaluations instead of 2^N, and its steps double as the marginal value of each sensor added:

| Step | Added | Robot after adding | Cost | Weighted success | Marginal gain | 95% CI (paired) | p | Significant |
|---:|---|---|---:|---:|---:|---|---:|---|
| 1 | (best single, the starting point) | front camera | $25.00 | 15% | -- | -- | -- | -- |
| 2 | Odometry pods | odometry pods + front camera | $219.85 | 25% | +9.6% | [+4.0%, +16.0%] | 0.001 | yes |

Greedy lands on the same robot as exhaustive search (odometry pods + front camera), so for this catalog the cheap search is sufficient -- worth knowing for anyone extending the parts list past the point where 2^N enumeration is affordable.


## Honest findings

- The best single suite (AprilTag (front camera) = front camera, $25.00) reaches 15% against the best bundle's 25% at $219.85. Bundling buys +10% for $+194.85 -- read the per-dollar column before reading that as an endorsement.
- Success rates here are lower across the board than `ftc_suite_writeup.md`'s, and that is expected, not a discrepancy: these profiles combine deviation axes, where the headline sweep isolates a single axis at the optimistic tier. The two studies' per-axis numbers agree where they overlap.
- Every dollar figure here is hardware only. A bundle's `parts` count is the closest this project gets to pricing integration effort, and it is not a dollar figure: a 5-part robot is five wiring harnesses, five failure modes, and five things to debug at 1am before a competition. ImuSuite costs $0 and is not free.
- The composition model is exact where it can be checked and approximate where it can't. A bundle reproduces the suites it's built from tick-for-tick (`ftc/scratch/bundle_test.py` fails otherwise, including the three-component bundle that must equal FullSuite), but combining two pose-fixing suites uses the union of their camera mounts on one detection pipeline rather than modeling two independent pipelines that could disagree with each other. Sensor FUSION conflict -- two sensors reporting different poses -- is not modeled at all here.
