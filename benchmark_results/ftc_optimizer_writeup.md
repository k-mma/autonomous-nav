# Which BUNDLE of sensors should an FTC team buy?

Every buildable combination of 7 sensor suites (ftc/sensors.py), up to 3 suites per bundle, composed by ftc/bundle.py and evaluated by ftc/optimizer.py over 5 scenario profiles x 25 seeded trials each. 63 raw combinations collapse to 24 distinct robots once bundles that buy identical hardware are recognized as the same purchase (3000 matches). Raw data in `ftc_optimizer_results.csv`, charts in `ftc_optimizer_frontier.png` and `ftc_optimizer_synergy.png`.

Two things make this different from `ftc_suite_writeup.md`'s seven-suite comparison:

- Bundles are costed over the UNION of their parts, so shared hardware is counted once. AprilTag + AprilTag-with-IMU is a $25 robot with one camera, not a $50 robot with two, and the two descriptions collapse to the same candidate before anything is simulated.
- Every candidate runs the identical seeded scenarios, so comparisons use a PAIRED bootstrap (nav/stats.py's `bootstrap_paired_diff_ci`) rather than checking whether two independent CIs overlap. Every "significant" below means a paired 95% CI that excludes zero AND a bootstrap p < 0.05, not a taller bar.

## The best robot and the most robust robot are the SAME robot

| Rank | Robot | Cost | Weighted success | Worst scenario | pp/$100 |
|---:|---|---:|---:|---:|---:|
| 1 | odometry pods + IMU + front camera + rear camera | $329.99 | 61% | 32% | +12.8 |
| 2 | odometry pods + front camera | $304.99 | 60% | 28% | +13.6 |
| 3 | odometry pods + IMU + front camera | $304.99 | 60% | 28% | +13.6 |
| 4 | odometry pods + front camera + rear camera | $329.99 | 60% | 28% | +12.6 |
| 5 | odometry pods + front camera + rear camera + 3 ToF sensors | $424.49 | 47% | 20% | +6.8 |
| 6 | odometry pods + IMU + front camera + rear camera + 3 ToF sensors | $424.49 | 47% | 20% | +6.8 |
| 7 | odometry pods + front camera + 3 ToF sensors | $399.49 | 46% | 20% | +6.8 |
| 8 | odometry pods + IMU + front camera + 3 ToF sensors | $399.49 | 46% | 20% | +6.8 |
| 9 | front camera + rear camera | $50.00 | 43% | 20% | +49.6 |
| 10 | IMU + front camera + rear camera | $50.00 | 43% | 20% | +49.6 |

Best average: odometry pods + IMU + front camera + rear camera ($329.99, 61%). Most robust -- highest success rate on its OWN WORST scenario, the right objective if you can't predict your division: odometry pods + IMU + front camera + rear camera ($329.99, 32% worst-case vs. 32% for the best-average robot). The two objectives pick the literal SAME robot here, not just a tie on worst-case rate -- there is no best-average-vs-most-robust tradeoff to report in this catalog, and a team should read that as "the choice was easy," not as a coincidence worth distrusting.

## Does bundling actually beat buying one sensor?

For each of the top bundles, a paired comparison against the best SINGLE suite that bundle itself contains -- the honest form of the question, since a bundle containing one strong sensor should not be credited with that sensor's performance:

| Bundle | Best single component | Bundle | Single | Gain | 95% CI (paired) | p | Extra cost | Verdict |
|---|---|---:|---:|---:|---|---:|---:|---|
| Odometry pods + IMU + Rear camera | Rear camera | 61% | 43% | +17.6% | [+10.4%, +24.8%] | 0.000 | $+279.99 | **significant** |
| Odometry pods + AprilTag (front camera) | AprilTag (front camera) | 60% | 42% | +17.6% | [+10.4%, +24.8%] | 0.000 | $+279.99 | **significant** |
| Odometry pods + AprilTag + IMU | AprilTag + IMU | 60% | 42% | +17.6% | [+10.4%, +25.6%] | 0.000 | $+279.99 | **significant** |
| Odometry pods + Rear camera | Rear camera | 60% | 43% | +16.8% | [+9.6%, +24.0%] | 0.000 | $+279.99 | **significant** |
| Odometry pods + Distance sensors + Rear camera | Rear camera | 47% | 43% | +4.0% | [-7.2%, +15.2%] | 0.259 | $+374.49 | not significant |
| Odometry pods + IMU + Rear camera + Distance sensors | Rear camera | 47% | 43% | +4.0% | [-7.2%, +15.2%] | 0.278 | $+374.49 | not significant |
| Odometry pods + Distance sensors + AprilTag (front camera) | AprilTag (front camera) | 46% | 42% | +3.2% | [-8.0%, +14.4%] | 0.322 | $+374.49 | not significant |
| Odometry pods + Distance sensors + AprilTag + IMU | AprilTag + IMU | 46% | 42% | +3.2% | [-8.8%, +13.6%] | 0.304 | $+374.49 | not significant |

4 of the 8 bundles tested beat their own best single component by a statistically significant margin. The cheapest of those upgrades is Odometry pods + IMU + Rear camera: +$279.99 over Rear camera for +17.6% success rate [95% CI +10.4%, +24.8%].

By marginal value -- gain per dollar of the UPGRADE, not of the whole robot -- the best combination to buy is Odometry pods + IMU + Rear camera at +6.3pp per extra $100 over Rear camera alone.

The mechanism is the one this project's own deviation-type analysis predicts, and it holds for every significant bundle above without exception: each spans more than one capability category (pose fixing, obstacle sensing, drift reduction, heading holding) AND adds a category its best single component did not have. Two sensors that fix the SAME failure mode mostly don't stack -- the second is correcting an error the first already removed -- while two that fix DIFFERENT failure modes do, because a match is lost to whichever deviation the robot has no answer for. That is the part of this result that should generalize past this specific catalog: buy across categories, not the two best sensors.

## The Pareto frontier: what's worth buying at each price

Every robot NOT on this list is one you should never buy -- something else is both cheaper and better.

| Robot | Cost | Weighted success | Worst scenario | Parts |
|---|---:|---:|---:|---:|
| encoders only | $0.00 | 18% | 8% | 0 |
| front camera | $25.00 | 42% | 16% | 1 |
| front camera + rear camera | $50.00 | 43% | 20% | 2 |
| odometry pods + front camera | $304.99 | 60% | 28% | 2 |
| odometry pods + IMU + front camera + rear camera | $329.99 | 61% | 32% | 4 |

### Best robot at each budget

| Budget | Best robot | Cost | Money left over | Weighted success |
|---:|---|---:|---:|---:|
| $50 | front camera + rear camera | $50.00 | $0.00 | 43% |
| $150 | front camera + rear camera | $50.00 | $100.00 | 43% |
| $300 | front camera + rear camera | $50.00 | $250.00 | 43% |
| $500 | odometry pods + IMU + front camera + rear camera | $329.99 | $170.01 | 61% |

Note the unspent columns: at $150 and $300, the best available robot is still front camera + rear camera at $50.00. Nothing purchasable in between improves on it -- the next rung of the frontier is out of reach, and the intermediate options are worse buys than something cheaper. A team at those budgets should bank the difference (or spend it on drivetrain/gearing, which ftc/drivetrain_benchmark.py and ftc/gearing_benchmark.py price separately) rather than stretch to a mid-priced sensor.

## Different scenarios, different answers

The single strongest argument against a one-number ranking: the winning robot changes with the kind of match you expect.

| Scenario | Best robot | Its success rate | Cost |
|---|---|---:|---:|
| Heavy pose drift | odometry pods + front camera | 56% | $304.99 |
| Field doesn't match the map | odometry pods | 84% | $279.99 |
| Opponent parks in the route | odometry pods + front camera + 3 ToF sensors | 68% | $399.49 |
| Everything at once (realistic fidelity) | odometry pods + IMU + front camera + rear camera | 32% | $329.99 |
| Tight corridor, mixed deviation | odometry pods + front camera | 80% | $304.99 |

4 different robots win at least one scenario out of 5. A team that knows its own dominant failure mode -- which is exactly what ftc/calibration.py's measured field/odometry CSVs are for -- can buy a cheaper robot than the overall ranking suggests and do better in the matches it actually plays.

## Exhaustive vs. greedy search

Greedy forward selection (start from the best single sensor, keep adding whichever one improves the objective most) is O(N^2) evaluations instead of 2^N, and its steps double as the marginal value of each sensor added:

| Step | Added | Robot after adding | Cost | Weighted success | Marginal gain | 95% CI (paired) | p | Significant |
|---:|---|---|---:|---:|---:|---|---:|---|
| 1 | (best single, the starting point) | front camera + rear camera | $50.00 | 43% | -- | -- | -- | -- |
| 2 | Odometry pods | odometry pods + front camera + rear camera | $329.99 | 60% | +16.8% | [+9.6%, +24.0%] | 0.000 | yes |
| 3 | IMU | odometry pods + IMU + front camera + rear camera | $329.99 | 61% | +0.8% | [+0.0%, +2.4%] | 0.368 | no |

Greedy lands on the same robot as exhaustive search (odometry pods + IMU + front camera + rear camera), so for this catalog the cheap search is sufficient -- worth knowing for anyone extending the parts list past the point where 2^N enumeration is affordable.

The greedy path stops paying at step 3: adding IMU costs $0.00 for +0.8% (95% CI [+0.0%, +2.4%], p=0.368) -- a gain this sweep cannot distinguish from noise. `--require-significant` makes ftc/optimizer.py's greedy search stop there rather than keep spending, which is the more honest stopping rule than "stop when the mean stops going up."

## Honest findings

- The best single suite (Rear camera = front camera + rear camera, $50.00) reaches 43% against the best bundle's 61% at $329.99. Bundling buys +18% for $+279.99 -- read the per-dollar column before reading that as an endorsement.
- Success rates here are lower across the board than `ftc_suite_writeup.md`'s, and that is expected, not a discrepancy: these profiles combine deviation axes and include a realistic-fidelity one, where the headline sweep isolates a single axis at the optimistic tier. The two studies' per-axis numbers agree where they overlap.
- Every dollar figure here is hardware only. A bundle's `parts` count is the closest this project gets to pricing integration effort, and it is not a dollar figure: a 5-part robot is five wiring harnesses, five failure modes, and five things to debug at 1am before a competition. ImuSuite costs $0 and is not free.
- The composition model is exact where it can be checked and approximate where it can't. A bundle reproduces the suites it's built from tick-for-tick (`ftc/scratch/bundle_test.py` fails otherwise, including the three-component bundle that must equal FullSuite), but combining two pose-fixing suites uses the union of their camera mounts on one detection pipeline rather than modeling two independent pipelines that could disagree with each other. Sensor FUSION conflict -- two sensors reporting different poses -- is not modeled at all here.
