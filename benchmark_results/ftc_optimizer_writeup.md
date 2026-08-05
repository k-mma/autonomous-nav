# Which BUNDLE of sensors should an FTC team buy?

Every buildable combination of 8 sensor suites (ftc/sensors.py), up to 3 suites per bundle, composed by ftc/bundle.py and evaluated by ftc/optimizer.py over 5 scenario profiles x 25 seeded trials each. 92 raw combinations collapse to 43 distinct robots once bundles that buy identical hardware are recognized as the same purchase (5375 matches). Raw data in `ftc_optimizer_results.csv`, charts in `ftc_optimizer_frontier.png` and `ftc_optimizer_synergy.png`.

Two things make this different from `ftc_suite_writeup.md`'s five-suite comparison:

- Bundles are costed over the UNION of their parts, so shared hardware is counted once. AprilTag + AprilTag-with-IMU is a $25 robot with one camera, not a $50 robot with two, and the two descriptions collapse to the same candidate before anything is simulated.
- Every candidate runs the identical seeded scenarios, so comparisons use a PAIRED bootstrap (nav/stats.py's `bootstrap_paired_diff_ci`) rather than checking whether two independent CIs overlap. Every "significant" below means a paired 95% CI that excludes zero AND a bootstrap p < 0.05, not a taller bar.

## The best robot, and the most robust robot, are not the same robot

| Rank | Robot | Cost | Weighted success | Worst scenario | pp/$100 |
|---:|---|---:|---:|---:|---:|
| 1 | odometry pods + IMU + front camera + rear camera + lidar | $429.99 | 66% | 32% | +11.2 |
| 2 | odometry pods + IMU + front camera + rear camera + 3 ToF sensors + lidar | $524.49 | 66% | 32% | +9.2 |
| 3 | odometry pods + front camera + rear camera + lidar | $429.99 | 66% | 32% | +11.0 |
| 4 | odometry pods + front camera + rear camera + 3 ToF sensors + lidar | $524.49 | 66% | 32% | +9.0 |
| 5 | odometry pods + front camera + lidar | $404.99 | 64% | 32% | +11.3 |
| 6 | odometry pods + IMU + front camera + lidar | $404.99 | 64% | 32% | +11.3 |
| 7 | odometry pods + IMU + front camera + rear camera | $329.99 | 61% | 32% | +12.8 |
| 8 | odometry pods + front camera | $304.99 | 60% | 28% | +13.6 |
| 9 | odometry pods + IMU + front camera | $304.99 | 60% | 28% | +13.6 |
| 10 | odometry pods + front camera + rear camera | $329.99 | 60% | 28% | +12.6 |

Best average: odometry pods + IMU + front camera + rear camera + lidar ($429.99, 66%). Most robust -- highest success rate on its OWN WORST scenario, the right objective if you can't predict your division: odometry pods + IMU + front camera + rear camera ($329.99, 32% worst-case vs. 32% for the best-average robot). Those worst-case rates are the same, so no robustness is being given up by taking the best-average robot here -- the two objectives happen to agree in this catalog. They differ in PRICE, though: $329.99 vs. $429.99 for +6% average success, which is the real choice on offer.

## Does bundling actually beat buying one sensor?

For each of the top bundles, a paired comparison against the best SINGLE suite that bundle itself contains -- the honest form of the question, since a bundle containing one strong sensor should not be credited with that sensor's performance:

| Bundle | Best single component | Bundle | Single | Gain | 95% CI (paired) | p | Extra cost | Verdict |
|---|---|---:|---:|---:|---|---:|---:|---|
| Odometry pods + Dual-camera AprilTag + Lidar + IMU | Dual-camera AprilTag | 66% | 43% | +23.2% | [+12.0%, +33.6%] | 0.000 | $+379.99 | **significant** |
| Odometry pods + Dual-camera AprilTag + Lidar + IMU + Distance sensors | Dual-camera AprilTag | 66% | 43% | +23.2% | [+12.8%, +34.4%] | 0.000 | $+474.49 | **significant** |
| Odometry pods + Dual-camera AprilTag + Lidar | Dual-camera AprilTag | 66% | 43% | +22.4% | [+11.2%, +32.8%] | 0.000 | $+379.99 | **significant** |
| Odometry pods + Dual-camera AprilTag + Lidar + Distance sensors | Dual-camera AprilTag | 66% | 43% | +22.4% | [+11.2%, +33.6%] | 0.000 | $+474.49 | **significant** |
| Odometry pods + AprilTag + Lidar | AprilTag | 64% | 42% | +21.6% | [+10.4%, +32.8%] | 0.000 | $+379.99 | **significant** |
| Odometry pods + AprilTag + IMU + Lidar | AprilTag + IMU | 64% | 42% | +21.6% | [+10.4%, +33.6%] | 0.000 | $+379.99 | **significant** |
| Odometry pods + IMU + Dual-camera AprilTag | Dual-camera AprilTag | 61% | 43% | +17.6% | [+10.4%, +24.8%] | 0.000 | $+279.99 | **significant** |
| Odometry pods + AprilTag | AprilTag | 60% | 42% | +17.6% | [+10.4%, +25.6%] | 0.000 | $+279.99 | **significant** |

8 of the 8 bundles tested beat their own best single component by a statistically significant margin. The cheapest of those upgrades is Odometry pods + IMU + Dual-camera AprilTag: +$279.99 over Dual-camera AprilTag for +17.6% success rate [95% CI +10.4%, +24.8%].

By marginal value -- gain per dollar of the UPGRADE, not of the whole robot -- the best combination to buy is Odometry pods + IMU + Dual-camera AprilTag at +6.3pp per extra $100 over Dual-camera AprilTag alone.

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
| odometry pods + front camera + lidar | $404.99 | 64% | 32% | 3 |
| odometry pods + IMU + front camera + rear camera + lidar | $429.99 | 66% | 32% | 5 |

### Best robot at each budget

| Budget | Best robot | Cost | Money left over | Weighted success |
|---:|---|---:|---:|---:|
| $50 | front camera + rear camera | $50.00 | $0.00 | 43% |
| $150 | front camera + rear camera | $50.00 | $100.00 | 43% |
| $300 | front camera + rear camera | $50.00 | $250.00 | 43% |
| $500 | odometry pods + IMU + front camera + rear camera + lidar | $429.99 | $70.01 | 66% |

Note the unspent columns: at $150 and $300, the best available robot is still front camera + rear camera at $50.00. Nothing purchasable in between improves on it -- the next rung of the frontier is out of reach, and the intermediate options are worse buys than something cheaper. A team at those budgets should bank the difference (or spend it on drivetrain/gearing, which ftc/drivetrain_benchmark.py and ftc/gearing_benchmark.py price separately) rather than stretch to a mid-priced sensor.

## Different scenarios, different answers

The single strongest argument against a one-number ranking: the winning robot changes with the kind of match you expect.

| Scenario | Best robot | Its success rate | Cost |
|---|---|---:|---:|
| Heavy pose drift | odometry pods + front camera | 56% | $304.99 |
| Field doesn't match the map | odometry pods + lidar | 100% | $379.99 |
| Opponent parks in the route | odometry pods + lidar | 100% | $379.99 |
| Everything at once (realistic fidelity) | odometry pods + IMU + front camera + rear camera + lidar | 48% | $429.99 |
| Tight corridor, mixed deviation | odometry pods + front camera | 80% | $304.99 |

3 different robots win at least one scenario out of 5. A team that knows its own dominant failure mode -- which is exactly what ftc/calibration.py's measured field/odometry CSVs are for -- can buy a cheaper robot than the overall ranking suggests and do better in the matches it actually plays.

## Exhaustive vs. greedy search

Greedy forward selection (start from the best single sensor, keep adding whichever one improves the objective most) is O(N^2) evaluations instead of 2^N, and its steps double as the marginal value of each sensor added:

| Step | Added | Robot after adding | Cost | Weighted success | Marginal gain | 95% CI (paired) | p | Significant |
|---:|---|---|---:|---:|---:|---|---:|---|
| 1 | (best single, the starting point) | front camera + rear camera | $50.00 | 43% | -- | -- | -- | -- |
| 2 | Odometry pods | odometry pods + front camera + rear camera | $329.99 | 60% | +16.8% | [+9.6%, +24.0%] | 0.000 | yes |
| 3 | Lidar | odometry pods + front camera + rear camera + lidar | $429.99 | 66% | +5.6% | [-4.0%, +16.0%] | 0.156 | no |
| 4 | IMU | odometry pods + IMU + front camera + rear camera + lidar | $429.99 | 66% | +0.8% | [+0.0%, +2.4%] | 0.367 | no |

Greedy lands on the same robot as exhaustive search (odometry pods + IMU + front camera + rear camera + lidar), so for this catalog the cheap search is sufficient -- worth knowing for anyone extending the parts list past the point where 2^N enumeration is affordable.

The greedy path stops paying at step 3: adding Lidar costs $100.00 for +5.6% (95% CI [-4.0%, +16.0%], p=0.156) -- a gain this sweep cannot distinguish from noise. `--require-significant` makes ftc/optimizer.py's greedy search stop there rather than keep spending, which is the more honest stopping rule than "stop when the mean stops going up."

## Honest findings

- The best single suite (Dual-camera AprilTag = front camera + rear camera, $50.00) reaches 43% against the best bundle's 66% at $429.99. Bundling buys +23% for $+379.99 -- read the per-dollar column before reading that as an endorsement.
- Success rates here are lower across the board than `ftc_suite_writeup.md`'s, and that is expected, not a discrepancy: these profiles combine deviation axes and include a realistic-fidelity one, where the headline sweep isolates a single axis at the optimistic tier. The two studies' per-axis numbers agree where they overlap.
- Every dollar figure here is hardware only. A bundle's `parts` count is the closest this project gets to pricing integration effort, and it is not a dollar figure: a 5-part robot is five wiring harnesses, five failure modes, and five things to debug at 1am before a competition. ImuSuite costs $0 and is not free.
- The composition model is exact where it can be checked and approximate where it can't. A bundle reproduces the suites it's built from tick-for-tick (`ftc/scratch/bundle_test.py` fails otherwise, including the three-component bundle that must equal FullSuite), but combining two pose-fixing suites uses the union of their camera mounts on one detection pipeline rather than modeling two independent pipelines that could disagree with each other. Sensor FUSION conflict -- two sensors reporting different poses -- is not modeled at all here.
