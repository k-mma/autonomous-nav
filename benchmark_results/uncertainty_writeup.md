# Open-loop vs. reactive vs. belief-based planning under map/reality deviation

20 trials per (policy, variance_level) point, 11 variance_level steps from 0.0 to 1.0, 25x25 grids at 12% obstacle density. Every policy at a given (variance_level, trial index) runs against the identical assumed grid and identical ground-truth grid (nav/field_variance.py), so a gap between policies reflects the policy, not which random grid it happened to get. Raw data in `uncertainty_results.csv`, chart in `uncertainty_comparison.png`.

## Crossover

variance_level = 0.2 is the first level where a closed-loop policy's 95% bootstrap CI on success rate no longer overlaps OpenLoopPolicy's, and stays non-overlapping for every level above it -- a statistical claim, not a fixed-margin one: at 20 trials/point OpenLoopPolicy's own CI here is about 30% wide, so the gap has to clear real sampling noise, not just a percentage-point threshold someone picked. At that point: open-loop 15%, reactive 100%, belief 100%.

## Success rate by variance_level

| variance_level | Open-loop | Reactive | Belief |
|---:|---:|---:|---:|
| 0.0 | 100% | 100% | 90% |
| 0.1 | 85% | 100% | 90% |
| 0.2 | 15% | 100% | 100% |
| 0.3 | 5% | 100% | 100% |
| 0.4 | 5% | 100% | 90% |
| 0.5 | 0% | 100% | 100% |
| 0.6 | 0% | 100% | 90% |
| 0.7 | 0% | 100% | 85% |
| 0.8 | 5% | 100% | 95% |
| 0.9 | 5% | 100% | 95% |
| 1.0 | 0% | 100% | 95% |

## Reactive vs. belief: is the extra complexity worth it?

Averaged across every variance_level, ReactivePolicy plans 1.28 times per trial past its bootstrap plan (0.472ms total planning time/trial); BeliefPolicy plans 15.20 times (9.728ms/trial) -- it replans every single step by construction (its expected-cost map changes with every sensor sweep, not just when a cell crosses the hard-obstacle threshold), so this gap is structural, not incidental. At variance_level >= 0.5, mean success rate is 100% for reactive vs. 93% for belief.

ReactivePolicy actually matches or beats BeliefPolicy's success rate (100% vs. 93%) at high deviation while planning far less -- expected-cost planning's extra complexity isn't paying for itself in this range.

Mean collision rate across the whole sweep: reactive 0.00/trial vs. belief 0.06/trial.

Belief's collision rate is nonzero even at variance_level=0.0 (10% of trials), where ground truth is cell-for-cell identical to the assumed map -- so map deviation isn't causing these particular collisions at all. BeliefGrid only treats a cell as a hard obstacle after enough repeated sightings to cross OCCUPANCY_OBSTACLE_THRESHOLD (roughly 3 consistent hits, given OCCUPANCY_LOGODDS_OCCUPIED); a single sighting just makes a cell *expensive* to enter, not impassable. If the cheapest route still runs through a real, already-glimpsed obstacle before belief has caught up to certainty, BeliefPolicy will sometimes take that gamble and collide -- a failure mode ReactivePolicy structurally cannot have, since it treats the very first sighting of any cell as fully trustworthy and permanently blocking. That's the real cost of belief-based planning's probabilistic calibration: it can rationally walk through a cell it isn't sure about yet, which is exactly what lets it degrade gracefully under real map deviation, but also what occasionally gets it hurt in a world that didn't deviate at all.

## Sensitivity: does the crossover move?

Same statistical crossover definition as above (find_crossover), rerun at 8 trials/point instead of 20 -- fewer trials per point, so treat these crossovers as noisier than the headline one, useful for direction/magnitude rather than a precise value.

| Sweep | Value | Crossover |
|---|---:|---:|
| sensor_radius | 2 | 0.2 |
| sensor_radius | 10 | 0.2 |
| density | 0.06 | 0.2 |
| density | 0.18 | 0.2 |

If a row's crossover comes in noticeably earlier (a smaller variance_level) than the headline 0.2, that parameter makes closing the loop start paying off sooner; later means the opposite -- sensing further or planning against a sparser field buys more headroom before deviation forces the issue.
