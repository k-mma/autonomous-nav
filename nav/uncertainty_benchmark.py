"""
When does closing the loop (sensing + replanning) stop being optional
and start being necessary -- versus when is a cheap open-loop plan good
enough? A physical competition field can't answer this: one noisy,
unrepeatable trial per match, no control over how much the real world
deviates from the assumed map, no ground truth to compare against.

This sweeps nav/field_variance.py's variance_level knob (how much a
"ground truth" grid diverges from the map a policy plans against) across
OpenLoopPolicy, ReactivePolicy, and BeliefPolicy (nav/policies.py),
running each policy against the *same* assumed grid and ground truth for
a given (variance_level, trial index) pair -- same reasoning as
nav/replan_benchmark.py sharing one obstacle_seed across its astar/D*
Lite comparison, so a difference in outcome is attributable to the
policy, not to which random grid it happened to get.

Writes benchmark_results/uncertainty_results.csv (every trial, raw),
benchmark_results/uncertainty_comparison.png (success rate / path cost /
collision rate vs. variance_level, one line per policy), and
benchmark_results/uncertainty_writeup.md (the crossover finding).
"""
import csv
import math
import random
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from nav.algorithms import astar
from nav.config import GRID_SIZE, LIDAR_RADIUS
from nav.field_variance import generate_ground_truth
from nav.grid import Grid
from nav.policies import BeliefPolicy, OpenLoopPolicy, ReactivePolicy
from nav.stats import bootstrap_ci

DENSITY = 0.12
MAX_ATTEMPTS_PER_TRIAL = 50
VARIANCE_LEVELS = [round(i / 10, 1) for i in range(11)]  # 0.0, 0.1, ..., 1.0
TRIALS_PER_COMBO = 20
OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

POLICIES = {
    "open_loop": OpenLoopPolicy,
    "reactive": ReactivePolicy,
    "belief": BeliefPolicy,
}
# Plot/table order and color -- fixed rather than dict iteration order so
# every chart and printout lists them the same way.
POLICY_ORDER = ["open_loop", "reactive", "belief"]
POLICY_LABELS = {"open_loop": "Open-loop", "reactive": "Reactive", "belief": "Belief"}
POLICY_COLORS = {"open_loop": "tab:red", "reactive": "tab:blue", "belief": "tab:green"}


@dataclass
class TrialOutcome:
    success: bool
    cost: float
    collisions: int
    steps: int


def _random_grid(rng, size=GRID_SIZE, density=DENSITY):
    grid = Grid(size=size)
    for row in range(size):
        for col in range(size):
            if rng.random() < density:
                grid.cells[row][col] = Grid.OBSTACLE
    free = [(r, c) for r in range(size) for c in range(size) if grid.cells[r][c] == Grid.FREE]
    if len(free) < 2:
        return None, None, None
    start, goal = rng.sample(free, 2)
    return grid, start, goal


def _solvable_scenario(trial_seed, density=DENSITY):
    """One (assumed grid, start, goal, assumed_path) tuple, reused across
    every policy and every variance_level for this trial index -- only
    `generate_ground_truth`'s output differs by variance_level, so a
    policy's outcome differs only because of the policy and the
    deviation, never because it got an easier or harder random grid.
    `density` is only ever overridden by run_sensitivity's density
    sweep; every other caller gets the module's fixed DENSITY."""
    rng = random.Random(trial_seed)
    for _ in range(MAX_ATTEMPTS_PER_TRIAL):
        grid, start, goal = _random_grid(rng, density=density)
        if grid is None:
            continue
        path, _, _ = astar(grid, start, goal)
        if path is None:
            continue
        return grid, start, goal, path
    raise RuntimeError(f"no solvable grid for seed {trial_seed} after {MAX_ATTEMPTS_PER_TRIAL} attempts")


def execute_trial(policy, ground_truth, actual_start, goal, max_steps):
    """Drive `policy` from `actual_start` to `goal` on `ground_truth`,
    one `.step()` call per tick, until it succeeds, gets stuck (`step`
    returns None), collides (steps onto a cell that's an obstacle in
    ground truth, or off the edge of the grid entirely -- the latter can
    only happen to OpenLoopPolicy, whose start-drift-shifted path isn't
    guaranteed to stay in bounds), or exhausts `max_steps`. Cost is the
    real Euclidean length of each executed step (nav.algorithms.
    weighted_path_length's approach, generalized the same way it
    generalizes RRT's not-always-unit-length steps -- OpenLoopPolicy's
    very first step after a start-drift offset isn't necessarily a
    cardinal/diagonal neighbor move either)."""
    current = actual_start
    cost = 0.0
    collisions = 0
    steps = 0
    for _ in range(max_steps):
        if current == goal:
            break
        next_cell = policy.step(ground_truth, current)
        if next_cell is None:
            break
        if not ground_truth.is_valid(*next_cell) or ground_truth.is_obstacle(*next_cell):
            collisions += 1
            break
        cost += math.hypot(next_cell[0] - current[0], next_cell[1] - current[1])
        current = next_cell
        steps += 1
    return TrialOutcome(success=(current == goal), cost=cost, collisions=collisions, steps=steps)


def run_variance_level(variance_level, num_trials, base_seed, density=DENSITY, sensor_radius=LIDAR_RADIUS):
    """`density` and `sensor_radius` default to this module's fixed
    constants for the headline sweep; run_sensitivity overrides them to
    ask whether the crossover finding moves when either changes."""
    rows = []
    for t in range(num_trials):
        trial_seed = base_seed + t
        grid, start, goal, path = _solvable_scenario(trial_seed, density=density)
        ground_truth, actual_start = generate_ground_truth(grid, start, goal, variance_level, seed=trial_seed)
        max_steps = min(4 * len(path), len(path) + 60)

        for policy_name in POLICY_ORDER:
            if policy_name == "open_loop":
                policy = POLICIES[policy_name](grid, start, goal, rng=random.Random(trial_seed))
            else:
                policy = POLICIES[policy_name](grid, start, goal, sensor_radius=sensor_radius,
                                                rng=random.Random(trial_seed))
            outcome = execute_trial(policy, ground_truth, actual_start, goal, max_steps)
            rows.append({
                "policy": policy_name,
                "variance_level": variance_level,
                "trial": t,
                "seed": trial_seed,
                "success": outcome.success,
                "path_cost": round(outcome.cost, 3),
                "steps": outcome.steps,
                "planning_time_ms": round(policy.planning_time_s * 1000, 4),
                "replans": policy.replans,
                "collisions": outcome.collisions,
            })
    return rows


def base_seed(level):
    """The headline sweep's own seed formula (ftc/suite_benchmark.py's
    base_seed() is the same idea, one sweep axis instead of two) --
    pulled out unchanged from the __main__ loop below so run_sweep can
    reconstruct it per level without threading a seed dict across a
    process boundary. Not used by run_sensitivity, which has its own
    per-sweep-index formula and stays out of scope for this change."""
    return 1_000_000 + round(level * 100)


def run_sweep(levels, num_trials, density=DENSITY, sensor_radius=LIDAR_RADIUS, max_workers=None):
    """Runs run_variance_level for every level in `levels` and returns
    every row concatenated in `levels`' order -- a drop-in replacement
    for the `for level in VARIANCE_LEVELS: ...` loop __main__ used to
    run directly. Each level is an independent unit of work (same
    reasoning as ftc/suite_benchmark.py's run_sweep: run_variance_level's
    own trial loop derives every trial's seed from base_seed + t, so
    nothing about which level runs on which worker, or in what order
    workers finish, can change a single seed).

    max_workers=1 skips ProcessPoolExecutor and runs every level
    serially in this process -- see
    nav/scratch/parallel_determinism_test.py, which diffs CSVs to prove
    this and the parallel path produce identical rows."""
    if max_workers == 1:
        results = {level: run_variance_level(level, num_trials, base_seed(level), density, sensor_radius)
                   for level in levels}
    else:
        with ProcessPoolExecutor(max_workers=max_workers) as executor:
            futures = {
                level: executor.submit(run_variance_level, level, num_trials, base_seed(level), density,
                                        sensor_radius)
                for level in levels
            }
            results = {level: future.result() for level, future in futures.items()}

    rows = []
    for level in levels:
        rows.extend(results[level])
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _bootstrap_seed(policy, level):
    """A plain int derived from (policy, level) rather than hashing the
    tuple directly -- Python's hash randomization makes str hashes
    (and therefore tuple hashes containing a str) differ run to run,
    which would make bootstrap_ci's CI width jitter between two runs
    over the *identical* trial data. POLICY_ORDER's index keeps this
    deterministic across runs and processes."""
    return 3_000_000 + POLICY_ORDER.index(policy) * 10_000 + round(level * 1000)


def aggregate(rows):
    """{(policy, variance_level): {success_rate, ci_lo, ci_hi, avg_cost,
    collision_rate, avg_planning_ms, avg_replans}}, avg_cost averaged
    over successful trials only (a partial, failed-trial cost isn't
    comparable to a full start->goal traversal). ci_lo/ci_hi is a 95%
    bootstrap CI on success_rate (nav/stats.py) -- at TRIALS_PER_COMBO
    == 20 points/policy/level, a bare percentage with no interval is the
    weakest part of this benchmark; see find_crossover for what actually
    reading these intervals changes about the crossover claim."""
    stats = {}
    for policy in POLICY_ORDER:
        for level in VARIANCE_LEVELS:
            matching = [r for r in rows if r["policy"] == policy and r["variance_level"] == level]
            successes = [r for r in matching if r["success"]]
            ci_lo, ci_hi = bootstrap_ci(len(successes), len(matching), seed=_bootstrap_seed(policy, level))
            stats[(policy, level)] = {
                "success_rate": len(successes) / len(matching) if matching else 0.0,
                "ci_lo": ci_lo,
                "ci_hi": ci_hi,
                "avg_cost": (sum(r["path_cost"] for r in successes) / len(successes)) if successes else 0.0,
                "collision_rate": sum(r["collisions"] for r in matching) / len(matching) if matching else 0.0,
                "avg_planning_ms": sum(r["planning_time_ms"] for r in matching) / len(matching) if matching else 0.0,
                "avg_replans": sum(r["replans"] for r in matching) / len(matching) if matching else 0.0,
            }
    return stats


def find_crossover(stats):
    """First variance_level where a closed-loop policy's (reactive or
    belief) 95% bootstrap CI on success rate no longer overlaps open_
    loop's -- i.e. the closed-loop policy's ci_lo is strictly above
    open_loop's ci_hi -- and stays non-overlapping for every remaining
    level, so a lone lucky open_loop run at one level surrounded by
    worse ones on both sides doesn't get reported as if the gap closed
    back up. This replaces the old fixed-margin version (a flat 15-point
    gap regardless of how much sampling noise 20 trials/point actually
    produces) with an honest statistical claim: "the intervals stop
    overlapping here," not "someone picked a margin that looked right."

    Returns None if the CIs never cleanly separate anywhere in the
    swept range -- that's a real, reportable answer (see write_writeup),
    not a failure of this function.
    """
    for i, level in enumerate(VARIANCE_LEVELS):
        gap_holds = True
        for later_level in VARIANCE_LEVELS[i:]:
            open_hi = stats[("open_loop", later_level)]["ci_hi"]
            best_closed_lo = max(stats[("reactive", later_level)]["ci_lo"],
                                  stats[("belief", later_level)]["ci_lo"])
            if best_closed_lo <= open_hi:
                gap_holds = False
                break
        if gap_holds:
            return level
    return None


def plot_results(stats, path, crossover):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 11), sharex=True)

    for policy in POLICY_ORDER:
        success = [stats[(policy, level)]["success_rate"] for level in VARIANCE_LEVELS]
        ci_lo = [stats[(policy, level)]["ci_lo"] for level in VARIANCE_LEVELS]
        ci_hi = [stats[(policy, level)]["ci_hi"] for level in VARIANCE_LEVELS]
        ax1.fill_between(VARIANCE_LEVELS, ci_lo, ci_hi, color=POLICY_COLORS[policy], alpha=0.15, linewidth=0)
        ax1.plot(VARIANCE_LEVELS, success, "o-", label=POLICY_LABELS[policy], color=POLICY_COLORS[policy])
    ax1.set_ylabel("Success rate")
    ax1.set_ylim(-0.05, 1.05)
    ax1.set_title(f"Planning policy vs. map/reality deviation ({TRIALS_PER_COMBO} trials/point, "
                   "shaded = 95% bootstrap CI)")
    if crossover is not None:
        ax1.axvline(crossover, color="black", linestyle="--", linewidth=1)
        ax1.annotate(f"crossover\nvariance_level={crossover}", xy=(crossover, 0.5),
                     xytext=(6, 0), textcoords="offset points", fontsize=9)
    ax1.legend()

    for policy in POLICY_ORDER:
        cost = [stats[(policy, level)]["avg_cost"] for level in VARIANCE_LEVELS]
        ax2.plot(VARIANCE_LEVELS, cost, "o-", label=POLICY_LABELS[policy], color=POLICY_COLORS[policy])
    ax2.set_ylabel("Avg. completion cost\n(successful trials only)")

    for policy in POLICY_ORDER:
        collisions = [stats[(policy, level)]["collision_rate"] for level in VARIANCE_LEVELS]
        ax3.plot(VARIANCE_LEVELS, collisions, "o-", label=POLICY_LABELS[policy], color=POLICY_COLORS[policy])
    ax3.set_xlabel("variance_level (map/reality deviation)")
    ax3.set_ylabel("Collision rate\n(collisions/trial)")

    fig.tight_layout()
    fig.savefig(path, dpi=150)


def write_writeup(stats, crossover, path, sensitivity=None):
    lines = [
        "# Open-loop vs. reactive vs. belief-based planning under map/reality deviation",
        "",
        f"{TRIALS_PER_COMBO} trials per (policy, variance_level) point, {len(VARIANCE_LEVELS)} "
        f"variance_level steps from 0.0 to 1.0, {GRID_SIZE}x{GRID_SIZE} grids at {DENSITY:.0%} "
        "obstacle density. Every policy at a given (variance_level, trial index) runs against the "
        "identical assumed grid and identical ground-truth grid (nav/field_variance.py), so a gap "
        "between policies reflects the policy, not which random grid it happened to get. Raw data "
        "in `uncertainty_results.csv`, chart in `uncertainty_comparison.png`.",
        "",
        "## Crossover",
        "",
    ]
    if crossover is None:
        lines.append(
            "No statistical crossover found in [0.0, 1.0]: at no level does a closed-loop policy's "
            "95% bootstrap CI on success rate separate cleanly from OpenLoopPolicy's and *stay* "
            "separated through variance_level=1.0. Saying so honestly here matters more than forcing "
            "a number -- see `uncertainty_comparison.png`'s shaded CI bands for where the curves "
            "actually sit relative to each other; they may still visually diverge without the "
            "intervals ever cleanly separating at this trial count."
        )
    else:
        open_at = stats[("open_loop", crossover)]["success_rate"]
        open_ci = stats[("open_loop", crossover)]["ci_hi"] - stats[("open_loop", crossover)]["ci_lo"]
        reactive_at = stats[("reactive", crossover)]["success_rate"]
        belief_at = stats[("belief", crossover)]["success_rate"]
        lines.append(
            f"variance_level = {crossover} is the first level where a closed-loop policy's 95% "
            "bootstrap CI on success rate no longer overlaps OpenLoopPolicy's, and stays "
            "non-overlapping for every level above it -- a statistical claim, not a fixed-margin one: "
            f"at {TRIALS_PER_COMBO} trials/point OpenLoopPolicy's own CI here is about "
            f"{open_ci:.0%} wide, so the gap has to clear real sampling noise, not just a percentage-"
            f"point threshold someone picked. At that point: open-loop {open_at:.0%}, reactive "
            f"{reactive_at:.0%}, belief {belief_at:.0%}."
        )
    lines += ["", "## Success rate by variance_level", "", "| variance_level | Open-loop | Reactive | Belief |",
              "|---:|---:|---:|---:|"]
    for level in VARIANCE_LEVELS:
        lines.append(
            f"| {level} | {stats[('open_loop', level)]['success_rate']:.0%} | "
            f"{stats[('reactive', level)]['success_rate']:.0%} | {stats[('belief', level)]['success_rate']:.0%} |"
        )

    avg_collision_reactive = sum(stats[("reactive", l)]["collision_rate"] for l in VARIANCE_LEVELS) / len(VARIANCE_LEVELS)
    avg_collision_belief = sum(stats[("belief", l)]["collision_rate"] for l in VARIANCE_LEVELS) / len(VARIANCE_LEVELS)
    avg_replans_reactive = sum(stats[("reactive", l)]["avg_replans"] for l in VARIANCE_LEVELS) / len(VARIANCE_LEVELS)
    avg_replans_belief = sum(stats[("belief", l)]["avg_replans"] for l in VARIANCE_LEVELS) / len(VARIANCE_LEVELS)
    avg_ms_reactive = sum(stats[("reactive", l)]["avg_planning_ms"] for l in VARIANCE_LEVELS) / len(VARIANCE_LEVELS)
    avg_ms_belief = sum(stats[("belief", l)]["avg_planning_ms"] for l in VARIANCE_LEVELS) / len(VARIANCE_LEVELS)
    high_levels = [l for l in VARIANCE_LEVELS if l >= 0.5]
    reactive_success_high = sum(stats[("reactive", l)]["success_rate"] for l in high_levels) / len(high_levels)
    belief_success_high = sum(stats[("belief", l)]["success_rate"] for l in high_levels) / len(high_levels)

    lines += [
        "",
        "## Reactive vs. belief: is the extra complexity worth it?",
        "",
        f"Averaged across every variance_level, ReactivePolicy plans {avg_replans_reactive:.2f} times "
        f"per trial past its bootstrap plan ({avg_ms_reactive:.3f}ms total planning time/trial); "
        f"BeliefPolicy plans {avg_replans_belief:.2f} times ({avg_ms_belief:.3f}ms/trial) -- it replans "
        "every single step by construction (its expected-cost map changes with every sensor sweep, not "
        "just when a cell crosses the hard-obstacle threshold), so this gap is structural, not incidental. "
        f"At variance_level >= 0.5, mean success rate is {reactive_success_high:.0%} for reactive vs. "
        f"{belief_success_high:.0%} for belief.",
        "",
        (
            "Belief-based planning is meaningfully more expensive per trial and does not clearly win on "
            "success rate at this deviation range -- see `uncertainty_comparison.png` for whether that gap "
            "is real or noise, and the success-rate table above for the level-by-level picture."
            if abs(reactive_success_high - belief_success_high) < 0.05 else
            (
                f"BeliefPolicy's {belief_success_high - reactive_success_high:+.0%} success-rate gap over "
                "reactive at high deviation is large enough to plausibly justify its extra planning cost."
                if belief_success_high > reactive_success_high else
                f"ReactivePolicy actually matches or beats BeliefPolicy's success rate "
                f"({reactive_success_high:.0%} vs. {belief_success_high:.0%}) at high deviation while "
                "planning far less -- expected-cost planning's extra complexity isn't paying for itself "
                "in this range."
            )
        ),
        "",
        f"Mean collision rate across the whole sweep: reactive {avg_collision_reactive:.2f}/trial vs. "
        f"belief {avg_collision_belief:.2f}/trial.",
    ]
    zero_level_belief_collisions = stats[("belief", 0.0)]["collision_rate"]
    if zero_level_belief_collisions > 0:
        lines.append("")
        lines.append(
            f"Belief's collision rate is nonzero even at variance_level=0.0 "
            f"({zero_level_belief_collisions:.0%} of trials), where ground truth is cell-for-cell "
            "identical to the assumed map -- so map deviation isn't causing these particular collisions "
            "at all. BeliefGrid only treats a cell as a hard obstacle after enough repeated sightings to "
            "cross OCCUPANCY_OBSTACLE_THRESHOLD (roughly 3 consistent hits, given "
            "OCCUPANCY_LOGODDS_OCCUPIED); a single sighting just makes a cell *expensive* to enter, not "
            "impassable. If the cheapest route still runs through a real, already-glimpsed obstacle "
            "before belief has caught up to certainty, BeliefPolicy will sometimes take that gamble and "
            "collide -- a failure mode ReactivePolicy structurally cannot have, since it treats the very "
            "first sighting of any cell as fully trustworthy and permanently blocking. That's the real "
            "cost of belief-based planning's probabilistic calibration: it can rationally walk through a "
            "cell it isn't sure about yet, which is exactly what lets it degrade gracefully under real "
            "map deviation, but also what occasionally gets it hurt in a world that didn't deviate at all."
        )
    if sensitivity:
        lines += ["", "## Sensitivity: does the crossover move?", "",
                   "Same statistical crossover definition as above (find_crossover), rerun at "
                   f"{SENSITIVITY_TRIALS} trials/point instead of {TRIALS_PER_COMBO} -- fewer trials "
                   "per point, so treat these crossovers as noisier than the headline one, useful for "
                   "direction/magnitude rather than a precise value.", "",
                   "| Sweep | Value | Crossover |", "|---|---:|---:|"]
        for label, value, cross in sensitivity:
            lines.append(f"| {label} | {value} | {cross if cross is not None else 'none found'} |")
        lines.append("")
        lines.append(
            "If a row's crossover comes in noticeably earlier (a smaller variance_level) than the "
            f"headline {crossover if crossover is not None else 'none found'}, that parameter makes "
            "closing the loop start paying off sooner; later means the opposite -- sensing further or "
            "planning against a sparser field buys more headroom before deviation forces the issue."
        )

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


SENSITIVITY_TRIALS = 8


def run_sensitivity():
    """Reruns the crossover analysis at a couple of alternate sensor
    radii and obstacle densities (fewer trials/point than the headline
    sweep, to keep this cheap) -- Phase 3's answer to "does the
    crossover move with sensor radius / element density?" A crossover
    that's stable across these variations is a much stronger claim than
    one that was only ever checked at one arbitrary radius/density.

    Returns a list of (label, value, crossover) tuples in a fixed,
    readable order.
    """
    sweeps = [
        ("sensor_radius", max(LIDAR_RADIUS // 2, 1), {"sensor_radius": max(LIDAR_RADIUS // 2, 1)}),
        ("sensor_radius", LIDAR_RADIUS * 2, {"sensor_radius": LIDAR_RADIUS * 2}),
        ("density", round(DENSITY / 2, 3), {"density": DENSITY / 2}),
        ("density", round(DENSITY * 1.5, 3), {"density": DENSITY * 1.5}),
    ]
    results = []
    for sweep_idx, (label, value, overrides) in enumerate(sweeps):
        rows = []
        for level in VARIANCE_LEVELS:
            base_seed = 2_000_000 + sweep_idx * 100_000 + round(level * 100)
            rows.extend(run_variance_level(level, SENSITIVITY_TRIALS, base_seed, **overrides))
        stats = aggregate(rows)
        results.append((label, value, find_crossover(stats)))
    return results


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    print(f"Running {len(VARIANCE_LEVELS)} variance_level points ({TRIALS_PER_COMBO} trials each) in parallel...")
    all_rows = run_sweep(VARIANCE_LEVELS, TRIALS_PER_COMBO)

    write_csv(all_rows, OUTPUT_DIR / "uncertainty_results.csv")
    stats = aggregate(all_rows)
    crossover = find_crossover(stats)

    print("\nrunning sensitivity sweep (sensor radius / density) ...")
    sensitivity = run_sensitivity()

    plot_results(stats, OUTPUT_DIR / "uncertainty_comparison.png", crossover)
    write_writeup(stats, crossover, OUTPUT_DIR / "uncertainty_writeup.md", sensitivity=sensitivity)

    print(f"\nWrote {len(all_rows)} trials to {OUTPUT_DIR / 'uncertainty_results.csv'}")
    print(f"Plot saved to {OUTPUT_DIR / 'uncertainty_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'uncertainty_writeup.md'}")
    print(f"\nCrossover: {crossover}")
    print(f"Sensitivity: {sensitivity}")
    for policy in POLICY_ORDER:
        print(f"\n{POLICY_LABELS[policy]}:")
        for level in VARIANCE_LEVELS:
            s = stats[(policy, level)]
            print(f"  variance_level={level:.1f}: success={s['success_rate']:.0%}  "
                  f"avg_cost={s['avg_cost']:.2f}  collision_rate={s['collision_rate']:.2f}  "
                  f"avg_replans={s['avg_replans']:.2f}  avg_planning_ms={s['avg_planning_ms']:.3f}")
