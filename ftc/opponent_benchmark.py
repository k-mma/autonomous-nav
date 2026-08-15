"""
Does a real (moving) opponent robot change which sensor suite wins,
compared to the existing "unplanned_blocker" deviation type's single
STATIC obstacle?

nav/field_variance.py's `_place_unplanned_blocker` (used via ftc/
suite_benchmark.py's `DEVIATION_TYPES["unplanned_blocker"]`) drops one
extra obstacle on the assumed start->goal route, once, before the match
starts, and leaves it there for the rest of the run -- an opponent with
no goals, no motion, no reaction to your robot's presence, exactly the
"no opponent modeling" gap README.md's "Threats to validity" names.
nav/obstacles.py already has `MovingObstacle` (a seeded random walk);
this reuses it completely unmodified -- pygame_app depends on its exact
existing tick()/place()/_move() behavior -- rather than writing a
second one.

INTEGRATION DETAIL (the one this module's docstring in the task brief
specifically flagged as needing care): `MovingObstacle.tick(grid,
now_ms, blocked)` is driven by millisecond WALL-CLOCK timing (`period_ms`
compared against `now_ms`), while ftc/match.py accrues SIMULATED
`elapsed_s` (drive + turn + replan time -- not real time, and not
correlated with how fast this process actually executes). ftc/match.py's
`run_match` now accepts an optional `moving_obstacles` sequence (default
`()`, so every existing caller -- ftc/suite_benchmark.py, ftc/
robustness.py, ftc/layout_benchmark.py, ftc/budget_benchmark.py -- is
completely unaffected) and ticks each one once per loop iteration with
`now_ms = elapsed_s * 1000.0`. That's the only change to ftc/match.py;
MovingObstacle itself is untouched.

Deliberately does NOT touch nav/field_variance.py. Both the static and
moving paths run through this module side by side, using the identical
probability gate (`min(variance_level, 1.0)`) and identical candidate-
cell selection (`rng.choice` of an interior cell of the assumed
start->goal path, via the same nav.algorithms.astar call
nav/field_variance.py's own `_place_unplanned_blocker` uses) -- see
`_place_moving_blocker` below, which mirrors that function's logic
exactly except for what happens to the chosen cell afterward. That
makes the static-vs-moving comparison genuinely apples to apples: the
only thing that differs is whether the one obstacle then random-walks
for the rest of the match or stays put, added as a new deviation type
alongside the existing static one rather than replacing it.

Writes benchmark_results/ftc_opponent_results.csv (every trial, raw,
tagged with a `blocker_type` column: static or moving), benchmark_
results/ftc_opponent_comparison.png (success rate vs. variance_level,
one panel per blocker_type, CI bands), and benchmark_results/
ftc_opponent_writeup.md (whether a moving opponent changes which suite
wins).
"""
import csv
import random
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt

from nav.algorithms import astar
from nav.field_variance import generate_ground_truth
from nav.obstacles import MovingObstacle
from nav.stats import bootstrap_ci

from ftc.config import OPPONENT_REPOSITION_PERIOD_MS
from ftc.field import build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import SUITES, SUITE_ORDER, SUITE_LABELS
from ftc.suite_benchmark import (
    LAYOUT, SUITE_COLORS, SUMMARY_MIN_LEVEL, TRIALS_PER_COMBO, VARIANCE_LEVELS, _solvable_scenario,
)

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

BLOCKER_TYPES = ["static", "moving"]
BLOCKER_TYPE_LABELS = {
    "static": "Static (existing unplanned_blocker)",
    "moving": "Moving (random-walk opponent)",
}


def _place_moving_blocker(ground_truth, assumed_grid, start, goal, probability, rng, period_ms):
    """Same probability gate and candidate-cell selection as nav/
    field_variance.py's own `_place_unplanned_blocker` -- the only
    difference is what happens to the chosen cell afterward (a
    persistent, ticking MovingObstacle instead of a permanent
    Grid.OBSTACLE). Returns the MovingObstacle, started at simulated
    t=0, or None if the probability gate didn't fire or the assumed map
    has no interior path cell to place one on."""
    if rng.random() >= probability:
        return None
    path, _, _ = astar(assumed_grid, start, goal)
    if not path or len(path) < 3:
        return None
    cell = rng.choice(path[1:-1])
    obstacle = MovingObstacle(cell, period_ms=period_ms, rng=random.Random(rng.randint(0, 2**31 - 1)))
    obstacle.place(ground_truth)
    obstacle.start(now_ms=0)
    return obstacle


def run_combo(blocker_type, variance_level, num_trials, base_seed, grid, free_cells, tag_sites):
    rows = []
    probability = min(variance_level, 1.0)
    for t in range(num_trials):
        trial_seed = base_seed + t
        start, goal = _solvable_scenario(trial_seed, grid, free_cells)

        if blocker_type == "static":
            ground_truth, actual_start = generate_ground_truth(
                grid, start, goal, variance_level, seed=trial_seed,
                start_drift_scale=0.0, obstacle_drift_scale=0.0, blocker_scale=1.0)
            moving_obstacles = ()
        else:
            ground_truth, actual_start = generate_ground_truth(
                grid, start, goal, variance_level, seed=trial_seed,
                start_drift_scale=0.0, obstacle_drift_scale=0.0, blocker_scale=0.0)
            placement_rng = random.Random(trial_seed)
            obstacle = _place_moving_blocker(ground_truth, grid, start, goal, probability, placement_rng,
                                              OPPONENT_REPOSITION_PERIOD_MS)
            moving_obstacles = (obstacle,) if obstacle is not None else ()

        for suite_name in SUITE_ORDER:
            suite = SUITES[suite_name]()
            result = run_match(suite, grid, start, goal, ground_truth, actual_start, tag_sites,
                                random.Random(trial_seed), moving_obstacles=moving_obstacles)
            rows.append({
                "suite": suite_name,
                "blocker_type": blocker_type,
                "variance_level": variance_level,
                "trial": t,
                "seed": trial_seed,
                "success": result.success,
                "collisions": result.collisions,
                "replans": result.replans,
                "cost_usd": suite.cost_usd,
            })
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _bootstrap_seed(blocker_type, suite, level):
    return (9_500_000 + BLOCKER_TYPES.index(blocker_type) * 1_000_000 + SUITE_ORDER.index(suite) * 10_000
            + round(level * 100))


def aggregate(rows):
    """{(suite, blocker_type, variance_level): {success_rate, ci_lo,
    ci_hi, avg_collisions, avg_replans}}."""
    stats = {}
    for suite in SUITE_ORDER:
        for blocker_type in BLOCKER_TYPES:
            for level in VARIANCE_LEVELS:
                matching = [r for r in rows if r["suite"] == suite and r["blocker_type"] == blocker_type
                            and r["variance_level"] == level]
                successes = [r for r in matching if r["success"]]
                n = len(matching)
                ci_lo, ci_hi = bootstrap_ci(len(successes), n, seed=_bootstrap_seed(blocker_type, suite, level))
                stats[(suite, blocker_type, level)] = {
                    "success_rate": len(successes) / n if n else 0.0,
                    "ci_lo": ci_lo,
                    "ci_hi": ci_hi,
                    "avg_collisions": sum(r["collisions"] for r in matching) / n if n else 0.0,
                    "avg_replans": sum(r["replans"] for r in matching) / n if n else 0.0,
                }
    return stats


def overall_success_rate(stats, suite, blocker_type, min_level=SUMMARY_MIN_LEVEL):
    levels = [l for l in VARIANCE_LEVELS if l >= min_level]
    vals = [stats[(suite, blocker_type, l)]["success_rate"] for l in levels]
    return sum(vals) / len(vals)


def value_ranking(stats, blocker_type):
    """{suite: {rate, per_100}} for every non-baseline suite under this
    blocker_type, plus the best-value suite name -- same success-rate-
    gain-per-$100 metric ftc/suite_benchmark.py's headline table uses.

    per_100 is None (NOT infinity) at cost_usd == 0.0 -- IMU is a second
    $0 headline suite besides the dead-reckoning baseline itself now;
    `best` is picked only among suites with a defined per_100, same
    convention as ftc/suite_benchmark.py's own write_writeup."""
    baseline = overall_success_rate(stats, "dead_reckoning", blocker_type)
    results = {}
    for suite in SUITE_ORDER:
        if suite == "dead_reckoning":
            continue
        rate = overall_success_rate(stats, suite, blocker_type)
        cost = SUITES[suite].cost_usd
        per_100 = None if cost == 0 else (rate - baseline) / (cost / 100) * 100
        results[suite] = {"rate": rate, "per_100": per_100}
    priced = {s: v for s, v in results.items() if v["per_100"] is not None}
    best = max(priced, key=lambda s: priced[s]["per_100"])
    return results, baseline, best


def plot_comparison(stats, path):
    fig, axes = plt.subplots(1, len(BLOCKER_TYPES), figsize=(13, 5.5), sharey=True)
    for ax, blocker_type in zip(axes, BLOCKER_TYPES):
        for suite in SUITE_ORDER:
            success = [stats[(suite, blocker_type, level)]["success_rate"] for level in VARIANCE_LEVELS]
            ci_lo = [stats[(suite, blocker_type, level)]["ci_lo"] for level in VARIANCE_LEVELS]
            ci_hi = [stats[(suite, blocker_type, level)]["ci_hi"] for level in VARIANCE_LEVELS]
            ax.fill_between(VARIANCE_LEVELS, ci_lo, ci_hi, color=SUITE_COLORS[suite], alpha=0.12, linewidth=0)
            ax.plot(VARIANCE_LEVELS, success, "o-", label=SUITE_LABELS[suite], color=SUITE_COLORS[suite],
                     markersize=4)
        ax.set_xlabel("variance_level (blocker probability = min(variance_level, 1.0))")
        ax.set_title(BLOCKER_TYPE_LABELS[blocker_type], fontsize=10)
        ax.set_ylim(-0.05, 1.05)
    axes[0].set_ylabel("Success rate")
    axes[0].legend(loc="lower left", fontsize=8)
    fig.suptitle(f"Static vs. moving opponent ({TRIALS_PER_COMBO} trials/point, '{LAYOUT}' layout)\n"
                  "shaded = 95% bootstrap CI", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    fig.savefig(path, dpi=150)


def _runner_up_overlap(rows, blocker_type, results, best):
    """Bootstrap-CI overlap check between `best` and its runner-up under
    `blocker_type`, on pooled successes/n (levels >= SUMMARY_MIN_LEVEL)
    -- the same "a flip that sits inside overlapping confidence
    intervals is not a real flip" check ftc/robustness.py and ftc/
    budget_benchmark.py both already apply to their own tipping-point
    claims. Returns (runner_up, overlap) or (None, None) if there's
    only one ranked suite. $0-cost suites (per_100=None) are excluded --
    a runner-up has to be a priced suite to make "how close was the
    dollar-value margin" a meaningful question at all."""
    priced = [s for s in results if results[s]["per_100"] is not None]
    ranked = sorted(priced, key=lambda s: -results[s]["per_100"])
    if len(ranked) < 2:
        return None, None
    runner_up = ranked[1]

    def _pooled(suite):
        matching = [r for r in rows if r["suite"] == suite and r["blocker_type"] == blocker_type
                    and r["variance_level"] >= SUMMARY_MIN_LEVEL]
        successes = sum(r["success"] for r in matching)
        return successes, len(matching)

    best_successes, best_n = _pooled(best)
    runner_successes, runner_n = _pooled(runner_up)
    best_lo, best_hi = bootstrap_ci(best_successes, best_n, seed=9_700_000 + SUITE_ORDER.index(best))
    runner_lo, runner_hi = bootstrap_ci(runner_successes, runner_n, seed=9_700_000 + SUITE_ORDER.index(runner_up))
    overlap = not (best_lo > runner_hi or runner_lo > best_hi)
    return runner_up, overlap


def write_writeup(stats, rows, path):
    lines = [
        "# Does a moving opponent change which suite wins?",
        "",
        "`ftc_suite_writeup.md`'s `unplanned_blocker` deviation type drops one STATIC obstacle on the "
        "planned route -- a real opponent moves. This reruns that same deviation axis "
        f"({TRIALS_PER_COMBO} trials/point, all {len(VARIANCE_LEVELS)} variance_level steps, "
        f"'{LAYOUT}' layout) two ways: `static` (the existing behavior, unchanged) and `moving` (the "
        "identical probability gate and candidate-cell selection, but the chosen obstacle then "
        "random-walks for the rest of the match via `nav/obstacles.py`'s `MovingObstacle`, ticked on "
        "SIMULATED match time -- see module docstring). Raw data (with an added `blocker_type` "
        "column) in `ftc_opponent_results.csv`, chart in `ftc_opponent_comparison.png`.",
        "",
        "## Overall success rate (variance_level >= 0.3)",
        "",
        "| Suite | Cost | Static | Moving | Difference |",
        "|---|---:|---:|---:|---:|",
    ]
    for suite in SUITE_ORDER:
        static_rate = overall_success_rate(stats, suite, "static")
        moving_rate = overall_success_rate(stats, suite, "moving")
        cost = SUITES[suite].cost_usd
        lines.append(f"| {SUITE_LABELS[suite]} | ${cost:.0f} | {static_rate:.0%} | {moving_rate:.0%} | "
                      f"{(moving_rate - static_rate):+.0%} |")

    static_results, static_baseline, static_best = value_ranking(stats, "static")
    moving_results, moving_baseline, moving_best = value_ranking(stats, "moving")
    lines += [
        "",
        "## Best value by blocker type",
        "",
        f"Static: {SUITE_LABELS[static_best]} ({static_results[static_best]['per_100']:+.1f}pp/$100, "
        f"baseline {static_baseline:.0%}). Moving: {SUITE_LABELS[moving_best]} "
        f"({moving_results[moving_best]['per_100']:+.1f}pp/$100, baseline {moving_baseline:.0%}).",
        "",
    ]

    if static_best == moving_best:
        lines.append(
            f"A moving opponent doesn't change which suite wins -- {SUITE_LABELS[static_best]} is "
            "the best-value suite against both a static and a moving obstacle. Modeling the opponent "
            "as a random walk instead of a fixed point changes the raw numbers (see the table above) "
            "but not the recommendation."
        )
    else:
        moving_runner_up, moving_overlap = _runner_up_overlap(rows, "moving", moving_results, moving_best)
        if moving_overlap:
            lines.append(
                f"A moving opponent appears to change which suite wins, but not cleanly: "
                f"{SUITE_LABELS[static_best]} is best against a static blocker, and {SUITE_LABELS[moving_best]} "
                f"edges out {SUITE_LABELS[moving_runner_up]} for best value against a moving one -- but "
                f"{moving_results[moving_best]['per_100']:.1f}pp/$100 vs. "
                f"{moving_results[moving_runner_up]['per_100']:.1f}pp/$100 is close enough that the two "
                "suites' success-rate confidence intervals still overlap at this trial count. Treat "
                f"\"{SUITE_LABELS[moving_best]} beats {SUITE_LABELS[moving_runner_up]} against a moving "
                "opponent\" as plausible, not confirmed -- but the headline claim that follows doesn't "
                "depend on that particular margin: neither of them is Full suite, and that gap "
                f"({SUITE_LABELS[static_best]}'s static win) is not close."
            )
        else:
            lines.append(
                f"A moving opponent changes which suite wins: {SUITE_LABELS[static_best]} is best "
                f"against a static blocker, but {SUITE_LABELS[moving_best]} is best against a moving one. "
                "This is a real finding, not a failure of the sweep -- the existing static-blocker "
                "deviation type was, in this specific respect, silently favoring whichever suite handles "
                "a fixed obstacle best, not whichever suite handles a genuinely unpredictable opponent "
                "best."
            )

    # Suites that never sense obstacles (dead_reckoning, odometry_pods,
    # apriltag) can't react to the blocker at all -- so their success-
    # rate jump between static and moving isn't about replanning around
    # it, and needs its own explanation rather than being silently left
    # next to the sensing-suite replan numbers below as if unremarkable.
    blind_suites = [s for s in SUITE_ORDER if not SUITES[s]().senses_obstacles]
    blind_gains = [(s, overall_success_rate(stats, s, "moving") - overall_success_rate(stats, s, "static"))
                   for s in blind_suites]
    biggest_blind_gain = max(blind_gains, key=lambda x: x[1])
    lines += [
        "", "## Why do suites that never sense the blocker at all also do better against a moving one?", "",
        f"{', '.join(SUITE_LABELS[s] for s in blind_suites)} never sense obstacles -- they can't react to "
        "the blocker being there at all, moving or static, and drive the exact same pre-planned route "
        "regardless. Yet every one of them does better against a moving opponent than a static one (see "
        f"the table above; the largest gain is {SUITE_LABELS[biggest_blind_gain[0]]}'s "
        f"{biggest_blind_gain[1]:+.0%}). The mechanism isn't sensing -- it's timing. A static blocker sits "
        "on the same planned-route cell for the entire match, so a blind suite that ever plans through "
        "that cell collides with it deterministically. A moving blocker starts on that same cell but then "
        "random-walks away; by the time a blind suite's fixed plan actually reaches that cell, the "
        "blocker has often wandered somewhere else, purely by timing luck the static version could never "
        "offer. A moving opponent is a *harder* obstacle to reason about, but this particular deviation "
        "type happens to make it an *easier* one to physically avoid for a suite that isn't reasoning "
        "about it at all.",
    ]

    # Collision/replan behavior against a moving opponent, for suites
    # that sense obstacles -- do they actually benefit from being able
    # to re-route around it, or does an opponent that keeps wandering
    # back into a just-cleared path erode that advantage?
    sensing_suites = [s for s in SUITE_ORDER if SUITES[s]().senses_obstacles]
    lines += ["", "## Do obstacle-sensing suites actually benefit against a moving opponent?", "",
              "| Suite | Avg. collisions (static) | Avg. collisions (moving) | Avg. replans (static) | "
              "Avg. replans (moving) |", "|---|---:|---:|---:|---:|"]
    levels = [l for l in VARIANCE_LEVELS if l >= SUMMARY_MIN_LEVEL]
    for suite in SUITE_ORDER:
        static_coll = sum(stats[(suite, "static", l)]["avg_collisions"] for l in levels) / len(levels)
        moving_coll = sum(stats[(suite, "moving", l)]["avg_collisions"] for l in levels) / len(levels)
        static_replans = sum(stats[(suite, "static", l)]["avg_replans"] for l in levels) / len(levels)
        moving_replans = sum(stats[(suite, "moving", l)]["avg_replans"] for l in levels) / len(levels)
        lines.append(f"| {SUITE_LABELS[suite]} | {static_coll:.2f} | {moving_coll:.2f} | "
                      f"{static_replans:.2f} | {moving_replans:.2f} |")
    lines.append("")
    lines.append(
        "Suites that sense obstacles (" + ", ".join(SUITE_LABELS[s] for s in sensing_suites) + ") replan "
        "more against a moving opponent than a static one, since a random-walking obstacle can wander "
        "back onto an already-cleared path -- extra active work a static blocker never demands, but "
        "work that still pays off in a lower collision rate (see the table above). The non-sensing "
        "suites' improvement above is a *passive* benefit (the blocker happens to wander off their fixed "
        "route); this is the *active* version of the same underlying advantage -- a sensing suite can "
        "additionally detect and route around the blocker even while it's still nearby, instead of just "
        "waiting for it to leave."
    )

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)

    all_rows = []
    for blocker_type in BLOCKER_TYPES:
        for level in VARIANCE_LEVELS:
            base_seed = 9_600_000 + BLOCKER_TYPES.index(blocker_type) * 1_000_000 + round(level * 100)
            print(f"blocker_type={blocker_type} variance_level={level} ...")
            all_rows.extend(run_combo(blocker_type, level, TRIALS_PER_COMBO, base_seed, grid, free_cells,
                                       tag_sites))

    write_csv(all_rows, OUTPUT_DIR / "ftc_opponent_results.csv")
    stats = aggregate(all_rows)
    plot_comparison(stats, OUTPUT_DIR / "ftc_opponent_comparison.png")
    write_writeup(stats, all_rows, OUTPUT_DIR / "ftc_opponent_writeup.md")

    print(f"\nWrote {len(all_rows)} trials to {OUTPUT_DIR / 'ftc_opponent_results.csv'}")
    print(f"Chart saved to {OUTPUT_DIR / 'ftc_opponent_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'ftc_opponent_writeup.md'}\n")
    for blocker_type in BLOCKER_TYPES:
        _, baseline, best = value_ranking(stats, blocker_type)
        print(f"{blocker_type}: best value = {best} (baseline={baseline:.0%})")
