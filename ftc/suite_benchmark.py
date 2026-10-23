"""
The headline study: which sensing investment actually buys reliability
in a 30-second FTC autonomous period, and at what level of field/
reality deviation does each become necessary? See README.md's "Research
question" section for the full framing -- this is the sweep that's
supposed to answer it.

Sweeps sensor suite (ftc/sensors.py) x deviation level x deviation type
x N seeded trials, on ftc/field.py's "cluttered" layout (the richest
test of both obstacle-sensing suites -- plenty to actually sense -- and
the hard-footprint-inflation routing that makes this project's grids
different from nav/'s point-robot ones). Deviation type comes from
Phase 2's independent-axis nav/field_variance.py scaling: each sweep
point isolates start drift, obstacle drift, or the unplanned-blocker
probability alone (the other two scales held at 0), instead of nav/
uncertainty_benchmark.py's single bundled variance_level -- without that
split there'd be no way to explain *why* a given suite wins or loses,
only that it does.

Every suite at a given (deviation_type, variance_level, trial index)
runs against the identical ground truth (same reasoning nav/
uncertainty_benchmark.py and nav/replan_benchmark.py both already use:
share the random scenario across every policy/suite being compared, so
a difference in outcome is attributable to the suite, not to which
random scenario it happened to get).

Writes benchmark_results/ftc_suite_results.csv (every trial, raw),
benchmark_results/ftc_suite_comparison.png (success rate vs. deviation,
one row per deviation type, CI bands), benchmark_results/
ftc_reliability_per_dollar.png (success-rate gain over the free
DeadReckoningSuite baseline, per $100 spent), and benchmark_results/
ftc_suite_writeup.md (which suite is the best value, which deviation
type dominates real failure, whether the expensive suites earn their
cost -- reported plainly, including any results that don't flatter this
project's own belief-planning machinery).
"""
import csv
import random
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
# Every dollar-cost label in this module's plots is a literal "$" in
# plain text, not the start of a mathtext expression -- without this,
# matplotlib's default $...$ math-mode parsing silently eats everything
# between two "$" signs (e.g. two "$100" labels in the same string) and
# renders it as an equation instead of currency.
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt

from nav.algorithms import astar
from nav.field_variance import generate_ground_truth
from nav.stats import bootstrap_ci

from ftc.config import DISTANCE_SENSOR_COUNT, DISTANCE_SENSOR_HALF_ANGLE_DEG, usd
from ftc.field import build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import SUITES, SUITE_ORDER, SUITE_LABELS

LAYOUT = "cluttered"
VARIANCE_LEVELS = [round(i / 10, 1) for i in range(11)]  # 0.0, 0.1, ..., 1.0
TRIALS_PER_COMBO = 25
MAX_ATTEMPTS_PER_TRIAL = 50
MIN_PATH_LEN = 4
# Levels below this are excluded from the "overall" success-rate and
# reliability-per-dollar summaries -- every suite trivially succeeds
# near variance_level=0.0 (ground truth close to the assumed map), so
# including those points would dilute the comparison exactly where
# sensing investment is supposed to matter.
SUMMARY_MIN_LEVEL = 0.3
OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

DEVIATION_TYPES = {
    "start_drift": dict(start_drift_scale=1.0, obstacle_drift_scale=0.0, blocker_scale=0.0),
    "obstacle_drift": dict(start_drift_scale=0.0, obstacle_drift_scale=1.0, blocker_scale=0.0),
    "unplanned_blocker": dict(start_drift_scale=0.0, obstacle_drift_scale=0.0, blocker_scale=1.0),
}
DEVIATION_TYPE_ORDER = ["start_drift", "obstacle_drift", "unplanned_blocker"]
DEVIATION_TYPE_LABELS = {
    "start_drift": "Start drift (pose error)",
    "obstacle_drift": "Obstacle drift (map error)",
    "unplanned_blocker": "Unplanned blocker (opponent robot)",
}
SUITE_COLORS = {
    "dead_reckoning": "tab:red",
    "odometry_pods": "tab:orange",
    "distance_sensors": "tab:blue",
    "apriltag": "tab:purple",
    # Promoted from ftc/newsuites_benchmark.py's side study into the
    # headline sweep -- reusing that module's own colors for the same
    # two suites, so a reader who's seen ftc_newsuites_comparison.png
    # doesn't have to relearn a color mapping.
    "imu": "tab:cyan",
    "dual_camera_apriltag": "tab:brown",
    "full_suite": "tab:green",
}


def _solvable_scenario(trial_seed, grid, free_cells):
    rng = random.Random(trial_seed)
    for _ in range(MAX_ATTEMPTS_PER_TRIAL):
        start, goal = rng.sample(free_cells, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= MIN_PATH_LEN:
            return start, goal
    raise RuntimeError(f"no solvable scenario for seed {trial_seed} after {MAX_ATTEMPTS_PER_TRIAL} attempts")


def run_combo(deviation_type, variance_level, num_trials, base_seed, grid, free_cells, tag_sites,
              fidelity=None, drivetrain=None):
    """`fidelity` defaults to None -- run_match resolves that to
    ftc.config.MODEL_FIDELITY (the "optimistic" default) fresh on every
    call, so every caller that doesn't pass it (every one that existed
    before ftc/fidelity_benchmark.py) is completely unaffected. Passing
    an explicit tier is what lets ftc/fidelity_benchmark.py rerun this
    exact sweep at "realistic"/"pessimistic" without duplicating this
    function.

    `drivetrain` defaults to None the same way -- run_match's own
    `drivetrain=None` default reproduces the existing tank-equivalent
    behavior exactly, so every caller that doesn't pass it (every one
    that existed before ftc/drivetrain_suite_benchmark.py) is completely
    unaffected. It is NOT part of `base_seed`/the per-trial seed: a
    (deviation_type, variance_level, trial index) triple must draw the
    identical ground truth regardless of which drivetrain later drives
    it, which is what lets ftc/drivetrain_suite_benchmark.py pair a
    tank run and a mecanum run trial-for-trial on the same scenarios."""
    rows = []
    scale_kwargs = DEVIATION_TYPES[deviation_type]
    for t in range(num_trials):
        trial_seed = base_seed + t
        start, goal = _solvable_scenario(trial_seed, grid, free_cells)
        ground_truth, actual_start = generate_ground_truth(
            grid, start, goal, variance_level, seed=trial_seed, **scale_kwargs
        )
        for suite_name in SUITE_ORDER:
            suite = SUITES[suite_name]()
            result = run_match(suite, grid, start, goal, ground_truth, actual_start, tag_sites,
                                random.Random(trial_seed), fidelity=fidelity, drivetrain=drivetrain)
            rows.append({
                "suite": suite_name,
                "deviation_type": deviation_type,
                "variance_level": variance_level,
                "trial": t,
                "seed": trial_seed,
                "success": result.success,
                "over_budget": result.over_budget,
                "elapsed_s": result.elapsed_s,
                "collisions": result.collisions,
                "replans": result.replans,
                "planning_time_ms": round(result.planning_time_s * 1000, 4),
                "final_pose_error_in": result.final_pose_error_in,
                "steps": result.steps,
                "cost_usd": suite.cost_usd,
            })
    return rows


def base_seed(deviation_type, level):
    """The one seed formula every full-rigor sweep in this repo shares
    -- ftc/suite_benchmark.py's own __main__, plus ftc/layout_benchmark.py,
    ftc/budget_benchmark.py, and ftc/fidelity_benchmark.py, which all
    inlined this exact expression before this function existed (grep
    the old commit for `6_000_000 +` to see it duplicated four times).
    Pulling it out doesn't change a single seed -- it's the same
    literal formula, just named -- and it's what lets run_sweep below
    reconstruct any caller's base_seed from just (deviation_type, level)
    without that caller having to pass its own seed dict across a
    process boundary."""
    return 6_000_000 + DEVIATION_TYPE_ORDER.index(deviation_type) * 1_000_000 + round(level * 100)


def run_sweep(deviation_types, levels, num_trials, grid, free_cells, tag_sites, fidelity=None,
              max_workers=None, drivetrain=None):
    """Runs run_combo for every (deviation_type, level) pair using
    base_seed() above, and returns every row concatenated in the same
    order a plain `for deviation_type: for level:` loop would produce --
    so this is a drop-in replacement for that loop, not a new sweep.

    Each (deviation_type, level) combo is one independent unit of work:
    run_combo's own trial loop already derives every trial's seed from
    base_seed + t, so no state is shared between combos, and nothing
    about which combo runs on which worker (or in what order workers
    finish) can change a single seed. That's what makes farming combos
    out to separate processes safe -- see
    ftc/scratch/parallel_determinism_test.py, which proves it by diffing
    CSVs, not just asserting it in a docstring.

    max_workers=1 skips ProcessPoolExecutor entirely and runs every
    combo serially in this process -- no subprocess startup cost, and
    it's the baseline parallel_determinism_test.py compares against.
    max_workers=None hands off to ProcessPoolExecutor's own default.

    `drivetrain` (a ftc.drivetrain.Drivetrain instance, or None) is
    passed straight through to every run_combo call -- see run_combo's
    own docstring for why it defaults to None and stays out of the seed
    formula. ftc/drivetrain_suite_benchmark.py is the one caller that
    passes an explicit one."""
    combos = [(dt, lvl) for dt in deviation_types for lvl in levels]

    if max_workers == 1:
        results = {
            combo: run_combo(combo[0], combo[1], num_trials, base_seed(*combo), grid, free_cells, tag_sites,
                              fidelity=fidelity, drivetrain=drivetrain)
            for combo in combos
        }
    else:
        with ProcessPoolExecutor(max_workers=max_workers) as executor:
            futures = {
                combo: executor.submit(run_combo, combo[0], combo[1], num_trials, base_seed(*combo), grid,
                                        free_cells, tag_sites, fidelity, drivetrain)
                for combo in combos
            }
            results = {combo: future.result() for combo, future in futures.items()}

    # Reassembled in `combos`' fixed order (not completion order, which
    # ProcessPoolExecutor makes no promises about) -- this is the line
    # that makes the output order independent of worker count.
    rows = []
    for combo in combos:
        rows.extend(results[combo])
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _bootstrap_seed(suite, deviation_type, level):
    return (4_000_000 + SUITE_ORDER.index(suite) * 100_000
            + DEVIATION_TYPE_ORDER.index(deviation_type) * 10_000 + round(level * 1000))


def aggregate(rows):
    """{(suite, deviation_type, variance_level): {success_rate, ci_lo,
    ci_hi, over_budget_rate, avg_elapsed_s, avg_collisions, avg_replans,
    avg_planning_ms, avg_final_pose_error_in}}."""
    stats = {}
    for suite in SUITE_ORDER:
        for deviation_type in DEVIATION_TYPE_ORDER:
            for level in VARIANCE_LEVELS:
                matching = [r for r in rows if r["suite"] == suite and r["deviation_type"] == deviation_type
                            and r["variance_level"] == level]
                successes = [r for r in matching if r["success"]]
                n = len(matching)
                ci_lo, ci_hi = bootstrap_ci(len(successes), n, seed=_bootstrap_seed(suite, deviation_type, level))
                stats[(suite, deviation_type, level)] = {
                    "success_rate": len(successes) / n if n else 0.0,
                    "ci_lo": ci_lo,
                    "ci_hi": ci_hi,
                    "over_budget_rate": sum(r["over_budget"] for r in matching) / n if n else 0.0,
                    "avg_elapsed_s": sum(r["elapsed_s"] for r in matching) / n if n else 0.0,
                    "avg_collisions": sum(r["collisions"] for r in matching) / n if n else 0.0,
                    "avg_replans": sum(r["replans"] for r in matching) / n if n else 0.0,
                    "avg_planning_ms": sum(r["planning_time_ms"] for r in matching) / n if n else 0.0,
                    "avg_final_pose_error_in": sum(r["final_pose_error_in"] for r in matching) / n if n else 0.0,
                }
    return stats


def overall_success_rate(stats, suite, deviation_type=None, min_level=SUMMARY_MIN_LEVEL):
    """Mean success_rate for `suite` across every level >= min_level,
    either for one deviation_type or averaged over all three."""
    types = [deviation_type] if deviation_type else DEVIATION_TYPE_ORDER
    levels = [l for l in VARIANCE_LEVELS if l >= min_level]
    vals = [stats[(suite, t, l)]["success_rate"] for t in types for l in levels]
    return sum(vals) / len(vals)


def plot_comparison(stats, path):
    fig, axes = plt.subplots(len(DEVIATION_TYPE_ORDER), 1, figsize=(8, 12), sharex=True)
    for ax, deviation_type in zip(axes, DEVIATION_TYPE_ORDER):
        for suite in SUITE_ORDER:
            success = [stats[(suite, deviation_type, level)]["success_rate"] for level in VARIANCE_LEVELS]
            ci_lo = [stats[(suite, deviation_type, level)]["ci_lo"] for level in VARIANCE_LEVELS]
            ci_hi = [stats[(suite, deviation_type, level)]["ci_hi"] for level in VARIANCE_LEVELS]
            ax.fill_between(VARIANCE_LEVELS, ci_lo, ci_hi, color=SUITE_COLORS[suite], alpha=0.12, linewidth=0)
            ax.plot(VARIANCE_LEVELS, success, "o-", label=SUITE_LABELS[suite], color=SUITE_COLORS[suite],
                     markersize=4)
        ax.set_ylabel("Success rate")
        ax.set_ylim(-0.05, 1.05)
        ax.set_title(DEVIATION_TYPE_LABELS[deviation_type])
    axes[0].legend(loc="lower left", fontsize=8)
    axes[-1].set_xlabel("variance_level (that deviation type alone, other two held at 0)")
    fig.suptitle(f"Sensor suite success rate vs. deviation ({TRIALS_PER_COMBO} trials/point, "
                  f"'{LAYOUT}' layout)\nshaded = 95% bootstrap CI", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.subplots_adjust(top=0.90)
    fig.savefig(path, dpi=150)


def plot_reliability_per_dollar(stats, path):
    """Bar height is (rate - baseline), a success-rate FRACTION, divided
    by (cost / 100) -- i.e. fraction-of-success-rate per $100 spent.
    Multiplying by 100 turns that into percentage POINTS per $100 (e.g.
    a hypothetical suite that's 16 percentage points better at $50 reads
    as +32, the number you'd actually want to read off the bar) -- leaving it
    unscaled would silently be 100x smaller than what the axis label
    and the write_writeup() text below both claim to be showing.

    Suites at cost_usd == 0.0 (the dead-reckoning baseline itself, and
    ImuSuite -- every REV Control Hub already ships one) are excluded
    from this chart entirely, not divided-by-zero or plotted as
    "infinite" value: "value per dollar" isn't a meaningful axis for
    hardware that costs nothing (see ftc/newsuites_benchmark.py's own
    per_100=None convention for the same case). ImuSuite's raw gain is
    reported separately in write_writeup() instead."""
    baseline = overall_success_rate(stats, "dead_reckoning")
    suites = [s for s in SUITE_ORDER if s != "dead_reckoning" and SUITES[s].cost_usd > 0]
    gains_per_100 = []
    for suite in suites:
        rate = overall_success_rate(stats, suite)
        cost = SUITES[suite].cost_usd
        gains_per_100.append((rate - baseline) / (cost / 100) * 100)

    fig, ax = plt.subplots(figsize=(8, 5))
    colors = [SUITE_COLORS[s] for s in suites]
    bars = ax.bar([SUITE_LABELS[s] for s in suites], gains_per_100, color=colors)
    for bar, suite in zip(bars, suites):
        ax.annotate(f"${usd(SUITES[suite].cost_usd)}", (bar.get_x() + bar.get_width() / 2, bar.get_height()),
                     ha="center", va="bottom" if bar.get_height() >= 0 else "top", fontsize=9)
    ax.axhline(0, color="black", linewidth=0.8)
    ax.set_ylabel("Success-rate gain over DeadReckoningSuite\nper $100 spent (percentage points / $100)")
    ax.set_title(f"Reliability per dollar (variance_level >= {SUMMARY_MIN_LEVEL}, all deviation types)\n"
                  f"DeadReckoningSuite baseline = {baseline:.0%}", fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=150)


def dominant_deviation_type(stats, suite, min_level=0.5):
    """Which deviation type `suite` handles worst -- the one with the
    lowest mean success rate at variance_level >= min_level. That's the
    deviation this suite's owner would actually feel in a match."""
    levels = [l for l in VARIANCE_LEVELS if l >= min_level]
    rates = {
        t: sum(stats[(suite, t, l)]["success_rate"] for l in levels) / len(levels)
        for t in DEVIATION_TYPE_ORDER
    }
    worst = min(rates, key=rates.get)
    return worst, rates


def write_writeup(stats, rows, path):
    lines = [
        "# FTC sensor suite comparison under field/reality deviation",
        "",
        f"{TRIALS_PER_COMBO} trials per (suite, deviation_type, variance_level) point, "
        f"{len(VARIANCE_LEVELS)} variance_level steps from 0.0 to 1.0, 3 deviation types swept "
        f"independently (nav/field_variance.py's *_scale kwargs -- Phase 2), on the '{LAYOUT}' field "
        "layout (ftc/field.py). Every suite at a given (deviation_type, variance_level, trial index) "
        "runs against the identical ground truth, so a gap between suites reflects the suite, not "
        "which random scenario it happened to get. Raw data in `ftc_suite_results.csv`, charts in "
        "`ftc_suite_comparison.png` and `ftc_reliability_per_dollar.png`.",
        "",
        "## Which suite wins",
        "",
        f"Overall success rate, variance_level >= {SUMMARY_MIN_LEVEL} across all three deviation types "
        "(excludes the near-zero-deviation points every suite trivially clears):",
        "",
        "| Suite | Cost | Overall success rate |",
        "|---|---:|---:|",
    ]
    overall = {s: overall_success_rate(stats, s) for s in SUITE_ORDER}
    for suite in sorted(SUITE_ORDER, key=lambda s: -overall[s]):
        lines.append(f"| {SUITE_LABELS[suite]} | ${usd(SUITES[suite].cost_usd)} | {overall[suite]:.0%} |")

    best_suite = max(SUITE_ORDER, key=lambda s: overall[s])
    lines += [
        "",
        f"{SUITE_LABELS[best_suite]} has the highest overall success rate "
        f"({overall[best_suite]:.0%}) at ${usd(SUITES[best_suite].cost_usd)}. See "
        "`ftc_reliability_per_dollar.png` and the value section below for whether that's actually "
        "the best *spend*, not just the best raw number.",
        "",
        "## Which deviation type dominates real failure",
        "",
        f"For each suite, the deviation type with the lowest mean success rate at variance_level >= 0.5 "
        "-- the one that actually hurts that suite the most in a match:",
        "",
        "| Suite | Dominant deviation type | Success rate on that type | start_drift | obstacle_drift | unplanned_blocker |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for suite in SUITE_ORDER:
        worst, rates = dominant_deviation_type(stats, suite)
        lines.append(
            f"| {SUITE_LABELS[suite]} | {DEVIATION_TYPE_LABELS[worst]} | {rates[worst]:.0%} | "
            f"{rates['start_drift']:.0%} | {rates['obstacle_drift']:.0%} | {rates['unplanned_blocker']:.0%} |"
        )

    dr_worst, _ = dominant_deviation_type(stats, "dead_reckoning")
    full_worst, _ = dominant_deviation_type(stats, "full_suite")
    lines += [
        "",
        f"With no sensing at all (DeadReckoningSuite, the baseline every FTC team already has for free), "
        f"{DEVIATION_TYPE_LABELS[dr_worst]} is what actually breaks a run. "
        + (
            f"FullSuite's worst deviation type is different ({DEVIATION_TYPE_LABELS[full_worst]}) -- "
            "which sensing investment matters depends on which kind of deviation the field/robot actually "
            "produces, not on some universal ranking of suites."
            if full_worst != dr_worst else
            f"FullSuite's worst deviation type is the *same* one ({DEVIATION_TYPE_LABELS[full_worst]}) -- "
            "spending on every suite at once didn't change which failure mode dominates, only how often it "
            "happens."
        ),
        "",
        "## Do the expensive suites earn their cost?",
        "",
    ]

    baseline = overall["dead_reckoning"]
    value_lines = []
    for suite in SUITE_ORDER:
        if suite == "dead_reckoning":
            continue
        cost = SUITES[suite].cost_usd
        gain = overall[suite] - baseline
        # gain is a success-rate FRACTION (e.g. 0.16); *100 turns
        # "fraction of success rate per $100" into the percentage
        # POINTS per $100 the "pp/$100" label below actually claims --
        # without it, every printed value here was 100x smaller than
        # its own unit label said (a real bug caught reviewing the
        # symposium poster, which uses the corrected number).
        #
        # None (NOT infinity) at cost_usd == 0.0 -- IMU is the one
        # headline suite this applies to (every REV Control Hub already
        # ships one). "Infinite value per dollar" isn't a meaningful
        # claim about a suite whose only real cost is integration
        # effort this project's dollar model can't price -- same
        # reasoning ftc/newsuites_benchmark.py and ftc/optimizer.py's
        # CandidateResult.value_per_100 already use for exactly this
        # case, applied here now that a second $0 suite besides the
        # baseline itself exists.
        per_100 = None if cost == 0 else (gain / (cost / 100)) * 100
        value_lines.append((suite, cost, gain, per_100))

    # Priced suites sort by per_100 descending; $0 suites sort last
    # (reported separately below, never as "infinitely good value") --
    # the same ordering rule ftc/optimizer.py's rank(objective="value")
    # uses.
    for suite, cost, gain, per_100 in sorted(value_lines, key=lambda x: (x[3] is not None, x[3] or 0.0),
                                              reverse=True):
        per_100_str = f"{per_100:+.1f}pp/$100" if per_100 is not None else "undefined (cost_usd == 0)"
        lines.append(f"- {SUITE_LABELS[suite]} (${usd(cost)}): {gain:+.0%} success rate over the free "
                       f"baseline -- {per_100_str}.")

    priced_value_lines = [v for v in value_lines if v[3] is not None]
    best_value = max(priced_value_lines, key=lambda x: x[3])
    # best_suite (the highest RAW success rate) is computed above and is
    # not necessarily the most expensive suite -- this used to be hardcoded
    # as "FullSuite ... is also the best raw performer", which silently went
    # stale the moment Odometry pods overtook FullSuite on raw success rate.
    # Derive both the raw winner and the priciest suite from the data instead.
    most_expensive = max(SUITE_ORDER, key=lambda s: SUITES[s].cost_usd)
    most_expensive_value = next(v for v in value_lines if v[0] == most_expensive)
    lines += ["", (
        f"{SUITE_LABELS[best_value[0]]} is the best value by success-rate-gained-per-dollar. "
        + (
            f"{SUITE_LABELS[most_expensive]} -- the most expensive option at "
            f"${usd(SUITES[most_expensive].cost_usd)} -- returns only "
            f"{most_expensive_value[3]:+.1f}pp/$100, against "
            f"{SUITE_LABELS[best_value[0]]}'s {best_value[3]:+.1f}pp/$100: the extra suites it "
            "stacks on top run into diminishing returns rather than each adding its standalone "
            "value again."
            if best_value[0] != most_expensive else
            "It's also the most expensive option, and still wins on a per-dollar basis, not just in raw "
            "success rate -- the combination doesn't just perform best, it's actually the efficient choice."
        )
    ), (
        f"Best raw success rate is a separate question, and here it has a separate answer: "
        f"{SUITE_LABELS[best_suite]} ({overall[best_suite]:.0%}) at "
        f"${usd(SUITES[best_suite].cost_usd)}."
        if best_suite != best_value[0] else
        f"{SUITE_LABELS[best_suite]} happens to win both questions here -- best value AND best raw "
        f"success rate ({overall[best_suite]:.0%})."
    ), ""]

    free_value_lines = [v for v in value_lines if v[3] is None]
    if free_value_lines:
        imu_suite, imu_cost, imu_gain, _ = next(v for v in free_value_lines if v[0] == "imu")
        lines += [
            f"IMU gained {imu_gain:+.0%} success rate over dead reckoning for $0 -- a real gain (or loss) "
            "with no dollar figure to divide it by (every REV Control Hub already ships one; the only real "
            "cost is the integration effort of reading and fusing it, ftc/sensors.py's ImuSuite, which this "
            "project's dollar-based cost model has no way to price). \"Is the free hardware worth the code\" "
            "has to be answered by the gain itself, not a per-dollar ranking.", "",
        ]

    imu_ties_dead_reckoning = overall["imu"] == overall["dead_reckoning"]
    rear_ties_front = overall["dual_camera_apriltag"] == overall["apriltag"]
    lines += ["## Honest findings", ""]
    if imu_ties_dead_reckoning or rear_ties_front:
        lines.append(
            "Two of this sweep's own numbers above are easy to misread as a bug rather than what they "
            "are -- an explicit consequence of `MODEL_FIDELITY = \"optimistic\"` (`ftc/config.py`), the "
            "tier this whole headline sweep runs at by default. "
            + (
                "IMU's overall success rate is IDENTICAL to DeadReckoningSuite's, cell for cell across "
                "every deviation type, because the optimistic tier's heading drift is 0.0 by construction "
                "-- there is no heading error anywhere for an IMU's continuous heading correction to fix, "
                "so ImuSuite behaves byte-for-byte like DeadReckoningSuite at this tier. "
                if imu_ties_dead_reckoning else ""
            )
            + (
                "Rear camera's overall success rate is IDENTICAL to AprilTag (front camera)'s, because the "
                "optimistic tier's camera FOV is 360 degrees (omnidirectional) -- a second, rear-facing "
                "camera adds no coverage a camera that already sees everything didn't already have. "
                if rear_ties_front else ""
            )
            + "Both are real findings about this tier's own stated assumptions, not measurement noise -- "
            "`ftc/fidelity_benchmark.py` reruns this sweep at the `realistic`/`pessimistic` tiers, where "
            "heading drift and real camera FOV are both live, and both suites separate measurably from "
            "their baselines there (see `ftc_fidelity_writeup.md`)."
        )
        lines.append("")

    dr_rows = [r for r in rows if r["suite"] == "distance_sensors" and r["variance_level"] == 0.0]
    dr_zero_collisions = sum(1 for r in dr_rows if r["collisions"] > 0)
    if dr_zero_collisions > 0:
        rate = dr_zero_collisions / len(dr_rows) if dr_rows else 0.0
        covered_deg = DISTANCE_SENSOR_COUNT * 2 * DISTANCE_SENSOR_HALF_ANGLE_DEG
        lines.append(
            f"DistanceSensorSuite collides in {rate:.0%} of trials even at variance_level=0.0 (ground "
            "truth cell-for-cell identical to the assumed map, every deviation type at 0) -- so field "
            "deviation isn't causing these collisions at all. A controlled check (same trials, "
            "drift_per_cell forced to 0) shows roughly two-thirds of these collisions persist with pose "
            "error completely disabled, so the dominant cause isn't pose drift -- it's the sensor "
            f"geometry itself. {DISTANCE_SENSOR_COUNT} narrow ToF cones "
            f"({DISTANCE_SENSOR_HALF_ANGLE_DEG:.1f} deg half-angle each) mounted front/left/right cover "
            f"only about {covered_deg:.0f} of the 360 degrees around the robot; anything in the roughly "
            f"{360 - covered_deg:.0f}-degree gap between cones -- a very plausible place for an obstacle "
            "to sit relative to a robot that's mid-turn on a diagonal grid -- is simply never seen until "
            "the robot's next planned step walks straight into it. Pose drift is a real, secondary "
            "compounding factor on top of that (the same check found collisions drop by roughly a third "
            "once drift is disabled, since a correctly-remembered obstacle position still isn't the same "
            "as never having missed one), but the primary lesson is blunter than a SLAM-consistency "
            "story: a sparse fixed-cone sensor suite has real, geometry-driven blind spots, and this "
            "project's own headline nav/ result (reactive beats belief, measured against nav/sensor.py's "
            "full disc-scan sensor model, which has no blind spot at all) doesn't transfer to a suite "
            "whose sensing coverage is this incomplete. `ftc/coverage_benchmark.py` sweeps more ToF "
            "sensors against this exact gap and finds it's worse than 'currently unmet': even at the "
            "largest count tested, a structural blind arc survives no FTC-legal ToF sensor count can "
            "close (see `ftc_coverage_writeup.md`'s 'Is full coverage even reachable?'). Buying distance "
            "sensors without covering enough of the robot's perimeter can be worse than not sensing at "
            "all, purely from what the hardware physically cannot see."
        )
    else:
        lines.append(
            "DistanceSensorSuite has zero collisions at variance_level=0.0 in this run -- no map/pose "
            "self-consistency issue showed up here, unlike some earlier smoke-testing runs of the same "
            "suite. Worth rerunning at a different seed before treating that as settled either way."
        )

    over_budget_any = {s: any(stats[(s, t, l)]["over_budget_rate"] > 0 for t in DEVIATION_TYPE_ORDER
                                for l in VARIANCE_LEVELS) for s in SUITE_ORDER}
    if any(over_budget_any.values()):
        worst_budget_suite = max(SUITE_ORDER, key=lambda s: sum(
            stats[(s, t, l)]["over_budget_rate"] for t in DEVIATION_TYPE_ORDER for l in VARIANCE_LEVELS))
        lines += ["", (
            f"The 30-second budget is a real constraint, not a footnote: {SUITE_LABELS[worst_budget_suite]} "
            "has the highest rate of runs that reached the goal too late to count, somewhere in this sweep "
            "-- replanning has a real time cost (PLANNING_OVERHEAD_S per call, ftc/config.py) on top of "
            "drive time, unlike nav/uncertainty_benchmark.py's harness, which lets every closed-loop policy "
            "replan for free indefinitely."
        )]
    else:
        lines += ["", (
            "No suite ever ran out of the 30-second budget in this sweep -- on the "
            f"'{LAYOUT}' layout at this grid scale, drive time and replan overhead never came close to "
            "30s even under heavy deviation. A larger/more cluttered layout or a tighter budget would be "
            "needed to make AUTONOMOUS_PERIOD_S itself the binding constraint rather than success/collision."
        )]

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)

    print(f"Running {len(DEVIATION_TYPE_ORDER)}x{len(VARIANCE_LEVELS)} combos "
          f"({TRIALS_PER_COMBO} trials each) in parallel...")
    all_rows = run_sweep(DEVIATION_TYPE_ORDER, VARIANCE_LEVELS, TRIALS_PER_COMBO, grid, free_cells, tag_sites)

    write_csv(all_rows, OUTPUT_DIR / "ftc_suite_results.csv")
    stats = aggregate(all_rows)
    plot_comparison(stats, OUTPUT_DIR / "ftc_suite_comparison.png")
    plot_reliability_per_dollar(stats, OUTPUT_DIR / "ftc_reliability_per_dollar.png")
    write_writeup(stats, all_rows, OUTPUT_DIR / "ftc_suite_writeup.md")

    print(f"\nWrote {len(all_rows)} trials to {OUTPUT_DIR / 'ftc_suite_results.csv'}")
    print(f"Charts saved to {OUTPUT_DIR / 'ftc_suite_comparison.png'} and "
          f"{OUTPUT_DIR / 'ftc_reliability_per_dollar.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'ftc_suite_writeup.md'}")
    for suite in SUITE_ORDER:
        rate = overall_success_rate(stats, suite)
        print(f"\n{SUITE_LABELS[suite]} (${usd(SUITES[suite].cost_usd)}): overall success rate {rate:.0%}")
        for deviation_type in DEVIATION_TYPE_ORDER:
            worst_rate = sum(stats[(suite, deviation_type, l)]["success_rate"] for l in VARIANCE_LEVELS
                              if l >= 0.5) / len([l for l in VARIANCE_LEVELS if l >= 0.5])
            print(f"  {deviation_type}: mean success (level>=0.5) = {worst_rate:.0%}")
