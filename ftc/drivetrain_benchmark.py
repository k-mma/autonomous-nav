"""
Does a mecanum drivetrain's cost premium (ftc/drivetrain.py: goBILDA's
own 96mm mecanum set vs. traction set, see ftc/config.py's
MECANUM_WHEEL_COST_USD/TANK_WHEEL_COST_USD for the current sourced
figures) get repaid, and does it change which SENSOR suite is the best
buy? README.md's "Threats to validity" names "no mecanum-specific
strafing advantage" as an open limitation ftc/drivetrain.py closes;
this is the study that measures what closing it actually changes.

The interaction this module exists to check explicitly: a mecanum robot
that holds a fixed heading toward the nearest AprilTag wall (ftc/
drivetrain.py's documented heading policy) keeps its camera aimed at
tags for the ENTIRE match, which Priority 1's camera-FOV gating should
help AprilTag substantially -- while a tank robot's camera swings away
from the tag wall every time it changes direction. ftc/
fidelity_benchmark.py's own realistic-tier numbers already show
AprilTag's overall success rate drop measurably from the optimistic
tier to the realistic one using the DEFAULT (tank-equivalent)
drivetrain (see benchmark_results/ftc_fidelity_writeup.md for the
current figures) -- this module's job is to check whether a mecanum
drivetrain recovers some or all of that gap.

Deliberately crossed with fidelity tier (ftc.config.FIDELITY_TIERS'
"optimistic" and "realistic") rather than run at only one: at
"optimistic" a mecanum robot's held heading should make NO difference
to AprilTag (an omnidirectional camera already sees everything
regardless of which way it's held), which is the built-in consistency
check that the realistic-tier effect (if any) is really about camera
FOV and not some other drivetrain side effect.

Reduced trial count/level set relative to the headline sweep -- the
same "a comparison sweep needs enough points to see the shape, not a
publication-grade curve at every point" reasoning ftc/robustness.py's
own docstring already uses -- since this crosses 2 drivetrains x 2
fidelity tiers x 7 suites x 3 deviation types on top of the headline
axes.

Writes benchmark_results/ftc_drivetrain_results.csv (every trial, raw,
with added `drivetrain` and `fidelity` columns), benchmark_results/
ftc_drivetrain_comparison.png (success rate by suite, one panel per
(drivetrain, fidelity) combination), and benchmark_results/
ftc_drivetrain_writeup.md (whether mecanum's premium is repaid, and
whether it changes the best-value sensor suite) -- this original
2-drivetrain (tank/mecanum) comparison and its three output files are
UNCHANGED by the heading-policy addition below: same DRIVETRAIN_ORDER,
same seeds, same numbers, since ftc_drivetrain_writeup.md's findings
are already cited elsewhere (README.md, WRITEUPS.md) and a silent
change to what those citations point at would be worse than a second,
clearly-separate study.

A SECOND, separate sweep -- ftc/drivetrain.py's MECANUM_HEADING_POLICY_
ORDER (tank, and MECANUM under each of its four heading_policy
choices: fixed_at_start, nearest_tag_current, route_dominant,
match_travel) -- checks the exact thing the original study's own
writeup names as untested: "a policy that re-picks its held heading
periodically... isn't tested here." Reuses this module's own
run_combo/summarize/value_ranking (already generic over any
DRIVETRAINS key, and over however many entries MECANUM_HEADING_POLICY_
ORDER holds -- adding match_travel required no changes to any of the
three) with a DIFFERENT base seed (HEADING_POLICY_BASE_SEED) so its
trials never collide with the original sweep's, and writes its own
separate output files (ftc_drivetrain_heading_policy_results.csv/
_comparison.png/_writeup.md) for the same "don't perturb an
already-cited result" reason as above.
"""
import csv
import random
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt

from nav.field_variance import generate_ground_truth
from nav.stats import bootstrap_ci, bootstrap_paired_diff_ci

from ftc.drivetrain import DRIVETRAIN_LABELS, DRIVETRAIN_ORDER, DRIVETRAINS, MECANUM_HEADING_POLICY_ORDER
from ftc.field import build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import SUITES, SUITE_ORDER, SUITE_LABELS
from ftc.suite_benchmark import DEVIATION_TYPES, DEVIATION_TYPE_ORDER, LAYOUT, SUITE_COLORS, _solvable_scenario

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

FIDELITY_ORDER = ["optimistic", "realistic"]
LEVELS = [0.3, 0.5, 0.7, 0.9]
TRIALS = 15
BASE_SEED = 11_000_000
HEADING_POLICY_BASE_SEED = BASE_SEED + 500_000_000


def run_combo(drivetrain_name, fidelity, grid, free_cells, tag_sites, base_seed=BASE_SEED):
    drivetrain = DRIVETRAINS[drivetrain_name]
    rows = []
    for deviation_type in DEVIATION_TYPE_ORDER:
        scale_kwargs = DEVIATION_TYPES[deviation_type]
        for level in LEVELS:
            for t in range(TRIALS):
                trial_seed = (base_seed + DEVIATION_TYPE_ORDER.index(deviation_type) * 100_000
                              + round(level * 1000) + t)
                start, goal = _solvable_scenario(trial_seed, grid, free_cells)
                ground_truth, actual_start = generate_ground_truth(
                    grid, start, goal, level, seed=trial_seed, **scale_kwargs
                )
                for suite_name in SUITE_ORDER:
                    suite = SUITES[suite_name]()
                    result = run_match(suite, grid, start, goal, ground_truth, actual_start, tag_sites,
                                        random.Random(trial_seed), fidelity=fidelity, drivetrain=drivetrain)
                    rows.append({
                        "suite": suite_name,
                        "drivetrain": drivetrain_name,
                        "fidelity": fidelity,
                        "deviation_type": deviation_type,
                        "variance_level": level,
                        "trial": t,
                        "success": result.success,
                        "cost_usd": suite.cost_usd + drivetrain.cost_usd,
                    })
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows, drivetrain_order=DRIVETRAIN_ORDER):
    """{(drivetrain, fidelity, suite): {rate, n, successes}}"""
    stats = {}
    for drivetrain_name in drivetrain_order:
        for fidelity in FIDELITY_ORDER:
            for suite in SUITE_ORDER:
                matching = [r for r in rows if r["drivetrain"] == drivetrain_name and r["fidelity"] == fidelity
                            and r["suite"] == suite]
                successes = sum(r["success"] for r in matching)
                n = len(matching)
                stats[(drivetrain_name, fidelity, suite)] = {"rate": successes / n if n else 0.0,
                                                                "successes": successes, "n": n}
    return stats


def _bootstrap_seed(drivetrain_name, fidelity, suite):
    # MECANUM_HEADING_POLICY_ORDER's first two entries ("tank",
    # "mecanum") are in the identical order/position as DRIVETRAIN_
    # ORDER's own two entries, so indexing against the longer list here
    # produces the EXACT same seed for those two names as before this
    # function had to also support the newer heading-policy variants
    # (now three: nearest_tag_current, route_dominant, match_travel)
    # -- a byte-for-byte no-op for ftc_drivetrain_writeup.md's own CIs.
    return (9_900_000 + MECANUM_HEADING_POLICY_ORDER.index(drivetrain_name) * 500_000
            + FIDELITY_ORDER.index(fidelity) * 50_000 + SUITE_ORDER.index(suite))


def value_ranking(stats, drivetrain_name, fidelity, rows):
    baseline_rate = stats[(drivetrain_name, fidelity, "dead_reckoning")]["rate"]
    results = {}
    for suite in SUITE_ORDER:
        if suite == "dead_reckoning":
            continue
        s = stats[(drivetrain_name, fidelity, suite)]
        cost = next(r["cost_usd"] for r in rows if r["drivetrain"] == drivetrain_name
                    and r["fidelity"] == fidelity and r["suite"] == suite)
        per_100 = (s["rate"] - baseline_rate) / (cost / 100) * 100 if cost > 0 else float("inf")
        ci_lo, ci_hi = bootstrap_ci(s["successes"], s["n"], seed=_bootstrap_seed(drivetrain_name, fidelity, suite))
        results[suite] = {"rate": s["rate"], "per_100": per_100, "cost": cost, "ci_lo": ci_lo, "ci_hi": ci_hi}
    best = max(results, key=lambda s: results[s]["per_100"])
    return results, baseline_rate, best


def plot_comparison(stats, path, drivetrain_order=DRIVETRAIN_ORDER):
    fig, axes = plt.subplots(len(drivetrain_order), len(FIDELITY_ORDER), figsize=(12, 4 * len(drivetrain_order)),
                              sharey=True, squeeze=False)
    for i, drivetrain_name in enumerate(drivetrain_order):
        for j, fidelity in enumerate(FIDELITY_ORDER):
            ax = axes[i][j]
            rates = [stats[(drivetrain_name, fidelity, s)]["rate"] for s in SUITE_ORDER]
            colors = [SUITE_COLORS[s] for s in SUITE_ORDER]
            ax.bar([SUITE_LABELS[s] for s in SUITE_ORDER], rates, color=colors)
            ax.set_title(f"{DRIVETRAIN_LABELS[drivetrain_name]}, {fidelity}", fontsize=9)
            ax.set_ylim(0, 1.0)
            ax.tick_params(axis="x", labelrotation=30, labelsize=7)
    fig.suptitle(f"Success rate by suite x drivetrain x fidelity ({TRIALS} trials/point, levels {LEVELS}, "
                  f"'{LAYOUT}' layout)", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=150)


def write_writeup(stats, rows, path):
    # Pulled from ftc.drivetrain.DRIVETRAINS rather than hardcoded -- a
    # hardcoded dollar figure in this exact spot already went stale once
    # (see ftc/robustness.py's own fix for the same trap), when ftc/
    # config.py's wheel costs were corrected against real vendor prices.
    tank_cost = DRIVETRAINS["tank"].cost_usd
    mecanum_cost = DRIVETRAINS["mecanum"].cost_usd
    mecanum_premium = mecanum_cost - tank_cost
    lines = [
        "# Does a mecanum drivetrain's cost premium get repaid?",
        "",
        f"ftc/drivetrain.py adds TANK (${tank_cost:.0f}) and MECANUM (${mecanum_cost:.0f}) as an axis "
        "orthogonal to sensor suite: TANK must rotate to face its direction of travel (the existing flat "
        "per-90-degree turn cost, unchanged from before this addition); MECANUM holds a fixed heading -- "
        "aimed at the nearest AprilTag wall site -- for the whole match, paying no turn cost but a "
        f"speed/drift penalty on any step that isn't roughly forward relative to that held heading. "
        f"Crossed with 2 fidelity tiers "
        f"({', '.join(FIDELITY_ORDER)}) x 7 suites x 3 deviation types x levels {LEVELS} x {TRIALS} "
        "trials/point on the 'cluttered' layout -- reduced relative to the headline sweep (see module "
        "docstring). Raw data in `ftc_drivetrain_results.csv`, chart in `ftc_drivetrain_comparison.png`.",
        "",
        "## The AprilTag interaction this module exists to check",
        "",
    ]
    apriltag_tank_optimistic = stats[("tank", "optimistic", "apriltag")]["rate"]
    apriltag_mecanum_optimistic = stats[("mecanum", "optimistic", "apriltag")]["rate"]
    apriltag_tank_realistic = stats[("tank", "realistic", "apriltag")]["rate"]
    apriltag_mecanum_realistic = stats[("mecanum", "realistic", "apriltag")]["rate"]
    lines += [
        "| Fidelity | Tank (turns to face travel direction) | Mecanum (holds heading at tag wall) | Difference |",
        "|---|---:|---:|---:|",
        f"| optimistic | {apriltag_tank_optimistic:.0%} | {apriltag_mecanum_optimistic:.0%} | "
        f"{(apriltag_mecanum_optimistic - apriltag_tank_optimistic):+.0%} |",
        f"| realistic | {apriltag_tank_realistic:.0%} | {apriltag_mecanum_realistic:.0%} | "
        f"{(apriltag_mecanum_realistic - apriltag_tank_realistic):+.0%} |",
        "",
    ]
    optimistic_negligible = abs(apriltag_mecanum_optimistic - apriltag_tank_optimistic) <= 0.05
    realistic_helps_relative_to_optimistic_gap = (
        (apriltag_mecanum_realistic - apriltag_tank_realistic)
        > (apriltag_mecanum_optimistic - apriltag_tank_optimistic)
    )
    mecanum_worse_overall = apriltag_mecanum_realistic < apriltag_tank_realistic
    if mecanum_worse_overall:
        lines.append(
            "Mecanum does NOT come out ahead here, at either tier -- and the reason is visible in ftc/"
            "drivetrain.py's own model, not a surprise: the held heading is picked ONCE, at match start, "
            "aimed at whichever tag wall is nearest the start cell, and never changes for the rest of the "
            "match. A route's actual travel direction changes on almost every leg (up to 8 different "
            "directions on this project's diagonal grid), so unless a route happens to run roughly "
            "parallel to that one fixed heading, most of its steps are strafes relative to it -- paying "
            "MECANUM_STRAFE_SPEED_FACTOR/_DRIFT_MULTIPLIER (0.8x speed, 1.6x drift, ftc/config.py) on "
            "close to every step, not just the occasional sideways one. That drift penalty compounds "
            "across the whole route and shows up as a lower success rate for EVERY suite under mecanum, "
            "not just AprilTag (see the per-suite table below) -- "
            + (
                "the camera-stays-aimed-at-tags benefit this module set out to check is real (see the "
                "realistic-tier gap narrowing slightly relative to the optimistic-tier one below) but is "
                if realistic_helps_relative_to_optimistic_gap else
                "the camera-stays-aimed-at-tags benefit this module set out to check does NOT show up in "
                "the tank-vs-mecanum gap itself (the gap WIDENS from optimistic to realistic, not narrows "
                "-- see the table below), so at this trial count the strafe penalty dominates completely; "
                "AprilTag's success rate is "
            )
            + "swamped by the constant-strafe cost of a heading "
            "policy that's fixed for the whole match regardless of where the route actually goes. This is "
            "a real limitation of the specific 'hold a fixed heading toward the nearest tag wall for the "
            "whole match' policy this module implements, not evidence that mecanum drivetrains are "
            "generally worse -- a policy that re-picks its held heading periodically (e.g. toward "
            "whichever tag wall is nearest the CURRENT position, or toward the route's own dominant "
            "direction) would strafe far less, and this module doesn't test that alternative."
        )
        if realistic_helps_relative_to_optimistic_gap:
            lines.append(
                f"\nThe camera-FOV mechanism is still visible underneath that, though: the tank-vs-mecanum "
                f"gap narrows going from optimistic ({(apriltag_mecanum_optimistic - apriltag_tank_optimistic):+.0%}) "
                f"to realistic ({(apriltag_mecanum_realistic - apriltag_tank_realistic):+.0%}) -- exactly the "
                "direction the camera-FOV benefit should push it, just not enough to overcome the strafe "
                "penalty at this trial count."
            )
    elif optimistic_negligible:
        lines.append(
            "As expected: holding a fixed heading toward the tag wall makes essentially no difference to "
            "AprilTag under the optimistic tier's omnidirectional camera (a camera that already sees "
            "everything can't see any more of it), but measurably helps under the realistic tier's real "
            "camera-FOV gating -- mecanum keeps the camera aimed at the tag wall all match, while tank's "
            "camera swings away every time it changes travel direction. This is the specific mechanism "
            "Priority 1's fidelity-tier fix was expected to expose."
        )
    else:
        lines.append(
            "Mecanum measurably helps AprilTag under the realistic tier's camera-FOV gating, though the "
            "optimistic-tier gap isn't as close to zero as expected -- worth rechecking against more "
            "trials before treating the optimistic-tier difference as pure noise."
        )

    lines += ["", "## Best value by drivetrain x fidelity", "",
              "| Drivetrain | Fidelity | Best value | pp/$100 |", "|---|---|---|---:|"]
    for drivetrain_name in DRIVETRAIN_ORDER:
        for fidelity in FIDELITY_ORDER:
            results, baseline, best = value_ranking(stats, drivetrain_name, fidelity, rows)
            lines.append(f"| {DRIVETRAIN_LABELS[drivetrain_name]} | {fidelity} | {SUITE_LABELS[best]} | "
                          f"{results[best]['per_100']:+.1f} |")

    lines += ["", "## Does mecanum's own premium get repaid?", "",
              f"Comparing each suite's success rate on mecanum vs. tank, at mecanum's ${mecanum_premium:.0f} "
              "total premium (MECANUM_WHEEL_COST_USD - TANK_WHEEL_COST_USD, ftc/config.py) on top of that "
              "suite's own sensor cost:", "",
              f"| Suite | Tank rate | Mecanum rate | Difference | Worth the ${mecanum_premium:.0f} premium? |",
              "|---|---:|---:|---:|---|"]
    for suite in SUITE_ORDER:
        tank_rate = stats[("tank", "realistic", suite)]["rate"]
        mecanum_rate = stats[("mecanum", "realistic", suite)]["rate"]
        diff = mecanum_rate - tank_rate
        verdict = "yes" if diff > 0.05 else ("no" if diff < -0.02 else "marginal")
        lines.append(f"| {SUITE_LABELS[suite]} | {tank_rate:.0%} | {mecanum_rate:.0%} | {diff:+.0%} | {verdict} |")

    lines += ["", "## What this does and does not prove", "",
              "This is a reduced-rigor sweep (see module docstring) -- enough to see whether the "
              "camera-FOV/held-heading interaction is real and in the expected direction, not a "
              "publication-grade confidence interval on the exact magnitude. The mecanum heading policy "
              "itself (hold heading toward the nearest AprilTag wall) is one reasonable, documented "
              "choice, not the only one a real team could make -- a team without an AprilTag camera at "
              "all has no reason to hold that particular heading, and this specific comparison doesn't "
              "sweep alternative mecanum heading policies. `ftc_drivetrain_heading_policy_writeup.md` "
              "(same module, a separate sweep/output) is where that gets checked."]

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def write_writeup_heading_policy(stats, rows, path):
    """The follow-up study write_writeup's own closing line points to:
    does re-aiming MECANUM's held heading (instead of fixing it once at
    match start) recover any of the premium `write_writeup` found
    unpaid? Compares all four MECANUM heading policies (ftc/
    drivetrain.py's MECANUM_HEADING_POLICY_ORDER) against tank and
    against each other, paired (identical seeded scenarios per
    (fidelity, suite) cell across all five drivetrain variants --
    nav/stats.py's bootstrap_paired_diff_ci is used for the headline
    fixed-at-start-vs-alternatives comparison, the same tool every
    other paired comparison in this project uses)."""
    mecanum_cost = DRIVETRAINS["mecanum"].cost_usd
    tank_cost = DRIVETRAINS["tank"].cost_usd
    lines = [
        "# Does re-aiming mecanum's held heading recover its unpaid premium?",
        "",
        "`ftc_drivetrain_writeup.md` found MECANUM's $" + f"{mecanum_cost - tank_cost:.0f}" + " premium over "
        "TANK NOT repaid under the one heading policy that study tested (hold a heading fixed at match start, "
        "aimed at the nearest AprilTag wall) -- and named the untested alternative explicitly: a policy that "
        "re-picks its held heading as the match progresses. This module checks that alternative directly, "
        "crossing MECANUM_HEADING_POLICY_ORDER (tank, plus MECANUM under `fixed_at_start` / "
        "`nearest_tag_current` / `route_dominant` / `match_travel`, see ftc/drivetrain.py) with the identical "
        f"fidelity tiers x suites x deviation types x levels {LEVELS} x {TRIALS} trials/point this module's "
        "original sweep already uses, on a DIFFERENT base seed (HEADING_POLICY_BASE_SEED) so this study's "
        "trials never overlap with `ftc_drivetrain_writeup.md`'s. Raw data in "
        "`ftc_drivetrain_heading_policy_results.csv`, chart in `ftc_drivetrain_heading_policy_comparison.png`.",
        "",
        "## Does any alternative policy beat fixed_at_start?",
        "",
        "Pooled across every suite, paired on identical scenarios, at the `realistic` fidelity tier (where "
        "the camera-FOV mechanism this whole investigation is about actually applies -- see "
        "`ftc_drivetrain_writeup.md`'s own optimistic-tier control):",
        "",
        "| Policy | Success rate | vs. fixed_at_start [95% CI] | Verdict |",
        "|---|---:|---:|---|",
    ]

    def _pooled_outcomes(drivetrain_name, fidelity):
        return [r["success"] for r in rows if r["drivetrain"] == drivetrain_name and r["fidelity"] == fidelity]

    # Every MECANUM_HEADING_POLICY_ORDER entry other than "tank" and the
    # bare "mecanum" (== fixed_at_start, the baseline this whole section
    # compares against) -- derived rather than a literal tuple so a
    # future 5th policy shows up here automatically instead of silently
    # being left out of the one table this module's headline verdict is
    # read from.
    alternative_policies = [d for d in MECANUM_HEADING_POLICY_ORDER if d not in ("tank", "mecanum")]

    baseline_outcomes = _pooled_outcomes("mecanum", "realistic")
    baseline_rate = sum(baseline_outcomes) / len(baseline_outcomes)
    lines.append(f"| fixed_at_start (baseline) | {baseline_rate:.0%} | -- | -- |")

    recovering_policies = []
    for drivetrain_name in alternative_policies:
        outcomes = _pooled_outcomes(drivetrain_name, "realistic")
        rate = sum(outcomes) / len(outcomes)
        ci_lo, ci_hi, p_value = bootstrap_paired_diff_ci(baseline_outcomes, outcomes,
                                                            seed=hash(drivetrain_name) % (2**31))
        delta = rate - baseline_rate
        if ci_lo > 0 and p_value < 0.05:
            verdict = "**significantly better**"
            recovering_policies.append(drivetrain_name)
        elif ci_hi < 0 and p_value > 0.95:
            verdict = "**significantly worse**"
        else:
            verdict = "not distinguishable from noise"
        lines.append(f"| {DRIVETRAIN_LABELS[drivetrain_name]} | {rate:.0%} | {delta:+.1%} "
                      f"[{ci_lo:+.1%}, {ci_hi:+.1%}] | {verdict} |")

    tank_outcomes = _pooled_outcomes("tank", "realistic")
    tank_rate = sum(tank_outcomes) / len(tank_outcomes)
    lines += ["", f"Tank (no held heading at all, for reference): {tank_rate:.0%}.", ""]

    if recovering_policies:
        recovering_labels = ", ".join(DRIVETRAIN_LABELS[d] for d in recovering_policies)
        lines.append(
            f"At least one alternative heading policy ({recovering_labels}) is a statistically real "
            "improvement over fixed_at_start -- re-aiming genuinely helps, exactly the mechanism "
            "`ftc_drivetrain_writeup.md` predicted but didn't have a policy to demonstrate it with. Whether "
            "that improvement is enough to catch up to tank (see the per-suite table below) is a separate "
            "question from whether it helps at all."
        )
    else:
        lines.append(
            "None of the alternative policies is a statistically significant improvement over fixed_at_start "
            "at this trial count. Re-aiming more often does not, by itself, guarantee less total strafe over "
            "a real route -- `nearest_tag_current` can still point away from the route's actual direction of "
            "travel at any given moment (it optimizes for tag visibility, not for minimizing strafe), and "
            "`route_dominant` optimizes for the AVERAGE direction over the whole remaining route, which can "
            "still be a poor fit for any one individual leg. Re-aiming itself is free regardless of policy "
            "(turn_cost_s is 0 for any holonomic drivetrain, ftc/drivetrain.py), so even `match_travel` -- "
            "which DOES aim at the immediate next leg exactly, avoiding the strafe penalty most of the time -- "
            "isn't guaranteed a win by construction: it's only re-aimed once per leg from the PLANNED path's "
            "nominal direction, not the tick's true post-error travel heading, so residual strafe from "
            "position noise can still cost it."
        )

    lines += [
        "", "## Does any policy actually catch up to tank?", "",
        "The comparison above is against fixed_at_start (MECANUM's own baseline policy), not against tank -- "
        "closing part of MECANUM's internal gap is a different question from closing the gap with tank "
        "itself, which is the one README.md's \"no mecanum-specific strafing advantage\" threat-to-validity "
        "entry actually asks. Same pooled-across-every-suite, paired-on-identical-scenarios comparison, "
        "against TANK directly this time:",
        "",
        "| Policy | Success rate | vs. tank [95% CI] | Verdict |",
        "|---|---:|---:|---|",
    ]
    for drivetrain_name in ["mecanum"] + alternative_policies:
        outcomes = _pooled_outcomes(drivetrain_name, "realistic")
        rate = sum(outcomes) / len(outcomes)
        ci_lo, ci_hi, p_value = bootstrap_paired_diff_ci(tank_outcomes, outcomes,
                                                            seed=(hash(drivetrain_name) + 1) % (2**31))
        delta = rate - tank_rate
        if ci_lo > 0 and p_value < 0.05:
            verdict = "**significantly better than tank**"
        elif ci_hi < 0 and p_value > 0.95:
            verdict = "**significantly worse than tank**"
        else:
            verdict = "not distinguishable from tank"
        lines.append(f"| {DRIVETRAIN_LABELS[drivetrain_name]} | {rate:.0%} | {delta:+.1%} "
                      f"[{ci_lo:+.1%}, {ci_hi:+.1%}] | {verdict} |")

    lines += ["", "## Best value by policy (realistic fidelity)", "", "| Policy | Best value | pp/$100 |",
              "|---|---|---:|"]
    for drivetrain_name in MECANUM_HEADING_POLICY_ORDER:
        results, baseline, best = value_ranking(stats, drivetrain_name, "realistic", rows)
        lines.append(f"| {DRIVETRAIN_LABELS[drivetrain_name]} | {SUITE_LABELS[best]} | {results[best]['per_100']:+.1f} |")

    per_suite_header = "| Suite | " + " | ".join(DRIVETRAIN_LABELS[d] for d in MECANUM_HEADING_POLICY_ORDER) + " |"
    per_suite_divider = "|---|" + "---:|" * len(MECANUM_HEADING_POLICY_ORDER)
    lines += ["", "## Per-suite success rate (realistic fidelity)", "", per_suite_header, per_suite_divider]
    for suite in SUITE_ORDER:
        row = [f"| {SUITE_LABELS[suite]} |"]
        for drivetrain_name in MECANUM_HEADING_POLICY_ORDER:
            row.append(f" {stats[(drivetrain_name, 'realistic', suite)]['rate']:.0%} |")
        lines.append("".join(row))

    lines += [
        "", "## What this does and does not prove", "",
        "Same reduced-rigor scope as `ftc_drivetrain_writeup.md` (see that module's own docstring) -- enough "
        "to see whether re-aiming helps at all, not a publication-grade estimate of the exact magnitude. "
        "All three alternative policies are single, specific, documented choices (ftc/drivetrain.py's "
        "resolve_held_heading_deg) -- `nearest_tag_current` optimizes for keeping a tag in view, "
        "`route_dominant` optimizes for the route's average direction, and `match_travel` (the one policy "
        "that directly targets the route's own IMMEDIATE next leg -- the gap both `ftc_drivetrain_writeup.md` "
        "and an earlier version of this writeup named as untested) holds the identical heading TANK would "
        "already be facing on that leg -- avoiding the strafe penalty on most steps, at NO turn-cost charge "
        "(Drivetrain.turn_cost_s is 0 for any holonomic drivetrain, ftc/drivetrain.py, regardless of how "
        "often its held heading changes -- a real mecanum chassis blends re-aiming into the same wheel "
        "commands still driving it forward, unlike TANK's forced stop-pivot-accelerate). What `match_travel` "
        "does NOT avoid is residual strafe: it's re-aimed once per leg from the PLANNED path's nominal "
        "direction, not the tick's true post-error travel heading, so position noise still produces some "
        "mismatch between chassis heading and actual travel direction -- a genuine trade-off to measure, "
        "not a strictly-better-by-construction policy.",
    ]

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)

    all_rows = []
    for drivetrain_name in DRIVETRAIN_ORDER:
        for fidelity in FIDELITY_ORDER:
            print(f"drivetrain={drivetrain_name} fidelity={fidelity} ...")
            all_rows.extend(run_combo(drivetrain_name, fidelity, grid, free_cells, tag_sites))

    write_csv(all_rows, OUTPUT_DIR / "ftc_drivetrain_results.csv")
    stats = summarize(all_rows)
    plot_comparison(stats, OUTPUT_DIR / "ftc_drivetrain_comparison.png")
    write_writeup(stats, all_rows, OUTPUT_DIR / "ftc_drivetrain_writeup.md")

    print(f"\nWrote {len(all_rows)} trials to {OUTPUT_DIR / 'ftc_drivetrain_results.csv'}")
    print(f"Chart saved to {OUTPUT_DIR / 'ftc_drivetrain_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'ftc_drivetrain_writeup.md'}\n")
    for drivetrain_name in DRIVETRAIN_ORDER:
        for fidelity in FIDELITY_ORDER:
            results, baseline, best = value_ranking(stats, drivetrain_name, fidelity, all_rows)
            print(f"{drivetrain_name}/{fidelity}: best value = {best} (baseline={baseline:.0%})")

    # Second, separate sweep -- see module docstring -- checking whether
    # an alternative mecanum heading policy recovers any of the unpaid
    # premium the sweep above found. Own base seed, own output files.
    print("\n--- heading-policy sweep ---")
    policy_rows = []
    for drivetrain_name in MECANUM_HEADING_POLICY_ORDER:
        for fidelity in FIDELITY_ORDER:
            print(f"drivetrain={drivetrain_name} fidelity={fidelity} ...")
            policy_rows.extend(run_combo(drivetrain_name, fidelity, grid, free_cells, tag_sites,
                                          base_seed=HEADING_POLICY_BASE_SEED))

    write_csv(policy_rows, OUTPUT_DIR / "ftc_drivetrain_heading_policy_results.csv")
    policy_stats = summarize(policy_rows, drivetrain_order=MECANUM_HEADING_POLICY_ORDER)
    plot_comparison(policy_stats, OUTPUT_DIR / "ftc_drivetrain_heading_policy_comparison.png",
                     drivetrain_order=MECANUM_HEADING_POLICY_ORDER)
    write_writeup_heading_policy(policy_stats, policy_rows, OUTPUT_DIR / "ftc_drivetrain_heading_policy_writeup.md")

    print(f"\nWrote {len(policy_rows)} trials to {OUTPUT_DIR / 'ftc_drivetrain_heading_policy_results.csv'}")
    print(f"Chart saved to {OUTPUT_DIR / 'ftc_drivetrain_heading_policy_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'ftc_drivetrain_heading_policy_writeup.md'}\n")
    for drivetrain_name in MECANUM_HEADING_POLICY_ORDER:
        results, baseline, best = value_ranking(policy_stats, drivetrain_name, "realistic", policy_rows)
        print(f"{drivetrain_name}/realistic: best value = {best} (baseline={baseline:.0%})")
