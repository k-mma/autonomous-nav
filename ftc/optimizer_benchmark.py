"""
The bundle study: across every buildable combination of ftc/sensors.py's
suites, which robot should a team actually buy -- and is any COMBINATION
of sensors significantly better than the best single sensor, or does the
whole idea of bundling wash out once the comparison is done honestly?

This is the question ftc/suite_benchmark.py structurally can't ask. That
sweep compares five fixed suites, one of which (FullSuite) is a
hand-built bundle of three others, and reports that it wins on raw
success rate but loses on value per dollar. Neither of those tells a
team whether buying two specific sensors together beats buying the best
one alone, whether the answer depends on which failure mode their
division actually produces, or what the best robot under a real budget
is. ftc/optimizer.py answers all three; this module runs it at full
rigor and writes the study.

What's different about the statistics here. Every other comparison in
this project reads two independent bootstrap CIs and checks whether they
overlap. That's the right tool when the trials aren't paired, and a
conservative one when they are -- which, in this project, they always
are: every candidate runs the identical seeded scenarios. This module
uses nav/stats.py's bootstrap_paired_diff_ci instead, resampling trial
indices so trial i's outcome is taken from both candidates or neither.
At these trial counts that routinely turns "the CIs overlap, call it a
wash" into a decisive answer in either direction, and it's what lets the
writeup below use the word "significant" about a specific bundle rather
than gesturing at a bar chart.

Both search strategies are run and compared, deliberately: exhaustive
enumeration finds the true optimum of the space, greedy forward
selection finds what a team reasoning "buy the best thing, then the next
best thing" would land on. Where they disagree is a real finding about
the shape of the problem (a pair of sensors worth buying only together
is invisible to greedy), and where they agree, the cheap search is
enough for anyone extending this to a bigger catalog.

Writes benchmark_results/ftc_optimizer_results.csv (every trial of every
candidate, raw), benchmark_results/ftc_optimizer_frontier.png (cost vs.
success with the Pareto frontier drawn, plus per-scenario winners),
benchmark_results/ftc_optimizer_synergy.png (each top bundle's paired
gain over its own best single component, with 95% CIs), and
benchmark_results/ftc_optimizer_writeup.md.

Run with `--profiles match` to evaluate ftc/optimizer.py's MATCH_PROFILES
catalog instead of the default DEFAULT_PROFILES -- the mixed-axis
scenarios that back the poster's scenario-deepdive figure, as opposed to
the single-axis-isolated ones this module's own writeup and Figure 4
depend on for their capability-attribution argument. That run writes to
ftc_optimizer_match_results.csv / _frontier.png / _synergy.png /
_writeup.md instead, so it never overwrites the default study.
"""
import argparse
import csv
from math import comb
from pathlib import Path
from textwrap import fill

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter

from ftc.config import usd
from ftc.optimizer import (
    BASELINE_SUITE, DEFAULT_COMPONENTS, DEFAULT_PROFILES, MATCH_PROFILES, PROFILES_BY_NAME, BundleOptimizer,
    best_per_profile, best_under_budget, exhaustive_search, greedy_search, pareto_frontier, rank,
    report_synergy,
)

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

# Which scenario catalog a run uses, and the output-file suffix that
# keeps it from clobbering the other one. "default" is DEFAULT_PROFILES
# -- the single-axis-isolated catalog Figure 4 and this module's own
# writeup depend on for their capability-attribution argument -- so it
# keeps the original, un-suffixed filenames; running with no arguments
# must reproduce them byte-identically. "match" is MATCH_PROFILES, the
# mixed-axis catalog that backs the poster's scenario-deepdive figure,
# and writes to its own ftc_optimizer_match_* files instead.
PROFILE_CATALOGS = {"default": DEFAULT_PROFILES, "match": MATCH_PROFILES}

TRIALS_PER_PROFILE = 25
# 3 components is where the space stops growing usefully: with 8
# components, sizes 1-3 already cover every distinct robot the
# overlapping suites can produce up to 5 physical parts (apriltag_imu
# and dual_camera_apriltag each carry two parts of their own), and
# sizes 4+ produce mostly duplicate part signatures the enumerator
# discards anyway. Greedy below is run to a higher max_size precisely
# to check that assumption rather than assume it.
MAX_BUNDLE_SIZE = 3
GREEDY_MAX_SIZE = 6
# Budgets a real team would plausibly have. $50 buys one or two cheap
# parts, $150 is a mid-season upgrade, $300 is roughly one odometry-pod
# set, $500 is effectively unconstrained in this catalog.
BUDGETS = [50.0, 150.0, 300.0, 500.0]
# How many top bundles get a full paired synergy test in the writeup.
SYNERGY_TOP_N = 8


def collect_rows(results, profile_labels):
    """One CSV row per (candidate, profile, trial) -- the raw data every
    number in the writeup is computed from, in the same
    every-trial-is-a-row shape as ftc_suite_results.csv."""
    rows = []
    for result in results:
        for profile_name in result.profile_order:
            outcome = result.per_profile[profile_name]
            for trial in range(outcome.n):
                rows.append({
                    "bundle": result.key,
                    "parts": "|".join(sorted(result.parts)),
                    "parts_label": result.parts_label,
                    "n_components": len(result.component_names),
                    "n_parts": result.part_count,
                    "cost_usd": round(result.cost_usd, 2),
                    "naive_sum_cost_usd": round(result.naive_sum_cost_usd, 2),
                    "capabilities": "|".join(sorted(result.capability_categories())) or "none",
                    "profile": profile_name,
                    "profile_label": profile_labels[profile_name],
                    "trial": trial,
                    "success": outcome.successes[trial],
                    "elapsed_s": outcome.elapsed_s[trial],
                    "collisions": outcome.collisions[trial],
                    "over_budget": outcome.over_budget[trial],
                })
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def plot_frontier(results, frontier, per_profile, path, profiles, trials):
    fig, (ax_scatter, ax_profiles) = plt.subplots(1, 2, figsize=(14, 6))

    singles = [r for r in results if len(r.component_names) == 1]
    bundles = [r for r in results if len(r.component_names) > 1]
    ax_scatter.scatter([r.cost_usd for r in bundles], [r.weighted_success_rate for r in bundles],
                       s=28, color="tab:blue", alpha=0.55, label="Bundles (2+ suites)")
    ax_scatter.scatter([r.cost_usd for r in singles], [r.weighted_success_rate for r in singles],
                       s=60, color="tab:orange", marker="s", label="Single suites", zorder=3)
    ax_scatter.plot([r.cost_usd for r in frontier], [r.weighted_success_rate for r in frontier],
                    "o-", color="tab:green", linewidth=2, markersize=7, label="Pareto frontier", zorder=4)
    # Frontier labels are long ("odometry pods + IMU + front camera +
    # rear camera + 8 ToF sensors"), and the expensive end of the frontier is
    # by construction at the right edge of the plot -- so anything past
    # the midpoint is labeled leftward, or the most interesting points
    # are the ones whose names run off the figure.
    max_cost = max(r.cost_usd for r in results)
    for result in frontier:
        right_half = result.cost_usd > 0.45 * max_cost
        ax_scatter.annotate(result.parts_label, (result.cost_usd, result.weighted_success_rate),
                            textcoords="offset points", xytext=(-7, 7) if right_half else (7, 7),
                            ha="right" if right_half else "left", fontsize=7)
    ax_scatter.set_xlim(-0.04 * max_cost, max_cost * 1.12)
    ax_scatter.set_xlabel("Bundle cost (USD, union of parts -- shared hardware counted once)")
    ax_scatter.set_ylabel("Weighted success rate across scenario profiles")
    ax_scatter.set_title("Every buildable robot: what you pay vs. what you get", fontsize=10)
    ax_scatter.yaxis.set_major_formatter(PercentFormatter(xmax=1.0))
    ax_scatter.legend(loc="lower right", fontsize=8)
    ax_scatter.grid(alpha=0.2)

    profile_names = list(per_profile)
    winners = [per_profile[n] for n in profile_names]
    rates = [per_profile[n].per_profile[n].success_rate for n in profile_names]
    ax_profiles.barh(range(len(profile_names)), rates, color="tab:purple", alpha=0.8)
    ax_profiles.set_yticks(range(len(profile_names)))
    ax_profiles.set_yticklabels([PROFILES_BY_NAME[n].label for n in profile_names], fontsize=8)
    for i, (rate, winner) in enumerate(zip(rates, winners)):
        # Wrapped, not truncated: these names are the answer the panel
        # exists to deliver, and a clipped one ("odometry pods + IMU +
        # front camera") names a different, cheaper robot than the
        # winner.
        ax_profiles.annotate(fill(f"{winner.parts_label} (${usd(winner.cost_usd)})", 42), (0.015, i),
                             va="center", fontsize=7, linespacing=1.3,
                             color="white" if rate > 0.35 else "black")
    ax_profiles.set_xlabel("Winning bundle's success rate on that scenario")
    ax_profiles.set_title("Different scenarios, different best robot", fontsize=10)
    ax_profiles.set_xlim(0, 1.0)
    ax_profiles.xaxis.set_major_formatter(PercentFormatter(xmax=1.0))
    ax_profiles.invert_yaxis()

    fig.suptitle(f"Sensor bundle optimizer ({trials} trials x {len(profiles)} scenario "
                 f"profiles per candidate)", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(path, dpi=150)


def plot_synergy(synergies, path):
    """Each bundle's PAIRED gain over its own best single component,
    with 95% CIs. A bar whose error bar crosses zero is a bundle that
    hasn't been shown to beat just buying the one sensor -- drawn that
    way on purpose, since "the bundle bar is taller" is exactly the
    reasoning this chart exists to prevent."""
    if not synergies:
        return
    fig, ax = plt.subplots(figsize=(10, max(4, 0.55 * len(synergies) + 2)))
    labels = [f"{s.bundle_label}\nvs {s.best_single_label}" for s in synergies]
    deltas = [s.delta for s in synergies]
    lower = [max(0.0, s.delta - s.ci_lo) for s in synergies]
    upper = [max(0.0, s.ci_hi - s.delta) for s in synergies]
    colors = ["tab:green" if s.significant else "tab:gray" for s in synergies]
    ax.barh(range(len(synergies)), deltas, color=colors, alpha=0.85)
    ax.errorbar(deltas, range(len(synergies)), xerr=[lower, upper], fmt="none", ecolor="black",
                elinewidth=1.1, capsize=3)
    ax.axvline(0, color="black", linewidth=0.9)
    ax.set_yticks(range(len(synergies)))
    ax.set_yticklabels(labels, fontsize=7)
    ax.invert_yaxis()
    ax.set_xlabel("Success-rate gain over the bundle's own best single component (paired, 95% CI)")
    ax.xaxis.set_major_formatter(PercentFormatter(xmax=1.0))
    ax.set_title("Does bundling actually help? (green = significant, gray = not distinguishable from noise)",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=150)


def write_writeup(summary_data, path):
    results = summary_data["results"]
    frontier = summary_data["frontier"]
    synergies = summary_data["synergies"]
    per_profile = summary_data["per_profile"]
    greedy_steps = summary_data["greedy_steps"]
    baseline_rate = summary_data["baseline_rate"]
    ranked = summary_data["ranked"]
    robust_ranked = summary_data["robust_ranked"]
    budget_picks = summary_data["budget_picks"]
    profiles = summary_data["profiles"]
    file_stem = summary_data["file_stem"]
    trials = summary_data["trials"]

    best = ranked[0]
    most_robust = robust_ranked[0]
    same_robot = best.parts == most_robust.parts
    best_single = max((r for r in results if len(r.component_names) == 1),
                      key=lambda r: r.weighted_success_rate)

    lines = [
        "# Which BUNDLE of sensors should an FTC team buy?",
        "",
        f"Every buildable combination of {len(DEFAULT_COMPONENTS)} sensor suites (ftc/sensors.py), up to "
        f"{MAX_BUNDLE_SIZE} suites per bundle, composed by ftc/bundle.py and evaluated by ftc/optimizer.py "
        f"over {len(profiles)} scenario profiles x {trials} seeded trials each. "
        f"{summary_data['raw_combinations']} raw combinations collapse to {len(results)} distinct robots "
        "once bundles that buy identical hardware are recognized as the same purchase "
        f"({len(results) * len(profiles) * trials} matches). Raw data in "
        f"`{file_stem}_results.csv`, charts in `{file_stem}_frontier.png` and "
        f"`{file_stem}_synergy.png`.",
        "",
        "Two things make this different from `ftc_suite_writeup.md`'s seven-suite comparison:",
        "",
        "- Bundles are costed over the UNION of their parts, so shared hardware is counted once. AprilTag "
        "+ AprilTag-with-IMU is a $25 robot with one camera, not a $50 robot with two, and the two "
        "descriptions collapse to the same candidate before anything is simulated.",
        "- Every candidate runs the identical seeded scenarios, so comparisons use a PAIRED bootstrap "
        "(nav/stats.py's `bootstrap_paired_diff_ci`) rather than checking whether two independent CIs "
        "overlap. Every \"significant\" below means a paired 95% CI that excludes zero AND a bootstrap "
        "p < 0.05, not a taller bar.",
        "",
        "## The best robot and the most robust robot" + (" are the SAME robot" if same_robot
                                                          else " are not the same robot"),
        "",
        "| Rank | Robot | Cost | Weighted success | Worst scenario | pp/$100 |",
        "|---:|---|---:|---:|---:|---:|",
    ]
    for i, result in enumerate(ranked[:10], start=1):
        value = result.value_per_100(baseline_rate)
        value_str = "--" if value is None else f"{value:+.1f}"
        lines.append(f"| {i} | {result.parts_label} | ${result.cost_usd:.2f} | "
                     f"{result.weighted_success_rate:.0%} | {result.worst_profile_rate:.0%} | {value_str} |")

    # Compared on the worst-case RATE, not on identity: rank() breaks
    # worst_case ties toward the cheaper robot, so the two rankings can
    # name different candidates that are equally robust. Claiming "these
    # are different robots" off a tie would be an artifact of the
    # tiebreaker, not a finding.
    robustness_gap = most_robust.worst_profile_rate - best.worst_profile_rate
    lines += [
        "",
        f"Best average: {best.parts_label} (${best.cost_usd:.2f}, {best.weighted_success_rate:.0%}). "
        f"Most robust -- highest success rate on its OWN WORST scenario, the right objective if you can't "
        f"predict your division: {most_robust.parts_label} (${most_robust.cost_usd:.2f}, "
        f"{most_robust.worst_profile_rate:.0%} worst-case vs. {best.worst_profile_rate:.0%} for the "
        "best-average robot)."
        + (
            " The two objectives pick the literal SAME robot here, not just a tie on worst-case rate -- "
            "there is no best-average-vs-most-robust tradeoff to report in this catalog, and a team should "
            "read that as \"the choice was easy,\" not as a coincidence worth distrusting."
            if same_robot else
            " These are genuinely different robots: averaging across scenarios rewards a bundle that is "
            "excellent at two things and helpless at a third, and a match schedule doesn't let you pick "
            "which one you get."
            if robustness_gap > 0.01 else
            f" Those worst-case rates are the same, so no robustness is being given up by taking the "
            f"best-average robot here -- the two objectives happen to agree on worst-case performance, "
            f"but still name different robots at different prices: ${most_robust.cost_usd:.2f} vs. "
            f"${best.cost_usd:.2f} for {best.weighted_success_rate - most_robust.weighted_success_rate:+.0%} "
            "average success, which is the real choice on offer."
        ),
        "",
        "## Does bundling actually beat buying one sensor?",
        "",
        "For each of the top bundles, a paired comparison against the best SINGLE suite that bundle itself "
        "contains -- the honest form of the question, since a bundle containing one strong sensor should "
        "not be credited with that sensor's performance:",
        "",
        "| Bundle | Best single component | Bundle | Single | Gain | 95% CI (paired) | p | Extra cost | Verdict |",
        "|---|---|---:|---:|---:|---|---:|---:|---|",
    ]
    for s in synergies:
        verdict = "**significant**" if s.significant else ("not significant" if s.delta > 0 else "no gain")
        lines.append(
            f"| {s.bundle_label} | {s.best_single_label} | {s.bundle_rate:.0%} | {s.best_single_rate:.0%} | "
            f"{s.delta:+.1%} | [{s.ci_lo:+.1%}, {s.ci_hi:+.1%}] | {s.p_value:.3f} | "
            f"${s.extra_cost_usd:+.2f} | {verdict} |"
        )

    significant = [s for s in synergies if s.significant]
    lines.append("")
    if significant:
        cheapest_significant = min(significant, key=lambda s: s.extra_cost_usd)
        best_marginal = max((s for s in significant if s.pp_per_extra_100 is not None),
                            key=lambda s: s.pp_per_extra_100, default=None)
        lines.append(
            f"{len(significant)} of the {len(synergies)} bundles tested beat their own best single "
            f"component by a statistically significant margin. The cheapest of those upgrades is "
            f"{cheapest_significant.bundle_label}: +${cheapest_significant.extra_cost_usd:.2f} over "
            f"{cheapest_significant.best_single_label} for {cheapest_significant.delta:+.1%} success rate "
            f"[95% CI {cheapest_significant.ci_lo:+.1%}, {cheapest_significant.ci_hi:+.1%}]."
        )
        if best_marginal is not None:
            lines.append(
                f"\nBy marginal value -- gain per dollar of the UPGRADE, not of the whole robot -- the best "
                f"combination to buy is {best_marginal.bundle_label} at "
                f"{best_marginal.pp_per_extra_100:+.1f}pp per extra $100 over "
                f"{best_marginal.best_single_label} alone."
            )
        # Checked against the data rather than asserted: does every
        # significant bundle actually span more than one capability
        # category, and does it add one its best single component
        # lacked? A tidy mechanism story that the results don't support
        # is worse than no story.
        by_key = {r.key: r for r in results}
        spans = []
        for s in significant:
            bundle_categories = by_key[s.bundle_key].capability_categories()
            single_categories = by_key[s.best_single_key].capability_categories()
            spans.append((len(bundle_categories) > 1, bool(bundle_categories - single_categories)))
        all_multi = all(multi for multi, _ in spans)
        all_added = all(added for _, added in spans)
        if all_multi and all_added:
            lines.append(
                "\nThe mechanism is the one this project's own deviation-type analysis predicts, and it "
                "holds for every significant bundle above without exception: each spans more than one "
                "capability category (pose fixing, obstacle sensing, drift reduction, heading holding) "
                "AND adds a category its best single component did not have. Two sensors that fix the "
                "SAME failure mode mostly don't stack -- the second is correcting an error the first "
                "already removed -- while two that fix DIFFERENT failure modes do, because a match is "
                "lost to whichever deviation the robot has no answer for. That is the part of this "
                "result that should generalize past this specific catalog: buy across categories, not "
                "the two best sensors."
            )
        else:
            lines.append(
                f"\nThe cross-category explanation only partly holds here: "
                f"{sum(1 for m, _ in spans if m)} of {len(spans)} significant bundles span more than one "
                f"capability category, and {sum(1 for _, a in spans if a)} add a category their best "
                "single component lacked. So \"combining across failure modes is what pays\" is "
                "supported but not the whole story in this run -- some gains come from more coverage of "
                "the SAME category (a second camera angle, more ToF cones), which the per-scenario "
                "table is the place to see."
            )
    else:
        lines.append(
            "No bundle in this sweep beat its own best single component by a statistically significant "
            "margin. That is a real (and, for a study built to find bundles, an inconvenient) result: at "
            f"{trials} trials per profile the gains from combining are inside the noise, and a "
            "team would be spending real money on a difference this study cannot demonstrate. Rerun with "
            "more trials before concluding either that the gains are absent or that they're merely "
            "unmeasured here."
        )

    lines += [
        "",
        "## The Pareto frontier: what's worth buying at each price",
        "",
        "Every robot NOT on this list is one you should never buy -- something else is both cheaper and "
        "better.",
        "",
        "| Robot | Cost | Weighted success | Worst scenario | Parts |",
        "|---|---:|---:|---:|---:|",
    ]
    for result in frontier:
        lines.append(f"| {result.parts_label} | ${result.cost_usd:.2f} | {result.weighted_success_rate:.0%} "
                     f"| {result.worst_profile_rate:.0%} | {result.part_count} |")

    lines += ["", "### Best robot at each budget", "",
              "| Budget | Best robot | Cost | Money left over | Weighted success |",
              "|---:|---|---:|---:|---:|"]
    for budget, pick in budget_picks:
        if pick is None:
            lines.append(f"| ${usd(budget)} | nothing in the catalog is this cheap | -- | -- | -- |")
        else:
            lines.append(f"| ${usd(budget)} | {pick.parts_label} | ${pick.cost_usd:.2f} | "
                         f"${budget - pick.cost_usd:.2f} | {pick.weighted_success_rate:.0%} |")
    # A budget whose best pick leaves most of the money unspent is a
    # finding, not a formatting quirk: it means the next rung of the
    # frontier is out of reach and everything in between is a worse buy
    # than something cheaper.
    unspent = [(b, p) for b, p in budget_picks if p is not None and b - p.cost_usd > 0.4 * b]
    if unspent:
        lines += ["", (
            "Note the unspent columns: at "
            + (" and ".join(f"${usd(b)}" for b, _ in unspent) if len(unspent) < 3 else
               ", ".join(f"${usd(b)}" for b, _ in unspent[:-1]) + f" and ${usd(unspent[-1][0])}")
            + f", the best available robot is still {unspent[0][1].parts_label} at "
            f"${unspent[0][1].cost_usd:.2f}. Nothing purchasable in between improves on it -- the next "
            "rung of the frontier is out of reach, and the intermediate options are worse buys than "
            "something cheaper. A team at those budgets should bank the difference (or spend it on "
            "drivetrain/gearing, which ftc/drivetrain_benchmark.py and ftc/gearing_benchmark.py price "
            "separately) rather than stretch to a mid-priced sensor."
        )]

    lines += [
        "",
        "## Different scenarios, different answers",
        "",
        "The single strongest argument against a one-number ranking: the winning robot changes with the "
        "kind of match you expect.",
        "",
        "| Scenario | Best robot | Its success rate | Cost |",
        "|---|---|---:|---:|",
    ]
    for name, winner in per_profile.items():
        lines.append(f"| {PROFILES_BY_NAME[name].label} | {winner.parts_label} | "
                     f"{winner.per_profile[name].success_rate:.0%} | ${winner.cost_usd:.2f} |")
    distinct_winners = {w.key for w in per_profile.values()}
    lines += ["", (
        f"{len(distinct_winners)} different robots win at least one scenario out of "
        f"{len(per_profile)}. A team that knows its own dominant failure mode -- which is exactly what "
        "ftc/calibration.py's measured field/odometry CSVs are for -- can buy a cheaper robot than the "
        "overall ranking suggests and do better in the matches it actually plays."
        if len(distinct_winners) > 1 else
        f"The same robot ({next(iter(per_profile.values())).parts_label}) wins every scenario in this "
        "sweep -- an unusually clean result, and one that makes the scenario-weighted ranking above safe "
        "to read as a straight recommendation."
    )]

    lines += [
        "",
        "## Exhaustive vs. greedy search",
        "",
        "Greedy forward selection (start from the best single sensor, keep adding whichever one improves "
        "the objective most) is O(N^2) evaluations instead of 2^N, and its steps double as the marginal "
        "value of each sensor added:",
        "",
        "| Step | Added | Robot after adding | Cost | Weighted success | Marginal gain | 95% CI (paired) | p | Significant |",
        "|---:|---|---|---:|---:|---:|---|---:|---|",
    ]
    for step in greedy_steps:
        if step.size == 1:
            lines.append(f"| 1 | (best single, the starting point) | {step.bundle.parts_label} | "
                         f"${step.bundle.cost_usd:.2f} | {step.bundle.weighted_success_rate:.0%} | -- | -- "
                         "| -- | -- |")
            continue
        lines.append(
            f"| {step.size} | {step.added_label} | {step.bundle.parts_label} | ${step.bundle.cost_usd:.2f} "
            f"| {step.bundle.weighted_success_rate:.0%} | {step.delta:+.1%} | "
            f"[{step.ci_lo:+.1%}, {step.ci_hi:+.1%}] | {step.p_value:.3f} | "
            f"{'yes' if step.significant else 'no'} |"
        )

    greedy_best = greedy_steps[-1].bundle
    agree = greedy_best.parts == best.parts
    lines += ["", (
        f"Greedy lands on the same robot as exhaustive search ({best.parts_label}), so for this catalog "
        "the cheap search is sufficient -- worth knowing for anyone extending the parts list past the "
        "point where 2^N enumeration is affordable."
        if agree else
        f"Greedy lands on {greedy_best.parts_label} ({greedy_best.weighted_success_rate:.0%}), NOT the "
        f"exhaustive optimum {best.parts_label} ({best.weighted_success_rate:.0%}). That gap is the "
        "known blind spot of forward selection and the reason both are reported: a pair of sensors that "
        "only pays off together is never picked up one at a time, because neither one improves the "
        "objective on its own. It is also a direct argument that bundle choice is not decomposable into "
        "\"rank the sensors, buy the top k\"."
    ), ""]

    first_insignificant = next((s for s in greedy_steps if s.size > 1 and not s.significant), None)
    if first_insignificant is not None:
        lines.append(
            f"The greedy path stops paying at step {first_insignificant.size}: adding "
            f"{first_insignificant.added_label} costs ${first_insignificant.extra_cost_usd:.2f} for "
            f"{first_insignificant.delta:+.1%} (95% CI [{first_insignificant.ci_lo:+.1%}, "
            f"{first_insignificant.ci_hi:+.1%}], p={first_insignificant.p_value:.3f}) -- a gain this "
            "sweep cannot distinguish from noise. `--require-significant` makes ftc/optimizer.py's greedy "
            "search stop there rather than keep spending, which is the more honest stopping rule than "
            "\"stop when the mean stops going up.\""
        )

    lines += [
        "",
        "## Honest findings",
        "",
        f"- The best single suite ({best_single.label} = {best_single.parts_label}, "
        f"${best_single.cost_usd:.2f}) reaches "
        f"{best_single.weighted_success_rate:.0%} against the best bundle's "
        f"{best.weighted_success_rate:.0%} at ${best.cost_usd:.2f}. Bundling buys "
        f"{best.weighted_success_rate - best_single.weighted_success_rate:+.0%} for "
        f"${best.cost_usd - best_single.cost_usd:+.2f} -- read the per-dollar column before reading that "
        "as an endorsement.",
        "- Success rates here are lower across the board than `ftc_suite_writeup.md`'s, and that is "
        "expected, not a discrepancy: these profiles combine deviation axes"
        + (" and include a realistic-fidelity one, where the headline sweep isolates a single axis at "
           "the optimistic tier."
           if any(p.fidelity == "realistic" for p in profiles) else
           ", where the headline sweep isolates a single axis at the optimistic tier.")
        + " The two studies' per-axis numbers agree where they overlap.",
        "- Every dollar figure here is hardware only. A bundle's `parts` count is the closest this project "
        "gets to pricing integration effort, and it is not a dollar figure: a 5-part robot is five wiring "
        "harnesses, five failure modes, and five things to debug at 1am before a competition. ImuSuite "
        "costs $0 and is not free.",
        "- The composition model is exact where it can be checked and approximate where it can't. A "
        "bundle reproduces the suites it's built from tick-for-tick (`ftc/scratch/bundle_test.py` fails "
        "otherwise, including the three-component bundle that must equal FullSuite), but combining two "
        "pose-fixing suites uses the union of their camera mounts on one detection pipeline rather than "
        "modeling two independent pipelines that could disagree with each other. Sensor FUSION conflict "
        "-- two sensors reporting different poses -- is not modeled at all here.",
    ]

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profiles", choices=sorted(PROFILE_CATALOGS), default="default",
                        help="Which scenario catalog to run at full rigor: 'default' (ftc/optimizer.py's "
                             "DEFAULT_PROFILES, single-axis isolated -- what Figure 4 and this module's own "
                             "writeup are built on) or 'match' (MATCH_PROFILES, mixed-axis and match-"
                             "realistic -- what the poster's scenario-deepdive figure reads). Running with "
                             "no arguments reproduces the original default-catalog outputs byte-for-byte; "
                             "'match' writes to separately-named files instead of overwriting them.")
    parser.add_argument("--trials", type=int, default=TRIALS_PER_PROFILE,
                        help=f"Trials per scenario profile (default {TRIALS_PER_PROFILE}, matching the "
                             "default catalog's rigor). Raising this only shrinks per-scenario sampling "
                             "noise -- it doesn't change methodology -- so it's most useful on a 'match' "
                             "run, where a 25-trial draw can leave one or two scenarios tied by chance "
                             "even when the underlying gap is real (checked at higher trial counts in "
                             "ftc/optimizer.py's MATCH_PROFILES comment). Left at the default, a 'default' "
                             "run still reproduces the original outputs byte-for-byte.")
    args = parser.parse_args()

    profiles = PROFILE_CATALOGS[args.profiles]
    # Only the match run gets a suffix, so the original, un-suffixed
    # filenames -- and everything downstream of them (Figure 4, ftc_
    # optimizer_writeup.md) -- stay untouched by this flag's existence.
    file_stem = "ftc_optimizer" if args.profiles == "default" else "ftc_optimizer_match"

    OUTPUT_DIR.mkdir(exist_ok=True)

    profile_labels = {p.name: p.label for p in profiles}
    optimizer = BundleOptimizer(profiles=profiles, trials_per_profile=args.trials,
                                on_progress=lambda b: print(f"  evaluating {b.parts_label()} "
                                                            f"(${b.cost_usd:.2f}) ..."))

    print(f"Scenario catalog: {args.profiles} ({', '.join(p.label for p in profiles)}), "
          f"{args.trials} trials/profile")
    print(f"Exhaustive search over {len(DEFAULT_COMPONENTS)} components, up to {MAX_BUNDLE_SIZE} per bundle:")
    results = exhaustive_search(optimizer, DEFAULT_COMPONENTS, min_size=1, max_size=MAX_BUNDLE_SIZE)
    raw_combinations = sum(comb(len(DEFAULT_COMPONENTS), k) for k in range(1, MAX_BUNDLE_SIZE + 1))

    print(f"\nGreedy forward selection (max {GREEDY_MAX_SIZE} components):")
    greedy_steps = greedy_search(optimizer, DEFAULT_COMPONENTS, max_size=GREEDY_MAX_SIZE)
    # Greedy can reach bundles larger than MAX_BUNDLE_SIZE, which the
    # exhaustive pass never enumerated -- fold them in so the ranking,
    # frontier and CSV cover every robot this study actually measured.
    results = list(optimizer._cache.values())

    baseline_rate = optimizer.evaluate(BASELINE_SUITE).weighted_success_rate
    ranked = rank(results, objective="weighted")
    robust_ranked = rank(results, objective="worst_case")
    frontier = pareto_frontier(results)
    per_profile = best_per_profile(results)
    budget_picks = [(b, best_under_budget(results, b)) for b in BUDGETS]

    bundles = [r for r in ranked if len(r.component_names) > 1]
    synergies = []
    for i, bundle_result in enumerate(bundles[:SYNERGY_TOP_N]):
        report = report_synergy(bundle_result, results, seed=i)
        if report:
            synergies.append(report)

    rows = collect_rows(results, profile_labels)
    write_csv(rows, OUTPUT_DIR / f"{file_stem}_results.csv")
    plot_frontier(results, frontier, per_profile, OUTPUT_DIR / f"{file_stem}_frontier.png", profiles,
                 args.trials)
    plot_synergy(synergies, OUTPUT_DIR / f"{file_stem}_synergy.png")
    write_writeup({
        "results": results, "frontier": frontier, "synergies": synergies, "per_profile": per_profile,
        "greedy_steps": greedy_steps, "baseline_rate": baseline_rate, "ranked": ranked,
        "robust_ranked": robust_ranked, "budget_picks": budget_picks,
        "raw_combinations": raw_combinations, "profiles": profiles, "file_stem": file_stem,
        "trials": args.trials,
    }, OUTPUT_DIR / f"{file_stem}_writeup.md")

    print(f"\nWrote {len(rows)} trials ({len(results)} distinct robots) to "
          f"{OUTPUT_DIR / (file_stem + '_results.csv')}")
    print(f"Charts saved to {OUTPUT_DIR / (file_stem + '_frontier.png')} and "
          f"{OUTPUT_DIR / (file_stem + '_synergy.png')}")
    print(f"Writeup saved to {OUTPUT_DIR / (file_stem + '_writeup.md')}\n")
    print(f"Best average:   {ranked[0].parts_label} (${ranked[0].cost_usd:.2f}, "
          f"{ranked[0].weighted_success_rate:.0%})")
    print(f"Most robust:    {robust_ranked[0].parts_label} (${robust_ranked[0].cost_usd:.2f}, "
          f"worst-case {robust_ranked[0].worst_profile_rate:.0%})")
    print(f"Significant synergies: {sum(1 for s in synergies if s.significant)} of {len(synergies)} tested")


if __name__ == "__main__":
    main()
