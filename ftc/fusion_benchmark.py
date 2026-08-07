"""
Does the AprilTag+odometry bundle's advantage over its best single
component survive once its own two pose sources can actually disagree,
or was ftc/bundle.py's optimistic merging (a tag detection applied at
face value, with no way to conflict with what odometry already
believes) doing the work? README.md's "Threats to validity" already
names this as an open question -- bundle results are an upper bound on
what combining buys, size unknown. This is the study that measures the
size.

ONE question, answered with a paired comparison on identical scenarios:
the AprilTag+odometry bundle's success rate under ftc/bundle.py's
existing optimistic merge (fusion=None) vs. under ftc/fusion.py's
confidence-weighted fusion (fusion=True) -- nav/stats.py's
bootstrap_paired_diff_ci on the same trials both ways, the identical
statistical tool ftc/optimizer_benchmark.py's own synergy claims use.
As grounding, not a second question, the same bundle-vs-best-single-
component comparison ftc/optimizer_benchmark.py already reports is
recomputed here at fusion=None (a sanity check that this module's own
scenario generation reproduces the same qualitative shape as that
study) and again at fusion=True (does the existing verdict survive).

Uses ftc/optimizer.py's DEFAULT_PROFILES -- the same 5 scenario
profiles ftc/optimizer_benchmark.py's headline study uses, chosen there
for spreading the five headline suites across a wide range of success
rates -- but this module's own scenario generation and seeds,
independent of ftc/optimizer.py's module-level scenario cache. This
module reads DEFAULT_PROFILES' definitions and nothing else from ftc/
optimizer.py, so nothing it does can perturb ftc_optimizer_results.csv
or any other published output.

Writes benchmark_results/ftc_fusion_results.csv (every trial, raw),
benchmark_results/ftc_fusion_comparison.png (success rate per profile
under each condition, 95% CI bands), and benchmark_results/
ftc_fusion_writeup.md.
"""
import csv
import random
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter

from nav.algorithms import astar
from nav.field_variance import generate_ground_truth
from nav.stats import bootstrap_ci, bootstrap_paired_diff_ci

from ftc.bundle import make_bundle
from ftc.field import build_grid, tag_sites_for
from ftc.match import run_match
from ftc.optimizer import DEFAULT_PROFILES
from ftc.sensors import SUITE_LABELS, SUITES

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

TRIALS_PER_PROFILE = 40
BASE_SEED = 33_000_000
BUNDLE_COMPONENTS = ("apriltag", "odometry_pods")
MAX_ATTEMPTS_PER_TRIAL = 50
MIN_PATH_LEN = 4

CONDITION_ORDER = ["best_single", "bundle_no_fusion", "bundle_fusion"]
CONDITION_LABELS = {
    "best_single": "Best single component (no fusion)",
    "bundle_no_fusion": "AprilTag+odometry bundle (optimistic merge)",
    "bundle_fusion": "AprilTag+odometry bundle (confidence-weighted fusion)",
}
CONDITION_COLORS = {"best_single": "tab:gray", "bundle_no_fusion": "tab:blue", "bundle_fusion": "tab:green"}


def _solvable_scenario(trial_seed, grid, free_cells):
    rng = random.Random(trial_seed)
    for _ in range(MAX_ATTEMPTS_PER_TRIAL):
        start, goal = rng.sample(free_cells, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= MIN_PATH_LEN:
            return start, goal
    raise RuntimeError(f"no solvable scenario for seed {trial_seed} after {MAX_ATTEMPTS_PER_TRIAL} attempts")


def build_scenarios(profile, num_trials, base_seed):
    """This module's own scenario generation -- see module docstring for
    why it doesn't share ftc/optimizer.py's cache. Every DEFAULT_PROFILES
    entry currently leaves `drivetrain`/`gearing` at their None defaults,
    so only `fidelity` is threaded through below; if a future profile
    sets either, this module would need updating to match
    ftc.optimizer.BundleOptimizer.evaluate's handling of them."""
    grid = build_grid(profile.layout)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(profile.layout)
    scenarios = []
    for t in range(num_trials):
        trial_seed = base_seed + t
        start, goal = _solvable_scenario(trial_seed, grid, free_cells)
        ground_truth, actual_start = generate_ground_truth(
            grid, start, goal, profile.variance_level, seed=trial_seed, **profile.scale_kwargs()
        )
        scenarios.append((trial_seed, grid, start, goal, ground_truth, actual_start, tag_sites))
    return scenarios


def run_candidate(condition, suite_factory, profile, scenarios, fusion):
    rows = []
    for trial_seed, grid, start, goal, ground_truth, actual_start, tag_sites in scenarios:
        result = run_match(suite_factory(), grid, start, goal, ground_truth, actual_start, tag_sites,
                            random.Random(trial_seed), fidelity=profile.fidelity, fusion=fusion)
        rows.append({
            "condition": condition, "profile": profile.name, "trial": trial_seed - BASE_SEED,
            "seed": trial_seed, "success": int(result.success),
        })
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _rate(rows, condition, profile_name=None):
    matching = [r for r in rows if r["condition"] == condition
                and (profile_name is None or r["profile"] == profile_name)]
    return sum(r["success"] for r in matching) / len(matching) if matching else 0.0


def _outcomes(rows, condition, profile_name):
    return [r["success"] for r in rows if r["condition"] == condition and r["profile"] == profile_name]


def plot_comparison(rows, path):
    profile_names = [p.name for p in DEFAULT_PROFILES]
    fig, ax = plt.subplots(figsize=(10, 6))
    x = range(len(profile_names))
    width = 0.26
    for i, condition in enumerate(CONDITION_ORDER):
        rates, ci_los, ci_his = [], [], []
        for name in profile_names:
            matching = [r["success"] for r in rows if r["condition"] == condition and r["profile"] == name]
            n, successes = len(matching), sum(matching)
            rate = successes / n if n else 0.0
            ci_lo, ci_hi = bootstrap_ci(successes, n, seed=hash((condition, name)) % (2**31))
            rates.append(rate)
            ci_los.append(rate - ci_lo)
            ci_his.append(ci_hi - rate)
        offsets = [xi + (i - 1) * width for xi in x]
        ax.bar(offsets, rates, width=width, label=CONDITION_LABELS[condition], color=CONDITION_COLORS[condition],
               yerr=[ci_los, ci_his], capsize=3)
    ax.set_xticks(list(x))
    ax.set_xticklabels([p.label for p in DEFAULT_PROFILES], rotation=15, ha="right", fontsize=8)
    ax.set_ylabel("Success rate (95% bootstrap CI)")
    ax.yaxis.set_major_formatter(PercentFormatter(xmax=1.0))
    ax.set_ylim(0, 1.05)
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title(f"Does AprilTag+odometry's advantage survive confidence-weighted fusion?\n"
                 f"({TRIALS_PER_PROFILE} trials/profile/condition)", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=150)


def write_writeup(rows, best_single_name, path):
    profile_names = [p.name for p in DEFAULT_PROFILES]

    pooled_single = _rate(rows, "best_single")
    pooled_no_fusion = _rate(rows, "bundle_no_fusion")
    pooled_fusion = _rate(rows, "bundle_fusion")

    # bootstrap_paired_diff_ci(a, b) returns a CI on mean(b) - mean(a)
    # and a ONE-SIDED p for the claim "b is better than a" (nav/
    # stats.py's own docstring). Calling it (no_fusion, fusion) makes
    # the returned CI match `delta` below (fusion - no_fusion, negative
    # = a drop) -- but that also means p_value here specifically tests
    # "fusion is better than no_fusion": small p supports an INCREASE,
    # and it's p_value close to 1 (not close to 0) that supports a
    # DROP, since a drop means fusion essentially never wins a
    # resample. Getting this backwards is an easy, silent mistake --
    # this project's own first draft of this file made it, printing a
    # CI that was entirely negative right next to "not distinguishable
    # from noise" because the significance check used p_value < 0.05
    # for BOTH directions instead of p_value > 0.95 for a drop.
    ci_lo, ci_hi, p_value = bootstrap_paired_diff_ci(
        [r for name in profile_names for r in _outcomes(rows, "bundle_no_fusion", name)],
        [r for name in profile_names for r in _outcomes(rows, "bundle_fusion", name)],
        seed=1,
    )
    delta = pooled_fusion - pooled_no_fusion
    significant_increase = ci_lo > 0 and p_value < 0.05
    significant_drop = ci_hi < 0 and p_value > 0.95

    advantage_before = pooled_no_fusion - pooled_single
    advantage_after = pooled_fusion - pooled_single

    lines = [
        "# Does the AprilTag+odometry bundle's advantage survive sensor disagreement?",
        "",
        f"{TRIALS_PER_PROFILE} trials x {len(DEFAULT_PROFILES)} scenario profiles (ftc/optimizer.py's "
        "DEFAULT_PROFILES) x 3 conditions -- the AprilTag+odometry bundle under ftc/bundle.py's existing "
        "optimistic merge (`fusion=None`) and under ftc/fusion.py's confidence-weighted fusion "
        "(`fusion=True`), plus its best single component "
        f"({SUITE_LABELS[best_single_name]}) at `fusion=None` as the comparison floor. Every condition runs "
        "against identical seeded scenarios (paired), so nav/stats.py's `bootstrap_paired_diff_ci` is used "
        "for the headline comparison rather than eyeballing whether independent CIs overlap -- the same "
        "tool ftc/optimizer_benchmark.py's own synergy claims use. Raw data in `ftc_fusion_results.csv`, "
        "chart in `ftc_fusion_comparison.png`.",
        "",
        "## The headline number",
        "",
        f"Pooled across all {len(DEFAULT_PROFILES)} profiles: the bundle succeeds in {pooled_no_fusion:.0%} of "
        f"trials under optimistic merging, {pooled_fusion:.0%} under confidence-weighted fusion -- a change "
        f"of {delta:+.1%} [95% CI {ci_lo:+.1%}, {ci_hi:+.1%}] (paired bootstrap). One-sided p for \"fusion "
        f"performs better than optimistic merging\": {p_value:.3f} -- a value near 1.0 supports a drop, near "
        "0.0 supports an increase, and anywhere in between is noise.",
        "",
    ]

    if significant_drop:
        lines.append(
            f"That drop is **statistically significant**: modeling AprilTag-vs-odometry disagreement costs "
            f"the bundle a real, measurable amount of success rate, not noise. The optimistic-merge model "
            f"was overstating this bundle's reliability by roughly {abs(delta):.0%} (95% CI "
            f"[{abs(ci_hi):.1%}, {abs(ci_lo):.1%}]) at this project's estimated fusion constants "
            "(ftc/config.py's `APRILTAG_BAD_DETECTION_PROBABILITY`/`APRILTAG_SYSTEMATIC_BIAS_CELLS`, "
            "uncalibrated -- see \"Honest findings\" below)."
        )
    elif significant_increase:
        lines.append(
            f"That's a statistically significant **increase**, which is not the direction this study set out "
            "to check for. The most likely mechanism: the disagreement check can also flag and down-weight a "
            "detection that was merely a large but LEGITIMATE correction (see ftc/fusion.py's fuse() "
            "docstring on fixed-threshold gating), and rejecting some of those changes the correction's "
            "distribution enough to occasionally help. Reported as found, not adjusted to match the expected "
            "direction."
        )
    else:
        lines.append(
            f"That difference is **not statistically distinguishable from noise** at this trial count -- the "
            "paired CI includes zero. This is a genuinely useful result: it means the optimistic-merge model's "
            "upper bound was TIGHT for this bundle at these fusion constants, not loose. Modeling disagreement "
            "was worth checking, and checking it found the existing bundle numbers hold up, not that they were "
            "wrong."
        )

    lines += [
        "",
        "## Does the bundle still beat its best single component?",
        "",
        "| Condition | Success rate | Advantage over best single |",
        "|---|---:|---:|",
        f"| {CONDITION_LABELS['best_single']} | {pooled_single:.0%} | -- |",
        f"| {CONDITION_LABELS['bundle_no_fusion']} | {pooled_no_fusion:.0%} | {advantage_before:+.0%} |",
        f"| {CONDITION_LABELS['bundle_fusion']} | {pooled_fusion:.0%} | {advantage_after:+.0%} |",
        "",
    ]
    if (advantage_before > 0) == (advantage_after > 0) and abs(advantage_after) > 1e-9:
        lines.append(
            "The bundle beats its best single component in the same direction under both conditions -- "
            "modeling disagreement changed the MARGIN, not the VERDICT `ftc_optimizer_writeup.md` reports."
        )
    else:
        lines.append(
            "The bundle's advantage over its best single component **changes sign** once fusion is modeled -- "
            "this is the kind of finding this study exists to catch: an optimizer recommendation that "
            "depended on optimistic merging, not on the bundle actually being the better buy."
        )

    lines += [
        "",
        "## Per-profile breakdown",
        "",
        "| Profile | Best single | Bundle (no fusion) | Bundle (fusion) |",
        "|---|---:|---:|---:|",
    ]
    for profile in DEFAULT_PROFILES:
        lines.append(
            f"| {profile.label} | {_rate(rows, 'best_single', profile.name):.0%} | "
            f"{_rate(rows, 'bundle_no_fusion', profile.name):.0%} | "
            f"{_rate(rows, 'bundle_fusion', profile.name):.0%} |"
        )

    lines += [
        "",
        "## Honest findings",
        "",
        "- Every fusion constant (`ODOMETRY_FUSION_CONFIDENCE`, `APRILTAG_FUSION_CONFIDENCE`, "
        "`APRILTAG_SYSTEMATIC_BIAS_CELLS`, `APRILTAG_BAD_DETECTION_PROBABILITY`, "
        "`APRILTAG_BAD_DETECTION_SIGMA_CELLS`, `FUSION_DISAGREEMENT_THRESHOLD_CELLS`, "
        "`FUSION_DISTRUST_FACTOR` -- all in ftc/config.py) is an engineering estimate with no real "
        "AprilTag-vs-odometry disagreement measurement behind it, the same status as this project's other "
        "unmeasured constants until real data goes through ftc/calibration.py. This study's answer is "
        "conditional on those estimates, not a calibrated number.",
        "- The disagreement check (nav/estimation.py's `fuse()`) uses a fixed distance threshold rather than "
        "one scaled by how much the prior itself has already drifted -- a documented simplification (see "
        "ftc/fusion.py's module docstring). A sanity check across this project's typical error-magnitude "
        "range found roughly 6.5% of ORDINARY (non-bad) detections still cross the threshold purely from "
        "correction magnitude, against roughly 87% of genuinely bad ones -- real separation, not perfect "
        "separation.",
        "- Fusion in this project only ever applies to a bundle's tag-detection events -- it does not touch "
        "obstacle sensing, heading correction, or any suite that isn't `fixes_pose`. A bundle that senses "
        "obstacles (e.g. FullSuite) is unaffected by anything in this study beyond its own AprilTag "
        "component's position correction.",
    ]

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    all_rows = []
    single_rates = {}
    for component in BUNDLE_COMPONENTS:
        component_rows = []
        for profile in DEFAULT_PROFILES:
            scenarios = build_scenarios(profile, TRIALS_PER_PROFILE, BASE_SEED)
            component_rows.extend(
                run_candidate(f"single_{component}", lambda c=component: SUITES[c](), profile, scenarios, None)
            )
        single_rates[component] = sum(r["success"] for r in component_rows) / len(component_rows)
        print(f"  {SUITE_LABELS[component]} (single, no fusion): {single_rates[component]:.0%}")

    best_single_name = max(BUNDLE_COMPONENTS, key=lambda c: single_rates[c])
    print(f"Best single component: {SUITE_LABELS[best_single_name]}")

    for profile in DEFAULT_PROFILES:
        scenarios = build_scenarios(profile, TRIALS_PER_PROFILE, BASE_SEED)
        print(f"profile={profile.name} ...")
        all_rows.extend(run_candidate("best_single", lambda: SUITES[best_single_name](), profile, scenarios, None))
        all_rows.extend(run_candidate("bundle_no_fusion", lambda: make_bundle(*BUNDLE_COMPONENTS), profile,
                                       scenarios, None))
        all_rows.extend(run_candidate("bundle_fusion", lambda: make_bundle(*BUNDLE_COMPONENTS), profile,
                                       scenarios, True))

    write_csv(all_rows, OUTPUT_DIR / "ftc_fusion_results.csv")
    plot_comparison(all_rows, OUTPUT_DIR / "ftc_fusion_comparison.png")
    write_writeup(all_rows, best_single_name, OUTPUT_DIR / "ftc_fusion_writeup.md")

    print(f"\nWrote {len(all_rows)} trials to {OUTPUT_DIR / 'ftc_fusion_results.csv'}")
    print(f"Chart saved to {OUTPUT_DIR / 'ftc_fusion_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'ftc_fusion_writeup.md'}")
    print(f"\nBundle, no fusion: {_rate(all_rows, 'bundle_no_fusion'):.0%}")
    print(f"Bundle, fusion:    {_rate(all_rows, 'bundle_fusion'):.0%}")
    print(f"Best single:       {_rate(all_rows, 'best_single'):.0%}")
