"""
Does nav/kalman.py's variance-aware fusion do any BETTER than ftc/
fusion.py's confidence-weighted fusion (benchmark_results/
ftc_fusion_writeup.md, Section 13 of the project report) at recovering
the AprilTag+odometry bundle's advantage once its own two pose sources
can disagree? ftc/fusion_benchmark.py already answers "does confidence-
weighted fusion survive" (no, it INVERTS the bundle's advantage -- see
that module). This module adds a THIRD fusion condition -- nav/
kalman.py's variance-aware update, run_match(fusion="kalman") -- on the
identical scenarios, and reports the same paired comparison nav/
stats.py's bootstrap_paired_diff_ci already gives every other headline
comparison in this project.

Deliberately a SEPARATE module writing a SEPARATE set of output files
(ftc_fusion_kalman_*) rather than modifying ftc/fusion_benchmark.py's
existing three-condition study in place: ftc_fusion_writeup.md's
existing numbers (63% -> 26%, a real published, cited finding) would
otherwise have every one of its sentences forced to carry a caveat that
has nothing to do with what that file is actually about (see
"IMPORTANT" below). This module reuses ftc/fusion_benchmark.py's own
build_scenarios (identical scenario generation, identical seeds) so a
reader can trust the two studies' "best_single"/"bundle_no_fusion"
numbers line up with each other.

IMPORTANT -- read before trusting the "kalman" column of anything this
module produces: nav/kalman.py's fusion path runs on ftc/fusion.py's
DEFAULT_APRILTAG_VARIANCE_MODEL, which is fit to ftc/calibration.py's
labeled SYNTHETIC PLACEHOLDER detection scatter -- not a real
measurement of any real AprilTag hardware (see ftc/calibration.py's
module docstring for exactly what "synthetic placeholder" means and
why). A result computed from an invented variance is not evidence about
real hardware; it is only evidence about how this project's OWN
placeholder numbers behave once run through a proper Kalman update
instead of a fixed confidence weight. This module's writeup restates
that every time it states a number, not just once at the top.

Writes benchmark_results/ftc_fusion_kalman_results.csv (every trial,
raw), benchmark_results/ftc_fusion_kalman_comparison.png (success rate
per profile under each of the 4 conditions, 95% CI bands), and
benchmark_results/ftc_fusion_kalman_writeup.md.
"""
import csv
import random
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter

from nav.stats import bootstrap_ci, bootstrap_paired_diff_ci

from ftc.bundle import make_bundle
from ftc.calibration import load_calibration
from ftc.fusion_benchmark import BUNDLE_COMPONENTS, BASE_SEED, TRIALS_PER_PROFILE, build_scenarios
from ftc.match import run_match
from ftc.optimizer import DEFAULT_PROFILES
from ftc.sensors import SUITE_LABELS, SUITES

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

CONDITION_ORDER = ["best_single", "bundle_no_fusion", "bundle_confidence", "bundle_kalman"]
CONDITION_LABELS = {
    "best_single": "Best single component (no fusion)",
    "bundle_no_fusion": "AprilTag+odometry bundle (optimistic merge)",
    "bundle_confidence": "AprilTag+odometry bundle (confidence-weighted fusion)",
    "bundle_kalman": "AprilTag+odometry bundle (Kalman fusion, SYNTHETIC variance)",
}
CONDITION_FUSION_KWARG = {"bundle_no_fusion": None, "bundle_confidence": True, "bundle_kalman": "kalman"}
CONDITION_COLORS = {"best_single": "tab:gray", "bundle_no_fusion": "tab:blue",
                     "bundle_confidence": "tab:green", "bundle_kalman": "tab:purple"}

PLACEHOLDER_NOTE = ("nav/kalman.py's fusion path runs on ftc/calibration.py's labeled SYNTHETIC PLACEHOLDER "
                     "AprilTag detection scatter, not real measured hardware data -- see this module's own "
                     "docstring")


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


def _pooled_outcomes(rows, condition):
    profile_names = [p.name for p in DEFAULT_PROFILES]
    return [r for name in profile_names for r in _outcomes(rows, condition, name)]


def plot_comparison(rows, path):
    profile_names = [p.name for p in DEFAULT_PROFILES]
    fig, ax = plt.subplots(figsize=(11, 6))
    x = range(len(profile_names))
    width = 0.2
    offsets_base = (len(CONDITION_ORDER) - 1) / 2
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
        offsets = [xi + (i - offsets_base) * width for xi in x]
        ax.bar(offsets, rates, width=width, label=CONDITION_LABELS[condition], color=CONDITION_COLORS[condition],
               yerr=[ci_los, ci_his], capsize=3)
    ax.set_xticks(list(x))
    ax.set_xticklabels([p.label for p in DEFAULT_PROFILES], rotation=15, ha="right", fontsize=8)
    ax.set_ylabel("Success rate (95% bootstrap CI)")
    ax.yaxis.set_major_formatter(PercentFormatter(xmax=1.0))
    ax.set_ylim(0, 1.05)
    ax.legend(fontsize=7, loc="upper right")
    ax.set_title(f"Optimistic merge vs. confidence-weighted vs. Kalman fusion\n"
                 f"({TRIALS_PER_PROFILE} trials/profile/condition -- Kalman path runs on SYNTHETIC "
                 "placeholder variance, see writeup)", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=150)


def _paired_line(rows, label_a, label_b, condition_a, condition_b, seed):
    outcomes_a = _pooled_outcomes(rows, condition_a)
    outcomes_b = _pooled_outcomes(rows, condition_b)
    rate_a = sum(outcomes_a) / len(outcomes_a)
    rate_b = sum(outcomes_b) / len(outcomes_b)
    ci_lo, ci_hi, p_value = bootstrap_paired_diff_ci(outcomes_a, outcomes_b, seed=seed)
    delta = rate_b - rate_a
    if ci_lo > 0 and p_value < 0.05:
        verdict = f"**{label_b} is significantly better**"
    elif ci_hi < 0 and p_value > 0.95:
        verdict = f"**{label_b} is significantly worse**"
    else:
        verdict = "not statistically distinguishable from noise at this trial count"
    return (f"| {label_a} vs. {label_b} | {rate_a:.0%} | {rate_b:.0%} | {delta:+.1%} "
            f"[{ci_lo:+.1%}, {ci_hi:+.1%}] | {verdict} |")


def write_writeup(rows, best_single_name, path):
    calibration_source = load_calibration().apriltag_variance.source
    lines = [
        "# Optimistic merge vs. confidence-weighted vs. Kalman fusion",
        "",
        f"**{PLACEHOLDER_NOTE} (calibration source = `{calibration_source}`). Every 'kalman' number below "
        "describes how this project's own placeholder behaves under a proper Kalman update -- it is not a "
        "claim about real AprilTag hardware.**",
        "",
        f"{TRIALS_PER_PROFILE} trials x {len(DEFAULT_PROFILES)} scenario profiles (ftc/optimizer.py's "
        "DEFAULT_PROFILES, identical to ftc/fusion_benchmark.py's own scenario generation and seeds) x "
        f"{len(CONDITION_ORDER)} conditions -- the AprilTag+odometry bundle's best single component as a "
        "floor, then the bundle itself under optimistic merging (`fusion=None`), confidence-weighted fusion "
        "(`fusion=True`, ftc/fusion.py), and Kalman fusion (`fusion=\"kalman\"`, nav/kalman.py). Every "
        "condition runs against identical seeded scenarios (paired), so nav/stats.py's "
        "`bootstrap_paired_diff_ci` is used for every headline comparison below rather than eyeballing "
        "whether independent CIs overlap. Raw data in `ftc_fusion_kalman_results.csv`, chart in "
        "`ftc_fusion_kalman_comparison.png`.",
        "",
        "## Headline pairwise comparisons",
        "",
        "| Comparison | Rate A | Rate B | B - A [95% CI] | Verdict |",
        "|---|---:|---:|---:|---|",
        _paired_line(rows, "Optimistic merge", "Confidence-weighted", "bundle_no_fusion", "bundle_confidence",
                      seed=101),
        _paired_line(rows, "Confidence-weighted", "Kalman (synthetic variance)", "bundle_confidence",
                      "bundle_kalman", seed=102),
        _paired_line(rows, "Optimistic merge", "Kalman (synthetic variance)", "bundle_no_fusion", "bundle_kalman",
                      seed=103),
        "",
    ]

    confidence_rate = _rate(rows, "bundle_confidence")
    kalman_rate = _rate(rows, "bundle_kalman")
    no_fusion_rate = _rate(rows, "bundle_no_fusion")
    single_rate = _rate(rows, "best_single")

    if kalman_rate > confidence_rate:
        lines.append(
            f"At this project's synthetic placeholder variance, Kalman fusion ({kalman_rate:.0%}) does better "
            f"than confidence-weighted fusion ({confidence_rate:.0%}) -- see the pairwise row above for "
            "whether that gap is statistically real or noise at this trial count. Even if real, this says "
            "the MATH is doing what it should (a properly gated, variance-aware update recovers more of the "
            "bundle's advantage than a fixed confidence weight) -- it does not mean a real AprilTag+odometry "
            f"bundle would perform this well, since {PLACEHOLDER_NOTE}."
        )
    else:
        lines.append(
            f"Kalman fusion ({kalman_rate:.0%}) does NOT come out ahead of confidence-weighted fusion "
            f"({confidence_rate:.0%}) at this project's synthetic placeholder variance. The likely mechanism: "
            "_apriltag_observation_variance_cells2 (ftc/fusion.py) derives the Kalman observation variance "
            "from `frac` (the same range/angle-degraded correction fraction the confidence-weighted path "
            "already uses as its confidence signal) rather than from real range/incidence geometry -- a "
            "documented approximation (see ftc/fusion.py's own docstring), and one real way this synthetic "
            "comparison could differ from a properly-instrumented one."
        )

    lines += [
        "",
        "## Does any fusion strategy beat the bundle's best single component?",
        "",
        "| Condition | Success rate | Advantage over best single |",
        "|---|---:|---:|",
        f"| {CONDITION_LABELS['best_single']} | {single_rate:.0%} | -- |",
        f"| {CONDITION_LABELS['bundle_no_fusion']} | {no_fusion_rate:.0%} | {no_fusion_rate - single_rate:+.0%} |",
        f"| {CONDITION_LABELS['bundle_confidence']} | {confidence_rate:.0%} | "
        f"{confidence_rate - single_rate:+.0%} |",
        f"| {CONDITION_LABELS['bundle_kalman']} | {kalman_rate:.0%} | {kalman_rate - single_rate:+.0%} |",
        "",
        "## Per-profile breakdown",
        "",
        "| Profile | Best single | Optimistic merge | Confidence-weighted | Kalman (synthetic) |",
        "|---|---:|---:|---:|---:|",
    ]
    for profile in DEFAULT_PROFILES:
        lines.append(
            f"| {profile.label} | {_rate(rows, 'best_single', profile.name):.0%} | "
            f"{_rate(rows, 'bundle_no_fusion', profile.name):.0%} | "
            f"{_rate(rows, 'bundle_confidence', profile.name):.0%} | "
            f"{_rate(rows, 'bundle_kalman', profile.name):.0%} |"
        )

    lines += [
        "",
        "## Honest findings / limitations",
        "",
        f"- {PLACEHOLDER_NOTE}. Pass real detection scatter to "
        "`ftc.calibration.load_calibration(apriltag_csv_path=...)` and its `.apriltag_variance.model` to "
        "`ftc.fusion.fused_tag_correction_kalman` (or `ftc.match.run_match`'s Kalman path, once a caller "
        "threads a model through) to replace this with a real one.",
        "- `_apriltag_observation_variance_cells2` (ftc/fusion.py) derives the Kalman path's observation "
        "variance from `frac` rather than from raw range/incidence -- see that function's own docstring for "
        "exactly what this approximates away.",
        "- Both fusion paths (confidence-weighted and Kalman) share the identical generative observation "
        "model (`_apriltag_observation_value`, ftc/fusion.py) -- this comparison is entirely about the fusion "
        "MATH, not about two different noise assumptions.",
        "- Every constant this comparison inherits from `bundle_confidence` (ODOMETRY_FUSION_CONFIDENCE, "
        "APRILTAG_FUSION_CONFIDENCE, APRILTAG_SYSTEMATIC_BIAS_CELLS, APRILTAG_BAD_DETECTION_PROBABILITY, "
        "APRILTAG_BAD_DETECTION_SIGMA_CELLS, all in ftc/config.py) is the same uncalibrated engineering "
        "estimate `ftc_fusion_writeup.md` already flags -- unaffected by this module's own additions.",
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
        for condition in ("bundle_no_fusion", "bundle_confidence", "bundle_kalman"):
            all_rows.extend(run_candidate(condition, lambda: make_bundle(*BUNDLE_COMPONENTS), profile, scenarios,
                                           CONDITION_FUSION_KWARG[condition]))

    write_csv(all_rows, OUTPUT_DIR / "ftc_fusion_kalman_results.csv")
    plot_comparison(all_rows, OUTPUT_DIR / "ftc_fusion_kalman_comparison.png")
    write_writeup(all_rows, best_single_name, OUTPUT_DIR / "ftc_fusion_kalman_writeup.md")

    print(f"\nWrote {len(all_rows)} trials to {OUTPUT_DIR / 'ftc_fusion_kalman_results.csv'}")
    print(f"Chart saved to {OUTPUT_DIR / 'ftc_fusion_kalman_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'ftc_fusion_kalman_writeup.md'}")
    for condition in CONDITION_ORDER:
        print(f"{CONDITION_LABELS[condition]}: {_rate(all_rows, condition):.0%}")
