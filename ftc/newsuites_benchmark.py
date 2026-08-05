"""
The two new suites Priority 1's fidelity-tier fix enables:
ImuSuite ($0 -- every REV Control Hub already ships one, ftc/config.py's
IMU_COST_USD) and DualCameraAprilTagSuite (~$80, front + rear camera).
Also includes AprilTagImuSuite (AprilTag + IMU stacked) as the
IMU-augmented variant of an existing suite the task brief asks for.

ImuSuite is the one upgrade in this whole project where the question is
"is the free hardware worth the code," which the success-rate-gain-
per-$100 metric every other study in this repo uses CANNOT express at
cost_usd == 0.0 -- this module reports that case explicitly as
"undefined," not as "infinite" (an earlier, less careful framing would
have divided by zero or silently called $0 "infinitely good value,"
which isn't a meaningful claim about integration effort).

DualCameraAprilTagSuite only does anything different from AprilTagSuite
under Priority 1's camera-FOV gating -- under the old omnidirectional
model a second camera would add literally nothing, since an
omnidirectional camera already sees everything the first one did. This
module crosses both new suites with fidelity tier (optimistic vs.
realistic) specifically to demonstrate that difference directly, the
same "crossed with fidelity" structure ftc/drivetrain_benchmark.py uses
for the same reason.

Reduced trial count/level set relative to the headline sweep (see ftc/
robustness.py's own docstring for the reasoning).

Writes benchmark_results/ftc_newsuites_results.csv (every trial, raw,
with an added `fidelity` column), benchmark_results/
ftc_newsuites_comparison.png (success rate by suite, one panel per
tier), and benchmark_results/ftc_newsuites_writeup.md.
"""
import csv
import random
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt

from nav.field_variance import generate_ground_truth
from nav.stats import bootstrap_ci

from ftc.field import build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import SUITES, SUITE_LABELS
from ftc.suite_benchmark import DEVIATION_TYPES, DEVIATION_TYPE_ORDER, LAYOUT, _solvable_scenario

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

# dead_reckoning is the $0 baseline every value ranking is measured
# against; apriltag is the existing headline best-value suite included
# for direct comparison; the rest are this module's own additions.
SUITE_ORDER_LOCAL = ["dead_reckoning", "apriltag", "imu", "apriltag_imu", "dual_camera_apriltag"]
SUITE_COLORS_LOCAL = {
    "dead_reckoning": "tab:red", "apriltag": "tab:purple", "imu": "tab:cyan",
    "apriltag_imu": "tab:olive", "dual_camera_apriltag": "tab:brown",
}
FIDELITY_ORDER = ["optimistic", "realistic"]
LEVELS = [0.3, 0.5, 0.7, 0.9]
TRIALS = 15
BASE_SEED = 14_000_000


def run_combo(fidelity, grid, free_cells, tag_sites):
    rows = []
    for deviation_type in DEVIATION_TYPE_ORDER:
        scale_kwargs = DEVIATION_TYPES[deviation_type]
        for level in LEVELS:
            for t in range(TRIALS):
                trial_seed = BASE_SEED + DEVIATION_TYPE_ORDER.index(deviation_type) * 100_000 + round(level * 1000) + t
                start, goal = _solvable_scenario(trial_seed, grid, free_cells)
                ground_truth, actual_start = generate_ground_truth(
                    grid, start, goal, level, seed=trial_seed, **scale_kwargs
                )
                for suite_name in SUITE_ORDER_LOCAL:
                    suite = SUITES[suite_name]()
                    result = run_match(suite, grid, start, goal, ground_truth, actual_start, tag_sites,
                                        random.Random(trial_seed), fidelity=fidelity)
                    rows.append({
                        "suite": suite_name,
                        "fidelity": fidelity,
                        "deviation_type": deviation_type,
                        "variance_level": level,
                        "trial": t,
                        "success": result.success,
                        "cost_usd": suite.cost_usd,
                    })
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows):
    stats = {}
    for fidelity in FIDELITY_ORDER:
        for suite in SUITE_ORDER_LOCAL:
            matching = [r for r in rows if r["fidelity"] == fidelity and r["suite"] == suite]
            successes = sum(r["success"] for r in matching)
            n = len(matching)
            ci_lo, ci_hi = bootstrap_ci(successes, n,
                                          seed=15_000_000 + FIDELITY_ORDER.index(fidelity) * 10_000
                                          + SUITE_ORDER_LOCAL.index(suite))
            stats[(fidelity, suite)] = {"rate": successes / n if n else 0.0, "successes": successes, "n": n,
                                          "ci_lo": ci_lo, "ci_hi": ci_hi, "cost": matching[0]["cost_usd"]}
    return stats


def value_ranking(stats, fidelity):
    """{suite: {rate, per_100_or_None, cost}} -- per_100 is None (NOT
    inf) when cost_usd == 0.0, since "infinite value per dollar" isn't a
    meaningful claim about a suite whose only real cost is integration
    effort this project's dollar model can't price. `best` picks the
    highest FINITE per_100 among suites that have one, reported
    separately from the free suite(s), which get their own callout."""
    baseline_rate = stats[(fidelity, "dead_reckoning")]["rate"]
    results = {}
    for suite in SUITE_ORDER_LOCAL:
        if suite == "dead_reckoning":
            continue
        s = stats[(fidelity, suite)]
        gain = s["rate"] - baseline_rate
        per_100 = None if s["cost"] == 0.0 else (gain / (s["cost"] / 100)) * 100
        results[suite] = {"rate": s["rate"], "per_100": per_100, "cost": s["cost"], "gain": gain}
    priced = {s: v for s, v in results.items() if v["per_100"] is not None}
    best_priced = max(priced, key=lambda s: priced[s]["per_100"]) if priced else None
    return results, baseline_rate, best_priced


def plot_comparison(stats, path):
    fig, axes = plt.subplots(1, len(FIDELITY_ORDER), figsize=(11, 5), sharey=True)
    for ax, fidelity in zip(axes, FIDELITY_ORDER):
        rates = [stats[(fidelity, s)]["rate"] for s in SUITE_ORDER_LOCAL]
        colors = [SUITE_COLORS_LOCAL[s] for s in SUITE_ORDER_LOCAL]
        ax.bar([SUITE_LABELS[s] for s in SUITE_ORDER_LOCAL], rates, color=colors)
        ax.set_title(fidelity, fontsize=10)
        ax.set_ylim(0, 1.0)
        ax.tick_params(axis="x", labelrotation=30, labelsize=8)
    axes[0].set_ylabel("Success rate")
    fig.suptitle(f"New suites vs. fidelity tier ({TRIALS} trials/point, levels {LEVELS}, '{LAYOUT}' layout)",
                  fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    fig.savefig(path, dpi=150)


def write_writeup(stats, path):
    lines = [
        "# New suites enabled by the fidelity-tier model: IMU and dual-camera AprilTag",
        "",
        "ImuSuite ($0 -- every REV Control Hub already ships one) corrects HEADING error only, "
        "continuously, with no replan cost; AprilTagImuSuite stacks it on AprilTagSuite; "
        "DualCameraAprilTagSuite adds a second (rear) camera to AprilTag's detection pipeline. Crossed "
        f"with fidelity tier ({', '.join(FIDELITY_ORDER)}) x 3 deviation types x levels {LEVELS} x "
        f"{TRIALS} trials/point on the 'cluttered' layout -- reduced relative to the headline sweep (see "
        "module docstring). Raw data in `ftc_newsuites_results.csv`, chart in "
        "`ftc_newsuites_comparison.png`.",
        "",
        "## Headline table",
        "",
        "| Suite | Cost | Optimistic | Realistic |",
        "|---|---:|---:|---:|",
    ]
    for suite in SUITE_ORDER_LOCAL:
        cost = stats[("optimistic", suite)]["cost"]
        lines.append(f"| {SUITE_LABELS[suite]} | ${cost:.0f} | {stats[('optimistic', suite)]['rate']:.0%} | "
                      f"{stats[('realistic', suite)]['rate']:.0%} |")

    lines += ["", "## Value ranking (success-rate gain over dead reckoning, per $100)", ""]
    for fidelity in FIDELITY_ORDER:
        results, baseline, best_priced = value_ranking(stats, fidelity)
        lines += [f"### {fidelity}", "", f"Dead-reckoning baseline: {baseline:.0%}", "",
                  "| Suite | Cost | Gain | pp/$100 |", "|---|---:|---:|---:|"]
        for suite, v in sorted(results.items(), key=lambda kv: (-kv[1]["per_100"] if kv[1]["per_100"] is not None
                                                                   else float("-inf"))):
            per_100_str = "undefined (cost_usd == 0)" if v["per_100"] is None else f"{v['per_100']:+.1f}"
            marker = " *best priced value*" if suite == best_priced else ""
            lines.append(f"| {SUITE_LABELS[suite]} | ${v['cost']:.0f} | {v['gain']:+.0%} | {per_100_str}{marker} |")
        imu_gain = results["imu"]["gain"]
        lines += ["", (
            f"ImuSuite gained {imu_gain:+.0%} success rate over dead reckoning for $0 -- a real gain (or "
            "loss) with no dollar figure to divide it by. \"Is the free hardware worth the code\" has to "
            "be answered by the gain itself, not a per-dollar ranking: this project's pp/$100 metric is "
            "silent on a $0 suite by construction, and reporting it as \"infinite value\" would be a more "
            "misleading claim than reporting it as undefined."
        ), ""]

    lines += ["## Does the dual camera actually matter -- and only where expected?", ""]
    apriltag_opt = stats[("optimistic", "apriltag")]["rate"]
    dual_opt = stats[("optimistic", "dual_camera_apriltag")]["rate"]
    apriltag_real = stats[("realistic", "apriltag")]["rate"]
    dual_real = stats[("realistic", "dual_camera_apriltag")]["rate"]
    lines += [
        "| Fidelity | AprilTag (1 camera) | Dual-camera AprilTag (2 cameras) | Difference |",
        "|---|---:|---:|---:|",
        f"| optimistic | {apriltag_opt:.0%} | {dual_opt:.0%} | {(dual_opt - apriltag_opt):+.0%} |",
        f"| realistic | {apriltag_real:.0%} | {dual_real:.0%} | {(dual_real - apriltag_real):+.0%} |",
        "",
    ]
    optimistic_negligible = abs(dual_opt - apriltag_opt) <= 0.05
    realistic_helps = dual_real > apriltag_real
    if optimistic_negligible and realistic_helps:
        lines.append(
            "Exactly as expected: a second camera makes essentially no difference under the optimistic "
            "tier's omnidirectional-camera assumption (an omnidirectional camera already sees everything "
            "a second one could add), but measurably helps under the realistic tier's real camera-FOV "
            "gating -- a clean demonstration of why the Priority 1 fidelity fix mattered. Under the OLD "
            "(pre-fidelity-tier) model, DualCameraAprilTagSuite would have been indistinguishable from "
            "AprilTagSuite; it isn't, once heading_deg_now actually gates detection."
        )
    else:
        lines.append(
            "The optimistic-tier gap isn't as close to zero as expected, or the realistic-tier gap didn't "
            "measurably favor the dual camera at this trial count -- worth rechecking with more trials "
            "before treating either direction as settled."
        )

    lines += ["", "## What this does and does not prove", "",
              "This is a reduced-rigor sweep (see module docstring), enough to see the shape of both "
              "effects, not a publication-grade confidence interval on the exact magnitude. ImuSuite's "
              "IMU_HEADING_CORRECTION_FACTOR and DualCameraAprilTagSuite's second-camera placement (front "
              "+ rear) are both documented ballpark engineering choices, same status as every other "
              "estimated constant in this project."]

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)

    all_rows = []
    for fidelity in FIDELITY_ORDER:
        print(f"fidelity={fidelity} ...")
        all_rows.extend(run_combo(fidelity, grid, free_cells, tag_sites))

    write_csv(all_rows, OUTPUT_DIR / "ftc_newsuites_results.csv")
    stats = summarize(all_rows)
    plot_comparison(stats, OUTPUT_DIR / "ftc_newsuites_comparison.png")
    write_writeup(stats, OUTPUT_DIR / "ftc_newsuites_writeup.md")

    print(f"\nWrote {len(all_rows)} trials to {OUTPUT_DIR / 'ftc_newsuites_results.csv'}")
    print(f"Chart saved to {OUTPUT_DIR / 'ftc_newsuites_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'ftc_newsuites_writeup.md'}\n")
    for fidelity in FIDELITY_ORDER:
        results, baseline, best_priced = value_ranking(stats, fidelity)
        print(f"{fidelity}: best priced value = {best_priced}, imu gain = {results['imu']['gain']:+.0%} "
              f"(baseline={baseline:.0%})")
