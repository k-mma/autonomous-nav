"""
Optional Priority 5: is a faster-motor purchase actually worth it now
that the 30-second budget genuinely binds? `ftc/budget_benchmark.py`
found `AUTONOMOUS_PERIOD_S` starts costing real matches around 15-20s
under the trapezoidal kinematics model (it barely bound at all under
the old naive drive-time formula) -- which is exactly the condition
under which a faster drivetrain would have something to win, IF it
actually bought more speed where it matters.

`ftc/config.py`'s `GEARING_OPTIONS` ("stock"/"fast"/"faster") is built
directly from goBILDA's published 5203-series RPM/torque table, not
invented multipliers -- and that data surfaces a real, non-obvious
finding: because torque falls as RPM rises for a fixed motor, and
because every option's accel-to-cruise distance is far larger than one
6in grid cell (so a single step never leaves `_trapezoidal_drive_time_s`'s
triangular, accel-only branch -- see ftc/scratch/gearing_test.py),
faster gearing is strictly SLOWER per cell in this model, not faster,
on top of `slip_factor` scaling `drift_per_cell` up. This is not a
speed-vs-slip tradeoff; it is a lose-lose at FTC's typical short-hop
distances, and this module measures exactly how much of a lose-lose it
is once translated into match success rate.

Crossed with AUTONOMOUS_PERIOD_S at three points: 30s (the real budget,
where `ftc/budget_benchmark.py` found it never binds -- gearing should
buy nothing here except more drift and slower per-cell drive time),
15s (right at where binding starts), and 10s (binds hard). Reduced
trial count/level set relative to the headline sweep (the same
reasoning ftc/robustness.py's own docstring already documents), since
this crosses 3 gearing options x 3 budgets x 7 suites x 3 deviation
types on top of the headline axes.

Writes benchmark_results/ftc_gearing_results.csv (every trial, raw,
with added `gearing` and `budget_s` columns), benchmark_results/
ftc_gearing_comparison.png (success rate by gearing, one panel per
budget), and benchmark_results/ftc_gearing_writeup.md (whether a
faster-motor purchase ever pays off, and at which budget).
"""
import csv
import random
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt

import ftc.match as match_module
from nav.field_variance import generate_ground_truth
from nav.stats import bootstrap_ci

from ftc.config import GEARING_LABELS, GEARING_ORDER, GEARING_OPTIONS
from ftc.field import build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import SUITES, SUITE_ORDER
from ftc.suite_benchmark import DEVIATION_TYPES, DEVIATION_TYPE_ORDER, LAYOUT, _solvable_scenario

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

BUDGET_LEVELS = [30.0, 15.0, 10.0]
LEVELS = [0.3, 0.5, 0.7, 0.9]
TRIALS = 15
BASE_SEED = 16_000_000


def run_combo(gearing, budget_s, grid, free_cells, tag_sites):
    original_budget = match_module.AUTONOMOUS_PERIOD_S
    rows = []
    try:
        match_module.AUTONOMOUS_PERIOD_S = budget_s
        for deviation_type in DEVIATION_TYPE_ORDER:
            scale_kwargs = DEVIATION_TYPES[deviation_type]
            for level in LEVELS:
                for t in range(TRIALS):
                    trial_seed = (BASE_SEED + DEVIATION_TYPE_ORDER.index(deviation_type) * 100_000
                                  + round(level * 1000) + t)
                    start, goal = _solvable_scenario(trial_seed, grid, free_cells)
                    ground_truth, actual_start = generate_ground_truth(
                        grid, start, goal, level, seed=trial_seed, **scale_kwargs
                    )
                    for suite_name in SUITE_ORDER:
                        suite = SUITES[suite_name]()
                        result = run_match(suite, grid, start, goal, ground_truth, actual_start, tag_sites,
                                            random.Random(trial_seed), gearing=gearing)
                        rows.append({
                            "suite": suite_name,
                            "gearing": gearing,
                            "budget_s": budget_s,
                            "deviation_type": deviation_type,
                            "variance_level": level,
                            "trial": t,
                            "success": result.success,
                            "over_budget": result.over_budget,
                            "final_pose_error_in": result.final_pose_error_in,
                        })
    finally:
        match_module.AUTONOMOUS_PERIOD_S = original_budget
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows):
    """{(gearing, budget_s): {rate, over_budget_rate, avg_pose_error_in}} averaged over every suite --
    this module's question is "does the gearing choice help at this budget," not a per-suite ranking
    (ftc/*_benchmark.py's other studies already own that question)."""
    stats = {}
    for gearing in GEARING_ORDER:
        for budget_s in BUDGET_LEVELS:
            matching = [r for r in rows if r["gearing"] == gearing and r["budget_s"] == budget_s]
            n = len(matching)
            successes = sum(r["success"] for r in matching)
            ci_lo, ci_hi = bootstrap_ci(successes, n,
                                          seed=17_000_000 + GEARING_ORDER.index(gearing) * 1000
                                          + BUDGET_LEVELS.index(budget_s))
            stats[(gearing, budget_s)] = {
                "rate": successes / n if n else 0.0, "ci_lo": ci_lo, "ci_hi": ci_hi,
                "over_budget_rate": sum(r["over_budget"] for r in matching) / n if n else 0.0,
                "avg_pose_error_in": sum(r["final_pose_error_in"] for r in matching) / n if n else 0.0,
            }
    return stats


def plot_gearing(stats, path):
    fig, axes = plt.subplots(1, len(BUDGET_LEVELS), figsize=(13, 5), sharey=True)
    colors = {"stock": "tab:gray", "fast": "tab:orange", "faster": "tab:red"}
    for ax, budget_s in zip(axes, BUDGET_LEVELS):
        rates = [stats[(g, budget_s)]["rate"] for g in GEARING_ORDER]
        ax.bar([GEARING_LABELS[g] for g in GEARING_ORDER], rates, color=[colors[g] for g in GEARING_ORDER])
        ax.set_title(f"budget = {budget_s:g}s", fontsize=10)
        ax.set_ylim(0, 1.0)
        ax.tick_params(axis="x", labelrotation=20, labelsize=8)
    axes[0].set_ylabel("Success rate (averaged across all 7 suites)")
    fig.suptitle(f"Gearing vs. budget ({TRIALS} trials/point, levels {LEVELS}, '{LAYOUT}' layout)", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    fig.savefig(path, dpi=150)


def write_writeup(stats, path):
    lines = [
        "# Is a faster-motor purchase worth it now that the budget binds?",
        "",
        "`ftc/budget_benchmark.py` found `AUTONOMOUS_PERIOD_S` starts binding around 15-20s under the "
        "trapezoidal kinematics model -- the first point at which a faster drivetrain would have anything "
        "to win, IF it actually bought more speed where it matters. `ftc/config.py`'s `GEARING_OPTIONS` is "
        "built from goBILDA's published 5203-series RPM/torque table, and that data says it doesn't: "
        "torque falls as RPM rises, and every option's accel-to-cruise distance is far larger than one "
        "6in grid cell, so a single step never reaches cruise speed regardless of gearing -- only "
        "acceleration governs per-cell drive time, and real acceleration is LOWER at every faster ratio "
        "(see ftc/scratch/gearing_test.py). `slip_factor` then scales `drift_per_cell` up on top of that. "
        "This is not a speed-vs-slip tradeoff; it's a lose-lose at this grid's cell scale. Crossed with "
        "budget at "
        f"{BUDGET_LEVELS} (30s = the real budget where it never binds; 15s = right at the binding point; "
        f"10s = binds hard), averaged across all 7 headline suites, {TRIALS} trials/point, levels "
        f"{LEVELS}, all 3 deviation types, 'cluttered' layout -- reduced relative to the headline sweep "
        "(see module docstring). Raw data in `ftc_gearing_results.csv`, chart in "
        "`ftc_gearing_comparison.png`.",
        "",
        "## Success rate by gearing x budget",
        "",
        "| Budget | Stock | Fast | Faster |", "|---:|---:|---:|---:|",
    ]
    for budget_s in BUDGET_LEVELS:
        lines.append(f"| {budget_s:g}s | {stats[('stock', budget_s)]['rate']:.0%} | "
                      f"{stats[('fast', budget_s)]['rate']:.0%} | {stats[('faster', budget_s)]['rate']:.0%} |")

    lines += ["", "## Accumulated pose error by gearing (the slip cost)", "",
              "| Budget | Stock | Fast | Faster |", "|---:|---:|---:|---:|"]
    for budget_s in BUDGET_LEVELS:
        lines.append(f"| {budget_s:g}s | {stats[('stock', budget_s)]['avg_pose_error_in']:.2f}in | "
                      f"{stats[('fast', budget_s)]['avg_pose_error_in']:.2f}in | "
                      f"{stats[('faster', budget_s)]['avg_pose_error_in']:.2f}in |")

    lines += ["", "## Does it pay off?", ""]
    for budget_s in BUDGET_LEVELS:
        stock = stats[("stock", budget_s)]
        best_gearing = max(GEARING_ORDER, key=lambda g: stats[(g, budget_s)]["rate"])
        best = stats[(best_gearing, budget_s)]
        # CI-overlap check on the extreme case (faster vs. stock) -- the
        # same "don't call a gap real until the CIs actually separate"
        # standard ftc/robustness.py, ftc/budget_benchmark.py, and ftc/
        # opponent_benchmark.py all already hold their own tipping
        # points to.
        faster = stats[("faster", budget_s)]
        overlap = not (faster["ci_lo"] > stock["ci_hi"] or stock["ci_lo"] > faster["ci_hi"])
        if best_gearing != "stock" and best["rate"] > stock["rate"] + 0.02:
            lines.append(
                f"At {budget_s:g}s, {GEARING_LABELS[best_gearing]} measurably beats stock: {best['rate']:.0%} "
                f"vs. {stock['rate']:.0%} -- unexpected given the per-cell kinematics (see module docstring), "
                "worth double-checking against ftc_gearing_results.csv directly rather than assumed away."
            )
        elif faster["rate"] < stock["rate"] - 0.02 and not overlap:
            lines.append(
                f"At {budget_s:g}s, faster gearing is worse, exactly as the per-cell kinematics predict: "
                f"{GEARING_LABELS['faster']} lands at {faster['rate']:.0%} vs. stock's {stock['rate']:.0%} "
                f"({(faster['rate'] - stock['rate']):+.0%}), and the two suites' success-rate confidence "
                f"intervals don't overlap at this trial count. This isn't a tradeoff that failed to pay off "
                "-- 'faster' gearing is strictly slower per cell AND drifts more (see the table above and "
                "ftc/scratch/gearing_test.py); there was never a time saving here for the drift cost to be "
                "weighed against. Buying speed without also buying something that corrects pose (odometry "
                "pods, AprilTag) makes the average suite's overall reliability worse, not better."
            )
        else:
            lines.append(
                f"At {budget_s:g}s, no gearing option measurably beats stock ({stock['rate']:.0%}) at this "
                f"trial count (fast: {stats[('fast', budget_s)]['rate']:.0%}, faster: {faster['rate']:.0%}) -- "
                + ("expected at 30s: the budget never binds there (ftc_budget_writeup.md), and neither "
                   "faster option even saves per-cell drive time at this grid scale (module docstring), so "
                   "there's nothing here for extra speed to win, only extra wheel-slip drift to lose." if budget_s == 30.0
                   else "the apparent drop doesn't clear the noise bar at this trial count; worth rechecking "
                   "with more trials before calling it either a real cost or a real non-effect.")
            )

    lines += ["", "## What this does and does not prove", "",
              "This is a reduced-rigor sweep (see module docstring), averaged across all 7 suites rather "
              "than reported per suite -- a real team would want to check this against the SPECIFIC suite "
              "it's actually running, since a suite that already fixes pose (AprilTag, odometry pods) can "
              "absorb the extra slip-driven drift better than one that can't (dead reckoning). The "
              "per-cell-slower finding itself is not a ballpark estimate -- it follows directly from "
              "goBILDA's own published RPM/torque table for the 5203 motor and this project's own grid "
              "cell size, both fixed facts, not tuned constants. `slip_factor` is the one number in "
              "GEARING_OPTIONS that remains an explicit ballpark engineering estimate: no vendor publishes "
              "slip-vs-gearing data, so it's a documented guess, same status as every other estimated "
              "constant in this project."]

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)

    all_rows = []
    for gearing in GEARING_ORDER:
        for budget_s in BUDGET_LEVELS:
            print(f"gearing={gearing} budget_s={budget_s} ...")
            all_rows.extend(run_combo(gearing, budget_s, grid, free_cells, tag_sites))

    write_csv(all_rows, OUTPUT_DIR / "ftc_gearing_results.csv")
    stats = summarize(all_rows)
    plot_gearing(stats, OUTPUT_DIR / "ftc_gearing_comparison.png")
    write_writeup(stats, OUTPUT_DIR / "ftc_gearing_writeup.md")

    print(f"\nWrote {len(all_rows)} trials to {OUTPUT_DIR / 'ftc_gearing_results.csv'}")
    print(f"Chart saved to {OUTPUT_DIR / 'ftc_gearing_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'ftc_gearing_writeup.md'}\n")
    for budget_s in BUDGET_LEVELS:
        for gearing in GEARING_ORDER:
            s = stats[(gearing, budget_s)]
            print(f"budget={budget_s:g}s gearing={gearing}: rate={s['rate']:.0%} "
                  f"avg_pose_error_in={s['avg_pose_error_in']:.2f}")
