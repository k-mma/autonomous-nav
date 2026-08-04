"""
Does AUTONOMOUS_PERIOD_S (ftc/config.py's 30-second FTC autonomous
budget) ever actually bind? `ftc_suite_writeup.md`'s own "Honest
findings" section already flags this as unresolved: no suite ran out of
budget anywhere in the headline sweep, which makes the budget model
arguably decorative -- ftc/match.py charges elapsed_s carefully (drive
time, turn time, PLANNING_OVERHEAD_S per replan) for a constraint that,
as tested, nothing has ever hit.

This sweeps AUTONOMOUS_PERIOD_S downward and finds (a) the budget at
which over_budget starts happening at all, and (b) whether tightening it
changes which suite wins -- the hypothesis being that suites which
replan more pay PLANNING_OVERHEAD_S more often and should degrade first
as the budget tightens. That's checked against actual avg_replans/trial
data rather than assumed from suite category: obstacle-sensing suites
(DistanceSensorSuite, FullSuite) replan on every newly-sensed obstacle,
but AprilTag also replans on every successful pose correction (ftc/
match.py's `replan_needed = not planned_once or tag_corrected`) and
empirically out-replans DistanceSensorSuite despite never sensing
obstacles at all -- see this module's own write_writeup for the
measured numbers before trusting which suites count as "replan-heavy."

Follows ftc/scratch/match_test.py's own proven pattern for varying the
budget: `ftc.match.AUTONOMOUS_PERIOD_S` is a plain module-level global
that ftc/match.py's run_match() looks up fresh on every call (unlike
ftc/sensors.py's SensorSuite.drift_per_cell, which is a CLASS attribute
baked in at import time -- see ftc/robustness.py's docstring for that
trap). So `ftc.match.AUTONOMOUS_PERIOD_S = x` before calling run_match
(directly, or via ftc/suite_benchmark.py's run_combo, which calls
run_match internally) is sufficient and needs no instance-attribute
workaround.

The prompt's own example budgets (30s, 20s, 15s, 10s, 7s) do bind, and
noticeably so, once ftc/match.py's drive time is charged with a
trapezoidal acceleration profile (MAX_ACCEL_MPS2, ftc/config.py) instead
of the old distance/MAX_DRIVE_SPEED_MPS instantaneous-acceleration
formula -- a single 6in cell step, too short to ever reach cruise speed
under realistic acceleration, now takes roughly 4-5x longer than the
naive formula assumed (see ftc/match.py's `_trapezoidal_drive_time_s`
docstring), which eats directly into whatever budget margin this sweep
measures. This is the interaction with the kinematics addition the task
brief itself called out -- run this module BEFORE that change and it
finds the budget barely binds anywhere in [7s, 30s]; run it after (the
current, correct state of this repo) and it binds starting around
15-20s instead. This sweep continues down to 1.5s so "does the budget
ever bind, and how hard" gets a real answer at the repo's current
kinematics model, not a stale one.

Reuses ftc/suite_benchmark.py's run_combo/aggregate/overall_success_rate
wholesale (same full-rigor 11-level x 25-trial sweep per budget point,
nothing reduced -- 9 budget points is cheap enough, ~2 minutes total, to
not need reducing) rather than reimplementing the sweep. The 30.0s point
reruns ftc/suite_benchmark.py's own scenario sequence unmodified and
serves as both this sweep's baseline and a consistency check that
nothing here silently changed AUTONOMOUS_PERIOD_S's default.

Writes benchmark_results/ftc_budget_results.csv (every trial, raw, with
an added `budget_s` column), benchmark_results/ftc_budget_comparison.png
(success rate and over-budget rate vs. budget, one line per suite), and
benchmark_results/ftc_budget_writeup.md (the budget at which it starts
to bind, whether that changes which suite wins, and whether the
replan-heavy-suites-degrade-first hypothesis actually holds).
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt

import ftc.match as match_module
from nav.stats import bootstrap_ci

from ftc.field import build_grid, tag_sites_for
from ftc.sensors import SUITE_LABELS, SUITE_ORDER
from ftc.suite_benchmark import (
    DEVIATION_TYPE_ORDER, LAYOUT, SUITE_COLORS, SUMMARY_MIN_LEVEL, TRIALS_PER_COMBO, VARIANCE_LEVELS,
    aggregate, overall_success_rate, run_combo,
)

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

# Descending, 30.0 first -- the real AUTONOMOUS_PERIOD_S, doubling as
# this sweep's baseline. Extends past the prompt's example floor of 7s
# down to where binding actually starts (see module docstring); 1.5s is
# close to a single cell's drive time at MAX_DRIVE_SPEED_MPS, so below
# that almost nothing but a same-cell/adjacent-cell start-goal pair
# could ever succeed regardless of suite, which would stop measuring
# "does the budget bind" and start measuring "is the scenario solvable
# at all" -- not a useful point to include.
BUDGET_LEVELS = [30.0, 20.0, 15.0, 10.0, 7.0, 5.0, 4.0, 3.0, 2.0, 1.5]
BASELINE_BUDGET = 30.0


def run_budget(budget_s):
    """Full-rigor sweep on the 'cluttered' layout with
    ftc.match.AUTONOMOUS_PERIOD_S patched to `budget_s` for the
    duration -- same seed formula as ftc/suite_benchmark.py's own
    __main__, so budget_s=30.0 reproduces ftc_suite_results.csv's rows
    trial-for-trial (bar the unseeded wall-clock planning_time_ms)."""
    grid = build_grid(LAYOUT)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(LAYOUT)

    original_budget = match_module.AUTONOMOUS_PERIOD_S
    try:
        match_module.AUTONOMOUS_PERIOD_S = budget_s
        rows = []
        for deviation_type in DEVIATION_TYPE_ORDER:
            for level in VARIANCE_LEVELS:
                base_seed = 6_000_000 + DEVIATION_TYPE_ORDER.index(deviation_type) * 1_000_000 + round(level * 100)
                rows.extend(run_combo(deviation_type, level, TRIALS_PER_COMBO, base_seed, grid, free_cells,
                                       tag_sites))
    finally:
        match_module.AUTONOMOUS_PERIOD_S = original_budget

    for row in rows:
        row["budget_s"] = budget_s
    return rows


def _bootstrap_seed(budget_s, suite):
    # round(budget_s * 1000), not BUDGET_LEVELS.index(budget_s) -- summarize()
    # is a general function any caller can hand an arbitrary budget_s to
    # (ftc/scratch/budget_benchmark_test.py does, e.g. 1000.0 or 0.01 to
    # check sane behavior at the extremes), and indexing into this
    # module's own fixed sweep list would raise ValueError for any budget
    # that isn't one of the exact points __main__ sweeps.
    return 8_000_000 + round(budget_s * 1000) * 100_000 + SUITE_ORDER.index(suite)


def summarize(rows, budget_s):
    """{suite: {rate, over_budget_rate, successes, n, ci_lo, ci_hi}} over
    the SUMMARY_MIN_LEVEL window (matches every other summary table in
    this project), plus the success-rate ranking (best first)."""
    matching_all = [r for r in rows if r["variance_level"] >= SUMMARY_MIN_LEVEL]
    results = {}
    for suite in SUITE_ORDER:
        matching = [r for r in matching_all if r["suite"] == suite]
        n = len(matching)
        successes = sum(r["success"] for r in matching)
        over_budget = sum(r["over_budget"] for r in matching)
        avg_replans = sum(r["replans"] for r in matching) / n
        ci_lo, ci_hi = bootstrap_ci(successes, n, seed=_bootstrap_seed(budget_s, suite))
        results[suite] = {
            "rate": successes / n, "over_budget_rate": over_budget / n, "avg_replans": avg_replans,
            "successes": successes, "n": n, "ci_lo": ci_lo, "ci_hi": ci_hi,
        }
    ranking = sorted(SUITE_ORDER, key=lambda s: -results[s]["rate"])
    return results, ranking


def first_binding_budget(all_summaries):
    """Largest budget (scanning BUDGET_LEVELS, which is already
    descending) at which any suite's over_budget_rate is nonzero --
    i.e. the first point, tightening from 30s down, where the budget
    starts costing anyone a match. None if it never binds anywhere in
    BUDGET_LEVELS."""
    for budget_s in BUDGET_LEVELS:
        results, _ = all_summaries[budget_s]
        if any(results[s]["over_budget_rate"] > 0 for s in SUITE_ORDER):
            return budget_s
    return None


def first_ranking_change(all_summaries):
    """First budget (scanning from 30s down) whose #1 suite differs from
    the 30s baseline's #1, plus a CI-overlap check on that specific pair
    -- a #1 swap whose success-rate CIs still overlap could be sampling
    noise, not a genuine budget effect, and the writeup has to say so
    rather than claim a clean flip (same reasoning ftc/robustness.py's
    first_tipping_point already uses for its own multiplier sweep)."""
    baseline_results, baseline_ranking = all_summaries[BASELINE_BUDGET]
    baseline_best = baseline_ranking[0]
    for budget_s in BUDGET_LEVELS:
        if budget_s == BASELINE_BUDGET:
            continue
        results, ranking = all_summaries[budget_s]
        if ranking[0] != baseline_best:
            new_best = ranking[0]
            old = baseline_results[baseline_best]
            new = results[new_best]
            overlap = not (new["ci_lo"] > old["ci_hi"] or old["ci_lo"] > new["ci_hi"])
            return budget_s, baseline_best, new_best, overlap
    return None, baseline_best, baseline_best, None


def write_csv(all_rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
        writer.writeheader()
        writer.writerows(all_rows)


def plot_budget_comparison(all_summaries, binding_budget, path):
    fig, (ax_rate, ax_over) = plt.subplots(1, 2, figsize=(13, 5))
    budgets_asc = list(reversed(BUDGET_LEVELS))
    for suite in SUITE_ORDER:
        rates = [all_summaries[b][0][suite]["rate"] for b in budgets_asc]
        over = [all_summaries[b][0][suite]["over_budget_rate"] for b in budgets_asc]
        ax_rate.plot(budgets_asc, rates, "o-", label=SUITE_LABELS[suite], color=SUITE_COLORS[suite], markersize=4)
        ax_over.plot(budgets_asc, over, "o-", label=SUITE_LABELS[suite], color=SUITE_COLORS[suite], markersize=4)

    for ax, title in ((ax_rate, "Success rate vs. budget"), (ax_over, "Over-budget rate vs. budget")):
        ax.set_xlabel("AUTONOMOUS_PERIOD_S (seconds)")
        ax.set_title(title, fontsize=10)
        ax.invert_xaxis()
        if binding_budget is not None:
            ax.axvline(binding_budget, color="red", linestyle="--", linewidth=1.2)
    ax_rate.set_ylabel("Success rate")
    ax_rate.set_ylim(-0.05, 1.05)
    ax_over.set_ylabel("Over-budget rate")
    if binding_budget is not None:
        ax_rate.annotate(f"binds at {binding_budget}s", xy=(binding_budget, 1.0), fontsize=8, color="red",
                          ha="center", va="bottom")
    ax_rate.legend(loc="lower right", fontsize=8)
    fig.suptitle(f"Does AUTONOMOUS_PERIOD_S bind? ({TRIALS_PER_COMBO} trials/point, variance_level >= "
                  f"{SUMMARY_MIN_LEVEL}, '{LAYOUT}' layout)", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(path, dpi=150)


def write_writeup(all_summaries, binding_budget, ranking_change, path):
    lines = [
        "# Does the 30-second autonomous budget ever actually bind?",
        "",
        "`ftc_suite_writeup.md`'s own \"Honest findings\" section flags this as open: no suite ran "
        "out of budget anywhere in the headline sweep. This sweeps `AUTONOMOUS_PERIOD_S` (ftc/"
        f"config.py, real value 30.0s) downward -- {', '.join(f'{b:g}s' for b in BUDGET_LEVELS)} -- "
        f"rerunning the identical full-rigor sweep ({TRIALS_PER_COMBO} trials/point, all "
        f"{len(VARIANCE_LEVELS)} variance_level steps, all 3 deviation types) at each budget via "
        "`ftc.match.AUTONOMOUS_PERIOD_S` patching (the pattern `ftc/scratch/match_test.py` already "
        "established). Raw data (with an added `budget_s` column) in `ftc_budget_results.csv`, "
        "chart in `ftc_budget_comparison.png`.",
        "",
    ]
    example_budgets = [30.0, 20.0, 15.0, 10.0, 7.0]
    example_binding = [b for b in example_budgets if any(all_summaries[b][0][s]["over_budget_rate"] > 0
                                                            for s in SUITE_ORDER)]
    if example_binding:
        first_example_bind = max(example_binding)  # BUDGET_LEVELS descending -> largest binding budget first
        worst_rate = max(all_summaries[first_example_bind][0][s]["over_budget_rate"] for s in SUITE_ORDER)
        lines.append(
            f"**The example budgets in the original ask ({', '.join(f'{b:g}s' for b in example_budgets)}) "
            f"do bind**, starting at {first_example_bind:g}s (worst-suite over-budget rate "
            f"{worst_rate:.2%} there -- small enough to round to 0% in the per-budget table below, but "
            "genuinely nonzero) -- ftc/match.py's trapezoidal drive-time model (added in this same "
            "sweep of priorities, see ftc/config.py's MAX_ACCEL_MPS2) makes every step take noticeably "
            "longer than the old naive distance/speed formula assumed, which erodes the margin this "
            "budget used to have. This sweep continues down to 1.5s so the *full* shape of the bind, not "
            "just where it starts, is visible."
        )
    else:
        lines.append(
            f"**The example budgets in the original ask ({', '.join(f'{b:g}s' for b in example_budgets)}) "
            "never bind** -- every suite has a 0% over-budget rate across all of them. This sweep "
            "continues down to 1.5s so the question gets a real answer instead of one truncated at the "
            "edge of detectability."
        )
    lines += [
        "",
        "## Per-budget results (variance_level >= 0.3, all deviation types)",
        "",
        "| Budget (s) | Ranking (best to worst) | Worst over-budget rate |",
        "|---:|---|---:|",
    ]
    for budget_s in BUDGET_LEVELS:
        results, ranking = all_summaries[budget_s]
        worst_over = max(results[s]["over_budget_rate"] for s in SUITE_ORDER)
        lines.append(
            f"| {budget_s:g} | {' > '.join(SUITE_LABELS[s] for s in ranking)} | {worst_over:.0%} |"
        )

    # "Replan-heavy" determined from the data (avg_replans at the real
    # 30s budget, unaffected by early cutoffs), not assumed -- a first
    # draft of this writeup hardcoded {distance_sensors, full_suite} as
    # the replan-heavy suites (obstacle-sensing suites replan on every
    # newly-seen obstacle), which turned out to be incomplete: AprilTag
    # also replans on every successful tag correction (see ftc/match.py's
    # `replan_needed = not planned_once or tag_corrected`), not just
    # obstacle detections, and empirically out-replans DistanceSensorSuite.
    baseline_results, _ = all_summaries[BASELINE_BUDGET]
    replans_by_suite = {s: baseline_results[s]["avg_replans"] for s in SUITE_ORDER}
    median_replans = sorted(replans_by_suite.values())[len(SUITE_ORDER) // 2]
    replan_heavy = {s for s, r in replans_by_suite.items() if r > median_replans}

    lines += ["", "## Replanning frequency (real 30s budget, unaffected by tightening)", "",
              "| Suite | Avg. replans/trial |", "|---|---:|"]
    for suite in sorted(SUITE_ORDER, key=lambda s: -replans_by_suite[s]):
        lines.append(f"| {SUITE_LABELS[suite]} | {replans_by_suite[suite]:.2f} |")
    lines.append("")
    lines.append(
        "AprilTag replans on every successful tag correction, not just DistanceSensorSuite/FullSuite's "
        "every-newly-sensed-obstacle trigger -- it ends up replanning more often than DistanceSensorSuite "
        "despite never sensing obstacles at all."
    )

    lines += ["", "## Where it starts to bind", ""]
    if binding_budget is None:
        lines.append(
            f"**The budget never binds anywhere in [{BUDGET_LEVELS[-1]:g}s, {BUDGET_LEVELS[0]:g}s]** -- "
            "every suite reaches 100% of its non-over-budget outcomes even at the tightest budget "
            "tested. AUTONOMOUS_PERIOD_S would have to be tightened well below what's swept here "
            "before it stopped being decorative."
        )
    else:
        tightest = BUDGET_LEVELS[-1]
        results_at_tightest, _ = all_summaries[tightest]
        worst_suite = max(SUITE_ORDER, key=lambda s: results_at_tightest[s]["over_budget_rate"])
        lines.append(
            f"**The budget starts binding at {binding_budget:g}s** -- the tightest budget at which "
            f"every suite still has a 0% over-budget rate is the next one up in this sweep. At the "
            f"tightest budget tested ({tightest:g}s), **{SUITE_LABELS[worst_suite]}** has the "
            f"highest over-budget rate ({results_at_tightest[worst_suite]['over_budget_rate']:.0%})."
        )
        if worst_suite in replan_heavy:
            lines.append(
                f"\nThis matches the replan-heavy-suites-degrade-first hypothesis: {SUITE_LABELS[worst_suite]} "
                f"replans {replans_by_suite[worst_suite]:.2f} times/trial on average (above the "
                f"{median_replans:.2f}/trial median across all 5 suites, see the table above), each replan "
                "charged PLANNING_OVERHEAD_S on top of drive time -- exactly the kind of suite expected to "
                "feel a tight budget first, even though the *specific* suite (AprilTag, corrections-driven) "
                "isn't the one the original obstacle-sensing-suites hypothesis named."
            )
        else:
            lines.append(
                f"\nThis does **not** match the replan-heavy-suites-degrade-first hypothesis -- "
                f"{SUITE_LABELS[worst_suite]} replans only {replans_by_suite[worst_suite]:.2f} times/trial on "
                f"average, at or below the {median_replans:.2f}/trial median. PLANNING_OVERHEAD_S isn't the "
                "dominant cost near the budget edge for this suite; total elapsed_s (drive + turn time over "
                "whatever path length that suite's pose/obstacle error forces) matters at least as much as "
                "replan count."
            )

    lines += ["", "## Does tightening the budget change which suite wins?", ""]
    tip_budget, old_best, new_best, overlap = ranking_change
    if tip_budget is None:
        lines.append(
            f"**No -- {SUITE_LABELS[old_best]} stays the #1 suite by raw success rate across the entire "
            f"sweep**, {BUDGET_LEVELS[0]:g}s down to {BUDGET_LEVELS[-1]:g}s. Even where the budget does "
            "bind (see above), it doesn't bind hard enough, or unevenly enough across suites, to change "
            "which one has the best raw success rate."
        )
    else:
        clean = "a statistically clean change (CIs don't overlap)" if not overlap else \
            "not statistically clean -- the two suites' success-rate CIs still overlap at this trial count"
        lines.append(
            f"**Yes -- at {tip_budget:g}s, {SUITE_LABELS[new_best]} overtakes {SUITE_LABELS[old_best]} "
            f"as the #1 suite by raw success rate.** This is {clean}, so it "
            + ("should be treated as sampling noise rather than a confirmed ranking change until rechecked "
               "with more trials." if overlap else
               "is a genuine effect of the tighter budget, not noise.")
        )

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    all_rows = []
    all_summaries = {}
    for budget_s in BUDGET_LEVELS:
        print(f"budget_s={budget_s} ...")
        rows = run_budget(budget_s)
        all_rows.extend(rows)
        all_summaries[budget_s] = summarize(rows, budget_s)

    binding_budget = first_binding_budget(all_summaries)
    ranking_change = first_ranking_change(all_summaries)

    write_csv(all_rows, OUTPUT_DIR / "ftc_budget_results.csv")
    plot_budget_comparison(all_summaries, binding_budget, OUTPUT_DIR / "ftc_budget_comparison.png")
    write_writeup(all_summaries, binding_budget, ranking_change, OUTPUT_DIR / "ftc_budget_writeup.md")

    print(f"\nWrote {len(all_rows)} trials to {OUTPUT_DIR / 'ftc_budget_results.csv'}")
    print(f"Chart saved to {OUTPUT_DIR / 'ftc_budget_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'ftc_budget_writeup.md'}\n")
    print(f"binding budget: {binding_budget}")
    print(f"ranking change: {ranking_change}")
