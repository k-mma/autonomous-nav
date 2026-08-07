"""
Does ftc/match.py's treatment of planning time hold at the TAIL, not
just on average? Every existing result in this repo reasons about
planning cost through PLANNING_OVERHEAD_S (ftc/config.py) -- a flat,
documented ballpark ("per-replan control-loop overhead on FTC-legal
onboard compute... not just the raw grid search, which is sub-
millisecond in Python and would understate what a real re-plan
actually costs") charged into elapsed_s on every replan. That constant
is a mean-ish estimate. A planner whose AVERAGE call is fine and whose
WORST call blows the budget is a planner that fails a real match, and
nothing in this repo has checked whether that gap is real.

WHAT run_match ACTUALLY DOES (read before any of this was written --
see WRITEUPS.md's "Planning latency at the tail" for the full trace).
There is no per-tick wall-clock deadline anywhere in ftc/match.py.
elapsed_s is a purely SIMULATED time accumulator; the only wall-clock
measurement it ever takes is around each `astar()` call
(`t0 = time.perf_counter(); ...; planning_time_s += time.perf_counter()
- t0`), and that measured value -- MatchResult.planning_time_s -- is
NEVER added to elapsed_s. What IS charged is a flat
`elapsed_s += PLANNING_OVERHEAD_S` on every replan AFTER the first
(`if planned_once: ... elapsed_s += PLANNING_OVERHEAD_S`) -- the very
FIRST planning call in a match costs 0 elapsed_s, charged or measured,
regardless of how long it actually took. So under the model exactly as
implemented, no measured planning latency, however large, can change
`over_budget`/`success` -- that's a structural fact about this file,
not something this benchmark is testing for. What this benchmark
actually measures: (a) the real distribution of per-call planning
latency this repo has never looked at, and (b) a COUNTERFACTUAL --
would a match currently reported as under budget go over budget if
elapsed_s used the REAL measured planning time instead of the flat
constant? -- which is the only way to ask "does the tail matter" that
doesn't beg the question against run_match's own accounting.

Non-invasive: `ftc.match.astar` (the name `from nav.algorithms import
astar` bound into ftc/match.py's own module namespace -- patching
`nav.algorithms.astar` instead would silently time nothing, since that
name was already bound into ftc.match at import time) is wrapped for
the duration of one run_match() call to record every individual
call's wall-clock latency, then restored. ftc/match.py's source is not
modified; no published number can be affected by this file existing.

GRID SIZE, deliberately: `ftc/field.py`'s `size` parameter is swept
(24/48/96/192/384 -- the native FTC scale and 4 synthetic multiples),
`cell_size_in` is NOT. Every `*_CELLS` sensor-range constant in ftc/
config.py (APRILTAG_RANGE_CELLS, DISTANCE_SENSOR_RANGE_CELLS, ...) is
computed once at import time from the native CELL_SIZE_IN and does not
read whatever `cell_size_in` a particular build_grid() call used --
sweeping `cell_size_in` would silently corrupt every suite's sensing
range relative to the grid it's actually driving on. Sweeping `size`
alone (a bigger physical field, same 6in cells) avoids that entirely --
the same methodology nav/scale_benchmark.py already uses for nav/'s
own domain-neutral grid. This is a real constraint on ftc/field.py's
`cell_size_in` parameter that this study surfaced, not a design choice
made for its own sake -- see "Honest findings" in the writeup.

REPLAN FREQUENCY: ftc/sensors.py's 5 headline suites (SUITE_ORDER),
reused as-is -- they already replan at different, real rates
(DeadReckoningSuite never replans past its first plan; AprilTag/
DistanceSensorSuite/FullSuite do, for different reasons -- see ftc/
budget_benchmark.py's own docstring). No synthetic "replan every N
ticks" knob was added.

Start/goal pairs are sampled within a bounded offset of each other
(MAX_OFFSET_CELLS), not uniformly across the whole grid, and rejected
outside [MIN_PATH_LEN, MAX_PATH_LEN] regardless of grid size -- so
DRIVE time (which scales with path length, and would otherwise dominate
elapsed_s at a much bigger grid, swamping any planning-latency effect
this study is actually looking for) stays comparable across every size
tested; only the SEARCH SPACE A* has to explore grows.

Writes benchmark_results/planning_latency.csv (one row per individual
planning call, match-level outcome columns denormalized onto every row
belonging to that match), benchmark_results/planning_latency_
comparison.png, and benchmark_results/planning_latency_writeup.md.
"""
import contextlib
import csv
import platform
import random
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.parse_math"] = False
import matplotlib.pyplot as plt

from nav.algorithms import astar
from nav.field_variance import generate_ground_truth
from nav.stats import bootstrap_ci

import ftc.match as match_module
from ftc.config import AUTONOMOUS_PERIOD_S, FTC_GRID_SIZE, PLANNING_OVERHEAD_S
from ftc.field import LAYOUTS, build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import SUITE_LABELS, SUITE_ORDER, SUITES

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"

RESOLUTION_MULTIPLIERS = [1, 2, 4, 8, 16]
LAYOUT_ORDER = list(LAYOUTS)
TRIALS_PER_COMBO = 12
BASE_SEED = 44_000_000

MIN_PATH_LEN = 4
MAX_PATH_LEN = 40
MAX_OFFSET_CELLS = 30
MAX_SCENARIO_ATTEMPTS = 200

# Reused, not invented -- the exact deviation mix ftc/optimizer.py's
# "corridor" ScenarioProfile already uses, chosen there for spreading
# suites across a range of outcomes. Fixed across the whole sweep so
# grid size and suite are the only two things varying.
DEVIATION_KWARGS = dict(variance_level=0.3, start_drift_scale=1.0, obstacle_drift_scale=1.0, blocker_scale=0.5)

COMPARE_COLUMNS = ["latency_s"]


@contextlib.contextmanager
def capture_astar_latencies():
    """Times every call ftc/match.py's run_match makes to `astar` for
    the duration of the `with` block, without modifying ftc/match.py.
    Patches `ftc.match.astar` specifically (the name bound into that
    module's namespace by its own `from nav.algorithms import astar`),
    not `nav.algorithms.astar` -- the two are different name bindings
    after import, and patching the wrong one silently captures nothing.
    Always restores the original function, including on an exception."""
    original = match_module.astar
    latencies = []

    def _timed(*args, **kwargs):
        t0 = time.perf_counter()
        result = original(*args, **kwargs)
        latencies.append(time.perf_counter() - t0)
        return result

    match_module.astar = _timed
    try:
        yield latencies
    finally:
        match_module.astar = original


def _bounded_scenario(trial_seed, grid):
    """Start/goal sampled within MAX_OFFSET_CELLS of each other (an
    offset from `start`, not a uniform draw from the whole grid --
    uniform sampling on a 384-cell grid would need many rejected
    attempts before landing near enough for MAX_PATH_LEN, and cost
    would grow with grid size for a reason that has nothing to do with
    what this study is measuring), rejected outside [MIN_PATH_LEN,
    MAX_PATH_LEN] regardless of grid size -- see module docstring."""
    rng = random.Random(trial_seed)
    size = grid.size
    for _ in range(MAX_SCENARIO_ATTEMPTS):
        sr, sc = rng.randrange(size), rng.randrange(size)
        if grid.cells[sr][sc] != 0:
            continue
        dr = rng.randint(-MAX_OFFSET_CELLS, MAX_OFFSET_CELLS)
        dc = rng.randint(-MAX_OFFSET_CELLS, MAX_OFFSET_CELLS)
        gr, gc = sr + dr, sc + dc
        if not (0 <= gr < size and 0 <= gc < size) or (gr, gc) == (sr, sc):
            continue
        if grid.cells[gr][gc] != 0:
            continue
        path, _, _ = astar(grid, (sr, sc), (gr, gc))
        if path is not None and MIN_PATH_LEN <= len(path) <= MAX_PATH_LEN:
            return (sr, sc), (gr, gc)
    raise RuntimeError(f"no bounded solvable scenario for seed {trial_seed} at size {size} "
                       f"after {MAX_SCENARIO_ATTEMPTS} attempts")


def run_combo(multiplier, layout, suite_name, num_trials, base_seed):
    size = FTC_GRID_SIZE * multiplier
    grid = build_grid(layout, size=size)
    tag_sites = tag_sites_for(layout)

    rows = []
    for t in range(num_trials):
        trial_seed = base_seed + t
        # Scenario generation's own astar() calls (finding a bounded
        # solvable start/goal) are NOT part of what a real robot's
        # onboard planner would run mid-match -- they're this study's
        # own offline scenario authoring, the equivalent of a team
        # surveying their field beforehand. Deliberately outside the
        # capture_astar_latencies() window below.
        start, goal = _bounded_scenario(trial_seed, grid)
        ground_truth, actual_start = generate_ground_truth(grid, start, goal, seed=trial_seed, **DEVIATION_KWARGS)
        suite = SUITES[suite_name]()

        with capture_astar_latencies() as latencies:
            result = run_match(suite, grid, start, goal, ground_truth, actual_start, tag_sites,
                                random.Random(trial_seed))

        # Sanity check on the instrumentation itself: the sum of what
        # was independently captured here should be CLOSE to what
        # run_match measured and reported on its own (MatchResult.
        # planning_time_s) -- confirming this is timing the SAME calls
        # run_match already measures, not some other code path. Not
        # exact, for two stacking reasons: (1) run_match's own t0/t1
        # bracket a call to `ftc.match.astar`, which -- once patched --
        # IS this wrapper, so run_match's measured interval includes
        # this wrapper's own overhead (a second perf_counter() pair, a
        # list append) on top of the real astar() time, while
        # `latencies` only records the inner call; (2) wall-clock
        # timing on a real, shared machine has genuine noise (OS
        # scheduling jitter, GC pauses) that scales with how long the
        # process runs, not with call count alone -- a fixed per-call
        # tolerance intermittently failed in practice (a real timing
        # discrepancy this loose bound is specifically here to absorb,
        # not a logic bug: see WRITEUPS.md). A relative-OR-absolute
        # bound tracks both sources without being so loose it would
        # miss an actual instrumentation bug (e.g. capturing a
        # different set of calls entirely, which would be off by
        # orders of magnitude, not a few percent).
        measured_total = sum(latencies)
        tolerance_s = max(2e-4, 0.05 * result.planning_time_s)
        assert abs(measured_total - result.planning_time_s) < tolerance_s, (
            f"capture_astar_latencies total ({measured_total}) != "
            f"MatchResult.planning_time_s ({result.planning_time_s}) -- instrumentation mismatch "
            f"(tolerance {tolerance_s})"
        )
        assert len(latencies) == result.replans + 1, (
            f"captured {len(latencies)} astar() calls but MatchResult.replans={result.replans} "
            f"(expected replans + 1, for the uncharged first plan)"
        )

        # The counterfactual: elapsed_s with the flat per-replan charge
        # replaced by the REAL measured total for this exact match --
        # including the first plan, which the flat model never charges
        # at all. charged_planning_s is exactly what run_match already
        # added to elapsed_s for planning (result.replans * PLANNING_
        # OVERHEAD_S, since only replans after the first are charged).
        charged_planning_s = result.replans * PLANNING_OVERHEAD_S
        counterfactual_elapsed_s = result.elapsed_s - charged_planning_s + measured_total
        counterfactual_over_budget = counterfactual_elapsed_s > AUTONOMOUS_PERIOD_S
        outcome_flipped = (not result.over_budget) and counterfactual_over_budget

        for i, latency_s in enumerate(latencies):
            rows.append({
                "resolution_multiplier": multiplier, "grid_size": size, "layout": layout, "suite": suite_name,
                "trial": t, "seed": trial_seed, "call_index": i, "is_first_plan": int(i == 0),
                "latency_s": round(latency_s, 6),
                "match_success": int(result.success), "match_over_budget": int(result.over_budget),
                "match_elapsed_s": round(result.elapsed_s, 4), "match_replans": result.replans,
                "match_planning_time_s": round(result.planning_time_s, 6),
                "match_num_calls": len(latencies),
                "counterfactual_elapsed_s": round(counterfactual_elapsed_s, 4),
                "counterfactual_over_budget": int(counterfactual_over_budget),
                "outcome_flipped": int(outcome_flipped),
            })
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _percentile(sorted_values, pct):
    if not sorted_values:
        return 0.0
    idx = min(int(pct * len(sorted_values)), len(sorted_values) - 1)
    return sorted_values[idx]


def latency_stats_by_multiplier(rows):
    """{multiplier: {median, p99, max, n}} pooled across layout/suite --
    the per-call latency distribution at each grid size."""
    stats = {}
    for m in RESOLUTION_MULTIPLIERS:
        values = sorted(r["latency_s"] for r in rows if r["resolution_multiplier"] == m)
        stats[m] = {
            "median": _percentile(values, 0.5), "p99": _percentile(values, 0.99),
            "max": values[-1] if values else 0.0, "n": len(values),
        }
    return stats


def matches_by_key(rows):
    """One representative row per MATCH (not per call) -- match-level
    columns are denormalized identically onto every call-row belonging
    to it, so any one of them (the first) carries the whole match's
    outcome."""
    seen = {}
    for r in rows:
        key = (r["resolution_multiplier"], r["layout"], r["suite"], r["trial"])
        if key not in seen:
            seen[key] = r
    return list(seen.values())


def flip_stats_by_multiplier(rows):
    """{multiplier: {n, flips, rate, ci_lo, ci_hi}} -- of matches the
    flat-cost model reports as under budget, how many flip to over
    budget under the measured-latency counterfactual."""
    matches = matches_by_key(rows)
    stats = {}
    for m in RESOLUTION_MULTIPLIERS:
        candidates = [r for r in matches if r["resolution_multiplier"] == m and not r["match_over_budget"]]
        flips = sum(r["outcome_flipped"] for r in candidates)
        n = len(candidates)
        ci_lo, ci_hi = bootstrap_ci(flips, n, seed=1000 + m) if n else (0.0, 0.0)
        stats[m] = {"n": n, "flips": flips, "rate": flips / n if n else 0.0, "ci_lo": ci_lo, "ci_hi": ci_hi}
    return stats


def replans_by_suite(rows):
    """{suite: mean replans/match} pooled across every grid size and
    layout -- the "replan frequency" axis this study reuses rather than
    inventing, and a check against ftc/budget_benchmark.py's own
    docstring claim that AprilTag out-replans DistanceSensorSuite."""
    matches = matches_by_key(rows)
    stats = {}
    for suite in SUITE_ORDER:
        matching = [r["match_replans"] for r in matches if r["suite"] == suite]
        stats[suite] = sum(matching) / len(matching) if matching else 0.0
    return stats


def plot_comparison(rows, path):
    latency_stats = latency_stats_by_multiplier(rows)
    replan_stats = replans_by_suite(rows)

    fig, (ax_latency, ax_replans) = plt.subplots(1, 2, figsize=(13, 5.5))

    sizes = [FTC_GRID_SIZE * m for m in RESOLUTION_MULTIPLIERS]
    medians = [latency_stats[m]["median"] * 1000 for m in RESOLUTION_MULTIPLIERS]
    p99s = [latency_stats[m]["p99"] * 1000 for m in RESOLUTION_MULTIPLIERS]
    maxes = [latency_stats[m]["max"] * 1000 for m in RESOLUTION_MULTIPLIERS]
    ax_latency.plot(sizes, medians, "o-", label="median", color="tab:blue")
    ax_latency.plot(sizes, p99s, "o-", label="p99", color="tab:orange")
    ax_latency.plot(sizes, maxes, "o-", label="max", color="tab:red")
    ax_latency.axhline(PLANNING_OVERHEAD_S * 1000, color="black", linestyle="--", linewidth=1,
                        label=f"PLANNING_OVERHEAD_S ({PLANNING_OVERHEAD_S * 1000:.0f}ms, flat charge/replan)")
    ax_latency.set_xscale("log")
    ax_latency.set_yscale("log")
    ax_latency.set_xlabel("Grid size (cells/side, log scale)")
    ax_latency.set_ylabel("Per-call planning latency, ms (log scale)")
    ax_latency.set_title("Planning latency distribution vs. grid size\n(pooled across layout + suite)",
                          fontsize=10)
    ax_latency.legend(fontsize=8)
    ax_latency.grid(alpha=0.2, which="both")

    suites = SUITE_ORDER
    ax_replans.bar([SUITE_LABELS[s] for s in suites], [replan_stats[s] for s in suites], color="tab:purple")
    ax_replans.set_ylabel("Mean replans / match")
    ax_replans.set_title("Replan frequency by suite\n(pooled across grid size + layout)", fontsize=10)
    ax_replans.tick_params(axis="x", labelrotation=20)
    for label in ax_replans.get_xticklabels():
        label.set_ha("right")

    fig.suptitle(f"Planning latency: distribution and replan frequency ({TRIALS_PER_COMBO} trials/combo)",
                 fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(path, dpi=150)


def write_writeup(rows, path):
    latency_stats = latency_stats_by_multiplier(rows)
    flip_stats = flip_stats_by_multiplier(rows)
    replan_stats = replans_by_suite(rows)
    total_matches = len(matches_by_key(rows))

    lines = [
        "# Does planning-time tail latency ever change a match outcome?",
        "",
        f"Measured on {platform.system()} {platform.release()}, {platform.machine()}, "
        f"Python {platform.python_version()} -- see \"Honest findings\" below for why this number does not "
        "transfer to different hardware, including the actual FTC-legal onboard compute "
        "(a REV Control Hub) this project's PLANNING_OVERHEAD_S constant is meant to represent.",
        "",
        f"{TRIALS_PER_COMBO} trials x {len(RESOLUTION_MULTIPLIERS)} grid sizes "
        f"({', '.join(str(FTC_GRID_SIZE * m) for m in RESOLUTION_MULTIPLIERS)} cells/side) x "
        f"{len(LAYOUT_ORDER)} layouts x {len(SUITE_ORDER)} suites = {total_matches} matches, "
        f"{len(rows)} individual planning calls. Raw data (one row per call) in "
        "`planning_latency.csv`, chart in `planning_latency_comparison.png`.",
        "",
        "## What ftc/match.py actually does with planning time",
        "",
        "There is no per-tick wall-clock deadline anywhere in this codebase. `elapsed_s` is a purely "
        "SIMULATED time accumulator; the only wall-clock measurement `run_match` ever takes is around each "
        "`astar()` call, and that measured value (`MatchResult.planning_time_s`) is never added to "
        "`elapsed_s`. What IS charged is a flat `PLANNING_OVERHEAD_S` "
        f"({PLANNING_OVERHEAD_S * 1000:.0f}ms, ftc/config.py) on every replan AFTER the first -- the very "
        "first planning call in a match costs 0 simulated seconds, charged or measured, no matter how long "
        "it actually took. This is a deliberate, documented choice (PLANNING_OVERHEAD_S's own comment: raw "
        "Python `astar()` \"is sub-millisecond... and would understate what a real re-plan actually costs\"), "
        "not an oversight -- but it has a structural consequence worth stating plainly: under the model "
        "exactly as implemented, no measured planning latency, however large, can change `over_budget` or "
        "`success`. Tail latency is measured and reported (`planning_time_s`, `avg_planning_ms` in other "
        "benchmarks' aggregates) but never charged.",
        "",
        "## The distribution",
        "",
        "| Grid size | n calls | median | p99 | max |",
        "|---:|---:|---:|---:|---:|",
    ]
    for m in RESOLUTION_MULTIPLIERS:
        s = latency_stats[m]
        lines.append(f"| {FTC_GRID_SIZE * m} | {s['n']} | {s['median'] * 1000:.2f}ms | {s['p99'] * 1000:.2f}ms "
                     f"| {s['max'] * 1000:.2f}ms |")

    crossover = next((m for m in RESOLUTION_MULTIPLIERS if latency_stats[m]["max"] > PLANNING_OVERHEAD_S), None)
    lines += ["", (
        f"The median stays well under PLANNING_OVERHEAD_S ({PLANNING_OVERHEAD_S * 1000:.0f}ms) at every "
        f"size tested -- planning is not the bottleneck on average, at any size here. The MAX does not: at "
        f"grid size {FTC_GRID_SIZE * crossover}, the single slowest observed planning call "
        f"({latency_stats[crossover]['max'] * 1000:.1f}ms) already exceeds the flat constant this project "
        "charges for an entire replan. That is exactly the average-fine/tail-not gap this study set out to "
        "check for -- on THIS machine, at THIS grid size, for THIS Python implementation of A*."
        if crossover is not None else
        f"The MAX never exceeds PLANNING_OVERHEAD_S ({PLANNING_OVERHEAD_S * 1000:.0f}ms) at any size tested, "
        f"up to {FTC_GRID_SIZE * RESOLUTION_MULTIPLIERS[-1]} cells/side. At this project's grid scale, on "
        "this machine, A* is fast enough that planning latency is not close to being the bottleneck, at the "
        "tail or on average."
    )]

    lines += [
        "",
        "## Does the tail change a match outcome?",
        "",
        "For every match the flat-cost model reports as under budget, the counterfactual: what if "
        "`elapsed_s` had used this exact match's REAL measured total planning time instead of "
        f"`replans * PLANNING_OVERHEAD_S`? (This also charges the first plan, which the flat model never "
        "charges at all.)",
        "",
        "| Grid size | Under-budget matches | Flipped to over-budget | Rate | 95% CI |",
        "|---:|---:|---:|---:|---|",
    ]
    any_flips = False
    for m in RESOLUTION_MULTIPLIERS:
        s = flip_stats[m]
        if s["flips"] > 0:
            any_flips = True
        lines.append(f"| {FTC_GRID_SIZE * m} | {s['n']} | {s['flips']} | {s['rate']:.1%} | "
                     f"[{s['ci_lo']:.1%}, {s['ci_hi']:.1%}] |")

    native_flips = flip_stats[1]["flips"]
    if native_flips == 0:
        native_summary = (
            f"At this project's actual published grid scale ({FTC_GRID_SIZE} cells/side, resolution "
            "multiplier 1x -- what every existing benchmark_results/ CSV in this repo was measured at), no "
            "match flips outcome under this counterfactual. The tail is real (see the distribution table "
            "above) but never large enough, at this grid size, to move `elapsed_s` past "
            "`AUTONOMOUS_PERIOD_S` on top of everything else already charged in a 30-second match -- "
            "planning latency's absolute scale (single-digit milliseconds even at p99) is just too small "
            "relative to the budget for this to matter here. This BOUNDS the question this project's other "
            "benchmarks leave unstated: not \"could tail latency ever matter\" (yes, at large enough scale "
            "-- see the rows above) but \"does it matter at the scale every published number in this repo "
            "was actually measured at\" (no)."
        )
    else:
        native_summary = (
            f"At this project's actual published grid scale ({FTC_GRID_SIZE} cells/side, resolution "
            f"multiplier 1x -- what every existing benchmark_results/ CSV in this repo was measured at), "
            f"{native_flips} match(es) flip outcome under this counterfactual. This means at least one of "
            "this project's own published headline trials could, in principle, have gone the other way "
            "under measured rather than assumed planning time -- worth treating as a real, if narrow, "
            "caveat on every over_budget-adjacent claim at native scale."
        )
    lines += ["", native_summary, ""]

    if any_flips and not native_flips:
        lines.append(
            "At synthetic grid sizes beyond this project's real scale, the counterfactual does flip some "
            "matches -- confirming the mechanism is real, just not triggered anywhere this project actually "
            "publishes numbers from."
        )
    elif not any_flips:
        lines.append(
            "No synthetic grid size tested flips any match either -- the gap between measured tail latency "
            "and the 30-second budget stays wide even at 16x this project's native grid size."
        )

    lines += [
        "",
        "## Replan frequency by suite",
        "",
        "| Suite | Mean replans / match |",
        "|---|---:|",
    ]
    for suite in SUITE_ORDER:
        lines.append(f"| {SUITE_LABELS[suite]} | {replan_stats[suite]:.2f} |")
    apriltag_vs_distance = replan_stats["apriltag"] - replan_stats["distance_sensors"]
    lines += ["", (
        f"AprilTag replans more often than DistanceSensorSuite in this sweep too "
        f"({replan_stats['apriltag']:.2f} vs. {replan_stats['distance_sensors']:.2f} per match) -- consistent "
        "with ftc/budget_benchmark.py's own docstring claim (AprilTag replans on every pose correction, not "
        "just on newly-sensed obstacles) rather than contradicting it."
        if apriltag_vs_distance > 0 else
        f"AprilTag replans LESS often than DistanceSensorSuite in this specific sweep "
        f"({replan_stats['apriltag']:.2f} vs. {replan_stats['distance_sensors']:.2f} per match) -- worth "
        "noting against ftc/budget_benchmark.py's docstring claim of the opposite; this sweep's fixed "
        "deviation mix and bounded path lengths differ from that module's own sweep, so the two aren't "
        "measuring identical conditions."
    ), ""]

    lines += [
        "## Honest findings",
        "",
        "- **Hardware.** Every latency figure above was measured on the machine named at the top of this "
        "file -- a developer laptop, not FTC-legal competition hardware (a REV Control Hub). "
        "PLANNING_OVERHEAD_S is explicitly documented as an estimate of REAL onboard-compute cost, "
        "acknowledged in that same comment to be well above raw Python astar() time on a machine like this "
        "one. These results say nothing about whether PLANNING_OVERHEAD_S is calibrated correctly for real "
        "hardware -- only ftc/calibration.py run against real onboard measurements could do that. A slower "
        "target would show this same tail-latency effect at a smaller grid size than reported here, not a "
        "different effect.",
        "- **`cell_size_in` was deliberately not swept.** Every `*_CELLS` sensor-range constant in ftc/"
        "config.py (APRILTAG_RANGE_CELLS, DISTANCE_SENSOR_RANGE_CELLS, LIDAR_RANGE_CELLS, ...) is computed "
        "once at import time from the native CELL_SIZE_IN and does not read whatever `cell_size_in` a "
        "particular `build_grid()` call used. This study only ever varies `size` (a bigger physical field, "
        "same 6in cells), which keeps every sensor's real-world range correct -- but it means `ftc/field.py`'s "
        "`cell_size_in` parameter is not currently safe to vary independently of the constants in ftc/"
        "config.py, a limitation this study surfaced while designing its own scenario generation rather than "
        "something already documented elsewhere in this repo.",
        "- **Grid sizes beyond 24 cells/side are a synthetic computational stress test, not a claim about a "
        "physically larger competition field.** `ftc/field.py`'s `DEFAULT_TAG_SITES` (and every named "
        "layout's own obstacle placement) are fixed in real inches and do not reposition as `size` grows -- "
        "at 16x scale, AprilTag's tag sites sit in a small corner of a much bigger grid rather than spread "
        "across its actual perimeter. The replan-frequency table above pools across sizes, so this doesn't "
        "distort it, but any AprilTag-specific number broken out BY size at the larger end should be read "
        "with that in mind.",
        "- Start/goal pairs are bounded to a comparable path-length range at every grid size (see module "
        "docstring) specifically so drive time doesn't confound this study's grid-size axis -- a genuinely "
        "unconstrained scenario sampler would make matches at large sizes fail from drive time alone long "
        "before planning latency became relevant, which would answer a different question than the one asked.",
    ]

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    all_rows = []
    for multiplier in RESOLUTION_MULTIPLIERS:
        for layout in LAYOUT_ORDER:
            for suite_name in SUITE_ORDER:
                base_seed = BASE_SEED + multiplier * 1_000_000 + LAYOUT_ORDER.index(layout) * 100_000 \
                    + SUITE_ORDER.index(suite_name) * 1_000
                print(f"size={FTC_GRID_SIZE * multiplier} layout={layout} suite={suite_name} ...")
                all_rows.extend(run_combo(multiplier, layout, suite_name, TRIALS_PER_COMBO, base_seed))

    write_csv(all_rows, OUTPUT_DIR / "planning_latency.csv")
    plot_comparison(all_rows, OUTPUT_DIR / "planning_latency_comparison.png")
    write_writeup(all_rows, OUTPUT_DIR / "planning_latency_writeup.md")

    print(f"\nWrote {len(all_rows)} planning-call rows to {OUTPUT_DIR / 'planning_latency.csv'}")
    print(f"Chart saved to {OUTPUT_DIR / 'planning_latency_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'planning_latency_writeup.md'}")

    stats = latency_stats_by_multiplier(all_rows)
    for m in RESOLUTION_MULTIPLIERS:
        s = stats[m]
        print(f"size={FTC_GRID_SIZE * m:>4}: median={s['median'] * 1000:.2f}ms p99={s['p99'] * 1000:.2f}ms "
              f"max={s['max'] * 1000:.2f}ms")
    flips = flip_stats_by_multiplier(all_rows)
    print(f"\nOutcome flips at native scale ({FTC_GRID_SIZE} cells): {flips[1]['flips']}/{flips[1]['n']}")
