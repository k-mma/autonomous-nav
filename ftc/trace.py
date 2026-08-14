"""
Turns one ftc/match.py run_match() call into a full tick-by-tick replay
trace instead of just a final MatchResult -- what pygame_app/ftc_viz/'s
animated field view (the RoadRunner-style "robot moving across the
field" demo) is built on. Kept in ftc/, not pygame_app/, since
recording a trace is a pure simulation concern independent of how (or
whether) anything ever draws it -- pygame_app/ftc_viz/ is the only
consumer today, but nothing about this module is pygame-specific, the
same reasoning that keeps ftc/sensors.py's ConeSensor out of nav/.

run_match()'s own `on_tick` hook is purely additive by design -- a
read-only callback invoked with already-computed snapshot data at a
handful of points in the match loop (see ftc/match.py's own docstring
for the full safety argument). That's what keeps this module safe to
add without touching the byte-for-byte optimistic-tier regression
guarantee ftc/scratch/fidelity_test.py enforces: recording a trace
cannot change what run_match() itself computes, only what else comes
back alongside it. ftc/scratch/trace_test.py verifies this directly
(same seeded match, with and without a trace recorded, byte-identical
MatchResult).

Since Priority 4's tag-diagnostics addition, every tick's snapshot also
carries `tag_diagnostics` (ftc/sensors.py's diagnose_tag_detections) --
per-tag-site range/FOV/camera-FOV/line-of-sight gate detail, not just
whether a correction happened to fire that tick. `python3 -m ftc.trace`
(see the CLI at the bottom of this file) is the human-readable front
end for that: run one real match and print exactly which tags were
geometrically in play, tick by tick, and which ticks actually produced
a correction -- the question "which AprilTags would the robot have
been able to detect along its path" answered by running the simulation
and reading off what happened, not by guessing.
"""
from dataclasses import dataclass, field

from ftc.match import run_match


@dataclass
class MatchTrace:
    """`result` is bit-for-bit the same ftc.match.MatchResult a plain
    run_match() call on the identical arguments would have returned.
    `ticks` is the ordered list of snapshot dicts run_match's on_tick
    callback was invoked with -- see ftc/match.py's _emit for exactly
    what each one contains and when it fires ("start" once, "step" per
    successful move, "collision" for a rejected move, "end" once with
    the final outcome). `suite` is the exact suite instance the match
    was run with (its .name/.cost_usd/.camera_mount_headings_deg etc.
    are what pygame_app/ftc_viz/ needs to know what to draw)."""
    result: object
    suite: object
    ticks: list = field(default_factory=list)


def record_match(suite, assumed_grid, start, goal, ground_truth, actual_start, tag_sites, rng, **kwargs):
    """Runs exactly one ftc.match.run_match call -- every kwarg
    (fidelity, drivetrain, gearing, moving_obstacles, ...) passes
    through unchanged -- and additionally captures its full tick-by-
    tick trace via run_match's on_tick hook. Returns a MatchTrace."""
    ticks = []
    result = run_match(suite, assumed_grid, start, goal, ground_truth, actual_start, tag_sites, rng,
                        on_tick=ticks.append, **kwargs)
    return MatchTrace(result=result, suite=suite, ticks=ticks)


def _format_tag_line(diag):
    gates = f"range={'OK' if diag['in_range'] else 'FAIL'}({diag['range_cells']:.1f}c)"
    gates += f" tag_fov={'OK' if diag['in_tag_fov'] else 'FAIL'}({diag['incidence_deg']:.0f}deg)"
    gates += f" cam_fov={'OK' if diag['in_camera_fov'] else 'FAIL'}"
    gates += f" los={'OK' if diag['in_line_of_sight'] else 'FAIL'}"
    verdict = "ELIGIBLE" if diag["geometrically_eligible"] else "not eligible"
    return f"tag[{diag['tag_index']}] {gates} -> {verdict}"


def print_detection_timeline(trace, only_eligible_or_detected=True):
    """Human-readable per-tick AprilTag detection timeline for one
    recorded MatchTrace -- the question "which tags would the robot
    have been able to detect along its path, and when" answered by
    reading off what a real run actually did, not by inspecting the
    geometry by hand. `only_eligible_or_detected` (default True) skips
    printing a tick line at all when nothing about it is interesting
    (no tag geometrically eligible, no correction fired) -- set False
    to see every single tick, including the quiet ones.
    """
    print(f"Suite: {trace.suite.name}  |  {len(trace.ticks)} ticks recorded  |  "
          f"success={trace.result.success}  final_pose_error_in={trace.result.final_pose_error_in:.2f}")
    print("-" * 78)
    any_detection = False
    for snap in trace.ticks:
        diagnostics = snap.get("tag_diagnostics") or []
        any_eligible = any(d["geometrically_eligible"] for d in diagnostics)
        tag_corrected = snap.get("tag_corrected", False)
        if tag_corrected:
            any_detection = True
        if only_eligible_or_detected and not (any_eligible or tag_corrected):
            continue
        pos = snap["true_position"]
        line = (f"tick {snap['tick']:>3} (t={snap['elapsed_s']:6.2f}s) event={snap['event']:<10} "
                f"pos=({pos[0]:.1f},{pos[1]:.1f}) heading={snap['heading_deg']:6.1f}deg")
        if tag_corrected:
            line += "  *** CORRECTED ***"
        print(line)
        for diag in diagnostics:
            if diag["geometrically_eligible"] or not only_eligible_or_detected:
                print(f"    {_format_tag_line(diag)}")
    print("-" * 78)
    if not any_detection:
        print("No AprilTag detection actually fired during this match "
              "(a tag may have been geometrically eligible without ever clearing dropout, or none ever were).")


def _build_cli_scenario(suite_name, layout, seed, deviation_type, level, fidelity):
    """Builds exactly the kind of scenario ftc/suite_benchmark.py's own
    run_combo builds, reused here rather than duplicated: same field/
    tag-site construction, same generate_ground_truth call shape. A
    thin CLI-only helper, not something any benchmark imports."""
    from nav.field_variance import generate_ground_truth

    from ftc.field import build_grid, tag_sites_for
    from ftc.sensors import SUITES
    from ftc.suite_benchmark import DEVIATION_TYPES, _solvable_scenario

    grid = build_grid(layout)
    free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    tag_sites = tag_sites_for(layout)
    start, goal = _solvable_scenario(seed, grid, free_cells)
    scale_kwargs = DEVIATION_TYPES[deviation_type]
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, level, seed=seed, **scale_kwargs)
    suite = SUITES[suite_name]()
    return suite, grid, start, goal, ground_truth, actual_start, tag_sites


if __name__ == "__main__":
    import argparse
    import random

    from ftc.sensors import SUITE_ORDER
    from ftc.suite_benchmark import DEVIATION_TYPE_ORDER

    parser = argparse.ArgumentParser(
        description="Run one real FTC match and print its AprilTag detection timeline tick by tick -- "
                     "which tags were geometrically in play, and which ticks actually produced a correction.")
    parser.add_argument("--suite", choices=SUITE_ORDER, default="apriltag")
    parser.add_argument("--layout", choices=["sparse", "cluttered", "corridor"], default="cluttered")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--deviation-type", choices=DEVIATION_TYPE_ORDER, default="start_drift")
    parser.add_argument("--level", type=float, default=0.5)
    parser.add_argument("--fidelity", choices=["optimistic", "realistic", "pessimistic"], default=None)
    parser.add_argument("--all-ticks", action="store_true",
                         help="Print every tick, including ones where no tag was eligible and none fired.")
    args = parser.parse_args()

    suite_reads_tags = args.suite in ("apriltag", "full_suite", "apriltag_imu", "dual_camera_apriltag")
    if not suite_reads_tags:
        print(f"NOTE: suite '{args.suite}' never reads a tag at all (fixes_pose is False or it doesn't use "
              "tag_correction) -- every tick's tag_diagnostics will be empty. Pass --suite apriltag (or "
              "full_suite/apriltag_imu/dual_camera_apriltag) to see real detection gate data.")

    suite, grid, start, goal, ground_truth, actual_start, tag_sites = _build_cli_scenario(
        args.suite, args.layout, args.seed, args.deviation_type, args.level, args.fidelity)

    print(f"Scenario: suite={args.suite} layout={args.layout} seed={args.seed} "
          f"deviation_type={args.deviation_type} level={args.level} fidelity={args.fidelity or '(default)'}")
    print(f"start={start} goal={goal} actual_start={actual_start} tag_sites={len(tag_sites)}")
    print()

    trace = record_match(suite, grid, start, goal, ground_truth, actual_start, tag_sites,
                          random.Random(args.seed), fidelity=args.fidelity)
    print_detection_timeline(trace, only_eligible_or_detected=not args.all_ticks)
