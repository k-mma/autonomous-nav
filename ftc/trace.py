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
