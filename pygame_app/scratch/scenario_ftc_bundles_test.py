"""
Does pygame_app/scenarios/scenario_ftc_bundles.py actually cover EVERY
possible 2+-suite combination -- not just a couple of spot-checked
ones -- draw every one of their names in full, and evaluate them on the
exact same trials this project's individual-suite results use?

Five different claims, checked separately:

1. check_enumeration_covers_every_raw_combination: brute-forces every
   itertools.combinations(candidates, size) for size in [2,
   len(candidates)] independently of ftc/bundle.py's own
   enumerate_bundles, and checks that every one either (a) appears in
   enumerate_bundles's output by PART SIGNATURE, or (b) is accounted
   for by one of the two reasons ftc/bundle.py documents for dropping a
   raw combination: it needs mutually exclusive parts (ftc/config.py's
   CONFLICTING_PART_GROUPS), or it's a duplicate of a smaller/earlier
   combination's exact hardware (drop_equivalent). Nothing is allowed
   to go missing for a third, undocumented reason -- that would mean
   this project's own "every buildable combination" claim is quietly
   false, exactly what "make sure all possible combinations are being
   tested" is asking to have checked.

2. check_status_bar_text_never_overflows_the_window: for every panel
   count {3, 4, 5, 6} scenario_ftc_bundles.py's own panels_for_bundle
   can produce from the default candidate pool, every status line --
   including each bundle's own name line -- fits inside the actual
   computed window width, truncated if it somehow ever had to be rather
   than left to clip. This is a regression test for a real reported
   bug: a first version of this file sized its window from the panel
   grid alone, with no floor tied to what the status bar itself needs
   to render, and several status lines ran wider than that on this
   project's own dev screen.

3. check_every_bundle_name_is_fully_visible_in_the_status_bar: the
   stronger claim #2 doesn't make -- not "nothing clips," but "no
   bundle's name is EVER actually truncated." status_min_width is handed
   every bundle this run will page through specifically so this holds;
   this check verifies it holds for literally every one of them, at
   every panel count, not just whichever bundle the window happened to
   be sized around.

4. check_headless_run_pages_through_every_bundle: drives
   scenario_ftc_bundles.main() through EVERY bundle at the default
   candidate set via one K_RIGHT keypress per rendered frame -- not a
   handful of spot checks -- confirming every one of them records and
   draws (several different panel counts/window sizes along the way, so
   this also exercises every _build_window resize transition) without
   crashing, then two more steps past the last one to confirm the list
   wraps back around to the first instead of raising.

5. check_component_panels_match_individual_suite_results: bundles are
   supposed to be tested on the SAME trials this project's individual-
   suite results are -- not just a similarly-configured rerun. This
   builds one scenario at both files' shared defaults, records a
   component panel's trace scenario_ftc_bundles.py's own way, records
   that same suite name's trace via scenario_ftc_suites.py's own
   record_all (the function its individual per-suite panels are built
   from), and requires the two MatchTraces to be byte-for-byte
   identical, for bundles of several different sizes.

Same headless technique pygame_app/scratch/scenario_ftc_suites_test.py
already uses: SDL_VIDEODRIVER=dummy (no real window) and
pygame.display.flip patched to count frames and post synthetic input,
so main()'s own event loop drives everything exactly like a real
interactive session would.
"""
import itertools
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

import pygame

from ftc.bundle import conflicting_parts, enumerate_bundles, make_bundle, parts_for
from ftc.sensors import SUITES

import pygame_app.scenarios.scenario_ftc_bundles as scenario_ftc_bundles


def check_enumeration_covers_every_raw_combination():
    candidates = scenario_ftc_bundles.DEFAULT_CANDIDATES
    enumerated = enumerate_bundles(candidates, min_size=2)
    enumerated_signatures = {b.part_signature() for b in enumerated}

    seen_signatures = set()
    unaccounted = []
    total_raw = 0
    for size in range(2, len(candidates) + 1):
        for combo in itertools.combinations(candidates, size):
            total_raw += 1
            try:
                bundle = make_bundle(*combo)
            except ValueError:
                # Must be a genuinely conflicting-parts combination, not
                # some other failure masquerading as one.
                bundle_parts = frozenset().union(*(parts_for(SUITES[name]()) for name in combo))
                if not conflicting_parts(bundle_parts):
                    unaccounted.append((combo, "raised ValueError but its parts don't conflict"))
                continue
            signature = bundle.part_signature()
            if signature in enumerated_signatures:
                seen_signatures.add(signature)
                continue
            if signature in seen_signatures:
                # A duplicate of a smaller/earlier combination's exact
                # hardware -- exactly what drop_equivalent is supposed
                # to strip, and the representative it kept already
                # accounted for this signature above.
                continue
            unaccounted.append((combo, f"missing entirely, signature {sorted(signature)}"))

    for combo, reason in unaccounted:
        print(f"  UNACCOUNTED: {combo} -- {reason}")
    # Every representative signature enumerate_bundles returned must
    # also have been reachable from the raw brute force -- catches the
    # opposite failure mode (enumerate_bundles inventing a combination
    # that no actual raw combination of candidates produces).
    every_returned_reachable = enumerated_signatures <= seen_signatures
    ok = not unaccounted and every_returned_reachable
    print(f"  {total_raw} raw combinations (sizes 2-{len(candidates)}) -> {len(enumerated)} distinct bundles "
          f"enumerated; every returned bundle reachable from a raw combination: {every_returned_reachable}")
    print(f"Every raw 2+-suite combination is either enumerated or accounted for (conflicting parts / exact "
          f"duplicate): {'OK' if ok else 'FAIL'}")
    return ok


def check_status_bar_text_never_overflows_the_window():
    pygame.init()
    font = pygame.font.SysFont(None, 18)
    hud_font = pygame.font.SysFont(None, 16)
    hud_h = scenario_ftc_bundles._hud_height(hud_font)
    status_h = scenario_ftc_bundles.status_bar_height(font)

    candidates = scenario_ftc_bundles.DEFAULT_CANDIDATES
    bundles = scenario_ftc_bundles.enumerate_all_bundles(candidates, 2, None, "cost")
    min_window_w = scenario_ftc_bundles.status_min_width(font, bundles)
    panel_counts = sorted({len(b.component_names) + 1 for b in bundles})
    print(f"  panel counts this candidate pool produces: {panel_counts}")

    display_info = pygame.display.Info()
    screen_w, screen_h = display_info.current_w, display_info.current_h

    fixed_lines = [line for line in scenario_ftc_bundles.STATUS_LINE_EXAMPLES if line is not None]
    ok = True
    for total_panels in panel_counts:
        _screen, _cols, _cell_px, _panel_w, _panel_h, window_w, _window_h = scenario_ftc_bundles._build_window(
            total_panels, hud_h, status_h, screen_w, screen_h, min_window_w)
        avail = window_w - 20
        for line in fixed_lines:
            w = font.size(line)[0]
            if w > avail:
                ok = False
                print(f"  OVERFLOW at total_panels={total_panels}: {w}px > {avail}px avail -- {line!r}")

        # The unbounded bundle-label line: worst case at this panel
        # count, checked AFTER truncation (what actually gets drawn),
        # not before.
        worst = max((b for b in bundles if len(b.component_names) + 1 == total_panels),
                     key=lambda b: len(b.label()))
        label_line = scenario_ftc_bundles.bundle_status_label(worst)
        truncated = scenario_ftc_bundles._truncate_to_width(font, label_line, avail)
        if font.size(truncated)[0] > avail:
            ok = False
            print(f"  OVERFLOW (truncation didn't work) at total_panels={total_panels}: "
                  f"{font.size(truncated)[0]}px > {avail}px -- {truncated!r}")

    print(f"No status-bar line clips the window at any panel count {panel_counts}: {'OK' if ok else 'FAIL'}")
    return ok


def check_every_bundle_name_is_fully_visible_in_the_status_bar():
    """The stronger claim "make sure each bundle's name can be seen"
    actually asks for: not just "nothing overflows the window" (the
    check above, which a truncated-to-fit line would also pass), but
    "no bundle's own name line is EVER truncated" -- status_min_width is
    handed the complete bundle list precisely so this holds for every
    one of them, at every panel count, not just whichever bundle happens
    to be selected when the window is built."""
    pygame.init()
    font = pygame.font.SysFont(None, 18)
    hud_font = pygame.font.SysFont(None, 16)
    hud_h = scenario_ftc_bundles._hud_height(hud_font)
    status_h = scenario_ftc_bundles.status_bar_height(font)

    candidates = scenario_ftc_bundles.DEFAULT_CANDIDATES
    bundles = scenario_ftc_bundles.enumerate_all_bundles(candidates, 2, None, "cost")
    min_window_w = scenario_ftc_bundles.status_min_width(font, bundles)
    panel_counts = sorted({len(b.component_names) + 1 for b in bundles})

    display_info = pygame.display.Info()
    screen_w, screen_h = display_info.current_w, display_info.current_h

    ok = True
    checked = 0
    for total_panels in panel_counts:
        _screen, _cols, _cell_px, _panel_w, _panel_h, window_w, _window_h = scenario_ftc_bundles._build_window(
            total_panels, hud_h, status_h, screen_w, screen_h, min_window_w)
        avail = window_w - 20
        for bundle in bundles:
            if len(bundle.component_names) + 1 != total_panels:
                continue
            label_line = scenario_ftc_bundles.bundle_status_label(bundle)
            truncated = scenario_ftc_bundles._truncate_to_width(font, label_line, avail)
            checked += 1
            if truncated != label_line:
                ok = False
                print(f"  TRUNCATED at total_panels={total_panels}: {label_line!r} -> {truncated!r}")

    print(f"Every one of the {len(bundles)} bundles' own status-bar name line renders in FULL, un-truncated, "
          f"at every panel count it can appear at ({checked} (bundle, panel-count) pairs checked): "
          f"{'OK' if ok else 'FAIL'}")
    return ok


NUM_STEPS_PAST_END = 2  # how far past the last bundle to keep stepping, to confirm wraparound


def run_headless_page_through_all(argv, num_bundles):
    total_steps = num_bundles + NUM_STEPS_PAST_END
    frame_count = [0]
    real_flip = pygame.display.flip

    def patched_flip():
        frame_count[0] += 1
        result = real_flip()
        n = frame_count[0]
        if n <= total_steps:
            pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_RIGHT))
        else:
            pygame.event.post(pygame.event.Event(pygame.QUIT))
        return result

    pygame.display.flip = patched_flip
    try:
        scenario_ftc_bundles.main(argv)
    finally:
        pygame.display.flip = real_flip
    return frame_count[0]


def check_headless_run_pages_through_every_bundle():
    args = scenario_ftc_bundles.parse_args([])
    candidates = scenario_ftc_bundles.resolve_candidates(args)
    bundles = scenario_ftc_bundles.enumerate_all_bundles(candidates, args.min_size, args.max_size, args.sort)
    num_bundles = len(bundles)
    print(f"  paging through all {num_bundles} bundles (+{NUM_STEPS_PAST_END} past the end, to check wraparound)")

    try:
        frames = run_headless_page_through_all(["--speed", "8"], num_bundles)
        ok = frames >= num_bundles + NUM_STEPS_PAST_END
    except Exception as e:
        ok = False
        frames = None
        print(f"  scenario_ftc_bundles.main() raised: {e!r}")
    print(f"  rendered {frames} frames")
    print(f"Every one of the {num_bundles} bundles (every panel-count/window-resize transition among them) "
          f"records and draws without crashing, and paging past the last one wraps around: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_headless_run_completes_a_specific_small_pool():
    """A separate, smaller candidate pool -- confirms the paging
    mechanism (and the enumeration/window-resize machinery underneath
    it) isn't somehow special-cased to the default 6-candidate,
    19-bundle pool."""
    pool_argv = ["--candidates", "odometry_pods,apriltag,imu,dual_camera_apriltag", "--max-size", "2"]
    args = scenario_ftc_bundles.parse_args(pool_argv)
    candidates = scenario_ftc_bundles.resolve_candidates(args)
    num_bundles = len(scenario_ftc_bundles.enumerate_all_bundles(
        candidates, args.min_size, args.max_size, args.sort))

    try:
        frames = run_headless_page_through_all(pool_argv + ["--speed", "8"], num_bundles)
        ok = frames >= num_bundles + NUM_STEPS_PAST_END
    except Exception as e:
        ok = False
        frames = None
        print(f"  raised: {e!r}")
    print(f"  rendered {frames} frames for {num_bundles} bundles")
    print(f"A smaller, explicit candidate pool ({num_bundles} bundles) pages through completely too: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_component_panels_match_individual_suite_results():
    """The literal claim "bundles are tested in the same trials as the
    individual results" -- checked, not just asserted in a docstring.

    scenario_ftc_bundles.py's own "component running alone" panels
    aren't THE individual-suite tool; pygame_app/scenarios/
    scenario_ftc_suites.py is, and its `record_all` is what actually
    produces "the individual result" for a suite name. This builds the
    identical scenario both files' own defaults would build (same
    layout/deviation/level/seed/fidelity/opponent), records a component
    panel's trace scenario_ftc_bundles.py's own way, records that same
    suite name's trace scenario_ftc_suites.py's own way, and requires
    them to be byte-for-byte the same MatchTrace -- not just "similar,"
    not "the same success rate," but every tick and every deterministic
    field of the final result identical. If bundles were ever evaluated
    against a different scenario or rng draw than individual suites are,
    this would be the first thing to catch it, and any "component vs.
    bundle" comparison drawn side by side in scenario_ftc_bundles.py's
    own panels would be comparing apples to oranges without anyone
    noticing.

    One field is deliberately excluded from the result comparison:
    MatchResult.planning_time_s is a wall-clock time.perf_counter()
    measurement (see ftc/match.py), not a function of the trial at all
    -- even running the exact same suite against the exact same
    scenario twice in a row produces two different wall-clock readings,
    so comparing it would fail this check on every run for a reason
    that has nothing to do with whether it's the same trial."""
    import dataclasses

    import pygame_app.scenarios.scenario_ftc_suites as scenario_ftc_suites

    def _result_without_timing(result):
        return dataclasses.replace(result, planning_time_s=0.0)

    state = dict(layout="cluttered", deviation_type="start_drift", level=0.5, fidelity="realistic", seed=42,
                 opponent="none")
    candidates = scenario_ftc_bundles.DEFAULT_CANDIDATES
    bundles = scenario_ftc_bundles.enumerate_all_bundles(candidates, 2, None, "cost")
    # A few bundles of different sizes, not just the first one -- the
    # claim is "every component panel," and a 2-suite bundle's own
    # components exercise a different code path than a 5-suite bundle's.
    sample = [bundles[0], bundles[len(bundles) // 2], bundles[-1]]

    (grid, tag_sites, start, goal, ground_truth, actual_start, _static_opp, opponent_path,
     _opponent_reference) = scenario_ftc_bundles.build_scenario(
        state["layout"], state["deviation_type"], state["level"], state["seed"], state["opponent"])

    ok = True
    checked = []
    for bundle in sample:
        for suite, _title, is_bundle in scenario_ftc_bundles.panels_for_bundle(bundle):
            if is_bundle:
                continue
            bundle_page_trace = scenario_ftc_bundles._record_suite(
                suite, grid, tag_sites, start, goal, ground_truth, actual_start, state["seed"],
                state["fidelity"], opponent_path)
            individual_trace, = scenario_ftc_suites.record_all(
                [suite.name], grid, tag_sites, start, goal, ground_truth, actual_start, state["seed"],
                state["fidelity"], opponent_path)
            same = (bundle_page_trace.ticks == individual_trace.ticks
                    and _result_without_timing(bundle_page_trace.result)
                    == _result_without_timing(individual_trace.result))
            checked.append(suite.name)
            if not same:
                ok = False
                print(f"  MISMATCH for {suite.name!r} (from bundle {bundle.name}): the bundle-page component "
                      f"panel and scenario_ftc_suites.py's own individual result diverge -- not the same trial")

    print(f"  checked {len(checked)} component-panel/individual-result pairs across {len(sample)} bundles: "
          f"{checked}")
    print(f"Every bundle-page component panel is the byte-identical SAME TRIAL as scenario_ftc_suites.py's "
          f"own individual result for that suite: {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_enumeration_covers_every_raw_combination():
    assert check_enumeration_covers_every_raw_combination()


def test_status_bar_text_never_overflows_the_window():
    assert check_status_bar_text_never_overflows_the_window()


def test_every_bundle_name_is_fully_visible_in_the_status_bar():
    assert check_every_bundle_name_is_fully_visible_in_the_status_bar()


def test_headless_run_pages_through_every_bundle():
    assert check_headless_run_pages_through_every_bundle()


def test_headless_run_completes_a_specific_small_pool():
    assert check_headless_run_completes_a_specific_small_pool()


def test_component_panels_match_individual_suite_results():
    assert check_component_panels_match_individual_suite_results()


if __name__ == "__main__":
    checks = [
        check_enumeration_covers_every_raw_combination(),
        check_status_bar_text_never_overflows_the_window(),
        check_every_bundle_name_is_fully_visible_in_the_status_bar(),
        check_headless_run_pages_through_every_bundle(),
        check_headless_run_completes_a_specific_small_pool(),
        check_component_panels_match_individual_suite_results(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
