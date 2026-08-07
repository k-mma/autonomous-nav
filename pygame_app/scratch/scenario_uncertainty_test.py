"""
Does pygame_app/scenarios/scenario_uncertainty.py's pygame rendering path
actually run -- not just the policy-stepping logic underneath it (Trial,
generate_ground_truth, the three nav.policies classes), which every
other test in this project already exercises without pygame at all.
scenario_uncertainty.py has its own standalone main()/event loop (it
isn't built on pygame_app.scenario.ScenarioConfig, so screenshots/pygame/
capture.py's scenario list can't drive it), and nobody had ever run it
under a headless display to confirm the drawing code itself doesn't
crash -- font rendering, the three draw_belief_panel branches (no
sensing / known-obstacle set / occupancy-probability lerp), the trail
polyline, the collision X-mark.

Same technique screenshots/pygame/capture.py uses: SDL_VIDEODRIVER=dummy
(no real window needed) and pygame.display.flip patched to count frames
and post synthetic input, so main()'s own event loop drives everything
exactly like a real interactive session would, just faster and without
a display.
"""
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

import pygame

import pygame_app.scenarios.scenario_uncertainty as scenario_uncertainty

# Frame numbers (at the scenario's own clock.tick(60)) to inject a
# policy-switch keypress at, so every draw_belief_panel branch actually
# renders at least once in a single headless run instead of only ever
# exercising K_1 (Open-loop)'s "no sensing" branch, the one main()
# starts on by default.
SWITCH_TO_REACTIVE_FRAME = 20
SWITCH_TO_BELIEF_FRAME = 40
# STEP_MS=250 at 60fps is one Trial.tick() every ~15 frames -- 70 frames
# is enough for several ticks under whichever policy is currently
# selected, so the trail-polyline and (for open-loop) collision-X-mark
# drawing branches get exercised too, not just the static first frame.
QUIT_AFTER_FRAME = 70


def run_headless():
    """Runs scenario_uncertainty.main() to completion under a dummy
    display, injecting policy-switch keys partway through. Returns
    nothing -- an exception escaping this function (a pygame/font/draw
    crash) is the failure this test exists to catch; the caller decides
    pass/fail from whether it raised."""
    frame_count = [0]
    real_flip = pygame.display.flip

    def patched_flip():
        frame_count[0] += 1
        result = real_flip()
        if frame_count[0] == SWITCH_TO_REACTIVE_FRAME:
            pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_2))
        elif frame_count[0] == SWITCH_TO_BELIEF_FRAME:
            pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_3))
        elif frame_count[0] == QUIT_AFTER_FRAME:
            pygame.event.post(pygame.event.Event(pygame.QUIT))
        return result

    pygame.display.flip = patched_flip
    try:
        scenario_uncertainty.main()
    finally:
        pygame.display.flip = real_flip


def check_headless_run_completes():
    try:
        run_headless()
        ok = True
    except Exception as e:
        ok = False
        print(f"  scenario_uncertainty.main() raised under a headless dummy display: {e!r}")
    print(f"scenario_uncertainty.py's pygame rendering path runs headlessly without crashing: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_headless_run_completes():
    assert check_headless_run_completes()


if __name__ == "__main__":
    checks = [check_headless_run_completes()]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
