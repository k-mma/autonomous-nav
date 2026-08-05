"""
Does pygame_app/scenarios/scenario_ftc_suites.py's rendering path
actually run -- not just ftc/trace.py's recording logic underneath it
(already covered headlessly by ftc/scratch/trace_test.py without
pygame at all)? Nobody had run this under a headless display to
confirm the drawing code itself doesn't crash: pygame_app/ftc_viz/
field_view.py's grid/tag/trail/robot/sensor-cone drawing, every
sensor_kind branch (cone/camera/lidar/none), the collision mark, and
scenario_ftc_suites.py's own interactive handlers (reroll, deviation-
type cycling, variance_level clamping, fidelity cycling).

Same technique pygame_app/scratch/scenario_uncertainty_test.py already
uses: SDL_VIDEODRIVER=dummy (no real window) and pygame.display.flip
patched to count frames and post synthetic input, so main()'s own event
loop drives everything exactly like a real interactive session would.

Runs with --suites all (every suite ftc/sensors.py defines, including
the Priority-3/4 additions the headline sweep never touches) so every
sensor_kind branch in field_view.draw_sensor_visual gets exercised at
least once in a single pass, not just whichever branch the 5 headline
suites happen to cover.
"""
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

import pygame

import pygame_app.scenarios.scenario_ftc_suites as scenario_ftc_suites

# Frame numbers (at the scenario's own clock.tick(60)) to inject an
# interactive keypress at, so the reroll/deviation-cycle/level-change/
# fidelity-cycle handlers (each of which re-records every trace) all
# actually run at least once in a single headless pass, not just the
# static first frame's default scenario.
REROLL_FRAME = 15
CYCLE_DEVIATION_FRAME = 30
RAISE_LEVEL_FRAME = 45
CYCLE_FIDELITY_FRAME = 60
STEP_BACK_FRAME = 75
QUIT_AFTER_FRAME = 90


def run_headless(argv):
    frame_count = [0]
    real_flip = pygame.display.flip

    def patched_flip():
        frame_count[0] += 1
        result = real_flip()
        n = frame_count[0]
        if n == REROLL_FRAME:
            pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_r))
        elif n == CYCLE_DEVIATION_FRAME:
            pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_n))
        elif n == RAISE_LEVEL_FRAME:
            pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_RIGHTBRACKET))
        elif n == CYCLE_FIDELITY_FRAME:
            pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_f))
        elif n == STEP_BACK_FRAME:
            pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_SPACE))  # pause
            pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_LEFT))
        elif n == QUIT_AFTER_FRAME:
            pygame.event.post(pygame.event.Event(pygame.QUIT))
        return result

    pygame.display.flip = patched_flip
    try:
        scenario_ftc_suites.main(argv)
    finally:
        pygame.display.flip = real_flip


def check_headless_run_completes_all_suites():
    try:
        run_headless(["--suites", "all", "--fps", "8"])
        ok = True
    except Exception as e:
        ok = False
        print(f"  scenario_ftc_suites.main(['--suites', 'all']) raised: {e!r}")
    print(f"scenario_ftc_suites.py runs headlessly (every sensor_kind branch: cone/camera/lidar/none) "
          f"without crashing: {'OK' if ok else 'FAIL'}")
    return ok


def check_headless_run_completes_single_suite():
    try:
        run_headless(["--suite", "apriltag", "--fps", "8"])
        ok = True
    except Exception as e:
        ok = False
        print(f"  scenario_ftc_suites.main(['--suite', 'apriltag']) raised: {e!r}")
    print(f"scenario_ftc_suites.py runs headlessly in single-suite (1-panel) mode without crashing: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_unknown_suite_name_raises_a_clear_error():
    try:
        scenario_ftc_suites.resolve_suite_names(scenario_ftc_suites.parse_args(["--suites", "not_a_real_suite"]))
        ok = False
        print("  expected SystemExit for an unknown suite name, none raised")
    except SystemExit as e:
        ok = "not_a_real_suite" in str(e)
    print(f"an unknown --suites name raises a clear error naming the bad value: {'OK' if ok else 'FAIL'}")
    return ok


if __name__ == "__main__":
    checks = [
        check_headless_run_completes_all_suites(),
        check_headless_run_completes_single_suite(),
        check_unknown_suite_name_raises_a_clear_error(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
