"""Captures one screenshot per scenarios/scenario_*.py, headless.

Runs pygame_app.visualizer.main(CONFIG) for real -- the exact same setup and
drawing code an interactive session uses -- under SDL's "dummy" video
driver (no real window/display needed) instead of duplicating any of
main()'s drawing logic here. pygame.display.flip is patched to save the
render target and post a QUIT event once the intended frame has been
drawn, so main()'s own event loop exits cleanly on its next iteration
exactly the way closing the window would.

Static/step-snapshot scenarios (no robot walking) are screenshotted on
their very first rendered frame -- every bit of scenario-specific state
(algorithm results, revealed step count) is already set up before the
loop starts, so frame 1 already shows the finished picture. auto_walk
scenarios (moving obstacles + a robot actually walking, sensed
mid-route) are instead let run for WALK_SETTLE_FRAMES real frames first,
so the screenshot catches the robot partway along its route with the
sensor/replan state that's the actual point of those two scenarios,
rather than a frozen t=0 frame where the robot hasn't moved yet.

    python3 screenshots/pygame/capture.py      # writes screenshots/pygame/*.png
"""
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import importlib
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
SCENARIOS_DIR = REPO_ROOT / "pygame_app" / "scenarios"
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(SCENARIOS_DIR))

import pygame

import pygame_app.visualizer as visualizer

OUT_DIR = Path(__file__).resolve().parent

# How many real frames (at the app's own clock.tick(60)) to let an
# auto_walk scenario run before capturing -- long enough for the robot
# to take several ROBOT_STEP_MS=300ms steps and for at least one moving
# obstacle (OBSTACLE_PERIOD_MS=700ms) to move and potentially trigger a
# replan, which is the entire point of those two scenarios.
WALK_SETTLE_FRAMES = 150

# module name -> output filename. Frame 1 (immediately after scenario
# setup, before any event-loop iteration) already shows the finished
# picture for every non-walking scenario -- see this file's docstring.
SCENARIOS = [
    ("scenario_bottleneck", "forest_corridor.png", 1),
    ("scenario_costmap", "costed_clearing.png", 1),
    ("scenario_maze", "deep_forest.png", 1),
    ("scenario_open", "open_field.png", 1),
    ("scenario_stepreplay", "search_snapshot.png", 1),
    ("scenario_sensor", "hidden_animals.png", WALK_SETTLE_FRAMES),
    ("scenario_noisy_sensor", "unreliable_sensor.png", WALK_SETTLE_FRAMES),
]


def capture_scenario(module_name, out_name, after_frames):
    module = importlib.import_module(module_name)
    out_path = OUT_DIR / out_name

    frame_count = [0]
    real_flip = pygame.display.flip

    def patched_flip():
        frame_count[0] += 1
        result = real_flip()
        if frame_count[0] == after_frames:
            pygame.image.save(pygame.display.get_surface(), str(out_path))
            pygame.event.post(pygame.event.Event(pygame.QUIT))
        return result

    pygame.display.flip = patched_flip
    try:
        visualizer.main(module.CONFIG)
    except SystemExit:
        pass
    finally:
        pygame.display.flip = real_flip

    print(f"wrote {out_path}")


def main():
    for module_name, out_name, after_frames in SCENARIOS:
        capture_scenario(module_name, out_name, after_frames)


if __name__ == "__main__":
    main()
