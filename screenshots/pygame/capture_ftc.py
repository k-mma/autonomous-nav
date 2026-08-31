"""Captures a screenshot of the FTC suite-comparison viewer, headless.

Same flip-patching trick as capture.py in this directory, but pointed at
pygame_app/scenarios/scenario_ftc_suites.py -- which is its own animated
real-time viewer rather than a pygame_app.visualizer.main(CONFIG)
scenario, so capture.py's SCENARIOS table can't drive it.

The frame is taken partway through playback (SETTLE_FRAMES) rather than
at t=0: every panel's robot is still parked on its start cell on frame 1,
and the whole point of this picture is several suites diverging from the
identical seeded scenario -- trails laid down, pose-error ghosts pulled
away from true position, some panels already collided.

    python3 screenshots/pygame/capture_ftc.py
"""
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "pygame_app" / "scenarios"))

import pygame

import scenario_ftc_suites

OUT_DIR = Path(__file__).resolve().parent

# ~13s of match time at 60fps (playback runs at 1.00x). Seed 15 is
# chosen because all four suites visibly diverge on it -- AprilTag
# reaches the goal at 12.4s while the other three are still driving --
# where on most seeds the AprilTag panel is pixel-identical to the dead
# reckoning one (at the "realistic" fidelity tier this viewer defaults
# to, camera-FOV gating often means no tag is ever in view, so the suite
# genuinely has nothing to correct with). Capturing just after 12.4s is
# what puts one panel on SUCCESS and three mid-route in the same frame.
SETTLE_FRAMES = 800

# A deliberately legible subset, not all 7 headline suites: at 7 panels
# the per-panel field is too small to read the robot/ghost/trail apart
# once the image is scaled to a README's width. These four are the ones
# the headline result is actually about -- the free baseline, the best
# value, the best raw performer, and the one that collides.
ARGV = [
    "--suites", "dead_reckoning,apriltag,odometry_pods,distance_sensors",
    "--layout", "cluttered",
    "--deviation-type", "start_drift",
    "--level", "0.5",
    "--fidelity", "realistic",
    "--seed", "15",
]


def main():
    out_path = OUT_DIR / "ftc_suite_replay.png"

    # Presentation-only overrides, scoped to this capture -- the viewer
    # itself is untouched. An interactive session auto-fits to the real
    # display, and under SDL's dummy driver that "display" is tiny, so a
    # straight capture comes out at 450px wide with 8px cells. Report a
    # large virtual screen, force all four panels onto ONE row (the app's
    # own _layout_cols picks 2x2 for n=4, which is portrait and awkward as
    # a README hero), and lift the 4-column cell-size cap so the fields are
    # actually readable at README width.
    scenario_ftc_suites.CELL_BY_COLS = {**scenario_ftc_suites.CELL_BY_COLS, 4: 22}
    scenario_ftc_suites._layout_cols = lambda n: n

    real_info = pygame.display.Info

    class _VirtualScreen:
        """pygame's own VidInfo is read-only, so hand the viewer a stand-in
        exposing just the two fields _fit_cell_px reads."""
        current_w, current_h = 5000, 3000

    pygame.display.Info = _VirtualScreen

    frame_count = [0]
    real_flip = pygame.display.flip

    def patched_flip():
        frame_count[0] += 1
        result = real_flip()
        if frame_count[0] == SETTLE_FRAMES:
            pygame.image.save(pygame.display.get_surface(), str(out_path))
            pygame.event.post(pygame.event.Event(pygame.QUIT))
        return result

    pygame.display.flip = patched_flip
    try:
        scenario_ftc_suites.main(ARGV)
    except SystemExit:
        pass
    finally:
        pygame.display.flip = real_flip
        pygame.display.Info = real_info

    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
