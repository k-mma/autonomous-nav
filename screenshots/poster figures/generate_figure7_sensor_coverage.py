"""Regenerates figure7_sensor_coverage.png for the SEED symposium
poster: the mechanism behind this project's headline NEGATIVE result --
three distance sensors cover only ~75 degrees of the robot's 360, so
most of the field around the robot is simply unsensed. This is what
explains the negative bar in figure2 and the worst point in figure3.

Every number here is read from ftc/config.py, not drawn to taste:
DISTANCE_SENSOR_COUNT (3), DISTANCE_SENSOR_HALF_ANGLE_DEG (12.5, so a
25-degree cone each), DISTANCE_SENSOR_MOUNT_HEADINGS_DEG (front, left,
right) and DISTANCE_SENSOR_RANGE_CELLS. Change the config and this
figure changes with it.

Regenerate after any change to ftc/config.py's distance-sensor block:

    python3 "screenshots/poster figures/generate_figure7_sensor_coverage.py"
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Wedge

OUT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(OUT_DIR))
from poster_common import REPO_ROOT  # noqa: E402,F401  (inserts repo on sys.path)

from ftc.config import (  # noqa: E402
    CELL_SIZE_IN, DISTANCE_SENSOR_COUNT, DISTANCE_SENSOR_HALF_ANGLE_DEG,
    DISTANCE_SENSOR_MOUNT_HEADINGS_DEG, DISTANCE_SENSOR_RANGE_CELLS,
)

OUT = OUT_DIR / "figure7_sensor_coverage.png"

SENSED = "#3aa17e"
UNSENSED = "#d8dde2"
ROBOT_NAVY = "#1e2a3a"
ALERT = "#d0742f"


def main():
    covered_deg = DISTANCE_SENSOR_COUNT * 2 * DISTANCE_SENSOR_HALF_ANGLE_DEG
    blind_deg = 360.0 - covered_deg
    range_in = DISTANCE_SENSOR_RANGE_CELLS * CELL_SIZE_IN

    fig, ax = plt.subplots(figsize=(8.6, 8.8))
    ax.set_aspect("equal")
    ax.axis("off")
    lim = 1.28
    ax.set_xlim(-lim, lim)
    # Extra headroom at the bottom so the blind-spot callout and the
    # legend swatches get their own bands instead of colliding.
    ax.set_ylim(-1.56, lim * 1.02)

    # Full 360 disc = everything within sensor range; the sensed wedges
    # are drawn on top of it, so what stays gray IS the blind region.
    ax.add_patch(Circle((0, 0), 1.0, facecolor=UNSENSED, edgecolor="#9aa5b1",
                         linewidth=1.6, linestyle=(0, (5, 4)), zorder=1))

    # Robot drawn facing UP (screen north). ftc/config.py's mount
    # headings are relative to the direction of travel (0 = straight
    # ahead, -90 = left, +90 = right), so "up" is heading 0 here and a
    # mount heading h sits at screen angle 90 - h.
    for h in DISTANCE_SENSOR_MOUNT_HEADINGS_DEG:
        centre = 90.0 - h
        ax.add_patch(Wedge((0, 0), 1.0,
                            centre - DISTANCE_SENSOR_HALF_ANGLE_DEG,
                            centre + DISTANCE_SENSOR_HALF_ANGLE_DEG,
                            facecolor=SENSED, edgecolor="white", linewidth=2, zorder=2))

    ax.add_patch(Circle((0, 0), 0.16, facecolor=ROBOT_NAVY, edgecolor="white",
                         linewidth=2, zorder=5))
    ax.text(0, 0, "ROBOT", color="white", fontsize=10, fontweight="bold",
            ha="center", va="center", zorder=6)

    # An obstacle sitting in the blind arc -- the failure this figure
    # exists to explain. Placed at 225 degrees screen (behind-left),
    # verified to be outside every sensed wedge.
    ang = np.radians(212.0)
    ox, oy = 0.74 * np.cos(ang), 0.74 * np.sin(ang)
    ax.add_patch(Circle((ox, oy), 0.075, facecolor=ALERT, edgecolor="white",
                         linewidth=2, zorder=6))
    ax.text(ox, oy, "!", color="white", fontsize=13, fontweight="bold",
            ha="center", va="center", zorder=7)
    ax.annotate("obstacle here is invisible\nuntil the robot drives into it",
                xy=(ox, oy), xytext=(-0.28, -1.17),
                arrowprops=dict(arrowstyle="-|>", color=ALERT, linewidth=2.1,
                                 shrinkA=2, shrinkB=8),
                color=ALERT, fontsize=11.5, fontweight="bold", ha="center", va="center",
                linespacing=1.25, zorder=7)

    ax.text(0, 1.15, f"{DISTANCE_SENSOR_COUNT} distance sensors cover only "
                      f"~{covered_deg:.0f}° of the robot's 360°",
            fontsize=15.5, fontweight="bold", ha="center", va="center")
    ax.text(0, 1.055, f"each cone ±{DISTANCE_SENSOR_HALF_ANGLE_DEG:g}°, "
                       f"~{range_in / 12:.0f} ft range — the remaining {blind_deg:.0f}° is unsensed",
            fontsize=11.5, color="#444444", ha="center", va="center")

    # Legend as two labeled swatches rather than a matplotlib legend --
    # only two categories, and placing them under the disc keeps the
    # figure square for a poster column.
    for x, color, label in ((-0.62, SENSED, f"sensed ({covered_deg:.0f}°)"),
                             (0.18, UNSENSED, f"unsensed ({blind_deg:.0f}°)")):
        ax.add_patch(plt.Rectangle((x, -1.475), 0.075, 0.055, facecolor=color,
                                    edgecolor="#9aa5b1", linewidth=1.1, zorder=6))
        ax.text(x + 0.105, -1.448, label, fontsize=11.5, ha="left", va="center")

    fig.tight_layout()
    fig.savefig(OUT, dpi=170, facecolor="white", bbox_inches="tight")
    print(f"wrote {OUT}  (covered {covered_deg}, blind {blind_deg})")


if __name__ == "__main__":
    main()
