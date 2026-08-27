"""Regenerates figure6_match_lifecycle.png: what one simulated match actually does, as
a loop rather than four paragraphs of prose.

Mirrors ftc/match.py's real control flow: plan once from the BELIEVED
start against the ASSUMED map, then repeat {drive one step -> sense ->
maybe re-plan} until one of three terminal conditions fires.

Layout notes (this is a redraw of an earlier version that read badly):
the four loop steps run left to right on ONE line with the repeat arc
carried ABOVE them, so nothing crosses anything; and all three terminal
outcomes fan out of a SINGLE check point rather than dangling off three
different steps. The single check point is also the more faithful
picture -- ftc/match.py tests all three conditions after every step, so
attaching "success" to the planning box or "timeout" to the re-plan box
(as the earlier draft did) implied a control flow the code doesn't have.

This figure is pure schematic -- it reads no CSV and runs no simulation,
so it only needs regenerating if ftc/match.py's control flow itself
changes:

    python3 "screenshots/poster figures/generate_figure6_match_lifecycle.py"
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

OUT_DIR = Path(__file__).resolve().parent
OUT = OUT_DIR / "figure6_match_lifecycle.png"

STEP_FACE, STEP_EDGE = "#dce7f6", "#2f5fb0"
WIN_FACE, WIN_EDGE = "#d8efdc", "#2e8b3d"
LOSE_FACE, LOSE_EDGE = "#f7dcd8", "#c0392b"
ARROW = "#3d4a5c"

STEP_W, STEP_H, STEP_Y = 2.2, 0.98, 3.55
STEP_X = [0.30, 2.95, 5.60, 8.25]
STEP_CX = [x + STEP_W / 2 for x in STEP_X]

OUT_W, OUT_H, OUT_Y = 2.5, 0.88, 0.40
OUT_X = [1.15, 4.10, 7.05]
OUT_CX = [x + OUT_W / 2 for x in OUT_X]

BUS_Y = 2.00          # the "checked after every step" bus line
LOOP_Y = 5.15         # the repeat arc, carried above the step row

STEPS = [
    ("1 · PLAN", "A* route from the BELIEVED\nstart, over the ASSUMED map"),
    ("2 · DRIVE", "Take one step. Pose error\ngrows with distance driven"),
    ("3 · SENSE", "Equipped sensors correct pose\nand/or reveal real obstacles"),
    ("4 · RE-PLAN?", "If the map changed,\nplan again from here"),
]

OUTCOMES = [
    (WIN_FACE, WIN_EDGE, "SUCCESS", "true position reaches the goal"),
    (LOSE_FACE, LOSE_EDGE, "FAILURE · collision", "true footprint hits a real obstacle"),
    (LOSE_FACE, LOSE_EDGE, "FAILURE · timeout", "30-second auto period expires"),
]


def box(ax, x, y, w, h, face, edge, title, body):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.045,rounding_size=0.09",
                                 facecolor=face, edgecolor=edge, linewidth=2, zorder=3))
    ax.text(x + w / 2, y + h * 0.70, title, fontsize=12.5, fontweight="bold",
            ha="center", va="center", color=edge, zorder=4)
    ax.text(x + w / 2, y + h * 0.28, body, fontsize=9.6, ha="center", va="center",
            color="#333333", linespacing=1.3, zorder=4)


def arrow(ax, start, end, color=ARROW, lw=2.1, ls="solid"):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle="-|>", color=color, linewidth=lw,
                                  linestyle=ls, mutation_scale=17, shrinkA=2, shrinkB=2, zorder=2))


def line(ax, start, end, color=ARROW, lw=2.1):
    ax.plot([start[0], end[0]], [start[1], end[1]], color=color, linewidth=lw,
            solid_capstyle="round", zorder=2)


def main():
    fig, ax = plt.subplots(figsize=(12.2, 6.1))
    ax.set_xlim(0, 10.75)
    ax.set_ylim(-0.05, 6.3)
    ax.axis("off")

    for x, (title, body) in zip(STEP_X, STEPS):
        box(ax, x, STEP_Y, STEP_W, STEP_H, STEP_FACE, STEP_EDGE, title, body)
    for x, (face, edge, title, body) in zip(OUT_X, OUTCOMES):
        box(ax, x, OUT_Y, OUT_W, OUT_H, face, edge, title, body)

    # Straight chain along the single step row -- no bends, no crossings.
    mid_y = STEP_Y + STEP_H / 2
    for i in range(3):
        arrow(ax, (STEP_X[i] + STEP_W, mid_y), (STEP_X[i + 1], mid_y))

    # Repeat arc, carried above the row: RE-PLAN? back to DRIVE.
    top = STEP_Y + STEP_H
    line(ax, (STEP_CX[3], top), (STEP_CX[3], LOOP_Y))
    line(ax, (STEP_CX[3], LOOP_Y), (STEP_CX[1], LOOP_Y))
    arrow(ax, (STEP_CX[1], LOOP_Y), (STEP_CX[1], top))
    ax.text((STEP_CX[1] + STEP_CX[3]) / 2, LOOP_Y, "repeat every step",
            fontsize=11, fontweight="bold", color=ARROW, ha="center", va="center", zorder=4,
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="none"))

    # One check point feeding all three outcomes.
    arrow(ax, (STEP_CX[1], STEP_Y), (STEP_CX[1], BUS_Y + 0.02))
    ax.text(STEP_CX[1] + 0.28, (STEP_Y + BUS_Y) / 2, "after every step,\nthe match checks:",
            fontsize=10.5, fontweight="bold", color=ARROW, ha="left", va="center",
            linespacing=1.3, zorder=4,
            bbox=dict(boxstyle="round,pad=0.24", facecolor="white", edgecolor="none"))
    line(ax, (OUT_CX[0], BUS_Y), (OUT_CX[2], BUS_Y))
    for cx, (_, edge, _, _) in zip(OUT_CX, OUTCOMES):
        arrow(ax, (cx, BUS_Y), (cx, OUT_Y + OUT_H), color=edge)

    ax.text(0.30, 6.02, "How one simulated match works",
            fontsize=16, fontweight="bold", ha="left", va="center")
    ax.text(0.30, 5.68,
            "The robot only ever acts on what it BELIEVES; the match is scored on what is TRUE.",
            fontsize=11.5, color="#444444", ha="left", va="center")

    fig.tight_layout()
    fig.savefig(OUT, dpi=170, facecolor="white", bbox_inches="tight")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
