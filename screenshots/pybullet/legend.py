"""Generates a standalone legend graphic (screenshots/pybullet/legend.png)
for the printed poster board -- the in-sim Hud (nav/sim3d/hud.py) renders
world-space debug text sized for an on-screen camera view, which reads as
an illegible speck once a screenshot is shrunk down to fit next to two
other panels on a printed board. This draws the same color/marker
vocabulary used across pybullet_main.py's three demos as flat swatches
with labels, sized to be read from a few feet away instead.

Every color is imported from nav/config.py and nav/sim3d/world.py rather
than re-typed here, so the legend can never silently drift out of sync
with what the sim actually renders.

    python3 screenshots/pybullet/legend.py     # writes screenshots/pybullet/legend.png
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from PIL import Image, ImageDraw, ImageFont

from nav.config import TERRAIN_COLORS, TERRAIN_COST, TERRAIN_NAMES, TERRAIN_CYCLE
from nav.sim3d.world import (
    BINARY_PATH_COLOR, COST_MAP_PATH_COLOR, SENSOR_PATH_COLOR,
    TERRAIN_NAIVE_PATH_COLOR, TERRAIN_AWARE_PATH_COLOR,
    FLASH_COLOR, LINGER_COLOR, TRIGGER_MARKER_COLOR,
    OBSTACLE_COLOR, HIDDEN_OBSTACLE_COLOR, COST_TINT_COLOR,
    START_COLOR, GOAL_COLOR, ROBOT_A_COLOR, ROBOT_B_COLOR,
)

WIDTH, HEIGHT = 2200, 1160
MARGIN = 60
BG = (255, 255, 255)
INK = (25, 25, 25)
SUBTLE = (120, 120, 120)
RULE = (210, 210, 210)
SWATCH = 70
ROW_GAP = 26
SECTION_GAP = 56
COL_GAP = 90
LABEL_OFFSET = 26

FONT_DIR = Path("/System/Library/Fonts/Supplemental")


def _font(name, size):
    return ImageFont.truetype(str(FONT_DIR / name), size)


TITLE_FONT = _font("Arial Bold.ttf", 64)
SECTION_FONT = _font("Arial Bold.ttf", 40)
LABEL_FONT = _font("Arial.ttf", 32)
SUB_FONT = _font("Arial.ttf", 26)


def to_rgb255(color):
    """Path/marker colors in nav/sim3d/world.py are 0-1 floats (3- or
    4-tuples, pybullet's rgbaColor convention); terrain colors in
    nav/config.py are already 0-255 ints (pygame's convention). Normalize
    either to a 0-255 RGB tuple PIL can draw."""
    r, g, b = color[0], color[1], color[2]
    if max(r, g, b) <= 1.0:
        return (round(r * 255), round(g * 255), round(b * 255))
    return (round(r), round(g), round(b))


def composite_over_white(rgba):
    """Alpha-blend a 0-1 RGBA color over a white background -- the same
    math PyBullet's own renderer does when it draws a translucent quad
    over the (pale) ground plane, so a legend swatch for e.g.
    HIDDEN_OBSTACLE_COLOR shows the same near-invisible result a viewer
    actually sees in the sim, not its raw (dark, fully-opaque-looking)
    base color."""
    r, g, b, a = rgba
    blended = tuple(round((c * a + 1.0 * (1 - a)) * 255) for c in (r, g, b))
    return blended


def draw_swatch_row(draw, x, y, color, label, sublabel=None, shape="square"):
    rgb = to_rgb255(color)
    box = [x, y, x + SWATCH, y + SWATCH]
    if shape == "square":
        draw.rectangle(box, fill=rgb, outline=INK, width=2)
    elif shape == "disc":
        draw.ellipse(box, fill=rgb, outline=INK, width=2)
    elif shape == "diamond":
        cx, cy = x + SWATCH / 2, y + SWATCH / 2
        draw.polygon([(cx, y), (x + SWATCH, cy), (cx, y + SWATCH), (x, cy)], fill=rgb, outline=INK, width=2)
    elif shape == "x":
        draw.rectangle(box, outline=RULE, width=2)
        draw.line([x + 8, y + 8, x + SWATCH - 8, y + SWATCH - 8], fill=rgb, width=8)
        draw.line([x + 8, y + SWATCH - 8, x + SWATCH - 8, y + 8], fill=rgb, width=8)
    elif shape == "line":
        mid = y + SWATCH / 2
        draw.rectangle(box, outline=RULE, width=1)
        draw.line([x + 6, mid, x + SWATCH - 6, mid], fill=rgb, width=14)

    text_x = x + SWATCH + LABEL_OFFSET
    draw.text((text_x, y + 4), label, font=LABEL_FONT, fill=INK)
    if sublabel:
        draw.text((text_x, y + 4 + 38), sublabel, font=SUB_FONT, fill=SUBTLE)
    return y + SWATCH + ROW_GAP


def draw_section(draw, x, y, title, rows):
    draw.text((x, y), title, font=SECTION_FONT, fill=INK)
    y += 58
    for color, label, sublabel, shape in rows:
        y = draw_swatch_row(draw, x, y, color, label, sublabel, shape)
    return y + SECTION_GAP - ROW_GAP


def main():
    img = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(img)

    draw.text((MARGIN, MARGIN), "Autonomous Navigation -- Legend", font=TITLE_FONT, fill=INK)
    draw.text((MARGIN, MARGIN + 78), "PyBullet demos: binary vs. cost-map / terrain-cost routing / lidar sensor discovery",
              font=SUB_FONT, fill=SUBTLE)
    draw.line([MARGIN, MARGIN + 130, WIDTH - MARGIN, MARGIN + 130], fill=RULE, width=2)

    col1_x = MARGIN
    col2_x = MARGIN + (WIDTH - 2 * MARGIN) // 3 + COL_GAP // 2
    col3_x = MARGIN + 2 * ((WIDTH - 2 * MARGIN) // 3) + COL_GAP
    top_y = MARGIN + 170

    y = draw_section(draw, col1_x, top_y, "Paths", [
        (BINARY_PATH_COLOR, "Binary-obstacle path", "no clearance margin", "line"),
        (COST_MAP_PATH_COLOR, "Cost-map path", "clearance-aware detour", "line"),
        (SENSOR_PATH_COLOR, "Sensor-limited path", "plans on lidar-known cells only", "line"),
        (TERRAIN_AWARE_PATH_COLOR, "Terrain-aware path", "routes around mud/water cost", "line"),
        (TERRAIN_NAIVE_PATH_COLOR, "Naive / binary baseline", "shared red = \"ignores the thing being compared\"", "line"),
    ])
    y = draw_section(draw, col1_x, y, "Replan Signals", [
        (FLASH_COLOR, "Just replanned", "bright flash, fades to real color", "line"),
        (LINGER_COLOR, "Superseded path", "old route, fades out", "line"),
        (TRIGGER_MARKER_COLOR, "Discovery trigger", "X marks the newly-confirmed cell", "x"),
    ])

    y2 = draw_section(draw, col2_x, top_y, "Terrain (cost multiplier)", [
        (TERRAIN_COLORS[t], TERRAIN_NAMES[t], f"x{TERRAIN_COST[t]:g} step cost", "square")
        for t in TERRAIN_CYCLE
    ])
    y2 = draw_section(draw, col2_x, y2, "Obstacles & Cost", [
        (OBSTACLE_COLOR, "Obstacle", "impassable", "square"),
        (composite_over_white(HIDDEN_OBSTACLE_COLOR), "Undiscovered obstacle",
         "sensor mode only, near-invisible until sensed", "square"),
        (composite_over_white((*COST_TINT_COLOR, 0.4)), "Clearance cost tint",
         "obstacle-proximity inflation (darker = costlier)", "square"),
    ])

    y3 = draw_section(draw, col3_x, top_y, "Markers", [
        (START_COLOR, "Start", "disc", "disc"),
        (GOAL_COLOR, "Goal", "diamond", "diamond"),
    ])
    y3 = draw_section(draw, col3_x, y3, "Multi-Robot (if shown)", [
        (ROBOT_A_COLOR, "Robot A", "", "square"),
        (ROBOT_B_COLOR, "Robot B", "", "square"),
    ])

    out_path = Path(__file__).resolve().parent / "legend.png"
    img.save(out_path)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
