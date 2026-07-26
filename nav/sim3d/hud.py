import pybullet as p

DEFAULT_COLOR = (0.05, 0.05, 0.05)
DEFAULT_TEXT_SIZE = 1.15


class Hud:
    """A multi-line status readout rendered as world-space debug text,
    mirroring nav/visualizer.py's status bar (active mode/algorithm, path
    cost or waypoint count, a replanning/waiting indicator, elapsed sim
    time). Updated in place every frame via replaceItemUniqueId so it
    never spawns a new debug text item per call -- PyBullet has no notion
    of a screen-space HUD, so this is pinned to a fixed world position
    instead and relies on debug text always billboarding toward the
    camera. No-op in headless/DIRECT mode, same as the rest of this
    module's debug-draw helpers -- addUserDebugText there just returns -1
    without erroring, so callers don't need to branch on `gui` themselves."""

    def __init__(self, position, gui, color=DEFAULT_COLOR, text_size=DEFAULT_TEXT_SIZE):
        self.position = position
        self.gui = gui
        self.color = color
        self.text_size = text_size
        self._item_id = -1

    def update(self, lines):
        if not self.gui:
            return
        self._item_id = p.addUserDebugText(
            "\n".join(lines),
            self.position,
            textColorRGB=self.color[:3],
            textSize=self.text_size,
            replaceItemUniqueId=self._item_id,
        )


class FollowLabel:
    """A short text label (e.g. "A") that tracks a moving position each
    frame, updated in place the same way Hud is. Used to tell two
    identical robot models apart in the multi-robot demo."""

    def __init__(self, text, gui, color=DEFAULT_COLOR, text_size=1.4, height_offset=0.9):
        self.text = text
        self.gui = gui
        self.color = color
        self.text_size = text_size
        self.height_offset = height_offset
        self._item_id = -1

    def update(self, position):
        if not self.gui:
            return
        x, y, z = position
        self._item_id = p.addUserDebugText(
            self.text,
            [x, y, z + self.height_offset],
            textColorRGB=self.color[:3],
            textSize=self.text_size,
            replaceItemUniqueId=self._item_id,
        )
