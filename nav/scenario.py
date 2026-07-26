from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

from nav.grid import Grid


@dataclass
class ScenarioConfig:
    """Preset state for one of the standalone scenario_*.py entry points
    -- passed to nav.visualizer.main(scenario=...), which applies it to
    the freshly built grid before the event loop starts, instead of each
    scenario file duplicating the visualizer's own setup/drawing logic.

    `build` and `moving_obstacles` are plain functions rather than
    precomputed data because both need the actual grid (its real size,
    whether --demo shrank it further to fit the screen) to place things
    correctly -- `moving_obstacles` in particular can't be decided until
    after `build` has placed obstacles/terrain, since it needs to know
    which cells are actually free.
    """

    # Shown in the window's title bar.
    title: str

    # Populates start/goal/obstacles/terrain on the freshly created grid.
    build: Callable[[Grid], None]

    # Called after build(grid) -- returns starting cells for moving
    # obstacles (animals). Defaults to none.
    moving_obstacles: Callable[[Grid], List[Tuple[int, int]]] = lambda grid: []

    sensor_enabled: bool = False
    noisy_sensor: bool = False

    # Run all three algorithms immediately on launch.
    auto_run: bool = True
    # Start the robot walking immediately after auto_run (ignored if
    # step_snapshot is set -- step mode and walking don't mix, see
    # nav/visualizer.py's start_robot).
    auto_walk: bool = False

    # If set, enters step mode and reveals this many cells/nodes per
    # panel instead of the full result -- for scenarios meant to
    # screenshot a mid-search state. Mutually exclusive with auto_run's
    # full-result display; overrides it when set.
    step_snapshot: Optional[int] = None
