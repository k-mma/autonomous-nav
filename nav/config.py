# Grid dimensions

GRID_SIZE = 25
CELL_SIZE = 28          # Pixels per cell

# --demo sizing (pygame_app/visualizer.py's --demo flag) -- smaller grid,
# bigger cells, so the 3-panel window still fits comfortably on a normal
# screen and reads well in a screenshot. Window dimensions themselves
# are no longer fixed constants here (Step 5): the 3-panel layout's
# width/height depend on which of GRID_SIZE/CELL_SIZE vs. these two are
# in effect, so pygame_app/visualizer.py computes them at startup instead.
DEMO_GRID_SIZE = 20
DEMO_CELL_SIZE = 30

STATUS_LINE_HEIGHT = 21
STATUS_LINES = 4
STATUS_BAR_HEIGHT = STATUS_LINES * STATUS_LINE_HEIGHT + 12

# Step 5: 3 side-by-side panels (Dijkstra / A* / RRT), each showing its
# own explored set and path against the same shared grid.
PANEL_DIVIDER_WIDTH = 4
PANEL_DIVIDER_COLOR = (120, 120, 120)
# Height reserved above each panel's grid for its algorithm-name label.
PANEL_LABEL_HEIGHT = 26
PANEL_LABEL_BG = (225, 225, 225)
PANEL_LABEL_TEXT = (30, 30, 30)


# Cell colors
#
# STEP 4 color audit: every color below (and TERRAIN_COLORS further down)
# was checked pairwise against the others for hue/value separation, since
# terrain now paints a background color on *every* cell and several other
# overlays (explored, cost tint, sensor ring) also have to stay legible
# against all four terrain colors, not just white. Each change below has
# its own comment explaining what it was checked against.
#
# Re-audit: the first pass only checked raw RGB distance, which missed a
# real clash -- ORANGE and the old MUD were nearly hue-twins (~5 deg
# apart on the color wheel), just at very different brightness/
# saturation, so the RGB-distance number looked fine while the actual
# hue was nearly identical. This pass fixed it from the terrain side
# (see TERRAIN_COLORS below) rather than moving ORANGE, since darkening/
# desaturating terrain overall -- so it reads as a muted backdrop
# markers clearly sit on top of -- was needed regardless of this one pair.

# Free cell -- background color math only now (grid lines, lerp base
# when a cell has no terrain painted, i.e. TERRAIN_GRASS -- see
# TERRAIN_COLORS). No longer the default cell fill; see draw_grid.
WHITE = (255, 255, 255)
# Obstacle -- a tree or rock, in this project's forest framing
BLACK = (30, 30, 30)
# Grid lines
GRAY = (200, 200, 200)
# Start cell -- STEP 4: brightened/re-hued from (50, 200, 100) to a more
# saturated, slightly bluer green so it stays clearly apart from
# TERRAIN_BUSH_COLOR (a duller, yellower olive) below -- both are
# "green," and start is drawn solid over whatever terrain sits under it.
GREEN = (34, 197, 94)
# Goal cell
RED = (220, 60, 60)
# STEP 4: Dijkstra's LIGHT_BLUE and A*'s LIGHT_PURPLE are merged into one
# EXPLORED_COLOR -- the visualizer only ever shows one algorithm's
# explored set at a time, so a viewer never needs the color itself to
# tell which algorithm produced it (the status line already says so).
# One shared color also means only one color needs to be kept distinct
# from the four terrain backgrounds instead of two. Chosen as a light
# lavender specifically because none of the four terrain hues (pale
# green, olive, brown, blue) drift anywhere near purple.
EXPLORED_COLOR = (196, 168, 255)
# Path cells
YELLOW = (255, 210, 50)
# Unreachable goal
DARK_RED = (160, 30, 30)
# Moving obstacle cells -- an animal, in this project's forest framing
ORANGE = (255, 140, 0)
# Robot marker -- STEP 4: raised brightness/saturation from (0, 190, 190)
# so it stays visible both over RRT_TREE_COLOR (a darker, less saturated
# teal) and over TERRAIN_WATER_COLOR (a muted, darker blue) -- pushed
# well above both so it doesn't blend into either.
CYAN = (0, 220, 255)

STATUS_BG = (245, 245, 245)
STATUS_TEXT = (60, 60, 60)

# Moving obstacles + robot animation

OBSTACLE_PERIOD_MS = 700
ROBOT_STEP_MS = 300
REPLAN_FLASH_MS = 700


# RRT

RRT_MAX_ITERS = 5000
RRT_STEP_SIZE = 2.0
RRT_GOAL_SAMPLE_RATE = 0.1
RRT_GOAL_RADIUS = 1.5
# Tree edges/nodes -- STEP 4: checked against EXPLORED_COLOR (a much
# lighter lavender, no hue overlap) and against all four terrain colors;
# closest is TERRAIN_WATER_COLOR (also blue-leaning) but far enough apart
# in hue (teal vs. blue-purple) and, being a thin drawn line rather than
# a cell fill, low risk of being confused with a terrain background even
# where the hues are somewhat close.
RRT_TREE_COLOR = (0, 150, 130)


# Cost map (weighted terrain / obstacle inflation, like Nav2's costmap)

COST_INFLUENCE_RADIUS = 3
COST_MAX_EXTRA = 4.0
# STEP 3/4: extra-cost tint blended toward from whatever the cell's
# terrain color is (not from flat WHITE anymore, now that terrain paints
# its own background -- see draw_grid in pygame_app/visualizer.py, which also
# divides the terrain cost multiplier back out before computing the
# blend amount, so this tint reflects obstacle-proximity inflation only,
# not terrain cost double-counted on top of its own color). Kept a warm
# peach specifically because every terrain color below is either cool
# (bush green, water blue) or a desaturated earth tone (mud), so a warm,
# fairly saturated tint reads clearly as "extra cost" against any of them.
COST_TINT = (255, 205, 150)


# Terrain (Step 3) -- purely a cosmetic/cost layer on top of the existing
# obstacle grid, framed as forest undergrowth: still obstacles are
# trees/rocks, moving obstacles are animals, terrain is the ground
# itself (grass/bush/mud/water). Every cell starts as TERRAIN_GRASS;
# painting a cell some other type multiplies its cost by the
# corresponding entry in TERRAIN_COST wherever the active grid's cost
# field gets refreshed (see Grid.refresh_cost_map), regardless of
# whether the obstacle-inflation cost map (K) is on -- unlike inflation,
# terrain cost is a real property of the ground, not a planning aid, so
# it always applies once painted. The robot can still traverse every
# terrain type; some just cost a lot more to cross.
TERRAIN_GRASS = 0
TERRAIN_BUSH = 1
TERRAIN_MUD = 2
TERRAIN_WATER = 3
# All four terrain types, in a fixed display order -- used where every
# type needs to be listed (e.g. the legend), regardless of what's
# actually paintable.
TERRAIN_CYCLE = [TERRAIN_GRASS, TERRAIN_BUSH, TERRAIN_MUD, TERRAIN_WATER]
# What the T key / left-click paint actually cycles through -- grass is
# excluded here since it's just the default tile every cell already
# starts as; there's nothing to "paint" it into.
TERRAIN_PAINT_CYCLE = [TERRAIN_BUSH, TERRAIN_MUD, TERRAIN_WATER]

TERRAIN_NAMES = {
    TERRAIN_GRASS: "Grass",
    TERRAIN_BUSH: "Bush",
    TERRAIN_MUD: "Mud",
    TERRAIN_WATER: "Water",
}

TERRAIN_COST = {
    TERRAIN_GRASS: 1.0,
    TERRAIN_BUSH: 2.0,
    TERRAIN_MUD: 3.5,
    TERRAIN_WATER: 5.0,
}

# Color re-audit: the first pass (Step 4) only checked raw RGB distance,
# which missed a real problem -- ORANGE (moving obstacle, hue ~33 deg)
# and the old MUD (hue ~28 deg) were nearly hue-twins, differing only in
# brightness/saturation, since RGB distance is dominated by that
# brightness gap rather than hue. Terrain is now deliberately darker and
# less saturated across the board (grass stays near-white/unobtrusive by
# design) so it reads as a muted backdrop that every marker color -- all
# of which stay near-maximum saturation/brightness -- clearly sits on
# top of, regardless of how close two hues land on the wheel. Bush, mud,
# and water all kept their original hue (still clearly bush/mud/water)
# but each had saturation and value pulled down roughly 30-40%.
TERRAIN_COLORS = {
    TERRAIN_GRASS: (231, 240, 223),
    TERRAIN_BUSH: (90, 115, 60),
    TERRAIN_MUD: (110, 90, 72),
    TERRAIN_WATER: (85, 120, 150),
}


# Lidar sensor model

LIDAR_RADIUS = 5
# Outline drawn around a real obstacle the robot hasn't sensed yet
HIDDEN_OBSTACLE_OUTLINE = (170, 170, 170)
# Sensor radius ring -- STEP 4: moved off blue (its old value,
# (0, 140, 200)) once TERRAIN_WATER_COLOR claimed that hue for the water
# terrain type. An amber replacement was tried first, but that's too
# close to YELLOW (path) and ORANGE (moving obstacle) -- the ring can
# pass directly over a yellow path cell, so it needs to read as a
# distinct hue there too, not just against terrain. Magenta/pink has no
# other claimant anywhere in this palette and sits far (RGB distance
# > 150) from every other color and all four terrain colors.
SENSOR_RING_COLOR = (230, 25, 170)

# Sensor noise -- both LidarSensor (nav/sensor.py) and Lidar3D
# (pybullet_app/sim3d/lidar.py) are perfect by default (every real obstacle in
# range is detected, at its exact cell, and nothing else is); passing
# noisy=True to either makes them imperfect in three independent ways,
# each governed by one of these rates:
#   - a real obstacle in range can go undetected this scan (false
#     negative), governed by NOISE_MISS_RATE;
#   - a detected obstacle's reported position can be off by a cell (2D)
#     or a bit of physical distance (3D) instead of exact, governed by
#     NOISE_POSITION_RATE;
#   - a free cell/empty ray can be "detected" as an obstacle that isn't
#     really there (false positive), governed by NOISE_FALSE_POSITIVE_RATE.
# Off by default (both sensors default to noisy=False) so every existing
# caller and test keeps its current, deterministic behavior unchanged.
NOISE_MISS_RATE = 0.15
NOISE_POSITION_RATE = 0.15
NOISE_FALSE_POSITIVE_RATE = 0.02
# How many separate scans have to (noisily) report the same cell before
# a caller that's using confirmed_obstacles() should trust it enough to
# replan on -- see LidarSensor.confirmed_obstacles / Lidar3D.confirmed_obstacles
# and WRITEUPS.md for why a single noisy reading isn't enough on its own.
CONFIRMATION_THRESHOLD = 2


# Occupancy belief model (nav/occupancy.py) -- a per-cell probability of
# being occupied, updated from LidarSensor observations via a log-odds
# rule (Thrun/Burgard/Fox's standard occupancy-grid-mapping update)
# instead of nav/sensor.py's binary known_obstacles set. Feeds
# nav/policies.py's BeliefPolicy.
OCCUPANCY_PRIOR = 0.5
# Log-odds increment applied per single free/occupied observation.
# Occupied is weighted more heavily than free so a handful of genuine
# detections outweighs a long run of "saw nothing here" observations --
# matching a real lidar's asymmetry, where a miss (NOISE_MISS_RATE) is
# possible but a hit essentially never lies about a cell being clear.
OCCUPANCY_LOGODDS_FREE = -0.4
OCCUPANCY_LOGODDS_OCCUPIED = 0.85
# Clamp on accumulated log-odds so a cell's probability can get very
# close to but never exactly reach 0.0/1.0 -- keeps a single
# contradicting observation able to move a saturated estimate back,
# instead of it being numerically stuck.
OCCUPANCY_LOGODDS_CLAMP = 6.0
# Probability at/above which BeliefGrid (nav/occupancy.py) treats a cell
# as a hard obstacle rather than just an expensive one to enter.
OCCUPANCY_OBSTACLE_THRESHOLD = 0.9
# Cost multipliers BeliefGrid interpolates between as a cell's occupancy
# probability climbs from 0 toward OCCUPANCY_OBSTACLE_THRESHOLD -- see
# nav/occupancy.py's BeliefGrid.
OCCUPANCY_FREE_COST_MULT = 1.0
OCCUPANCY_BLOCKED_COST_MULT = 12.0


# Field variance (nav/field_variance.py) -- how far a "ground truth" grid
# is allowed to drift from the assumed map a policy plans against, at
# variance_level == 1.0 (nav/uncertainty_benchmark.py's sweep knob).
# Each deviation type scales linearly with variance_level from 0 at 0.0
# up to the bound below at 1.0 -- see generate_ground_truth's docstring
# for the exact mapping.
FIELD_VARIANCE_MAX_START_DRIFT_RADIUS = 3
FIELD_VARIANCE_MAX_OBSTACLE_DRIFT_COUNT = 6
