"""
Composes two or more of ftc/sensors.py's suites into ONE working suite
that ftc/match.py can drive unchanged -- the missing piece this project
needed before it could ask "which BUNDLE of sensors is worth buying,"
rather than only "which of these five fixed suites is."

The seven headline suites already answer the second question, and one
of them (FullSuite) is itself a hand-built bundle -- distance sensors +
AprilTag + odometry pods, hardcoded as its own class. That hardcoding is
the limitation: there are 8 suites in SUITES now, and the interesting
combinations (odometry pods + rear camera? IMU + dual camera + 6 ToF
sensors?) would each need a hand-written class before anything could
measure them. BundleSuite below builds any of them on demand, and
ftc/optimizer.py searches over the space they form.

Two things make this more than a `class Bundle(A, B)` one-liner:

1. COST. Suites price themselves with a flat cost_usd, which
   double-counts shared hardware the moment two suites overlap:
   AprilTagSuite ($25, one webcam) + AprilTagImuSuite ($25, the same
   webcam plus a free IMU) is a $25 robot, not a $50 one. So a bundle
   costs the UNION of its components' PARTS (ftc/config.py's
   PART_COSTS_USD), never the sum of their prices. A bundle of
   overlapping suites is therefore often cheaper than its parts list
   suggests, and two different-looking bundles can turn out to be the
   same robot -- which is exactly what part_signature() exists to let
   ftc/optimizer.py detect and deduplicate before paying to simulate
   both.

2. CAPABILITY MERGING that stays exact. Every capability composes the
   physically honest way: senses_obstacles/fixes_pose/fixes_heading are
   ORed (a robot has a capability if any of its hardware provides it),
   drift_per_cell takes the MINIMUM (the best localization hardware on
   the robot sets its drift rate -- bolting encoders onto a robot that
   also has odometry pods doesn't make it drift more), obstacle sensors
   are UNIONed into one CompositeObstacleSensor, camera mounts are
   UNIONed (two suites each carrying a camera give the bundle both
   cones), and heading correction takes the strongest available.

   The payoff of getting that right is a guarantee, not just tidiness:
   a bundle reproduces the suite(s) it's built from EXACTLY, tick for
   tick and rng-draw for rng-draw. make_bundle("apriltag") is
   indistinguishable from AprilTagSuite(), and make_bundle(
   "distance_sensors", "apriltag", "odometry_pods") is indistinguishable
   from FullSuite() -- same cost, same capabilities, same match results
   on the same seed. ftc/scratch/bundle_test.py fails if either stops
   holding, which is what keeps the published headline numbers safe from
   this module's existence: the optimizer's search space CONTAINS the
   headline suites rather than approximating them.

Nothing in ftc/sensors.py, ftc/match.py, or any existing benchmark is
modified or imported-into by this module -- it is strictly additive.
"""
import copy
import itertools

from ftc.config import (
    CONFLICTING_PART_GROUPS, DISTANCE_SENSOR_COUNT, DISTANCE_SENSOR_COUNTS_SWEPT, PART_COSTS_USD,
)
from ftc.sensors import SUITES, SUITE_LABELS, AprilTagSuite, SensorSuite

# Which physical parts each suite in ftc/sensors.py is made of. Kept
# here rather than as a `parts` attribute on the suite classes so this
# whole feature stays additive -- ftc/sensors.py holds the published
# headline suites and isn't touched by it. Every mapping is checked
# against that suite's own cost_usd by part_cost_mismatches() below.
SUITE_PARTS = {
    "dead_reckoning": frozenset(),  # built-in motor encoders -- no purchasable part
    "odometry_pods": frozenset({"odometry_pods"}),
    "distance_sensors": frozenset({"distance_sensors_3"}),
    "apriltag": frozenset({"camera_front"}),
    "full_suite": frozenset({"distance_sensors_3", "camera_front", "odometry_pods"}),
    "imu": frozenset({"imu"}),
    "apriltag_imu": frozenset({"camera_front", "imu"}),
    "dual_camera_apriltag": frozenset({"camera_front", "camera_rear"}),
    # ftc/sensors.py's make_distance_sensor_suite(n) variants, which
    # exist only at runtime (ftc/coverage_benchmark.py's sweep) and name
    # themselves "distance_sensors_{n}" -- the same string their part is
    # priced under in ftc/config.py's PART_COSTS_USD, so a swept ToF
    # layout can be bundled with anything else without needing its own
    # `parts` attribute set at every call site.
    **{f"distance_sensors_{n}": frozenset({f"distance_sensors_{n}"})
       for n in sorted({DISTANCE_SENSOR_COUNT, *DISTANCE_SENSOR_COUNTS_SWEPT})},
}


# Display names for the parts above, and the order a parts list is
# written in (localization first, then vision, then obstacle sensing --
# roughly the order a team would install them). Needed because a
# bundle's COMPONENT names can describe the same robot in more than one
# way: "AprilTag + Dual-camera AprilTag" and "Dual-camera AprilTag" are
# both a front and a rear camera, and only the parts list says so
# unambiguously.
PART_LABELS = {
    "odometry_pods": "odometry pods",
    "camera_front": "front camera",
    "camera_rear": "rear camera",
    "imu": "IMU",
}
PART_LABELS.update({f"distance_sensors_{n}": f"{n} ToF sensors" for n in DISTANCE_SENSOR_COUNTS_SWEPT})
PART_LABELS[f"distance_sensors_{DISTANCE_SENSOR_COUNT}"] = f"{DISTANCE_SENSOR_COUNT} ToF sensors"
_PART_DISPLAY_ORDER = ["odometry_pods", "imu", "camera_front", "camera_rear"] + \
    [f"distance_sensors_{n}" for n in sorted({DISTANCE_SENSOR_COUNT, *DISTANCE_SENSOR_COUNTS_SWEPT})]


def parts_label(parts):
    """A parts set written out as the robot it is: "odometry pods +
    front camera + rear camera". Empty (dead reckoning) reads as
    "encoders only", which is exactly what that robot has."""
    ordered = sorted(parts, key=lambda p: (_PART_DISPLAY_ORDER.index(p)
                                            if p in _PART_DISPLAY_ORDER else len(_PART_DISPLAY_ORDER), p))
    return " + ".join(PART_LABELS.get(p, p) for p in ordered) if ordered else "encoders only"


def parts_for(suite):
    """The parts a suite instance is built from. Prefers an instance's
    own `parts` attribute if it has one -- that's how ftc/sensors.py's
    make_distance_sensor_suite(n) variants (built by INSTANCE override,
    see its docstring) declare their differing ToF layouts without
    needing an entry in SUITE_PARTS keyed by a name that only exists at
    runtime."""
    own = getattr(suite, "parts", None)
    if own is not None:
        return frozenset(own)
    if suite.name not in SUITE_PARTS:
        raise KeyError(
            f"no parts declared for suite '{suite.name}' -- add it to ftc/bundle.py's SUITE_PARTS "
            "(or set a `parts` attribute on the instance) before bundling or costing it"
        )
    return SUITE_PARTS[suite.name]


def part_cost(parts):
    """Dollar cost of a set of parts. Unknown parts raise rather than
    silently costing $0 -- a bundle quietly under-priced is worse than
    one that refuses to be priced."""
    unknown = set(parts) - set(PART_COSTS_USD)
    if unknown:
        raise KeyError(f"no price for part(s) {sorted(unknown)} -- add them to ftc/config.py's PART_COSTS_USD")
    return sum(PART_COSTS_USD[p] for p in parts)


def conflicting_parts(parts):
    """Parts in `parts` that physically can't coexist on one robot (ftc/
    config.py's CONFLICTING_PART_GROUPS) -- e.g. two different ToF mount
    layouts. Returns the offending subset, empty if the bundle is
    buildable."""
    for group in CONFLICTING_PART_GROUPS:
        overlap = set(parts) & group
        if len(overlap) > 1:
            return overlap
    return set()


def part_cost_mismatches(tolerance=0.005):
    """Every suite whose SUITE_PARTS entry doesn't reprice to its own
    published cost_usd, as (name, suite_cost, parts_cost). Empty means
    the part model is a faithful refactor of ftc/sensors.py's flat
    prices rather than a second, drifting source of truth -- see ftc/
    scratch/bundle_test.py, which fails on any non-empty result."""
    mismatches = []
    for name, suite_cls in SUITES.items():
        declared = suite_cls.cost_usd
        from_parts = part_cost(SUITE_PARTS[name])
        if abs(declared - from_parts) > tolerance:
            mismatches.append((name, declared, from_parts))
    return mismatches


class CompositeObstacleSensor:
    """Every obstacle sensor on the robot, scanned as one. Satisfies the
    same sense(grid, position, heading_deg_now) -> newly-seen-cells
    contract ftc/sensors.py's ConeSensor already does, so ftc/match.py
    needs no knowledge that a bundle exists.

    Exactly equivalent to its single child when it wraps only one: each
    child already filters its own previously-seen cells, so with one
    child `seen - self.known_obstacles` is a no-op subtraction and the
    returned set is identical, cell for cell. With several children it
    also dedupes ACROSS them -- a cell a front-mounted cone saw twenty
    ticks ago and a side-mounted cone sees now is not "newly seen," and
    reporting it as such would trigger replans ftc/match.py has no
    reason to pay for."""

    def __init__(self, sensors):
        self.sensors = list(sensors)
        self.known_obstacles = set()

    def sense(self, grid, position, heading_deg_now):
        seen = set()
        for sensor in self.sensors:
            seen |= sensor.sense(grid, position, heading_deg_now)
        newly_seen = seen - self.known_obstacles
        self.known_obstacles |= seen
        return newly_seen


class BundleSuite(SensorSuite):
    """Two or more suites bought together, presented to ftc/match.py as
    one. Built via make_bundle() below rather than directly.

    Capability merging is documented at module level; the one piece
    worth restating here is tag_correction. A bundle does NOT call each
    pose-fixing component's tag_correction in turn -- that would consume
    a fresh rng draw per component and per tick, so a two-camera bundle
    would diverge from the identical single-camera robot on a shared
    seed for reasons that have nothing to do with its extra camera.
    Instead the bundle runs ftc/sensors.py's AprilTagSuite.tag_correction
    ONCE, with `camera_mount_headings_deg` set to the union of its
    components' mounts, which is what the robot physically has: one
    detection pipeline reading whichever camera sees a tag. Every
    pose-fixing suite in ftc/sensors.py uses exactly that model
    (AprilTagSuite, AprilTagImuSuite and DualCameraAprilTagSuite all
    delegate to it, differing only in mounts), so this is an exact
    reproduction for every bundle buildable today, not an approximation
    -- a future pose-fixing suite with a genuinely different correction
    model would need this method extended rather than inherited."""

    def __init__(self, components):
        self.components = list(components)
        self.component_names = [c.name for c in self.components]
        self.name = "+".join(self.component_names)
        self.parts = frozenset().union(*(parts_for(c) for c in self.components)) if self.components \
            else frozenset()

        conflict = conflicting_parts(self.parts)
        if conflict:
            raise ValueError(
                f"bundle {self.name} needs mutually exclusive parts {sorted(conflict)} -- a robot mounts "
                "one of these layouts, not several (ftc/config.py's CONFLICTING_PART_GROUPS)"
            )

        self.cost_usd = part_cost(self.parts)
        # Naive sum of the components' own prices. Kept only so callers
        # can report what the union-costing actually saved; nothing in
        # this project uses it as a price.
        self.naive_sum_cost_usd = sum(c.cost_usd for c in self.components)
        # Distinct purchasable parts -- a plain proxy for integration
        # effort, which this project's dollar model cannot price (see
        # ftc/sensors.py's ImuSuite: $0 hardware, real firmware work).
        # Reported alongside cost by ftc/optimizer.py so "cheapest" and
        # "simplest" stay visibly different questions.
        self.part_count = len(self.parts)

        self.senses_obstacles = any(c.senses_obstacles for c in self.components)
        self.fixes_pose = any(c.fixes_pose for c in self.components)
        self.fixes_heading = any(c.fixes_heading for c in self.components)
        # The best localization hardware on the robot sets its drift
        # rate; adding a camera to a robot that has odometry pods does
        # not make it drift like one that doesn't.
        self.drift_per_cell = min(c.drift_per_cell for c in self.components)

        mounts = []
        for component in self.components:
            if not component.fixes_pose:
                continue
            for mount in component.camera_mount_headings_deg:
                if mount not in mounts:
                    mounts.append(mount)
        # Preserved in component order (not sorted) so a single-component
        # bundle hands AprilTagSuite.tag_correction the identical list
        # object contents that component would have used itself.
        self.camera_mount_headings_deg = mounts or list(SensorSuite.camera_mount_headings_deg)

        self.heading_correction_factor = max(
            (getattr(c, "heading_correction_factor", 0.0) for c in self.components), default=0.0
        )
        self.integration_notes = " | ".join(
            f"{SUITE_LABELS.get(c.name, c.name)}: {c.integration_notes}" for c in self.components
        )

    def label(self):
        return " + ".join(SUITE_LABELS.get(n, n) for n in self.component_names)

    def parts_label(self):
        """What this robot IS, rather than which suites were named to
        build it -- the unambiguous display name (see PART_LABELS)."""
        return parts_label(self.parts)

    def fresh(self):
        """An equivalent bundle built from fresh component objects --
        what a caller running many trials should use per trial, matching
        every benchmark in this project's `suite = SUITES[name]()`
        per-trial construction. copy.copy rather than type(c)() so a
        component carrying INSTANCE overrides (ftc/sensors.py's
        make_distance_sensor_suite variants override cost_usd and
        mount_headings_deg on the instance) keeps them; suites hold no
        mutable per-match state of their own -- the one stateful object,
        the obstacle sensor, is constructed per match by
        make_obstacle_sensor -- so a shallow copy is a genuinely fresh
        suite, not a shared one."""
        return BundleSuite([copy.copy(c) for c in self.components])

    def part_signature(self):
        """The robot this bundle actually is, independent of which
        suites were named to build it. Two bundles with the same
        signature are the same purchase and will behave identically, so
        ftc/optimizer.py evaluates only one of them."""
        return self.parts

    def make_obstacle_sensor(self):
        sensors = [s for s in (c.make_obstacle_sensor() for c in self.components) if s is not None]
        if not sensors:
            return None
        return CompositeObstacleSensor(sensors)

    def tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites, rng, fidelity=None):
        if not self.fixes_pose:
            return None
        return AprilTagSuite.tag_correction(self, true_grid, true_position, heading_deg_now, tag_sites,
                                              rng, fidelity)

    def heading_correction(self, rng):
        return self.heading_correction_factor


def make_bundle(*suite_names):
    """A BundleSuite from ftc/sensors.py suite names (or ready-made
    suite instances, which is how ftc/coverage_benchmark.py-style
    variants from make_distance_sensor_suite get bundled).

    Duplicate names collapse to one component. A single name is allowed
    and returns a bundle that behaves exactly like that suite -- useful
    to ftc/optimizer.py, which ranks singles and bundles side by side
    and wants one code path for both."""
    if not suite_names:
        raise ValueError("make_bundle needs at least one suite")
    components, seen = [], set()
    for item in suite_names:
        suite = SUITES[item]() if isinstance(item, str) else item
        if suite.name in seen:
            continue
        seen.add(suite.name)
        components.append(suite)
    return BundleSuite(components)


def enumerate_bundles(suite_names, min_size=2, max_size=None, drop_equivalent=True):
    """Every buildable combination of `suite_names` sized [min_size,
    max_size], as BundleSuites.

    Two filters keep the search space honest rather than merely large:
    combinations needing conflicting parts (two ToF layouts) are
    dropped as unbuildable, and -- with drop_equivalent -- so is any
    combination whose part signature a smaller/earlier combination
    already produced. The second one matters more than it sounds:
    {apriltag, apriltag_imu} and {apriltag_imu} are the same robot, so
    simulating both would burn trials to reproduce a number twice and
    then report the redundant bundle as a separate "option" a team
    could choose. The kept representative is the one with the fewest
    components, i.e. the simplest way to describe that robot."""
    if min_size < 1:
        raise ValueError("min_size must be >= 1")
    names = list(dict.fromkeys(suite_names))
    max_size = len(names) if max_size is None else min(max_size, len(names))

    bundles, by_signature = [], {}
    for size in range(min_size, max_size + 1):
        for combo in itertools.combinations(names, size):
            try:
                bundle = make_bundle(*combo)
            except ValueError:  # conflicting parts -- not a buildable robot
                continue
            signature = bundle.part_signature()
            if drop_equivalent and signature in by_signature:
                continue
            by_signature[signature] = bundle
            bundles.append(bundle)
    return bundles
