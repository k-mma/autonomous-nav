"""
Searches the space of sensor BUNDLES (ftc/bundle.py) for the best robot
a team could actually buy, and -- the part that makes this more than a
ranking -- says whether a bundle is *significantly* better than any one
sensor bought alone, or whether it just looks better because it was
measured on different luck.

The gap this fills. ftc/suite_benchmark.py ranks seven fixed suites on
one axis of deviation at a time; ftc/recommend.py predicts a single
suite's success rate on one calibrated field. Neither can answer the
question a team with $300 actually has: given everything on the shelf,
which COMBINATION should we buy, does combining beat buying the single
best part, and does the answer change depending on what kind of match
we expect? This module answers all three, over an arbitrary set of
candidate suites.

Three ideas do the work:

1. SHARED SCENARIOS, PAIRED STATISTICS. Every candidate is evaluated on
   the identical list of pre-generated scenarios (same seeds, same
   ground truth, same start/goal) -- the design ftc/suite_benchmark.py
   already uses, taken one step further: because the pairing is exact,
   two candidates can be compared with nav/stats.py's
   bootstrap_paired_diff_ci instead of by eyeballing whether their
   independent CIs overlap. That is a much stronger test at this
   project's trial counts (see that function's docstring), and it is
   the whole basis for the word "significantly" anywhere in this
   module's output. A bundle that wins by 6 points with a paired CI of
   [+1, +11] is a real finding; one that wins by 6 points with [-9,
   +21] is reported as noise, not as a recommendation.

2. SCENARIO PROFILES, not one blended average. A ScenarioProfile is a
   named match the team might face -- heavy pose drift, a map that
   doesn't match the field, an opponent parking in the route, a tight
   corridor, everything at once at realistic fidelity. Each candidate
   is scored on all of them, and the summary reports the weighted mean,
   the WORST profile (minimax -- the robot you want if you can't
   predict your division), and the per-profile winner. "Which bundle is
   best" genuinely has different answers under those three questions,
   and collapsing them into one number hides exactly the tradeoff a
   team is choosing between.

3. THE SEARCH KNOWS WHAT IT COSTS. Bundles are costed over the union of
   their parts (ftc/bundle.py), duplicate robots are evaluated once,
   and the results are reduced to a Pareto frontier over (cost, success
   rate) plus a best-under-budget lookup -- so the output is "here is
   the frontier, and here is the best thing you can buy for $150,"
   never a single "optimal" number that quietly assumes money is free.
   Exhaustive search covers the whole space when it's small; greedy
   forward selection scales past it and, as a side effect, produces the
   marginal value of each sensor added -- each step tested against the
   previous one with the same paired statistics as above.

Runs as a CLI:

    python3 -m ftc.optimizer                                  # default components, exhaustive
    python3 -m ftc.optimizer --budget 150                     # best robot for $150
    python3 -m ftc.optimizer --objective worst_case           # most robust, not best-on-average
    python3 -m ftc.optimizer --search greedy --max-size 4     # marginal value of each addition
    python3 -m ftc.optimizer --components apriltag,imu,dual_camera_apriltag,odometry_pods --trials 20

ftc/optimizer_benchmark.py runs the same machinery at full rigor and
writes the CSV/chart/writeup study; this module holds the reusable
parts and stays free of matplotlib so importing it costs nothing.
"""
import argparse
import random
from dataclasses import dataclass, field

from nav.algorithms import astar
from nav.field_variance import generate_ground_truth
from nav.stats import bootstrap_ci, bootstrap_paired_diff_ci

from ftc.bundle import BundleSuite, enumerate_bundles, make_bundle
from ftc.config import DEAD_RECKONING_DRIFT_PER_CELL
from ftc.drivetrain import DRIVETRAINS
from ftc.field import build_grid, tag_sites_for
from ftc.match import run_match
from ftc.sensors import SUITES, SUITE_LABELS

MAX_ATTEMPTS_PER_TRIAL = 50
MIN_PATH_LEN = 4
DEFAULT_TRIALS_PER_PROFILE = 12
DEFAULT_BASE_SEED = 21_000_000
# The free-hardware baseline every value figure in this project is
# measured against (ftc/suite_benchmark.py, ftc/recommend.py, ftc/
# newsuites_benchmark.py all use the same one).
BASELINE_SUITE = "dead_reckoning"
# All 8 suites in ftc/sensors.py minus full_suite, which is itself a
# hardcoded bundle of three of the others -- ftc/bundle.py rebuilds it
# exactly from its components, so including it as a separate "part"
# would just create duplicate-signature candidates the search would
# then have to discard.
DEFAULT_COMPONENTS = [name for name in SUITES if name != "full_suite"]


@dataclass(frozen=True)
class ScenarioProfile:
    """One kind of match a team might face. The deviation axes are
    nav/field_variance.py's independent *_scale kwargs (Phase 2), the
    same ones ftc/suite_benchmark.py sweeps one at a time -- the
    difference here is that a profile can turn on several at once,
    because a real match does."""
    name: str
    label: str
    layout: str = "cluttered"
    variance_level: float = 0.6
    start_drift_scale: float = 0.0
    obstacle_drift_scale: float = 0.0
    blocker_scale: float = 0.0
    fidelity: str = None      # None -> ftc.config.MODEL_FIDELITY, resolved inside run_match
    drivetrain: str = None    # None -> ftc/match.py's legacy no-drivetrain model
    gearing: str = None       # None -> "stock"
    weight: float = 1.0

    def scale_kwargs(self):
        return dict(start_drift_scale=self.start_drift_scale,
                    obstacle_drift_scale=self.obstacle_drift_scale,
                    blocker_scale=self.blocker_scale)


# Five profiles spanning the failure modes this project has actually
# measured, rather than five arbitrary difficulty settings. The first
# three isolate one deviation axis each (so a bundle's result can be
# ATTRIBUTED to a capability -- pose fixing, obstacle sensing, or
# reacting to a blocker); the last two combine all three, one on a
# different layout and one at "realistic" fidelity where camera-FOV
# gating and heading drift are live.
#
# The levels are chosen for DISCRIMINATION, not for being impressive or
# punishing: each one is a point where the seven headline suites spread
# out across a wide range of success rates (checked against ftc_suite_
# writeup.md's published per-axis numbers, which this harness
# reproduces). A profile everything fails, or everything survives,
# costs a full set of trials and separates nothing -- the first draft
# of this list ran the combined profiles at variance_level 0.5 with
# every axis at 1.0 and scored 0% for every candidate including the
# $399 one, which is a measurement of the profile, not of the robots.
DEFAULT_PROFILES = [
    ScenarioProfile("pose_drift", "Heavy pose drift", variance_level=0.5, start_drift_scale=1.0),
    ScenarioProfile("map_error", "Field doesn't match the map", variance_level=0.5, obstacle_drift_scale=1.0),
    ScenarioProfile("opponent", "Opponent parks in the route", variance_level=0.5, blocker_scale=1.0),
    ScenarioProfile("combined_realistic", "Everything at once (realistic fidelity)", variance_level=0.3,
                    start_drift_scale=1.0, obstacle_drift_scale=1.0, blocker_scale=1.0,
                    fidelity="realistic"),
    ScenarioProfile("corridor", "Tight corridor, mixed deviation", layout="corridor", variance_level=0.3,
                    start_drift_scale=1.0, obstacle_drift_scale=1.0, blocker_scale=0.5),
]
PROFILES_BY_NAME = {p.name: p for p in DEFAULT_PROFILES}


@dataclass
class Scenario:
    seed: int
    grid: object
    start: tuple
    goal: tuple
    ground_truth: object
    actual_start: tuple
    tag_sites: tuple


_GRID_CACHE = {}
_SCENARIO_CACHE = {}


def _grid_for(layout):
    if layout not in _GRID_CACHE:
        grid = build_grid(layout)
        free_cells = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
        _GRID_CACHE[layout] = (grid, free_cells, tag_sites_for(layout))
    return _GRID_CACHE[layout]


def _solvable_scenario(trial_seed, grid, free_cells):
    """Same start/goal sampling ftc/suite_benchmark.py uses. Duplicated
    (it's eight lines) rather than imported from there, so this module
    doesn't pull matplotlib in through that import just to draw two
    random cells."""
    rng = random.Random(trial_seed)
    for _ in range(MAX_ATTEMPTS_PER_TRIAL):
        start, goal = rng.sample(free_cells, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= MIN_PATH_LEN:
            return start, goal
    raise RuntimeError(f"no solvable scenario for seed {trial_seed} after {MAX_ATTEMPTS_PER_TRIAL} attempts")


def build_scenarios(profile, num_trials, base_seed):
    """`num_trials` fully-generated scenarios for one profile, cached.

    Generated ONCE and reused by every candidate -- that's what makes
    the comparison paired (see module docstring), and it also means the
    astar/ground-truth generation cost is paid once per profile instead
    of once per candidate, which is most of what keeps an exhaustive
    search affordable."""
    key = (profile, num_trials, base_seed)
    if key in _SCENARIO_CACHE:
        return _SCENARIO_CACHE[key]
    grid, free_cells, tag_sites = _grid_for(profile.layout)
    scenarios = []
    for t in range(num_trials):
        trial_seed = base_seed + t
        start, goal = _solvable_scenario(trial_seed, grid, free_cells)
        ground_truth, actual_start = generate_ground_truth(
            grid, start, goal, profile.variance_level, seed=trial_seed, **profile.scale_kwargs()
        )
        scenarios.append(Scenario(trial_seed, grid, start, goal, ground_truth, actual_start, tag_sites))
    _SCENARIO_CACHE[key] = scenarios
    return scenarios


@dataclass
class ProfileOutcome:
    """One candidate's per-trial results on one profile. `successes` is
    kept as the raw per-trial 0/1 vector, not just its mean, because the
    paired comparisons this module's conclusions rest on need trial i of
    one candidate lined up against trial i of another."""
    profile: str
    successes: list
    elapsed_s: list
    collisions: list
    over_budget: list

    @property
    def n(self):
        return len(self.successes)

    @property
    def success_rate(self):
        return sum(self.successes) / self.n if self.n else 0.0

    @property
    def over_budget_rate(self):
        return sum(self.over_budget) / self.n if self.n else 0.0

    @property
    def avg_elapsed_s(self):
        return sum(self.elapsed_s) / self.n if self.n else 0.0

    @property
    def avg_collisions(self):
        return sum(self.collisions) / self.n if self.n else 0.0


@dataclass
class CandidateResult:
    key: str
    label: str          # the suites named to build it ("AprilTag + IMU")
    parts_label: str    # the robot it actually is ("front camera + IMU")
    component_names: list
    parts: frozenset
    cost_usd: float
    part_count: int
    naive_sum_cost_usd: float
    senses_obstacles: bool
    fixes_pose: bool
    fixes_heading: bool
    low_drift: bool
    per_profile: dict
    profile_order: list
    weights: dict

    def capability_categories(self):
        """Which KINDS of failure this robot has an answer for. The
        categories are the project's own independent deviation axes
        (ftc/suite_benchmark.py's deviation types) rather than a
        hardware taxonomy, which is what makes them predictive: a
        bundle spanning two categories covers two ways a match is lost,
        while two sensors in the same category are largely correcting
        an error the other one already removed."""
        categories = set()
        if self.fixes_pose:
            categories.add("pose")
        if self.senses_obstacles:
            categories.add("obstacle")
        if self.low_drift:
            categories.add("drift")
        if self.fixes_heading:
            categories.add("heading")
        return categories

    def outcome_vector(self):
        """Every trial's success flag, concatenated in a fixed profile
        order -- the paired vector two candidates get compared on."""
        return [s for name in self.profile_order for s in self.per_profile[name].successes]

    @property
    def n_trials(self):
        return sum(self.per_profile[name].n for name in self.profile_order)

    @property
    def weighted_success_rate(self):
        total_weight = sum(self.weights[name] for name in self.profile_order)
        return sum(self.per_profile[name].success_rate * self.weights[name]
                   for name in self.profile_order) / total_weight

    @property
    def pooled_success_rate(self):
        """Unweighted rate over all pooled trials -- what the paired
        significance tests actually operate on (see report_synergy)."""
        vector = self.outcome_vector()
        return sum(vector) / len(vector) if vector else 0.0

    @property
    def worst_profile_rate(self):
        """The minimax score: how this candidate does on its WORST kind
        of match. The right objective for a team that can't predict
        which failure mode it'll face."""
        return min(self.per_profile[name].success_rate for name in self.profile_order)

    @property
    def worst_profile(self):
        return min(self.profile_order, key=lambda name: self.per_profile[name].success_rate)

    @property
    def avg_elapsed_s(self):
        return sum(self.per_profile[n].avg_elapsed_s for n in self.profile_order) / len(self.profile_order)

    @property
    def over_budget_rate(self):
        return sum(self.per_profile[n].over_budget_rate for n in self.profile_order) / len(self.profile_order)

    @property
    def avg_collisions(self):
        return sum(self.per_profile[n].avg_collisions for n in self.profile_order) / len(self.profile_order)

    def success_ci(self, seed=0):
        vector = self.outcome_vector()
        return bootstrap_ci(sum(vector), len(vector), seed=seed)

    def value_per_100(self, baseline_rate):
        """Percentage POINTS of success rate gained over the free
        baseline per $100 spent -- the same metric (and the same *100
        scaling fix) as ftc/suite_benchmark.py's write_writeup. None,
        never infinity, when the candidate is free: "infinite value per
        dollar" isn't a meaningful claim about a robot whose only real
        cost is integration effort (ftc/newsuites_benchmark.py makes the
        same call for ImuSuite)."""
        if self.cost_usd <= 0:
            return None
        return (self.weighted_success_rate - baseline_rate) / (self.cost_usd / 100) * 100


class BundleOptimizer:
    """Evaluates and searches sensor bundles over a set of scenario
    profiles. One instance holds the scenario set and the evaluation
    cache, so every candidate it scores is directly comparable to every
    other one it has scored."""

    def __init__(self, profiles=None, trials_per_profile=DEFAULT_TRIALS_PER_PROFILE,
                 base_seed=DEFAULT_BASE_SEED, on_progress=None):
        self.profiles = list(profiles) if profiles else list(DEFAULT_PROFILES)
        self.profile_order = [p.name for p in self.profiles]
        self.weights = {p.name: p.weight for p in self.profiles}
        self.trials_per_profile = trials_per_profile
        self.base_seed = base_seed
        self.on_progress = on_progress
        # Keyed by part signature, NOT by name: two differently-named
        # bundles that buy the same hardware are the same robot and get
        # the same answer, so the second one is free.
        self._cache = {}

    def _as_bundle(self, candidate):
        if isinstance(candidate, BundleSuite):
            return candidate
        if isinstance(candidate, str):
            return make_bundle(candidate)
        return make_bundle(candidate)  # a ready-made suite instance

    def evaluate(self, candidate):
        """Run every profile's trials for one candidate (a suite name, a
        suite instance, or a BundleSuite) and return a CandidateResult.

        A fresh suite instance is built per trial, exactly as every
        benchmark in this project does -- suites carry per-match state
        (ConeSensor.known_obstacles) that must not leak across trials.
        The rng handed to run_match is seeded from the trial seed, so
        rerunning any single row of any table this module prints
        reproduces it."""
        bundle = self._as_bundle(candidate)
        signature = bundle.part_signature()
        if signature in self._cache:
            return self._cache[signature]

        if self.on_progress:
            self.on_progress(bundle)

        per_profile = {}
        for profile in self.profiles:
            scenarios = build_scenarios(profile, self.trials_per_profile, self.base_seed)
            drivetrain = DRIVETRAINS[profile.drivetrain] if profile.drivetrain else None
            successes, elapsed, collisions, over_budget = [], [], [], []
            for scenario in scenarios:
                result = run_match(
                    bundle.fresh(), scenario.grid, scenario.start, scenario.goal, scenario.ground_truth,
                    scenario.actual_start, scenario.tag_sites, random.Random(scenario.seed),
                    fidelity=profile.fidelity, drivetrain=drivetrain, gearing=profile.gearing,
                )
                successes.append(int(result.success))
                elapsed.append(result.elapsed_s)
                collisions.append(result.collisions)
                over_budget.append(int(result.over_budget))
            per_profile[profile.name] = ProfileOutcome(profile.name, successes, elapsed, collisions,
                                                        over_budget)

        result = CandidateResult(
            key=bundle.name, label=bundle.label(), parts_label=bundle.parts_label(),
            component_names=list(bundle.component_names),
            parts=signature, cost_usd=bundle.cost_usd, part_count=bundle.part_count,
            naive_sum_cost_usd=bundle.naive_sum_cost_usd,
            senses_obstacles=bundle.senses_obstacles, fixes_pose=bundle.fixes_pose,
            fixes_heading=bundle.fixes_heading,
            # Anything below the plain encoder-only rate is a real
            # drift reduction; today only odometry pods provide one.
            low_drift=bundle.drift_per_cell < DEAD_RECKONING_DRIFT_PER_CELL,
            per_profile=per_profile,
            profile_order=list(self.profile_order), weights=dict(self.weights),
        )
        self._cache[signature] = result
        return result

    def evaluate_all(self, candidates):
        return [self.evaluate(c) for c in candidates]

    def baseline_rate(self):
        """The free dead-reckoning robot's weighted success rate -- what
        every value-per-dollar figure is measured against."""
        return self.evaluate(BASELINE_SUITE).weighted_success_rate


@dataclass
class SynergyReport:
    """Whether a bundle beats the best single sensor it contains -- the
    literal question "is this combination worth buying instead of just
    one of these." `significant` requires BOTH a positive lower CI bound
    and a bootstrap p-value under `alpha`, on the PAIRED difference."""
    bundle_key: str
    bundle_label: str
    bundle_rate: float
    best_single_key: str
    best_single_label: str
    best_single_rate: float
    delta: float
    ci_lo: float
    ci_hi: float
    p_value: float
    significant: bool
    cost_usd: float
    best_single_cost_usd: float
    extra_cost_usd: float
    pp_per_extra_100: float = None

    def verdict(self):
        if self.significant:
            return (f"{self.bundle_label} beats its own best single component "
                    f"({self.best_single_label}) by {self.delta:+.1%} "
                    f"[95% CI {self.ci_lo:+.1%}, {self.ci_hi:+.1%}], p={self.p_value:.3f} -- real synergy")
        if self.delta > 0:
            return (f"{self.bundle_label} is {self.delta:+.1%} over {self.best_single_label} but the paired "
                    f"CI [{self.ci_lo:+.1%}, {self.ci_hi:+.1%}] includes 0 (p={self.p_value:.3f}) -- not "
                    "distinguishable from noise at this trial count")
        if self.delta == 0:
            # Equal POOLED rates across profiles doesn't mean equal
            # behavior: a bundle can win one scenario and lose another
            # by the same margin (the per-scenario table is where that
            # shows up), so this says "ties," not "buys nothing."
            return (f"{self.bundle_label} ties {self.best_single_label} on pooled trials -- the extra "
                    f"${self.extra_cost_usd:.2f} buys no overall gain, though the per-scenario table may "
                    "still show it trading one kind of match for another")
        return (f"{self.bundle_label} does not beat {self.best_single_label} ({self.delta:+.1%}) -- "
                "the combination buys nothing this single sensor doesn't already provide")


def report_synergy(bundle_result, single_results, alpha=0.05, seed=0, restrict_to_components=True):
    """Paired comparison of a bundle against the best SINGLE candidate.

    By default the comparison is against the best single suite the
    bundle itself contains (`restrict_to_components`) -- that's the
    honest form of "does combining help": a bundle whose only merit is
    containing one strong sensor should not be credited with that
    sensor's performance. Pass restrict_to_components=False to compare
    against the best single option available at all, which is the
    stricter shopping question ("should we buy this bundle instead of
    the best single thing on the shelf").

    The test resamples the pooled trial vector across all profiles. With
    the default equal profile weights that pooled rate is exactly the
    weighted rate; with unequal weights it isn't, and the significance
    claim then applies to the pooled trials rather than to the weighted
    objective -- stated here because the two only coincide by default.
    """
    singles = [r for r in single_results if len(r.component_names) == 1]
    if restrict_to_components:
        singles = [r for r in singles if r.component_names[0] in bundle_result.component_names]
    if not singles:
        return None
    best_single = max(singles, key=lambda r: r.pooled_success_rate)

    a = best_single.outcome_vector()
    b = bundle_result.outcome_vector()
    ci_lo, ci_hi, p_value = bootstrap_paired_diff_ci(a, b, seed=seed)
    delta = bundle_result.pooled_success_rate - best_single.pooled_success_rate
    extra_cost = bundle_result.cost_usd - best_single.cost_usd
    return SynergyReport(
        bundle_key=bundle_result.key, bundle_label=bundle_result.label,
        bundle_rate=bundle_result.pooled_success_rate,
        best_single_key=best_single.key, best_single_label=best_single.label,
        best_single_rate=best_single.pooled_success_rate,
        delta=delta, ci_lo=ci_lo, ci_hi=ci_hi, p_value=p_value,
        significant=(ci_lo > 0 and p_value < alpha),
        cost_usd=bundle_result.cost_usd, best_single_cost_usd=best_single.cost_usd,
        extra_cost_usd=extra_cost,
        pp_per_extra_100=(delta / (extra_cost / 100) * 100) if extra_cost > 0 else None,
    )


def pareto_frontier(results, objective="weighted"):
    """The candidates nothing else beats on both price and performance:
    every result for which no other result is at least as cheap AND at
    least as good, with at least one of those strict. Everything off the
    frontier is a robot you should never buy -- something else is
    cheaper and better.

    Ties (identical cost and rate) keep the one with fewer parts, i.e.
    the simpler robot to actually integrate."""
    score = _objective_fn(objective)
    ordered = sorted(results, key=lambda r: (r.cost_usd, -score(r), r.part_count))
    frontier, best_rate = [], None
    for result in ordered:
        rate = score(result)
        if best_rate is None or rate > best_rate:
            frontier.append(result)
            best_rate = rate
    return frontier


def _objective_fn(objective):
    if objective == "weighted":
        return lambda r: r.weighted_success_rate
    if objective == "worst_case":
        return lambda r: r.worst_profile_rate
    if objective == "pooled":
        return lambda r: r.pooled_success_rate
    raise ValueError(f"unknown objective '{objective}' (weighted, worst_case, pooled)")


def rank(results, objective="weighted", baseline_rate=None):
    """Best first. "value" needs a baseline rate to measure gain
    against; the others don't."""
    if objective == "value":
        if baseline_rate is None:
            raise ValueError("objective 'value' needs baseline_rate")
        # Free candidates have no defined pp/$100 (see
        # CandidateResult.value_per_100) and sort last rather than
        # being silently treated as infinitely good value.
        return sorted(results, key=lambda r: (r.value_per_100(baseline_rate) is not None,
                                              r.value_per_100(baseline_rate) or 0.0), reverse=True)
    score = _objective_fn(objective)
    return sorted(results, key=lambda r: (score(r), -r.cost_usd), reverse=True)


def best_under_budget(results, budget_usd, objective="weighted"):
    """Best candidate costing <= budget_usd. Ties on performance break
    toward the cheaper, then simpler, robot -- a team should not pay
    more for the same measured result."""
    affordable = [r for r in results if r.cost_usd <= budget_usd]
    if not affordable:
        return None
    score = _objective_fn(objective)
    return max(affordable, key=lambda r: (score(r), -r.cost_usd, -r.part_count))


def best_per_profile(results):
    """{profile_name: winning CandidateResult} -- the "different
    scenarios have different answers" table. Ties break toward the
    cheaper candidate."""
    winners = {}
    for name in results[0].profile_order:
        winners[name] = max(results, key=lambda r: (r.per_profile[name].success_rate, -r.cost_usd))
    return winners


@dataclass
class GreedyStep:
    size: int
    added: str
    added_label: str
    bundle: CandidateResult
    previous: CandidateResult
    delta: float
    ci_lo: float
    ci_hi: float
    p_value: float
    significant: bool
    extra_cost_usd: float
    considered: list = field(default_factory=list)


def greedy_search(optimizer, component_names=None, objective="weighted", budget_usd=None, max_size=None,
                  alpha=0.05, require_significant=False, seed=0):
    """Forward selection: start from the best single sensor, then keep
    adding whichever remaining sensor improves the objective most, until
    nothing improves it (or the budget/size cap stops it).

    Why bother when exhaustive search exists: exhaustive is 2^N
    candidates and this project already has 8 usable components (256
    bundles x 5 profiles x 12 trials = 15,360 matches), so the space
    stops being free to enumerate the moment anyone adds a few more
    sensors. Greedy is O(N^2) evaluations and, more usefully, its output
    IS the answer to "what is each additional sensor actually worth" --
    every step reports the paired delta over the previous step's bundle,
    with the same significance test report_synergy uses.

    `require_significant` makes it stop as soon as an addition can't be
    distinguished from noise -- a stricter and more honest stopping rule
    than "stop when the mean stops going up," since at 60 trials a
    +2-point mean improvement routinely isn't real. Greedy is not
    guaranteed to find the exhaustive optimum (two sensors that are only
    worth buying together will never be picked up one at a time -- the
    exact blind spot a synergy search cares about), which is why
    ftc/optimizer_benchmark.py runs both and reports where they differ
    rather than trusting either alone.
    """
    names = list(component_names or DEFAULT_COMPONENTS)
    score = _objective_fn(objective)
    max_size = len(names) if max_size is None else max_size

    singles = optimizer.evaluate_all(names)
    affordable_singles = [r for r in singles if budget_usd is None or r.cost_usd <= budget_usd]
    if not affordable_singles:
        return []
    current = max(affordable_singles, key=score)
    chosen = list(current.component_names)
    steps = [GreedyStep(size=1, added=chosen[0], added_label=current.label, bundle=current,
                        previous=current, delta=0.0, ci_lo=0.0, ci_hi=0.0, p_value=1.0,
                        significant=False, extra_cost_usd=current.cost_usd,
                        considered=[r.key for r in affordable_singles])]

    while len(chosen) < max_size:
        best_candidate, best_score, considered = None, score(current), []
        for name in names:
            if name in chosen:
                continue
            try:
                candidate_bundle = make_bundle(*chosen, name)
            except ValueError:  # conflicting parts (e.g. two ToF layouts)
                continue
            if budget_usd is not None and candidate_bundle.cost_usd > budget_usd:
                continue
            candidate = optimizer.evaluate(candidate_bundle)
            considered.append(candidate.key)
            if score(candidate) > best_score:
                best_candidate, best_score = candidate, score(candidate)
        if best_candidate is None:
            break

        added = next(n for n in best_candidate.component_names if n not in chosen)
        ci_lo, ci_hi, p_value = bootstrap_paired_diff_ci(
            current.outcome_vector(), best_candidate.outcome_vector(), seed=seed + len(chosen)
        )
        significant = ci_lo > 0 and p_value < alpha
        steps.append(GreedyStep(
            size=len(chosen) + 1, added=added, added_label=SUITE_LABELS.get(added, added),
            bundle=best_candidate, previous=current,
            delta=best_candidate.pooled_success_rate - current.pooled_success_rate,
            ci_lo=ci_lo, ci_hi=ci_hi, p_value=p_value, significant=significant,
            extra_cost_usd=best_candidate.cost_usd - current.cost_usd, considered=considered,
        ))
        if require_significant and not significant:
            break
        chosen = list(best_candidate.component_names)
        current = best_candidate

    return steps


def exhaustive_search(optimizer, component_names=None, min_size=1, max_size=None, budget_usd=None):
    """Every buildable, non-duplicate bundle in the space, evaluated.
    Bundles priced above `budget_usd` are skipped before being
    simulated, which is what makes a budget-constrained search cheaper
    than an unconstrained one rather than just a filter on its
    output."""
    names = list(component_names or DEFAULT_COMPONENTS)
    bundles = enumerate_bundles(names, min_size=min_size, max_size=max_size)
    if budget_usd is not None:
        bundles = [b for b in bundles if b.cost_usd <= budget_usd]
    return optimizer.evaluate_all(bundles)


def summarize(results, optimizer, objective="weighted", budget_usd=None, alpha=0.05,
              synergy_top_n=5, seed=0):
    """Everything a caller needs to report on a set of results, as one
    dict -- ranking, Pareto frontier, per-profile winners, synergy
    reports for the top bundles, and the budget pick.

    "value" (pp/$100) is a ranking objective but not a Pareto or budget
    one: the frontier and the budget pick are both defined over
    (cost, performance), and cost is already the axis "value" divides
    by -- ranking a frontier by value-per-dollar would just re-sort it
    by cheapness. Both fall back to the weighted rate in that case."""
    baseline = optimizer.evaluate(BASELINE_SUITE)
    baseline_rate = baseline.weighted_success_rate
    ranked = rank(results, objective=objective, baseline_rate=baseline_rate)
    bundles = [r for r in ranked if len(r.component_names) > 1]
    synergies = []
    for i, bundle_result in enumerate(bundles[:synergy_top_n]):
        report = report_synergy(bundle_result, results, alpha=alpha, seed=seed + i)
        if report:
            synergies.append(report)
    return {
        "baseline": baseline,
        "baseline_rate": baseline_rate,
        "ranked": ranked,
        "pareto": pareto_frontier(results, objective=objective if objective != "value" else "weighted"),
        "per_profile": best_per_profile(results),
        "synergies": synergies,
        "budget_pick": best_under_budget(results, budget_usd,
                                         objective=objective if objective != "value" else "weighted")
        if budget_usd is not None else None,
    }


def format_result_row(result, baseline_rate):
    """Rows are labeled by PARTS, not by component suites: two rows
    reading "Odometry pods + AprilTag + Dual-camera AprilTag" and
    "Odometry pods + Dual-camera AprilTag" are the same robot described
    two ways, and a ranking that prints both under near-identical names
    is unreadable exactly where it matters most."""
    value = result.value_per_100(baseline_rate)
    value_str = "     --" if value is None else f"{value:+6.1f}"
    return (f"{result.parts_label:<52} ${result.cost_usd:>7.2f}  "
            f"success {result.weighted_success_rate:>4.0%}  "
            f"worst {result.worst_profile_rate:>4.0%}  "
            f"pp/$100 {value_str}  parts {result.part_count}")


def _print_summary(summary, objective, budget_usd, top_n):
    baseline_rate = summary["baseline_rate"]
    print(f"\n=== Ranking by {objective} (top {top_n}) ===")
    print(f"(free dead-reckoning baseline: {baseline_rate:.0%} weighted success)\n")
    for result in summary["ranked"][:top_n]:
        print("  " + format_result_row(result, baseline_rate))

    print("\n=== Pareto frontier (nothing is both cheaper and better) ===\n")
    for result in summary["pareto"]:
        print("  " + format_result_row(result, baseline_rate))

    print("\n=== Best bundle per scenario ===\n")
    for name, winner in summary["per_profile"].items():
        rate = winner.per_profile[name].success_rate
        label = PROFILES_BY_NAME[name].label if name in PROFILES_BY_NAME else name
        print(f"  {label:<40} -> {winner.parts_label:<44} {rate:>4.0%}  ${winner.cost_usd:.2f}")

    print("\n=== Does bundling actually beat one sensor alone? ===\n")
    if not summary["synergies"]:
        print("  (no multi-sensor bundles in this result set)")
    for report in summary["synergies"]:
        print(f"  {report.verdict()}")
        if report.significant and report.pp_per_extra_100 is not None:
            print(f"      +${report.extra_cost_usd:.2f} over the single sensor "
                  f"-> {report.pp_per_extra_100:+.1f}pp per extra $100")

    if budget_usd is not None:
        print(f"\n=== Best robot for ${budget_usd:.2f} ===\n")
        pick = summary["budget_pick"]
        if pick is None:
            print(f"  Nothing in the search space costs <= ${budget_usd:.2f}.")
        else:
            print("  " + format_result_row(pick, baseline_rate))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--components", default=",".join(DEFAULT_COMPONENTS),
                        help="Comma-separated ftc/sensors.py suite names to combine.")
    parser.add_argument("--search", choices=["exhaustive", "greedy"], default="exhaustive")
    parser.add_argument("--objective", choices=["weighted", "worst_case", "pooled", "value"],
                        default="weighted",
                        help="weighted: mean success across scenarios. worst_case: minimax, the robot that "
                             "never falls apart. value: success gained per $100. pooled: unweighted pool.")
    parser.add_argument("--min-size", type=int, default=1,
                        help="1 (default) ranks single suites alongside bundles; 2 searches bundles only.")
    parser.add_argument("--max-size", type=int, default=3,
                        help="Cap on components per bundle (exhaustive search is 2^N without one).")
    parser.add_argument("--budget", type=float, default=None, help="Dollar cap; also reports the best pick "
                                                                    "at or under it.")
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS_PER_PROFILE,
                        help="Trials per scenario profile per candidate.")
    parser.add_argument("--profiles", default=None,
                        help=f"Comma-separated subset of {','.join(PROFILES_BY_NAME)}.")
    parser.add_argument("--require-significant", action="store_true",
                        help="Greedy search only: stop adding sensors once an addition is no longer "
                             "statistically distinguishable from noise.")
    parser.add_argument("--top", type=int, default=12, help="How many ranked rows to print.")
    parser.add_argument("--seed", type=int, default=DEFAULT_BASE_SEED)
    args = parser.parse_args()

    components = [c.strip() for c in args.components.split(",") if c.strip()]
    unknown = [c for c in components if c not in SUITES]
    if unknown:
        parser.error(f"unknown suite(s) {unknown} -- choose from {sorted(SUITES)}")
    profiles = ([PROFILES_BY_NAME[p.strip()] for p in args.profiles.split(",")] if args.profiles
                else DEFAULT_PROFILES)

    evaluated = []
    optimizer = BundleOptimizer(profiles=profiles, trials_per_profile=args.trials, base_seed=args.seed,
                                on_progress=lambda b: evaluated.append(b.name))

    print(f"Components: {', '.join(components)}")
    print(f"Scenario profiles ({args.trials} trials each): "
          f"{', '.join(p.label for p in profiles)}")
    print(f"Search: {args.search}, objective: {args.objective}"
          + (f", budget: ${args.budget:.2f}" if args.budget is not None else ""))

    if args.search == "greedy":
        steps = greedy_search(optimizer, components, objective=args.objective, budget_usd=args.budget,
                              max_size=args.max_size, require_significant=args.require_significant)
        print("\n=== Greedy forward selection (marginal value of each sensor added) ===\n")
        for step in steps:
            if step.size == 1:
                print(f"  start: {step.bundle.parts_label} -- {step.bundle.weighted_success_rate:.0%} "
                      f"at ${step.bundle.cost_usd:.2f}")
                continue
            mark = "significant" if step.significant else "NOT significant"
            print(f"  + {step.added_label} (+${step.extra_cost_usd:.2f}) -> "
                  f"{step.bundle.weighted_success_rate:.0%} "
                  f"({step.delta:+.1%}, 95% CI [{step.ci_lo:+.1%}, {step.ci_hi:+.1%}], "
                  f"p={step.p_value:.3f}) -- {mark}")
        results = list(optimizer._cache.values())
    else:
        results = exhaustive_search(optimizer, components, min_size=args.min_size,
                                    max_size=args.max_size, budget_usd=args.budget)
        # Singles are needed as the comparison floor for every synergy
        # report even when the search itself only enumerates bundles.
        if args.min_size > 1:
            results = optimizer.evaluate_all(components) + results

    summary = summarize(results, optimizer, objective=args.objective, budget_usd=args.budget)
    print(f"\nEvaluated {len(optimizer._cache)} distinct robots "
          f"({len(optimizer._cache) * len(profiles) * args.trials} matches).")
    _print_summary(summary, args.objective, args.budget, args.top)


if __name__ == "__main__":
    main()
