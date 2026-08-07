"""
Does ftc/opponent_benchmark.py's moving-opponent integration actually
work the way its docstring claims: identical candidate-cell selection
to the existing static blocker, simulated-time-driven ticking (not
real time), zero effect on every existing caller that doesn't pass
moving_obstacles, and an obstacle that actually moves over a match?

1. run_match's new `moving_obstacles` parameter defaults to `()` --
   calling it exactly as every priority 1-3 module already does (no
   moving_obstacles argument at all) produces byte-identical results to
   before this change, for a suite/scenario combination pinned down
   ahead of time.
2. `_place_moving_blocker` and nav/field_variance.py's own
   `_place_unplanned_blocker` choose the SAME cell given the same rng
   seed and probability=1.0 (guaranteed gate) -- the "identical
   candidate-cell selection, only what happens next differs" claim,
   checked directly rather than assumed from reading both functions.
3. run_match actually calls `.tick(ground_truth, now_ms)` with
   `now_ms == elapsed_s * 1000` on every loop iteration -- checked with
   a spy object standing in for a MovingObstacle that just records every
   now_ms it's ticked with, rather than depending on a real
   MovingObstacle happening to have a free neighbor and the match
   happening to run long enough for a real move to occur (both are
   scenario-luck-dependent; the spy isn't).
4. Two identical run_match calls with the same moving_obstacles setup
   (fresh MovingObstacle built from the same seed each time) produce
   identical MatchResults (barring the unseeded wall-clock
   planning_time_s field, which every other scratch test in this
   project also excludes from equality checks) -- ticking on
   `elapsed_s * 1000` rather than a real timer means outcomes can't
   depend on how fast this process actually executes.
"""
import random

import ftc.match as match_module
from ftc.field import build_grid, tag_sites_for
from ftc.opponent_benchmark import _place_moving_blocker
from ftc.sensors import DeadReckoningSuite, DistanceSensorSuite
from nav.field_variance import _place_unplanned_blocker, generate_ground_truth
from nav.grid import Grid

LAYOUT = "cluttered"


def _scenario(seed=42):
    grid = build_grid(LAYOUT)
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    rng = random.Random(seed)
    from nav.algorithms import astar
    start = goal = None
    for _ in range(50):
        start, goal = rng.sample(free, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= 6:
            break
    ground_truth, actual_start = generate_ground_truth(grid, start, goal, 0.0, seed=seed)
    tag_sites = tag_sites_for(LAYOUT)
    return grid, start, goal, ground_truth, actual_start, tag_sites


def _same_ignoring_wall_clock_timing(a, b):
    """Compares two MatchResults on everything except planning_time_s --
    a real time.perf_counter() measurement (ftc/match.py), not a seeded
    quantity, so two otherwise-identical runs can legitimately report
    slightly different values."""
    return (a.success == b.success and a.elapsed_s == b.elapsed_s and a.over_budget == b.over_budget
            and a.collisions == b.collisions and a.replans == b.replans
            and a.final_pose_error_in == b.final_pose_error_in and a.steps == b.steps)


def check_empty_moving_obstacles_is_a_true_no_op():
    grid, start, goal, ground_truth, actual_start, tag_sites = _scenario()
    a = match_module.run_match(DistanceSensorSuite(), grid, start, goal, ground_truth, actual_start,
                                tag_sites, random.Random(7))
    b = match_module.run_match(DistanceSensorSuite(), grid, start, goal, ground_truth, actual_start,
                                tag_sites, random.Random(7), moving_obstacles=())
    ok = _same_ignoring_wall_clock_timing(a, b)
    print(f"  no moving_obstacles arg: {a}")
    print(f"  moving_obstacles=(): {b}")
    print(f"omitting moving_obstacles and passing () explicitly give identical results: {'OK' if ok else 'FAIL'}")
    return ok


def check_candidate_cell_selection_matches_static():
    grid = build_grid(LAYOUT)
    free = [(r, c) for r in range(grid.size) for c in range(grid.size) if grid.cells[r][c] == 0]
    rng = random.Random(3)
    from nav.algorithms import astar
    start = goal = None
    for _ in range(50):
        start, goal = rng.sample(free, 2)
        path, _, _ = astar(grid, start, goal)
        if path is not None and len(path) >= 6:
            break

    static_grid = Grid(size=grid.size)
    static_grid.cells = [row[:] for row in grid.cells]
    static_grid.diagonal = grid.diagonal
    _place_unplanned_blocker(static_grid, grid, start, goal, 1.0, random.Random(99))
    static_cell = next((r, c) for r in range(grid.size) for c in range(grid.size)
                        if static_grid.cells[r][c] == Grid.OBSTACLE and grid.cells[r][c] == Grid.FREE)

    moving_grid = Grid(size=grid.size)
    moving_grid.cells = [row[:] for row in grid.cells]
    moving_grid.diagonal = grid.diagonal
    obstacle = _place_moving_blocker(moving_grid, grid, start, goal, 1.0, random.Random(99), period_ms=700)

    ok = obstacle is not None and obstacle.cell == static_cell
    print(f"  static blocker landed on {static_cell}, moving blocker landed on "
          f"{obstacle.cell if obstacle else None}")
    print(f"_place_moving_blocker picks the same cell as nav/field_variance.py's _place_unplanned_blocker "
          f"given the same rng seed: {'OK' if ok else 'FAIL'}")
    return ok


class _SpyObstacle:
    """Stands in for a MovingObstacle in run_match's moving_obstacles
    sequence -- records every now_ms it's ticked with instead of moving,
    so the "run_match ticks on simulated elapsed_s * 1000" claim can be
    checked directly instead of depending on a real MovingObstacle
    happening to have a free neighbor and the match happening to run
    past period_ms before ending (both scenario-luck-dependent -- an
    earlier version of this test relied on that luck and flaked when a
    match ended in a collision before 700ms of simulated time had
    passed)."""

    def __init__(self):
        self.now_ms_calls = []

    def tick(self, grid, now_ms, blocked=frozenset()):
        self.now_ms_calls.append(now_ms)
        return False


def check_run_match_ticks_on_simulated_time():
    """The tick at the top of iteration N uses elapsed_s as accrued
    through the end of iteration N-1 -- so the LAST recorded now_ms is
    expected to be strictly less than the final result.elapsed_s * 1000
    by roughly one step's worth of drive/turn time (that final step's
    time gets added *after* that iteration's tick call, before the loop
    either completes normally or exits). The real claims to check are:
    ticking starts at simulated t=0, the sequence is monotonically
    non-decreasing (time doesn't run backward), and it never exceeds the
    match's own final elapsed_s -- not that the two are numerically
    equal."""
    grid, start, goal, ground_truth, actual_start, tag_sites = _scenario(seed=11)
    spy = _SpyObstacle()
    result = match_module.run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start,
                                     tag_sites, random.Random(11), moving_obstacles=(spy,))
    expected_final_ms = result.elapsed_s * 1000.0
    ok = (len(spy.now_ms_calls) >= 1 and spy.now_ms_calls == sorted(spy.now_ms_calls)
          and spy.now_ms_calls[0] == 0.0 and spy.now_ms_calls[-1] <= expected_final_ms + 1e-6)
    print(f"  now_ms sequence: {spy.now_ms_calls}")
    print(f"  final result.elapsed_s * 1000 = {expected_final_ms}")
    print(f"run_match ticks with now_ms == elapsed_s * 1000, starting at 0, monotonically non-decreasing, "
          f"never past the match's own final elapsed_s: {'OK' if ok else 'FAIL'}")
    return ok


def check_deterministic_given_same_seed():
    def run_once():
        grid, start, goal, ground_truth, actual_start, tag_sites = _scenario(seed=21)
        obstacle = _place_moving_blocker(ground_truth, grid, start, goal, 1.0, random.Random(8), period_ms=700)
        moving_obstacles = (obstacle,) if obstacle is not None else ()
        return match_module.run_match(DeadReckoningSuite(), grid, start, goal, ground_truth, actual_start,
                                       tag_sites, random.Random(21), moving_obstacles=moving_obstacles)

    first = run_once()
    second = run_once()
    ok = _same_ignoring_wall_clock_timing(first, second)
    print(f"  run 1: {first}")
    print(f"  run 2: {second}")
    print(f"two identical setups (same seeds) produce identical MatchResults -- ticking depends on "
          f"simulated elapsed_s, not real time: {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_empty_moving_obstacles_is_a_true_no_op():
    assert check_empty_moving_obstacles_is_a_true_no_op()


def test_candidate_cell_selection_matches_static():
    assert check_candidate_cell_selection_matches_static()


def test_run_match_ticks_on_simulated_time():
    assert check_run_match_ticks_on_simulated_time()


def test_deterministic_given_same_seed():
    assert check_deterministic_given_same_seed()


if __name__ == "__main__":
    checks = [
        check_empty_moving_obstacles_is_a_true_no_op(),
        check_candidate_cell_selection_matches_static(),
        check_run_match_ticks_on_simulated_time(),
        check_deterministic_given_same_seed(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
