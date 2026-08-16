"""
Does D* Lite's incremental repair (nav/dstar_lite.py) actually replan
faster than re-running A* from scratch, on the two scenarios this
project already replans repeatedly on a live grid? Recreates both:

- **Moving obstacle** (nav/obstacles.py's MovingObstacle + the replanning
  policy in pygame_app/visualizer.py): an obstacle random-walks between free
  cardinal neighbors; every time it moves, the robot needs an up-to-date
  route from wherever it currently is to the goal.
- **Sensor discovery** (nav/sensor.py's LidarSensor/KnownGrid + the
  replanning policy in pygame_app/visualizer.py, `nav/scratch/lidar_test.py`):
  the robot only knows about obstacles it's sensed; every time a scan
  reveals something new that's actually on its planned route (the exact
  trigger condition pygame_app/visualizer.py uses:
  `newly_seen and (newly_seen & remaining or robot_pos in newly_seen)`),
  it needs a fresh route.

Both scenarios run twice on the identical event sequence (same obstacle
bounce timing / same sensed cells in the same order) -- once calling
`nav.algorithms.astar` fresh every time a replan is triggered (what every
one of the call sites named above actually does today), once using one
persistent `DStarLite` instance repaired incrementally. Only the planning
calls themselves are timed.

Like nav/scale_benchmark.py, this sweeps grid size (this project's actual
interactive grid is a fixed 25x25 -- the sweep exists specifically to
show whether/how the answer changes at a size this project doesn't
actually run at). `KnownGrid` gained an optional `size` parameter
(nav/sensor.py) for exactly this, the same reason `Grid` grew one for
nav/scale_benchmark.py.
"""
import csv
import random
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from nav.algorithms import astar
from nav.dstar_lite import DStarLite
from nav.grid import Grid
from nav.obstacles import MovingObstacle, find_free_neighbor
from nav.sensor import LidarSensor, KnownGrid

SIZES = [25, 50, 100, 200]
TRIALS_PER_SIZE = 6
DENSITY = 0.15
OUTPUT_DIR = Path(__file__).resolve().parent.parent / "benchmark_results"


def _clone_grid(grid):
    g = Grid(size=grid.size)
    g.cells = [row[:] for row in grid.cells]
    g.cost = [row[:] for row in grid.cost]
    g.diagonal = grid.diagonal
    g.cost_map_enabled = grid.cost_map_enabled
    return g


def _random_grid(size, rng, density=DENSITY):
    grid = Grid(size=size)
    for row in range(size):
        for col in range(size):
            if rng.random() < density:
                grid.cells[row][col] = Grid.OBSTACLE
    free = [(r, c) for r in range(size) for c in range(size) if grid.cells[r][c] == Grid.FREE]
    if len(free) < 2:
        return None, None, None
    start, goal = rng.sample(free, 2)
    return grid, start, goal


# -- Scenario 1: moving obstacle --


def _place_moving_obstacle(grid, size, rng, avoid):
    # Only needs to return a single starting cell now -- MovingObstacle
    # random-walks from one cell instead of bouncing between two fixed
    # ones (see nav/obstacles.py). Still requires at least one free
    # neighbor so the obstacle actually has somewhere to walk to; a cell
    # with zero free neighbors would never move, defeating this
    # benchmark's whole point of measuring repeated replans.
    free = [(r, c) for r in range(size) for c in range(size)
            if grid.cells[r][c] == Grid.FREE and (r, c) not in avoid]
    rng.shuffle(free)
    for cell in free:
        if find_free_neighbor(grid, cell) is not None:
            return cell
    return None


def _moving_obstacle_astar(grid, start, goal, cell, obstacle_seed, num_events):
    obstacle = MovingObstacle(cell, rng=random.Random(obstacle_seed))
    current = start
    total_s = 0.0
    replans = 0
    for _ in range(num_events):
        obstacle._move(grid)
        t0 = time.perf_counter()
        path, _, _ = astar(grid, current, goal)
        total_s += time.perf_counter() - t0
        replans += 1
        if path is None or len(path) < 2:
            continue
        current = path[1]
        if current == goal:
            break
    return total_s, replans


def _moving_obstacle_dstar(grid, start, goal, cell, obstacle_seed, num_events):
    # Same obstacle_seed as _moving_obstacle_astar's obstacle -- both
    # calls get their own MovingObstacle instance (each on its own cloned
    # grid), but seeding both from the same value keeps their random-walk
    # move sequences identical, which is what makes the astar-vs-D*-Lite
    # timing comparison apples to apples (see module docstring).
    obstacle = MovingObstacle(cell, rng=random.Random(obstacle_seed))
    # Timed like every other planning call below (and like _sensor_dstar's
    # own initial DStarLite(...) construction) -- the constructor's
    # internal compute_shortest_path() is D* Lite's first full solve, the
    # direct equivalent of _moving_obstacle_astar's very first astar()
    # call. Excluding it from total_s would give D* Lite a free head
    # start here that the sensor-discovery scenario doesn't give it,
    # making the two scenarios' reported speedups non-comparable.
    t0 = time.perf_counter()
    planner = DStarLite(grid, start, goal)
    total_s = time.perf_counter() - t0
    replans = 1
    current = start
    for _ in range(num_events):
        old_cell = obstacle.position
        obstacle._move(grid)
        new_cell = obstacle.position
        t0 = time.perf_counter()
        planner.update_edge_costs([old_cell, new_cell])
        planner.compute_shortest_path()
        path = planner.extract_path()
        total_s += time.perf_counter() - t0
        replans += 1
        if path is None or len(path) < 2:
            continue
        current = path[1]
        planner.move_start(current)
        if current == goal:
            break
    return total_s, replans


def run_moving_obstacle_trial(size, seed):
    rng = random.Random(seed)
    for _ in range(50):
        grid, start, goal = _random_grid(size, rng)
        if grid is None:
            continue
        p, _, _ = astar(grid, start, goal)
        if p is None:
            continue
        cell = _place_moving_obstacle(grid, size, rng, avoid={start, goal})
        if cell is None:
            continue
        # MovingObstacle.place() marks its own starting cell as an
        # obstacle immediately (see nav/obstacles.py) -- needed here too
        # so both planners' very first plan already accounts for it,
        # which means re-checking solvability *with* it placed, since
        # adding it could have closed the only route.
        grid.cells[cell[0]][cell[1]] = Grid.OBSTACLE
        p, _, _ = astar(grid, start, goal)
        if p is None:
            continue
        break
    else:
        raise RuntimeError(f"moving-obstacle scenario at size {size}: no solvable setup found")

    # Give a bigger grid proportionally more events to replan over, same
    # reasoning nav/scale_benchmark.py scales RRT's step_size/max_iters
    # with grid size -- a fixed event count would let a huge grid finish
    # its whole trial without the obstacle ever mattering.
    num_events = min(200, max(40, len(p)))
    obstacle_seed = rng.randrange(2 ** 31)

    a_s, a_replans = _moving_obstacle_astar(_clone_grid(grid), start, goal, cell, obstacle_seed, num_events)
    d_s, d_replans = _moving_obstacle_dstar(_clone_grid(grid), start, goal, cell, obstacle_seed, num_events)
    return {
        "scenario": "moving_obstacle",
        "grid_size": size,
        "seed": seed,
        "events": num_events,
        "astar_total_ms": round(a_s * 1000, 4),
        "astar_replans": a_replans,
        "dstar_total_ms": round(d_s * 1000, 4),
        "dstar_replans": d_replans,
        "speedup": round(a_s / d_s, 3) if d_s else 0,
    }


# -- Scenario 2: sensor discovery --


def _sensor_astar(true_grid, start, goal, size, radius, max_steps):
    lidar = LidarSensor(radius)
    current = start
    total_s = 0.0
    replans = 0

    lidar.sense(true_grid, current)
    t0 = time.perf_counter()
    known = KnownGrid(lidar.known_obstacles, size=size)
    path, _, _ = astar(known, current, goal)
    total_s += time.perf_counter() - t0
    replans += 1
    if path is None:
        return total_s, replans

    idx = 0
    for _ in range(max_steps):
        if idx >= len(path) - 1:
            break
        idx += 1
        current = path[idx]
        newly = lidar.sense(true_grid, current)
        remaining = set(path[idx:])
        if newly and (newly & remaining or current in newly):
            t0 = time.perf_counter()
            known = KnownGrid(lidar.known_obstacles, size=size)
            new_path, _, _ = astar(known, current, goal)
            total_s += time.perf_counter() - t0
            replans += 1
            if new_path is None:
                continue
            path, idx = new_path, 0
    return total_s, replans


def _sensor_dstar(true_grid, start, goal, size, radius, max_steps):
    lidar = LidarSensor(radius)
    current = start

    lidar.sense(true_grid, current)
    known = KnownGrid(lidar.known_obstacles, size=size)
    t0 = time.perf_counter()
    planner = DStarLite(known, current, goal)
    path = planner.extract_path()
    total_s = time.perf_counter() - t0
    replans = 1
    if path is None:
        return total_s, replans

    idx = 0
    for _ in range(max_steps):
        if idx >= len(path) - 1:
            break
        idx += 1
        current = path[idx]
        planner.move_start(current)
        newly = lidar.sense(true_grid, current)
        remaining = set(path[idx:])
        if newly and (newly & remaining or current in newly):
            t0 = time.perf_counter()
            for cell in newly:
                known.cells[cell[0]][cell[1]] = Grid.OBSTACLE
            planner.update_edge_costs(newly)
            planner.compute_shortest_path()
            new_path = planner.extract_path()
            total_s += time.perf_counter() - t0
            replans += 1
            if new_path is None:
                continue
            path, idx = new_path, 0
    return total_s, replans


def run_sensor_trial(size, seed):
    rng = random.Random(seed + 10_000_000)
    for _ in range(50):
        grid, start, goal = _random_grid(size, rng)
        if grid is None:
            continue
        p, _, _ = astar(grid, start, goal)
        if p is None:
            continue
        break
    else:
        raise RuntimeError(f"sensor scenario at size {size}: no solvable setup found")

    radius = max(3, size // 15)
    max_steps = min(4 * len(p), len(p) + 60)

    a_s, a_replans = _sensor_astar(grid, start, goal, size, radius, max_steps)
    d_s, d_replans = _sensor_dstar(grid, start, goal, size, radius, max_steps)
    return {
        "scenario": "sensor_discovery",
        "grid_size": size,
        "seed": seed,
        "events": max_steps,
        "astar_total_ms": round(a_s * 1000, 4),
        "astar_replans": a_replans,
        "dstar_total_ms": round(d_s * 1000, 4),
        "dstar_replans": d_replans,
        "speedup": round(a_s / d_s, 3) if d_s else 0,
    }


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def average(rows, key, size):
    matching = [r[key] for r in rows if r["grid_size"] == size]
    return sum(matching) / len(matching)


def plot_results(mo_rows, sd_rows, path):
    fig, axes = plt.subplots(1, 2, figsize=(11, 5))

    for ax, rows, title in ((axes[0], mo_rows, "Moving obstacle"), (axes[1], sd_rows, "Sensor discovery")):
        a_avgs = [average(rows, "astar_total_ms", s) for s in SIZES]
        d_avgs = [average(rows, "dstar_total_ms", s) for s in SIZES]
        ax.plot(SIZES, a_avgs, "o-", label="astar (from scratch)", color="tab:red")
        ax.plot(SIZES, d_avgs, "o-", label="D* Lite (incremental)", color="tab:blue")
        ax.set_xlabel("Grid size N (for an NxN grid)")
        ax.set_ylabel("Avg total planning time per trial (ms)")
        ax.set_yscale("log")
        ax.set_title(title)
        ax.legend()

    fig.tight_layout()
    fig.savefig(path, dpi=150)


# A speedup inside this band of 1.0 is reported as a tie rather than as
# a win or a loss -- at TRIALS_PER_SIZE trials, a few percent either way
# is run-to-run timing noise, and the earlier hand-written version of
# this writeup got into trouble precisely by narrating small margins as
# decisive.
TIE_BAND = 0.10
INTERACTIVE_SIZE = 25


def _verdict(speedup):
    if speedup > 1 + TIE_BAND:
        return "win"
    if speedup < 1 - TIE_BAND:
        return "loss"
    return "tie"


def _crossover(speedups):
    """Smallest swept size whose speedup clears the tie band, or None if
    D* Lite never gets there. Reported as an interval (the size before
    it, and it) because the sweep only samples 4 sizes -- claiming a
    precise crossover from 4 points would overstate the resolution."""
    for i, size in enumerate(SIZES):
        if speedups[size] > 1 + TIE_BAND:
            return (SIZES[i - 1], size) if i else (None, size)
    return None


def write_writeup(mo_rows, sd_rows, path):
    """Regenerates replan_writeup.md from the run that just happened.

    This function exists because it didn't: replan_writeup.md used to be
    hand-maintained while the CSV next to it was machine-generated, so
    every re-run silently invalidated the prose beside the data. It
    drifted far enough to claim D* Lite "wins at every size tested" and
    "2.4x faster even at the small 25x25 grid" while the CSV in the same
    directory showed a tie there. Every number and every win/loss/tie
    characterization below is therefore derived from `mo_rows`/`sd_rows`,
    not written down -- the mechanism explanations are static prose,
    because those explain *why* the numbers come out as they do and stay
    true regardless of the exact values."""
    stats = {}
    for name, rows in (("mo", mo_rows), ("sd", sd_rows)):
        a = {s: average(rows, "astar_total_ms", s) for s in SIZES}
        d = {s: average(rows, "dstar_total_ms", s) for s in SIZES}
        stats[name] = {"a": a, "d": d, "x": {s: a[s] / d[s] for s in SIZES}}

    mo, sd = stats["mo"], stats["sd"]
    small = INTERACTIVE_SIZE
    big = SIZES[-1]

    def phrase(sc, size):
        x = sc["x"][size]
        v = _verdict(x)
        if v == "win":
            return f"{x:.2f}x faster"
        if v == "loss":
            return f"{x:.2f}x -- i.e. {1 / x:.1f}x *slower*"
        return f"{x:.2f}x, a tie"

    def crossover_sentence(sc, label):
        cross = _crossover(sc["x"])
        if cross is None:
            return (f"D* Lite never clears the tie band on {label} at any size swept here, topping out at "
                    f"{sc['x'][big]:.2f}x on {big}x{big}.")
        prev, at = cross
        if prev is None:
            return (f"D* Lite is already ahead on {label} at the smallest size swept "
                    f"({small}x{small}, {sc['x'][small]:.2f}x).")
        return (f"On {label} the crossover sits between {prev}x{prev} and {at}x{at}, reaching "
                f"{sc['x'][big]:.2f}x by {big}x{big}.")

    lines = [
        "# D* Lite vs from-scratch A*: moving-obstacle and sensor-discovery replanning",
        "",
        f"{TRIALS_PER_SIZE} trials per grid size ({', '.join(f'{s}x{s}' for s in SIZES)}), "
        f"{DENSITY:.0%} obstacle density, both scenarios recreating the exact replanning policy this",
        "project already uses live (see `nav/replan_benchmark.py`'s docstring for",
        "the precise correspondence to `nav/obstacles.py` and `nav/sensor.py`).",
        "Raw data in `replan_results.csv`, plot in `replan_comparison.png`.",
        "Correctness (does D* Lite actually agree with A*, not just run faster) is",
        "checked separately and exhaustively in `nav/scratch/dstar_lite_test.py`.",
        "",
        "Generated by `python3 -m nav.replan_benchmark` -- do not hand-edit; it is",
        "overwritten on every run.",
        "",
        "## What the data shows",
        "",
        "| Size | Moving obstacle: astar | D* Lite | speedup | Sensor discovery: astar | D* Lite | speedup |",
        "|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for s in SIZES:
        lines.append(f"| {s}x{s} | {mo['a'][s]:.3f}ms | {mo['d'][s]:.3f}ms | {mo['x'][s]:.2f}x "
                     f"| {sd['a'][s]:.3f}ms | {sd['d'][s]:.3f}ms | {sd['x'][s]:.2f}x |")

    lines += [
        "",
        "(Both totals are summed over every replan in the trial, not per-replan --",
        "\"speedup\" is total astar time / total D* Lite time for the whole trial,",
        f"averaged over the {TRIALS_PER_SIZE} trials at that size. A speedup within "
        f"{TIE_BAND:.0%} of 1.0 is called a tie rather than a win or a loss: at this",
        "trial count a few percent either way is timing noise.)",
        "",
        f"Moving obstacle: at the {small}x{small} grid this project actually runs "
        f"interactively at, D* Lite is {phrase(mo, small)}. " + crossover_sentence(mo, "this scenario"),
        "This is the scenario where the mechanism plays out cleanly: an obstacle bouncing",
        "between two fixed cells only ever invalidates a small, localized",
        "neighborhood of the search each time it moves, and D* Lite's",
        "`update_edge_costs` + `compute_shortest_path` only ever touches that",
        "neighborhood -- not the whole grid, regardless of how big the grid is.",
        "A* has no way to reuse anything between calls; it re-explores from the",
        "robot's current cell outward every single time, and that cost grows with",
        "distance-to-goal (hence with grid size), while D* Lite's repair cost",
        "stays close to flat.",
        "",
        f"Sensor discovery: at {small}x{small}, D* Lite is {phrase(sd, small)}. "
        + crossover_sentence(sd, "sensor discovery"),
        "Two things are going on here, both real, not artifacts:",
        "",
        "1. Sensor discovery only ever adds obstacles, never removes them",
        "   (`known_obstacles` is monotonically growing -- see WRITEUPS.md's",
        "   sensor-model section), so each replan event tends to touch more newly",
        "   -blocked cells at once than the single bouncing obstacle in the other",
        "   scenario does, and D* Lite's per-vertex bookkeeping (heap push/pop",
        "   with lazy-deletion, dict lookups for g/rhs/entry, a full neighbor scan",
        "   per `update_vertex` call) is real, non-trivial Python-level constant-",
        "   factor overhead. On a small grid, A*'s search itself is *so* cheap",
        "   that D* Lite's bookkeeping overhead per event costs more than the",
        "   search it's replacing.",
        "2. Building a `KnownGrid` from scratch every replan -- exactly what",
        "   `pygame_app/visualizer.py`'s `planning_grid()` and this benchmark's astar",
        f"   baseline both do -- also isn't very expensive at {small}x{small}, so there's",
        "   less baseline cost to begin with for an incremental approach to beat.",
        "",
        "Both effects shrink relative to D* Lite's real advantage as the grid",
        f"grows -- a from-scratch A* search's cost climbs about "
        f"{sd['a'][big] / sd['a'][small]:.0f}x from {small}x{small} to {big}x{big}, while D* Lite's "
        f"incremental repair cost climbs only about {sd['d'][big] / sd['d'][small]:.0f}x over the same",
        "range, because the number of *newly relevant* cells per sensor update",
        "doesn't grow with total grid size, only with how far the robot has moved",
        "and the sensor's fixed radius.",
        "",
        "## The honest takeaway",
        "",
        "Neither scenario supports a flat \"D* Lite is just faster\" claim -- the",
        "real, useful finding is that its advantage is asymptotic, and where the",
        "crossover sits genuinely depends on how localized a single replan event",
        "is.",
    ]

    verdicts = {sc: _verdict(stats[sc]["x"][small]) for sc in ("mo", "sd")}
    if all(v == "win" for v in verdicts.values()):
        lines.append(f"D* Lite already pays off at this project's actual {small}x{small} scale on both "
                     "scenarios, so the incremental algorithm is simply the right default here.")
    elif all(v in ("loss", "tie") for v in verdicts.values()):
        lines.append(f"At this project's actual {small}x{small} scale D* Lite wins neither scenario "
                     f"(moving obstacle {mo['x'][small]:.2f}x, sensor discovery {sd['x'][small]:.2f}x); both "
                     "only pay off on grids several times larger than anything this project's pygame "
                     "visualizer or pybullet demos actually use.")
    else:
        won = "moving obstacle" if verdicts["mo"] == "win" else "sensor discovery"
        lost = "sensor discovery" if verdicts["mo"] == "win" else "moving obstacle"
        lines.append(f"At {small}x{small} the two scenarios disagree: {won} already favors D* Lite while "
                     f"{lost} does not, which is exactly the point -- how localized a replan event is "
                     "decides whether incremental repair is worth its bookkeeping.")

    lines += [
        "",
        "That's not a reason to dismiss the algorithm; it's the",
        "same \"wrong tool at this project's actual scale, right tool at a bigger",
        "one\" conclusion `benchmark_results/writeup.md` already reached for plain",
        "RRT, arrived at independently and for a structurally different reason",
        "(constant-factor bookkeeping overhead here, versus an O(n) nearest-",
        "neighbor scan there) -- and it's exactly the kind of claim this project's",
        "own stated ethos requires being run and measured rather than assumed.",
    ]

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    mo_rows = [run_moving_obstacle_trial(size, size * 1000 + t) for size in SIZES for t in range(TRIALS_PER_SIZE)]
    sd_rows = [run_sensor_trial(size, size * 1000 + t) for size in SIZES for t in range(TRIALS_PER_SIZE)]

    write_csv(mo_rows + sd_rows, OUTPUT_DIR / "replan_results.csv")
    plot_results(mo_rows, sd_rows, OUTPUT_DIR / "replan_comparison.png")
    write_writeup(mo_rows, sd_rows, OUTPUT_DIR / "replan_writeup.md")

    print(f"Wrote {len(mo_rows) + len(sd_rows)} trials to {OUTPUT_DIR / 'replan_results.csv'}")
    print(f"Plot saved to {OUTPUT_DIR / 'replan_comparison.png'}")
    print(f"Writeup saved to {OUTPUT_DIR / 'replan_writeup.md'}")

    for label, rows in (("Moving obstacle", mo_rows), ("Sensor discovery", sd_rows)):
        print(f"\n{label} ({TRIALS_PER_SIZE} trials/size):")
        for size in SIZES:
            a_avg = average(rows, "astar_total_ms", size)
            d_avg = average(rows, "dstar_total_ms", size)
            print(f"  {size:4d}x{size:<4d}: astar {a_avg:9.3f}ms | D* Lite {d_avg:9.3f}ms | "
                  f"speedup {a_avg / d_avg:.2f}x")
