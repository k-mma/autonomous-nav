"""
Does D* Lite's incremental repair (nav/dstar_lite.py) actually replan
faster than re-running A* from scratch, on the two scenarios this
project already replans repeatedly on a live grid? Recreates both:

- **Moving obstacle** (nav/obstacles.py's MovingObstacle + the replanning
  policy in nav/visualizer.py): an obstacle random-walks between free
  cardinal neighbors; every time it moves, the robot needs an up-to-date
  route from wherever it currently is to the goal.
- **Sensor discovery** (nav/sensor.py's LidarSensor/KnownGrid + the
  replanning policy in nav/visualizer.py, `nav/scratch/lidar_test.py`):
  the robot only knows about obstacles it's sensed; every time a scan
  reveals something new that's actually on its planned route (the exact
  trigger condition nav/visualizer.py uses:
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


if __name__ == "__main__":
    OUTPUT_DIR.mkdir(exist_ok=True)

    mo_rows = [run_moving_obstacle_trial(size, size * 1000 + t) for size in SIZES for t in range(TRIALS_PER_SIZE)]
    sd_rows = [run_sensor_trial(size, size * 1000 + t) for size in SIZES for t in range(TRIALS_PER_SIZE)]

    write_csv(mo_rows + sd_rows, OUTPUT_DIR / "replan_results.csv")
    plot_results(mo_rows, sd_rows, OUTPUT_DIR / "replan_comparison.png")

    print(f"Wrote {len(mo_rows) + len(sd_rows)} trials to {OUTPUT_DIR / 'replan_results.csv'}")
    print(f"Plot saved to {OUTPUT_DIR / 'replan_comparison.png'}")

    for label, rows in (("Moving obstacle", mo_rows), ("Sensor discovery", sd_rows)):
        print(f"\n{label} ({TRIALS_PER_SIZE} trials/size):")
        for size in SIZES:
            a_avg = average(rows, "astar_total_ms", size)
            d_avg = average(rows, "dstar_total_ms", size)
            print(f"  {size:4d}x{size:<4d}: astar {a_avg:9.3f}ms | D* Lite {d_avg:9.3f}ms | "
                  f"speedup {a_avg / d_avg:.2f}x")
