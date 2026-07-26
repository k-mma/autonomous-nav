import heapq
import math

from nav.grid import DIAGONAL_COST
from nav.heuristics import manhattan, octile


def dijkstra(grid, start, goal, order_out=None):
    """
    Shortest path from start to goal by cost, expanding the cheapest known
    cell first. Guaranteed optimal for any grid.get_neighbors() step costs.

    `order_out`, if given a list, gets each cell appended to it in the
    exact order it's settled -- a plain `set` (what's returned as
    `settled`) doesn't preserve that order, and nav/visualizer.py's
    step-by-step replay mode (Step 6) needs it to reveal cells one at a
    time in the order they were actually explored. Optional and additive
    so every existing caller that doesn't pass it sees no change at all.

    Returns (path, settled, came_from). path is None if goal is unreachable.
    settled is every cell the search finished expanding (for visualizing
    and comparing search effort against astar).
    """
    dist = {start: 0}
    came_from = {}
    settled = set()
    # pq: cost, row, col
    pq = [(0, start[0], start[1])]

    while pq:
        cost, row, col = heapq.heappop(pq)
        current = (row, col)

        # Ignore cell already explored with a lower cost
        if current in settled:
            continue
        settled.add(current)
        if order_out is not None:
            order_out.append(current)

        if current == goal:
            return reconstruct(came_from, start, goal), settled, came_from

        for neighbor, step_cost in grid.get_neighbors(row, col):
            new_cost = cost + step_cost

            if new_cost < dist.get(neighbor, float('inf')):
                dist[neighbor] = new_cost
                came_from[neighbor] = current
                heapq.heappush(pq, (new_cost, neighbor[0], neighbor[1]))

    # Return if no path found (queue is exhausted)
    return None, settled, came_from


def astar(grid, start, goal, heuristic=None, order_out=None):
    """
    Like dijkstra, but expands the cell with the lowest f = g + h first,
    where g is the cost so far and h = heuristic(cell, goal) estimates the
    remaining cost. Only guaranteed optimal if `heuristic` never
    overestimates the true remaining cost (see nav/heuristics.py) -- an
    inadmissible heuristic can make it settle for a longer path (see
    WRITEUPS.md).

    `heuristic` defaults to Manhattan distance on a 4-directional grid and
    octile distance once `grid.diagonal` is on -- Manhattan is inadmissible
    the moment diagonal movement is allowed (see WRITEUPS.md's "Diagonal
    movement and why Manhattan breaks"), so the default has to track
    `grid.diagonal` the same way nav/dstar_lite.py's does.

    `order_out` -- see dijkstra's docstring; same optional replay hook.

    Returns (path, settled, came_from). path is None if goal is unreachable.
    """
    heuristic = heuristic or (octile if grid.diagonal else manhattan)
    g_score = {start: 0}
    came_from = {}
    settled = set()
    # pq: (f, g, row, col)
    # f = g + h (heuristic estimate to goal)
    # g = cost so far
    pq = [(heuristic(start, goal), 0, start[0], start[1])]

    while pq:
        f, g, row, col = heapq.heappop(pq)
        current = (row, col)

        # Ignore cell already explored with a lower cost
        if current in settled:
            continue
        settled.add(current)
        if order_out is not None:
            order_out.append(current)

        if current == goal:
            return reconstruct(came_from, start, goal), settled, came_from

        for neighbor, step_cost in grid.get_neighbors(row, col):
            new_g = g + step_cost

            if new_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = new_g
                new_f = new_g + heuristic(neighbor, goal)
                came_from[neighbor] = current
                heapq.heappush(pq, (new_f, new_g, neighbor[0], neighbor[1]))

    # Return if no path found (queue is exhausted)
    return None, settled, came_from


def reconstruct(came_from, start, goal):
    """Walk came_from backwards from goal to start and return start->goal order."""
    path = []
    cell = goal
    while cell != start:
        path.append(cell)
        cell = came_from[cell]
    path.append(start)
    path.reverse()
    return path


def path_cost(grid, path):
    """Total movement cost along path, by the exact same measure
    get_neighbors uses to compute it (this just re-derives it per edge
    instead of re-deriving the whole neighbor list): sqrt(2) for a
    diagonal step else 1, scaled by the terrain cost of the cell being
    entered (see Grid.cost / compute_cost_map -- 1.0 everywhere when
    cost-map mode is off, so this collapses to the old diagonal-only
    cost in that case), plus an elevation surcharge for climbing (see
    Grid._elevation_cost -- 0 everywhere unless elevation_aware is on)."""
    cost = 0
    for (r1, c1), (r2, c2) in zip(path, path[1:]):
        step = DIAGONAL_COST if r1 != r2 and c1 != c2 else 1
        cost += step * grid.cost[r2][c2] + grid._elevation_cost(r1, c1, r2, c2)
    return cost


def weighted_path_length(grid, path):
    """Like path_cost, generalized to paths whose steps aren't always
    exactly 1 or sqrt(2) cells long -- RRT's waypoints can be up to
    RRT_STEP_SIZE cells apart, so path_cost's "diagonal or 1" step-cost
    assumption doesn't hold for it. Uses the real Euclidean length of
    each edge instead, scaled by the terrain cost of the cell being
    entered, same as path_cost. For a grid-adjacent path (every
    Dijkstra/A* path) this gives the exact same number path_cost does,
    since hypot(1, 0) == 1 and hypot(1, 1) == sqrt(2) -- so
    nav/visualizer.py's 3-panel comparison (Step 5) can use this one
    function for all three algorithms instead of branching per algorithm.
    """
    total = 0.0
    for (r1, c1), (r2, c2) in zip(path, path[1:]):
        dist = math.hypot(r2 - r1, c2 - c1)
        total += dist * grid.cost[r2][c2] + grid._elevation_cost(r1, c1, r2, c2)
    return total


def validate_endpoints(grid, start, goal):
    """Reason a search from start to goal can't even be attempted, or None if it's fine."""
    if start == goal:
        return "same_cell"
    if grid.is_obstacle(*start):
        return "start_blocked"
    if grid.is_obstacle(*goal):
        return "goal_blocked"
    return None


def find_path(grid, algo_name, start, goal, heuristic=None, order_out=None):
    """
    Run dijkstra/astar/rrt/rrt_star from start to goal, handling the edge
    cases the raw algorithms don't check for: same start/goal cell, and
    either endpoint sitting on an obstacle. `heuristic` is only used when
    algo_name is "astar"; see astar()'s docstring for its default (Manhattan,
    or octile once `grid.diagonal` is on). `order_out`
    -- see dijkstra's docstring -- is only honored for "dijkstra", "astar",
    and "rrt" (the three nav/visualizer.py actually replays step-by-step);
    passed through unused otherwise.

    Returns (path, explored, reason, came_from). reason is None on an
    ordinary run, "no_path" if the search exhausted its budget without
    reaching goal, and "same_cell" / "start_blocked" / "goal_blocked" if
    the endpoints stopped the search from running at all. came_from is the
    raw parent map each algorithm builds -- the visualizer uses it to draw
    RRT's tree edges.
    """
    reason = validate_endpoints(grid, start, goal)
    if reason == "same_cell":
        return [start], set(), reason, {}
    if reason is not None:
        return None, set(), reason, {}

    if algo_name == "astar":
        path, explored, came_from = astar(grid, start, goal, heuristic=heuristic,
                                           order_out=order_out)
    elif algo_name == "rrt":
        # Imported here, not at module level, because nav.rrt imports
        # `reconstruct` from this module -- a top-level import here would
        # be circular.
        from nav.rrt import rrt
        path, explored, came_from = rrt(grid, start, goal, order_out=order_out)
    elif algo_name == "rrt_star":
        # Same circular-import reasoning as "rrt" above.
        from nav.rrt_star import rrt_star
        path, explored, came_from = rrt_star(grid, start, goal)
    elif algo_name == "dstar_lite":
        # Same circular-import reasoning as "rrt" above. Note this is the
        # one-shot wrapper -- see nav/dstar_lite.py's docstring for why a
        # single find_path call can't demonstrate D* Lite's actual point
        # (incremental repair across multiple calls on the same object).
        from nav.dstar_lite import dstar_lite
        path, explored, came_from = dstar_lite(grid, start, goal)
    else:
        path, explored, came_from = dijkstra(grid, start, goal, order_out=order_out)

    if path is None:
        return None, explored, "no_path", came_from
    return path, explored, None, came_from