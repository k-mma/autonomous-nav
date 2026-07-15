import heapq

from nav.grid import DIAGONAL_COST
from nav.heuristics import manhattan


def dijkstra(grid, start, goal):
    """
    Shortest path from start to goal by cost, expanding the cheapest known
    cell first. Guaranteed optimal for any grid.get_neighbors() step costs.

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


def astar(grid, start, goal, heuristic=manhattan):
    """
    Like dijkstra, but expands the cell with the lowest f = g + h first,
    where g is the cost so far and h = heuristic(cell, goal) estimates the
    remaining cost. Only guaranteed optimal if `heuristic` never
    overestimates the true remaining cost (see nav/heuristics.py) -- an
    inadmissible heuristic can make it settle for a longer path (see
    WRITEUPS.md).

    Returns (path, settled, came_from). path is None if goal is unreachable.
    """
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


def path_cost(path):
    """Total movement cost along path, counting diagonal steps as sqrt(2)."""
    cost = 0
    for (r1, c1), (r2, c2) in zip(path, path[1:]):
        cost += DIAGONAL_COST if r1 != r2 and c1 != c2 else 1
    return cost


def validate_endpoints(grid, start, goal):
    """Reason a search from start to goal can't even be attempted, or None if it's fine."""
    if start == goal:
        return "same_cell"
    if grid.is_obstacle(*start):
        return "start_blocked"
    if grid.is_obstacle(*goal):
        return "goal_blocked"
    return None


def find_path(grid, algo_name, start, goal, heuristic=None):
    """
    Run dijkstra/astar from start to goal, handling the edge cases the raw
    algorithms don't check for: same start/goal cell, and either endpoint
    sitting on an obstacle. `heuristic` is only used when algo_name is
    "astar"; it defaults to Manhattan distance.

    Returns (path, explored, reason). reason is None on an ordinary run,
    "no_path" if the search exhausted the grid without reaching goal, and
    "same_cell" / "start_blocked" / "goal_blocked" if the endpoints stopped
    the search from running at all.
    """
    reason = validate_endpoints(grid, start, goal)
    if reason == "same_cell":
        return [start], set(), reason
    if reason is not None:
        return None, set(), reason

    if algo_name == "astar":
        path, explored, _ = astar(grid, start, goal, heuristic=heuristic or manhattan)
    else:
        path, explored, _ = dijkstra(grid, start, goal)

    if path is None:
        return None, explored, "no_path"
    return path, explored, None