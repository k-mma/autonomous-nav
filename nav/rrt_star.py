import math
import random

from nav.config import RRT_MAX_ITERS, RRT_STEP_SIZE, RRT_GOAL_SAMPLE_RATE, RRT_GOAL_RADIUS
from nav.kdtree import KDTree
from nav.rrt import _sample_free_cell, _steer, _clear_line

# RRT*'s rewiring radius, textbook version, shrinks with tree size:
# gamma * (log(n) / n) ** (1/d). This project uses a much simpler fixed
# radius -- a constant multiple of step_size -- instead. That's a common
# practical simplification (plenty of teaching/reference RRT*
# implementations do the same) at the cost of not being the
# asymptotically tightest possible radius; see WRITEUPS.md for why that
# tradeoff was made here.
RRT_STAR_NEIGHBOR_FACTOR = 2.0


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _is_ancestor(came_from, start, node, candidate):
    """True if `candidate` appears in `node`'s parent chain back to
    `start`. Guards the rewire step below against creating a cycle:
    rewiring a node to point to one of its own descendants would turn
    the tree into a loop that `reconstruct` would walk forever."""
    current = node
    while current != start:
        current = came_from[current]
        if current == candidate:
            return True
    return False


def rrt_star(grid, start, goal, max_iters=RRT_MAX_ITERS, step_size=RRT_STEP_SIZE,
             goal_sample_rate=RRT_GOAL_SAMPLE_RATE, goal_radius=RRT_GOAL_RADIUS,
             neighbor_radius=None, rng=None):
    """
    RRT*: plain RRT (nav/rrt.py) plus two changes that give it an
    asymptotic-optimality guarantee plain RRT doesn't have --

    1. **Choose the cheapest parent, not just the nearest one.** Plain RRT
       always connects a new node to its single nearest existing
       neighbor. RRT* instead looks at every existing node within
       `neighbor_radius` and connects to whichever one gives the lowest
       total cost-to-come (`cost[parent] + distance(parent, new_node)`),
       as long as the straight edge between them is collision-free.
    2. **Rewire nearby nodes through the new one.** After adding the new
       node, RRT* checks every one of those same nearby nodes again: if
       reaching it *through* the node just added would now be cheaper
       than its current parent, its parent is switched. This is the step
       plain RRT has no equivalent of at all, and it's what lets earlier,
       locally-suboptimal connections get corrected later as the tree
       fills in -- the mechanism behind "asymptotically optimal" (path
       quality keeps improving as iterations continue, in the limit
       converging to the true shortest path).

    Unlike `rrt()`, this does *not* stop as soon as a node reaches
    `goal_radius` -- it keeps iterating for the full `max_iters` budget,
    since rewiring after that point can still improve the path to a
    goal-radius node found early on. The best goal-radius node found by
    the end (lowest recorded cost) is the one actually connected to
    `goal`.

    Known simplification: rewiring a node's parent does *not* cascade the
    cost update to that node's own descendants (a full implementation
    would). A rewired node's cost is always still correct for *that*
    node, and its descendants' costs are always still valid (real edges,
    real distances) -- they just may not reflect a downstream improvement
    until *they* happen to get rewired directly. This trades a small
    amount of solution quality for a lot of implementation simplicity;
    see WRITEUPS.md for the measured effect (still meaningfully shorter
    paths than plain RRT despite skipping this).

    Returns (path, tree_nodes, came_from) -- same shape as
    dijkstra/astar/rrt.
    """
    rng = rng or random.Random()
    neighbor_radius = neighbor_radius if neighbor_radius is not None else step_size * RRT_STAR_NEIGHBOR_FACTOR

    came_from = {}
    cost = {start: 0.0}
    nodes = [start]
    tree = KDTree()
    tree.insert(start)
    goal_radius_nodes = set()

    for _ in range(max_iters):
        sample = goal if rng.random() < goal_sample_rate else _sample_free_cell(grid, rng)
        nearest = tree.nearest(sample)
        new_node = _steer(nearest, sample, step_size)

        if new_node == nearest or new_node in cost:
            continue
        if not _clear_line(grid, nearest, new_node):
            continue

        neighbors = [n for n in tree.within_radius(new_node, neighbor_radius) if n in cost]
        if nearest not in neighbors:
            neighbors.append(nearest)

        best_parent, best_cost = nearest, cost[nearest] + _dist(nearest, new_node)
        for n in neighbors:
            if n == nearest or not _clear_line(grid, n, new_node):
                continue
            c = cost[n] + _dist(n, new_node)
            if c < best_cost:
                best_parent, best_cost = n, c

        came_from[new_node] = best_parent
        cost[new_node] = best_cost
        nodes.append(new_node)
        tree.insert(new_node)

        for n in neighbors:
            if n == best_parent or n == start:
                continue
            if _is_ancestor(came_from, start, new_node, n):
                continue
            if not _clear_line(grid, new_node, n):
                continue
            c = best_cost + _dist(new_node, n)
            if c < cost[n]:
                came_from[n] = new_node
                cost[n] = c

        if _dist(new_node, goal) <= goal_radius:
            goal_radius_nodes.add(new_node)

    if goal not in cost:
        if not goal_radius_nodes:
            return None, set(nodes), came_from
        best_goal_parent = min(goal_radius_nodes, key=lambda n: cost[n])
        came_from[goal] = best_goal_parent
        cost[goal] = cost[best_goal_parent] + _dist(best_goal_parent, goal)
        nodes.append(goal)

    # Local import: nav.algorithms imports this module lazily too, to
    # avoid a circular import (same reason nav/rrt.py does this).
    from nav.algorithms import reconstruct
    return reconstruct(came_from, start, goal), set(nodes), came_from
