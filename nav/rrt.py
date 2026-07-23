import math
import random

from nav.config import RRT_MAX_ITERS, RRT_STEP_SIZE, RRT_GOAL_SAMPLE_RATE, RRT_GOAL_RADIUS
from nav.kdtree import KDTree


def _sample_free_cell(grid, rng):
    size = len(grid.cells)
    while True:
        cell = (rng.randrange(size), rng.randrange(size))
        if not grid.is_obstacle(*cell):
            return cell


def _nearest(nodes, point):
    """Linear-scan fallback, kept only for anything that still wants to
    call it directly (e.g. a scratch test comparing against the k-d
    tree). `rrt()` itself no longer uses this -- see the KDTree import
    above and WRITEUPS.md for why an unindexed O(n) scan over every tree
    node, repeated every iteration, was RRT's actual bottleneck at scale."""
    return min(nodes, key=lambda n: math.hypot(n[0] - point[0], n[1] - point[1]))


def _steer(frm, to, step_size):
    """Move from `frm` toward `to` by at most `step_size`, landing on an
    integer grid cell -- this RRT grows directly in grid-cell space rather
    than continuous space, same coordinates every other planner here uses."""
    dr, dc = to[0] - frm[0], to[1] - frm[1]
    dist = math.hypot(dr, dc)
    if dist <= step_size:
        return to
    scale = step_size / dist
    return (round(frm[0] + dr * scale), round(frm[1] + dc * scale))


def _clear_line(grid, frm, to):
    """True if every cell on the straight line from frm to to (Bresenham's
    line algorithm) is free -- keeps a tree edge from clipping through an
    obstacle sitting between two sampled points."""
    r0, c0 = frm
    r1, c1 = to
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    r, c = r0, c0
    while (r, c) != (r1, c1):
        if grid.is_obstacle(r, c):
            return False
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return not grid.is_obstacle(r1, c1)


def rrt(grid, start, goal, max_iters=RRT_MAX_ITERS, step_size=RRT_STEP_SIZE,
        goal_sample_rate=RRT_GOAL_SAMPLE_RATE, goal_radius=RRT_GOAL_RADIUS, rng=None):
    """
    Rapidly-exploring Random Tree: grow a tree from `start` by repeatedly
    sampling a random free cell, stepping from the nearest tree node toward
    it, and adding the new node if the edge doesn't cross an obstacle.
    Stops as soon as a node lands within `goal_radius` of `goal`.

    Unlike Dijkstra/A*, this doesn't search a fixed neighbor graph -- it
    grows through whatever open space it happens to sample, so it doesn't
    guarantee the shortest path (or even the same path twice). `rng` is
    exposed so callers (e.g. the benchmark) can get reproducible runs.

    Returns (path, tree_nodes, came_from) -- same shape as dijkstra/astar
    so find_path and the visualizer can treat all three identically.
    tree_nodes is every node grown (RRT's equivalent of "cells explored").
    path is None if the goal wasn't reached within max_iters.
    """
    rng = rng or random.Random()
    came_from = {}
    nodes = [start]
    tree = KDTree()
    tree.insert(start)

    for _ in range(max_iters):
        sample = goal if rng.random() < goal_sample_rate else _sample_free_cell(grid, rng)
        nearest = tree.nearest(sample)
        new_node = _steer(nearest, sample, step_size)

        if new_node == nearest or new_node == start or new_node in came_from:
            continue
        if not _clear_line(grid, nearest, new_node):
            continue

        came_from[new_node] = nearest
        nodes.append(new_node)
        tree.insert(new_node)

        if math.hypot(new_node[0] - goal[0], new_node[1] - goal[1]) <= goal_radius:
            if new_node != goal:
                came_from[goal] = new_node
                nodes.append(goal)
            # Local import: nav.algorithms imports this module lazily too,
            # to avoid a circular import between the two.
            from nav.algorithms import reconstruct
            return reconstruct(came_from, start, goal), set(nodes), came_from

    return None, set(nodes), came_from
