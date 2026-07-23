"""
D* Lite (Koenig & Likhachev, 2002): an incremental replanner. Every other
planner in this project (dijkstra/astar/rrt/rrt_star, all in
nav/algorithms.py, nav/rrt.py, nav/rrt_star.py) solves a path from
scratch, every single time it's called -- which is exactly what
nav/obstacles.py's moving-obstacle replanning and nav/sensor.py's
discovery-triggered replanning both do, over and over, on a grid that's
barely changed since the last call. D* Lite instead keeps a single
persistent search around and *repairs* it: when a handful of cells change
(an obstacle appears/disappears, a lidar scan reveals new obstacles, the
robot moves one cell), only the vertices whose optimal cost could
actually have been affected get re-examined -- not the whole grid.

How it stays correct while only touching what changed
-------------------------------------------------------
The search runs *backwards*, from `goal` outward, and tracks two costs
per cell instead of one:

- `g(s)`: the best cost-to-goal found so far for cell `s` (like
  Dijkstra's `dist`, just measured *to* the goal instead of *from* the
  start).
- `rhs(s)`: a one-step lookahead -- `min` over `s`'s neighbors `v` of
  `edge_cost(s, v) + g(v)`. `rhs(goal) = 0` always.

A cell is "consistent" when `g(s) == rhs(s)` -- its cost-to-goal is
locally correct given its neighbors' costs. When an edge cost changes,
only the cells whose `rhs` depends on that edge go *inconsistent*
(`g != rhs`), and only inconsistent cells ever get pushed onto the open
queue for re-examination. Everywhere else in the grid, the previously
computed `g` values are still exactly as valid as they were before --
nothing has to be recomputed just because *something, somewhere* on the
grid changed.

Because the search is anchored at the (fixed) goal rather than the
(moving) start, the robot advancing one cell doesn't invalidate any of
this -- it only shifts the priority-queue ordering slightly, corrected
by a single scalar (`km`, "key modifier": the heuristic distance the
robot has moved since its last query) instead of requiring a fresh
search. This is the actual mechanism behind "incremental": not a vague
promise of being faster, but a specific proof that a bounded change to
the graph only invalidates a bounded, generally small, region of the
previous search.

Since grid adjacency here is symmetric (if `v` is a neighbor of `u`, `u`
is a neighbor of `v` -- true even for the diagonal corner-cut rule in
`Grid.get_neighbors`, which checks the same two corner cells regardless
of which direction you're moving between `u` and `v`), a cell's
predecessors are exactly the same *set* of cells as its successors --
just interpreted with the edge cost in the other direction. That's what
lets `_neighbors` below double as both without needing a separate
"reverse graph."

See WRITEUPS.md for the full derivation and nav/replan_benchmark.py for
the actual measured speed difference against re-running A* from scratch
on the two scenarios this project already replans repeatedly on:
nav/obstacles.py's bouncing MovingObstacle, and nav/sensor.py's
lidar-discovery replanning.
"""
import heapq

from nav.heuristics import manhattan, octile

INF = float("inf")


def _neighbor_cells(grid, cell):
    """Just the neighbor cells of `cell`, ignoring cost -- both the
    successor set (used to compute rhs) and the predecessor set (used to
    decide what to re-examine after a cost changes), since grid adjacency
    is symmetric. See the module docstring for why that's true even with
    diagonal movement on."""
    return [c for c, _ in grid.get_neighbors(*cell)]


class DStarLite:
    """
    A persistent incremental planner for one (grid, goal) pair. Construct
    once; call `move_start`/`update_edge_costs` as the world changes, and
    `extract_path` whenever the caller wants the current best route.
    `goal` is fixed for the lifetime of an instance -- if the goal itself
    needs to move, build a new DStarLite (this project's use cases, an
    obstacle moving or a sensor revealing more of the map, never move the
    goal, only the start and the obstacle layout).
    """

    def __init__(self, grid, start, goal, heuristic=None):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.heuristic = heuristic or (octile if grid.diagonal else manhattan)
        self.g = {}
        self.rhs = {goal: 0.0}
        self.km = 0.0
        self.queue = []
        # cell -> the key it was last pushed with. A queue entry is only
        # "live" if it matches this -- anything else popped off the heap
        # is a stale duplicate from before a rewrite and gets discarded
        # instead of processed (the standard lazy-deletion trick for a
        # binary heap that doesn't support decrease-key directly).
        self.entry = {}
        self._push(goal)
        self.compute_shortest_path()

    # -- key/queue bookkeeping --

    def _key(self, cell):
        g = self.g.get(cell, INF)
        rhs = self.rhs.get(cell, INF)
        m = min(g, rhs)
        if m == INF:
            return (INF, INF)
        return (m + self.heuristic(self.start, cell) + self.km, m)

    def _push(self, cell):
        key = self._key(cell)
        self.entry[cell] = key
        heapq.heappush(self.queue, (key, cell))

    def _pop_top(self):
        while self.queue:
            key, cell = heapq.heappop(self.queue)
            if self.entry.get(cell) == key:
                del self.entry[cell]
                return key, cell
        return None

    def _top_key(self):
        while self.queue:
            key, cell = self.queue[0]
            if self.entry.get(cell) == key:
                return key
            heapq.heappop(self.queue)
        return (INF, INF)

    # -- core algorithm --

    def update_vertex(self, u):
        """Recompute rhs(u) from its current neighbors and re-queue it if
        it's inconsistent (g != rhs) -- or drop it from the queue if it
        just became consistent. Called on exactly the cells that could
        have been affected by whatever just changed: a popped cell's
        neighbors during compute_shortest_path, or the changed cell (and
        its neighbors) after an edge-cost change."""
        if u != self.goal:
            best = INF
            for v, cost in self.grid.get_neighbors(*u):
                c = self.g.get(v, INF) + cost
                if c < best:
                    best = c
            self.rhs[u] = best
        if u in self.entry:
            del self.entry[u]
        if self.g.get(u, INF) != self.rhs.get(u, INF):
            self._push(u)

    def compute_shortest_path(self):
        """Process the open queue until the start is consistent and
        nothing left in the queue could possibly produce a cheaper key
        for it -- i.e. until the search has converged as far out as it
        needs to for `start` to have a final answer. Cells far from
        `start` that were never inconsistent are never touched at all."""
        while self.queue and (
            self._top_key() < self._key(self.start)
            or self.rhs.get(self.start, INF) != self.g.get(self.start, INF)
        ):
            k_old, u = self._pop_top()
            k_new = self._key(u)
            if k_old < k_new:
                # u's key increased since it was queued (some cost went
                # up) -- requeue with the corrected key instead of
                # processing it as if the old, now-wrong key still held.
                self.entry[u] = k_new
                heapq.heappush(self.queue, (k_new, u))
                continue

            if self.g.get(u, INF) > self.rhs.get(u, INF):
                # Overconsistent -> consistent: u's cost improved.
                # Everything that might route through u needs a look.
                self.g[u] = self.rhs[u]
                for p in _neighbor_cells(self.grid, u):
                    self.update_vertex(p)
            else:
                # Underconsistent: u's old cost was wrong (too good --
                # something it depended on got worse or disappeared).
                # Invalidate it and let it, and everything that might
                # route through it, get recomputed from its neighbors.
                self.g[u] = INF
                self.update_vertex(u)
                for p in _neighbor_cells(self.grid, u):
                    self.update_vertex(p)

    # -- world-change hooks --

    def move_start(self, new_start):
        """Call when the robot actually moves to `new_start`. Folding the
        heuristic distance traveled into `km` is what lets every
        previously computed g-value stay valid without being recomputed
        relative to the new start -- the search doesn't need to know
        `start` moved at all except to correct its own priority
        ordering."""
        if new_start == self.start:
            return
        self.km += self.heuristic(self.start, new_start)
        self.start = new_start

    def update_edge_costs(self, changed_cells):
        """Call after `changed_cells` (an iterable of (row, col)) have had
        their obstacle state or terrain cost change in `self.grid`.
        Works uniformly for both: an obstacle appearing/disappearing
        changes which edges *exist*; a cost-map change changes what they
        *cost* -- either way, it's exactly the changed cells and their
        neighbors whose rhs could now be wrong. Does not itself
        recompute the path -- call `compute_shortest_path` after (and
        `move_start` first, if the robot moved too)."""
        touched = set()
        for cell in changed_cells:
            touched.add(cell)
            touched.update(_neighbor_cells(self.grid, cell))
        for u in touched:
            self.update_vertex(u)

    # -- reading out the answer --

    def extract_path(self, max_steps=None):
        """Greedily follow the cheapest neighbor from `start` toward
        `goal` -- D* Lite's equivalent of `nav.algorithms.reconstruct`.
        Once the search has converged, g(s) is the true cost-to-goal for
        every consistent cell, so always stepping to whichever neighbor
        minimizes `edge_cost + g(neighbor)` traces out a shortest path
        with no explicit parent map needed. Returns None if `start` has
        no known route to `goal`."""
        if self.g.get(self.start, INF) == INF:
            return None

        path = [self.start]
        current = self.start
        limit = max_steps or (len(self.grid.cells) ** 2 + 10)
        for _ in range(limit):
            if current == self.goal:
                return path
            best_cell, best_cost = None, INF
            for v, cost in self.grid.get_neighbors(*current):
                c = cost + self.g.get(v, INF)
                if c < best_cost:
                    best_cell, best_cost = v, c
            if best_cell is None:
                return None
            current = best_cell
            path.append(current)
        return None


def dstar_lite(grid, start, goal, heuristic=None):
    """One-shot convenience wrapper matching dijkstra/astar/rrt's
    (path, explored, came_from) signature, for uniform testing and for
    plugging into nav.algorithms.find_path as "dstar_lite". This throws
    away D* Lite's entire reason for existing -- a fresh DStarLite object
    here does a full search from nothing, same as any other planner would
    -- so it's useful for correctness testing and as a one-shot baseline,
    but the actual point (repairing a *persistent* instance instead of
    rebuilding) only shows up by holding onto the object across multiple
    calls, which is what nav/replan_benchmark.py actually demonstrates."""
    planner = DStarLite(grid, start, goal, heuristic=heuristic)
    path = planner.extract_path()
    explored = set(planner.g) | set(planner.rhs)
    return path, explored, {}
