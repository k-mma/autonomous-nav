"""
Conflict-Based Search (Sharon, Stern, Felner & Sturtevant, 2012) for
multi-agent pathfinding: plan every agent independently first, then
repair conflicts pairwise instead of ever searching the joint state
space of all agents at once (which is exponential in the number of
agents -- a joint search over N robots on this project's 25x25 grid
would need a state space of roughly (625 cells)^N, intractable past two
or three agents).

Why the existing two-robot coordination doesn't generalize
-------------------------------------------------------------
pybullet_multi_robot_main.py's NavAgent policy has each of exactly two
robots treat the *other's current cell* as a temporary obstacle and
replan around it, symmetrically, every REPLAN_PERIOD_S. That works
specifically because there are only two robots: each one only ever has
one "other" to react to. With three or more, "treat everyone else's
current cell as an obstacle" doesn't have a clean symmetric extension --
whose cell does agent A treat as blocked when B and C are both nearby
and might each move differently depending on what A does? Real conflicts
between 3+ agents are inherently *joint*: resolving one pair's conflict
can create a new conflict with a third agent, requiring the search to
reason about combinations of agents' plans, not just pairs reacting to
each other's current position.

The CBS idea
------------
1. **High level**: search over a tree of constraint sets. Each node
   holds one set of constraints per agent (a constraint says "agent X
   cannot be at cell C at time T", or "agent X cannot move from cell C1
   to cell C2 at time T") plus a concrete solution -- one path per agent,
   computed by an independent single-agent search subject to that node's
   constraints for that agent alone.
2. The root node has no constraints; every agent just plans its own
   shortest path, completely ignoring every other agent.
3. **Validate**: check the current set of paths for the first conflict
   (two agents at the same cell at the same time, or two agents swapping
   cells between consecutive time steps). If there isn't one, this node
   *is* a valid, conflict-free joint solution -- done.
4. **Branch**: if there's a conflict between agents A and B at time T,
   create two child nodes -- one adds a constraint forbidding *A* from
   the conflicting cell/edge at time T, the other forbids *B* instead.
   Only the newly-constrained agent needs to replan (a single-agent
   search, not a joint one); every other agent's path is reused unchanged
   from the parent. Both children go on a priority queue ordered by
   total solution cost (sum of every agent's path length).
5. Repeat: pop the cheapest node, validate, branch, until a conflict-free
   node is found.

This scales to arbitrary agent counts because the *low-level* search
(one agent, subject to a handful of constraints) never gets more
expensive as the agent count grows -- only the number of high-level tree
nodes explored does, and in practice that stays small unless agents are
in heavy, repeated contention (see WRITEUPS.md for how this project's
intersection scenario behaves with 3 and 4 agents).

Time-expanded low-level search
-------------------------------
Unlike dijkstra/astar/rrt elsewhere in this project, CBS's low-level
search state is *(cell, time)*, not just *cell* -- a constraint like
"can't be at (12, 12) at time 7" only makes sense if time is part of the
state. Every timestep, an agent either moves to a grid-neighbor or
explicitly waits in place (staying is itself a distinct action with its
own cost) -- waiting is what lets one agent yield to another instead of
the two being forced into head-on contact.
"""
import heapq
import itertools

from nav.heuristics import manhattan

INF = float("inf")


def _neighbor_cells(grid, cell):
    return [c for c, _ in grid.get_neighbors(*cell)]


def _reconstruct(came_from, start_state, goal_state):
    path = []
    state = goal_state
    while state != start_state:
        path.append(state[0])
        state = came_from[state]
    path.append(start_state[0])
    path.reverse()
    return path


def low_level_search(grid, start, goal, vertex_constraints, edge_constraints, max_time):
    """
    Time-expanded A* for a single agent: state = (cell, t). Actions are
    every neighbor cell (cost 1) plus waiting in place (also cost 1).

    `vertex_constraints`: a set of (cell, t) this agent may not occupy.
    `edge_constraints`: a set of (from_cell, to_cell, t) this agent may
    not use to move from `from_cell` at time `t` to `to_cell` at t+1.
    `max_time`: a horizon cap, both a safety valve and what makes the
    goal-acceptance rule below well-defined.

    An agent can only safely settle at `goal` forever once no constraint
    could still force it to move away later -- so `goal` is only
    accepted at a time *after* the latest time referenced by any of this
    agent's own constraints. (A tighter version would check per-cell
    constraints only; this conservative version -- the latest time
    across *all* this agent's constraints -- is simpler and only costs a
    handful of extra wait steps in practice, not correctness.)

    Returns (path, cost) or (None, None) if unreachable within max_time.
    """
    referenced_times = [t for _, t in vertex_constraints] + [t for _, _, t in edge_constraints]
    must_survive_until = max(referenced_times, default=-1)

    start_state = (start, 0)
    g_score = {start_state: 0}
    came_from = {}
    open_heap = [(manhattan(start, goal), 0, start, 0)]
    closed = set()

    while open_heap:
        _, g, cell, t = heapq.heappop(open_heap)
        state = (cell, t)
        if state in closed:
            continue
        closed.add(state)

        if cell == goal and t > must_survive_until:
            return _reconstruct(came_from, start_state, state), g

        if t >= max_time:
            continue

        for nxt in _neighbor_cells(grid, cell) + [cell]:
            nt = t + 1
            if (nxt, nt) in vertex_constraints:
                continue
            if (cell, nxt, t) in edge_constraints:
                continue
            ng = g + 1
            nstate = (nxt, nt)
            if ng < g_score.get(nstate, INF):
                g_score[nstate] = ng
                came_from[nstate] = state
                heapq.heappush(open_heap, (ng + manhattan(nxt, goal), ng, nxt, nt))

    return None, None


def _cell_at(path, t):
    """Where the agent is at time `t`, treating it as parked at its last
    cell forever after its path ends (matches how an "arrived" robot
    behaves in pybullet_multi_robot_main.py -- once done, it just sits,
    and other agents still need to not run into it)."""
    return path[t] if t < len(path) else path[-1]


def first_conflict(paths):
    """The earliest-time conflict across every pair of agents in `paths`
    (a dict agent_id -> path), or None if the joint solution has none.
    Two kinds:

    - `("vertex", i, j, cell, t)`: agents i and j both occupy `cell` at
      time `t`.
    - `("edge", i, j, cell_i, cell_j, t)`: i moves cell_i -> cell_j while
      j moves cell_j -> cell_i between t and t+1 -- a head-on swap that a
      vertex check alone would miss entirely (neither agent is ever at
      the *same* cell at the *same* time, they just pass through each
      other mid-edge)."""
    agent_ids = list(paths.keys())
    horizon = max(len(p) for p in paths.values())

    for t in range(horizon):
        occupied = {}
        for a in agent_ids:
            cell = _cell_at(paths[a], t)
            if cell in occupied:
                return ("vertex", occupied[cell], a, cell, t)
            occupied[cell] = a

        for idx_i in range(len(agent_ids)):
            for idx_j in range(idx_i + 1, len(agent_ids)):
                i, j = agent_ids[idx_i], agent_ids[idx_j]
                ci_t, ci_t1 = _cell_at(paths[i], t), _cell_at(paths[i], t + 1)
                cj_t, cj_t1 = _cell_at(paths[j], t), _cell_at(paths[j], t + 1)
                if ci_t1 != ci_t and ci_t == cj_t1 and ci_t1 == cj_t:
                    return ("edge", i, j, ci_t, ci_t1, t)

    return None


def solution_cost(paths):
    return sum(len(p) - 1 for p in paths.values())


def cbs(grid, agents, max_expansions=5000, max_time_horizon=None):
    """
    `agents`: a dict (or anything dict()-able) of agent_id -> (start,
    goal) -- any number of agents, each with independent cells.

    Returns a dict agent_id -> path (list of cells, index = timestep,
    all agents implicitly synchronized to the same global clock -- see
    pybullet_cbs_main.py for why driving them has to preserve that), or
    None if no conflict-free solution was found within `max_expansions`
    constraint-tree nodes (either genuinely unsolvable, or the budget was
    too small -- this doesn't distinguish the two, same as RRT's
    max_iters).
    """
    agent_ids = list(agents.keys())
    max_time_horizon = max_time_horizon or (4 * len(grid.cells) + 40)

    empty = (frozenset(), frozenset())
    root_constraints = {a: empty for a in agent_ids}
    root_paths = {}
    for a in agent_ids:
        start, goal = agents[a]
        path, _ = low_level_search(grid, start, goal, frozenset(), frozenset(), max_time_horizon)
        if path is None:
            return None
        root_paths[a] = path

    counter = itertools.count()
    open_heap = [(solution_cost(root_paths), next(counter), root_constraints, root_paths)]

    for _ in range(max_expansions):
        if not open_heap:
            return None
        _, _, constraints, paths = heapq.heappop(open_heap)

        conflict = first_conflict(paths)
        if conflict is None:
            return paths

        if conflict[0] == "vertex":
            _, i, j, cell, t = conflict
            branch_constraints = {i: ("vertex", cell, t), j: ("vertex", cell, t)}
        else:
            _, i, j, ci, cj, t = conflict
            branch_constraints = {i: ("edge", ci, cj, t), j: ("edge", cj, ci, t)}

        for agent, new_constraint in branch_constraints.items():
            v_constraints, e_constraints = constraints[agent]
            if new_constraint[0] == "vertex":
                _, cell, t = new_constraint
                v_constraints = v_constraints | {(cell, t)}
            else:
                _, frm, to, t = new_constraint
                e_constraints = e_constraints | {(frm, to, t)}

            start, goal = agents[agent]
            new_path, _ = low_level_search(grid, start, goal, v_constraints, e_constraints, max_time_horizon)
            if new_path is None:
                continue  # this branch is infeasible -- prune it, don't queue it

            new_constraints = dict(constraints)
            new_constraints[agent] = (v_constraints, e_constraints)
            new_paths = dict(paths)
            new_paths[agent] = new_path

            heapq.heappush(open_heap, (solution_cost(new_paths), next(counter), new_constraints, new_paths))

    return None
