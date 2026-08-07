import heapq


GRID = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

START = (0, 0)
GOAL = (11, 7)
ROWS = len(GRID)
COLS = len(GRID[0])


def get_neighbors(row, col):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = []
    for dr, dc in directions:
        r = row + dr
        c = col + dc
        if 0 <= r < ROWS and 0 <= c < COLS and GRID[r][c] == 0:
            neighbors.append((r, c))
    return neighbors


def dijkstra(start, goal):
    dist = {start: 0}
    came_from = {}
    # pq: cost, row, col
    pq = [(0, start[0], start[1])]
    cells_explored = 0
    while pq:
        cost, row, col = heapq.heappop(pq)
        cells_explored += 1
        if (row, col) == goal:
            path = []
            cell = goal
            while cell != start:
                path.append(cell)
                cell = came_from[cell]
            path.append(start)
            path.reverse()
            return path, cells_explored
        if cost > dist.get((row, col), float('inf')):
            continue
        for neighbor in get_neighbors(row, col):
            new_cost = cost + 1
            if new_cost < dist.get(neighbor, float('inf')):
                dist[neighbor] = new_cost
                came_from[neighbor] = (row, col)
                heapq.heappush(pq, (new_cost, neighbor[0], neighbor[1]))
    return None, cells_explored


def print_grid(path):
    if path:
        path_set = set(path)
    else:
        path_set = set()
    symbols = {0: ".", 1: "#"}
    for r in range(ROWS):
        row_str = ""
        for c in range(COLS):
            if (r, c) == START:
                row_str += "S "
            elif (r, c) == GOAL:
                row_str += "G "
            elif (r, c) in path_set:
                row_str += "* "
            else:
                row_str += symbols[GRID[r][c]] + " "
        print(row_str)


# --- pytest entry points --------------------------------------------------
# Same reasoning as nav/scratch/astar_test.py: an obstacle-free grid has a
# known-optimal path length, Manhattan(START, GOAL) + 1.

def test_finds_the_optimal_path_on_an_open_grid():
    path, explored = dijkstra(START, GOAL)
    assert path is not None
    assert path[0] == START
    assert path[-1] == GOAL
    assert len(path) == 19  # Manhattan(START, GOAL) + 1
    for (r1, c1), (r2, c2) in zip(path, path[1:]):
        assert abs(r1 - r2) + abs(c1 - c2) == 1, f"non-cardinal step {(r1, c1)}->{(r2, c2)}"
    assert 0 < explored <= ROWS * COLS


if __name__ == "__main__":
    path, explored = dijkstra(START, GOAL)
    if path:
        print(f"Path found! Length: {len(path)} steps, Cells explored: {explored}\n")
        print_grid(path)
        print(f"\nPath as coordinates: {path}")
    else:
        print(f"No path found. Cells explored before giving up: {explored}")