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


def heuristic(row, col):
    return abs(row - GOAL[0]) + abs(col - GOAL[1])


def astar(start, goal):
    g_score = {start: 0}
    came_from = {}
    # pq: (f, g, row, col)
    # f = g + h (heuristic estimate to goal)
    # g = cost so far
    pq = [(heuristic(start[0], start[1]), 0, start[0], start[1])]
    cells_explored = 0
    while pq:
        f, g, row, col = heapq.heappop(pq)
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
        if g > g_score.get((row, col), float('inf')):
            continue
        for neighbor in get_neighbors(row, col):
            new_g = g + 1
            if new_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = new_g
                came_from[neighbor] = (row, col)
                new_f = new_g + heuristic(neighbor[0], neighbor[1])
                heapq.heappush(pq, (new_f, new_g, neighbor[0], neighbor[1]))
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


if __name__ == "__main__":
    path, explored = astar(START, GOAL)
    if path:
        print(f"Path found! Length: {len(path)} steps, Cells explored: {explored}\n")
        print_grid(path)
        print(f"\nPath as coordinates: {path}")
    else:
        print(f"No path found. Cells explored before giving up: {explored}")