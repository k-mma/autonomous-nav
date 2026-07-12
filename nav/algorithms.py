import heapq


def dijkstra(grid, start, goal):
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
        
        for neighbor in grid.get_neighbors(row, col):
            new_cost = cost + 1
            
            if new_cost < dist.get(neighbor, float('inf')):
                dist[neighbor] = new_cost
                came_from[neighbor] = current
                heapq.heappush(pq, (new_cost, neighbor[0], neighbor[1]))
    
    # Return if no path found (queue is exhausted)
    return None, settled, came_from


def astar(grid, start, goal):

    def heuristic(row, col):
        return abs(row - goal[0]) + abs(col - goal[1])
    
    g_score = {start: 0}
    came_from = {}
    settled = set()
    # pq: (f, g, row, col)
    # f = g + h (heuristic estimate to goal)
    # g = cost so far
    pq = [(heuristic(start[0], start[1]), 0, start[0], start[1])]

    while pq:
        f, g, row, col = heapq.heappop(pq)
        current = (row, col)
        
        # Ignore cell already explored with a lower cost
        if current in settled:
            continue
        settled.add(current)
        
        if current == goal:
            return reconstruct(came_from, start, goal), settled, came_from
        
        for neighbor in grid.get_neighbors(row, col):
            new_g = g + 1
            
            if new_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = new_g
                new_f = new_g + heuristic(neighbor[0], neighbor[1])
                came_from[neighbor] = current
                heapq.heappush(pq, (new_f, new_g, neighbor[0], neighbor[1]))
    
    # Return if no path found (queue is exhausted)
    return None, settled, came_from


def reconstruct(came_from, start, goal):
    path = []
    cell = goal
    while cell != start:
        path.append(cell)
        cell = came_from[cell]
    path.append(start)
    path.reverse()
    return path