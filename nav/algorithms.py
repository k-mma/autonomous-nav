# FUNCTIONS

# Dijkstra's

# Pseudocode
'''
for each vertex v:
    dist[v] = infinity
    prev[v] = none
dist[source] = 0
set all vertices to unexplored
while destination not explored:
    v = least-vlaued unexplored vertex
    set v to explored
    for each edge (v, w):
        if dist[v] + len(v, w) < dist[w]:
            dist[w] = dist[v] + len(v, w)
            prev[w] = v
'''
