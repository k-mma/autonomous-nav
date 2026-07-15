import math


def manhattan(a, b):
    """Correct, admissible heuristic for 4-directional movement."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a, b):
    """Admissible for both 4- and 8-directional movement, but a looser
    (less informative) bound than Manhattan on a 4-directional grid."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def chebyshev(a, b):
    """Admissible for 8-directional movement if diagonal cost == 1."""
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def octile(a, b):
    """Correct, admissible heuristic for 8-directional movement with
    diagonal steps costing sqrt(2)."""
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dr, dc) + (math.sqrt(2) - 1) * min(dr, dc)


def scaled(heuristic, factor):
    """Wrap `heuristic` so it returns factor times its normal estimate.
    factor > 1 makes it inadmissible (can overestimate the true cost)."""
    def wrapped(a, b):
        return heuristic(a, b) * factor
    return wrapped
