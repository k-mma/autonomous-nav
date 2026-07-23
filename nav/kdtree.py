"""
A minimal 2D k-d tree over (row, col) points, built incrementally one
point at a time -- exactly how nav/rrt.py grows its tree, one new node
per iteration. Supports the two queries RRT/RRT* actually need:

- `nearest(point)`: the single closest existing point (RRT's per-iteration
  "which tree node do I steer from").
- `within_radius(point, radius)`: every existing point within `radius`
  (RRT*'s per-iteration "which nearby nodes could I rewire").

Both run in roughly O(log n) for a reasonably balanced tree, replacing
nav.rrt's original O(n) linear scan over every node -- see
benchmark_results/scale_writeup.md for why that scan was identified as
RRT's actual bottleneck at scale, and WRITEUPS.md for the before/after
numbers once this replaced it.

No rebalancing. A k-d tree built by naive recursive insertion can
degrade toward a linked list under an adversarial insertion order (e.g.
points fed in sorted order). RRT's insertion order is nowhere near
adversarial -- each new node is steered toward a uniformly random sample
-- so this stays close enough to balanced in practice without needing the
maintenance a general-purpose incremental k-d tree would. Worth stating
plainly rather than leaving implicit: this is a real limitation of the
data structure, just not one this project's one caller (RRT) actually
triggers.
"""


class _Node:
    __slots__ = ("point", "left", "right")

    def __init__(self, point):
        self.point = point
        self.left = None
        self.right = None


def _dist2(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


class KDTree:
    def __init__(self):
        self.root = None
        self.size = 0

    def insert(self, point):
        self.size += 1
        if self.root is None:
            self.root = _Node(point)
            return

        node = self.root
        depth = 0
        while True:
            axis = depth % 2
            if point[axis] < node.point[axis]:
                if node.left is None:
                    node.left = _Node(point)
                    return
                node = node.left
            else:
                if node.right is None:
                    node.right = _Node(point)
                    return
                node = node.right
            depth += 1

    def nearest(self, target):
        """The single closest inserted point to `target`, or None if the
        tree is empty. Standard k-d tree nearest-neighbor: descend toward
        the half-space containing `target`, then only backtrack into the
        far half-space if it could possibly hold something closer than
        the best found so far (the axis-aligned distance to the splitting
        plane is a lower bound on anything on the far side)."""
        if self.root is None:
            return None

        best_point = [None]
        best_dist2 = [float("inf")]

        def visit(node, depth):
            if node is None:
                return
            d2 = _dist2(node.point, target)
            if d2 < best_dist2[0]:
                best_point[0] = node.point
                best_dist2[0] = d2

            axis = depth % 2
            diff = target[axis] - node.point[axis]
            near, far = (node.left, node.right) if diff < 0 else (node.right, node.left)
            visit(near, depth + 1)
            # Only worth checking the far side if the splitting plane
            # itself is closer than the best distance found so far.
            if diff * diff < best_dist2[0]:
                visit(far, depth + 1)

        visit(self.root, 0)
        return best_point[0]

    def within_radius(self, target, radius):
        """Every inserted point within `radius` of `target` (inclusive),
        as a plain list. Same pruning idea as `nearest`, just collecting
        matches instead of tracking a single best."""
        results = []
        r2 = radius * radius

        def visit(node, depth):
            if node is None:
                return
            if _dist2(node.point, target) <= r2:
                results.append(node.point)

            axis = depth % 2
            diff = target[axis] - node.point[axis]
            near, far = (node.left, node.right) if diff < 0 else (node.right, node.left)
            visit(near, depth + 1)
            if diff * diff <= r2:
                visit(far, depth + 1)

        visit(self.root, 0)
        return results
