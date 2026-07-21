"""
A* on a 4-directional grid produces waypoints with sharp 90-degree
corners -- fine for a search algorithm, not something a real robot base
can follow without stopping to pivot at every single one. These three
functions turn that jagged waypoint list into something a differential
drive can actually track.
"""


def simplify_collinear(points):
    """Drop interior points that lie on a straight line between their
    neighbors. A* on a 4-directional grid emits one waypoint per cell, so
    a 10-cell straight run is 10 collinear points -- collapsing them to
    just the two endpoints gives the smoothers below a handful of
    meaningful corners to work with instead of dozens of redundant knots
    along every straight stretch."""
    if len(points) < 3:
        return list(points)

    simplified = [points[0]]
    for i in range(1, len(points) - 1):
        x0, y0 = simplified[-1]
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        # Cross product of (p1-p0) and (p2-p0): zero iff collinear.
        cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
        if abs(cross) > 1e-9:
            simplified.append(points[i])
    simplified.append(points[-1])
    return simplified


def chaikin_smooth(points, iterations=3):
    """Corner-cutting (Chaikin's algorithm): each pass replaces every
    corner with two points 1/4 and 3/4 of the way along its adjacent
    edges, rounding it off. Cheap and simple -- the "first" smoothing
    pass the plan calls for -- but it only approaches the original path,
    it doesn't pass through the original waypoints except the two ends
    (which are kept exact here so the robot still starts/ends in the
    right cell)."""
    pts = list(points)
    if len(pts) < 3:
        return pts

    for _ in range(iterations):
        new_pts = [pts[0]]
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            q = (0.75 * x0 + 0.25 * x1, 0.75 * y0 + 0.25 * y1)
            r = (0.25 * x0 + 0.75 * x1, 0.25 * y0 + 0.75 * y1)
            new_pts.extend([q, r])
        new_pts.append(pts[-1])
        pts = new_pts
    return pts


def _catmull_rom_point(p0, p1, p2, p3, t):
    t2, t3 = t * t, t * t * t
    x = 0.5 * (
        2 * p1[0]
        + (-p0[0] + p2[0]) * t
        + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
        + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3
    )
    y = 0.5 * (
        2 * p1[1]
        + (-p0[1] + p2[1]) * t
        + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
        + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3
    )
    return (x, y)


def catmull_rom_spline(points, samples_per_segment=8):
    """Fit a Catmull-Rom spline through `points` -- unlike Chaikin, this
    curve passes exactly through every original waypoint (not just the
    endpoints), while still arriving smoothly rather than turning on a
    dime at each one. This is the "spline fit if time allows" upgrade
    over plain corner-cutting; the visualizer drives this by default."""
    pts = list(points)
    if len(pts) < 3:
        return pts

    # Pad with a duplicated point at each end so every real waypoint gets
    # a full 4-point neighborhood to interpolate from.
    padded = [pts[0]] + pts + [pts[-1]]
    curve = []
    for i in range(1, len(padded) - 2):
        p0, p1, p2, p3 = padded[i - 1], padded[i], padded[i + 1], padded[i + 2]
        for s in range(samples_per_segment):
            t = s / samples_per_segment
            curve.append(_catmull_rom_point(p0, p1, p2, p3, t))
    curve.append(pts[-1])
    return curve
