"""Small polygon helpers on Vec2 tuples, shared by configuration and tactical geometry."""

import math

from .model import Vec2


def rectangle(x0: float, y0: float, x1: float, y1: float) -> tuple[Vec2, ...]:
    return Vec2(x0, y0), Vec2(x1, y0), Vec2(x1, y1), Vec2(x0, y1)


def signed_area(points) -> float:
    return 0.5 * sum(a.x * b.y - b.x * a.y for a, b in zip(points, (*points[1:], points[0])))


def ccw(points) -> tuple[Vec2, ...]:
    points = tuple(points)
    return points if signed_area(points) >= 0 else tuple(reversed(points))


def area_polygon(item) -> tuple[Vec2, ...]:
    """Counter-clockwise vertices of an [x0, y0, x1, y1] rectangle or an object with .polygon."""
    polygon = getattr(item, "polygon", None)
    return ccw(polygon if polygon is not None else rectangle(*item))


def is_convex(points) -> bool:
    points = ccw(points)
    n = len(points)
    if n < 3 or abs(signed_area(points)) <= 1e-6:
        return False
    for i in range(n):
        a, b, c = points[i], points[(i + 1) % n], points[(i + 2) % n]
        if (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x) < -1e-9:
            return False
    return True


def contains(points, p: Vec2) -> bool:
    """Point in a simple polygon, boundary included."""
    inside = False
    n = len(points)
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)
        if (abs(cross) <= 1e-9 and min(a.x, b.x) - 1e-9 <= p.x <= max(a.x, b.x) + 1e-9
                and min(a.y, b.y) - 1e-9 <= p.y <= max(a.y, b.y) + 1e-9):
            return True
        if (a.y > p.y) != (b.y > p.y):
            x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y)
            if p.x < x:
                inside = not inside
    return inside


def centroid(points) -> Vec2:
    area = signed_area(points)
    if abs(area) <= 1e-12:
        return Vec2(sum(p.x for p in points) / len(points), sum(p.y for p in points) / len(points))
    cx = cy = 0.0
    for a, b in zip(points, (*points[1:], points[0])):
        cross = a.x * b.y - b.x * a.y
        cx += (a.x + b.x) * cross
        cy += (a.y + b.y) * cross
    return Vec2(cx / (6 * area), cy / (6 * area))


def mirror_point(p: Vec2, bounds) -> Vec2:
    """Point reflection through the field centre (red <-> blue for RMUC fields)."""
    x0, y0, x1, y1 = bounds
    return Vec2(x0 + x1 - p.x, y0 + y1 - p.y)


def mirror_yaw(yaw: float) -> float:
    return math.atan2(-math.sin(yaw), -math.cos(yaw))


def _meet(n1, n2, v: Vec2, radius: float) -> Vec2:
    det = n1[0] * n2[1] - n1[1] * n2[0]
    if abs(det) < 1e-12:
        return Vec2(v.x + radius * n1[0], v.y + radius * n1[1])
    return Vec2(v.x + radius * (n2[1] - n1[1]) / det, v.y + radius * (n1[0] - n2[0]) / det)


def inflate(points, radius: float):
    """Grow a convex polygon by ``radius``.

    Returns half-plane rows ``(nx, ny, c)`` (inside iff ``nx*x + ny*y <= c`` for every
    row) and the grown vertices, each with the summed unit normals of its two edges.
    Corners sharper than 90 degrees get a bevel tangent to the radius circle instead
    of a long miter spike. Rectangles keep square corners.
    """
    points = ccw(points)
    n = len(points)
    normals = []
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        length = math.hypot(b.x - a.x, b.y - a.y)
        normals.append(((b.y - a.y) / length, (a.x - b.x) / length))
    rows = [(nx, ny, nx * points[i].x + ny * points[i].y + radius)
            for i, (nx, ny) in enumerate(normals)]
    vertices = []
    for i, v in enumerate(points):
        before, after = normals[i - 1], normals[i]
        if before[0] * after[0] + before[1] * after[1] < -1e-9:
            bx, by = before[0] + after[0], before[1] + after[1]
            norm = math.hypot(bx, by)
            bevel = (bx / norm, by / norm)
            rows.append((bevel[0], bevel[1], bevel[0] * v.x + bevel[1] * v.y + radius))
            vertices.append((_meet(before, bevel, v, radius),
                             (before[0] + bevel[0], before[1] + bevel[1])))
            vertices.append((_meet(bevel, after, v, radius),
                             (bevel[0] + after[0], bevel[1] + after[1])))
        else:
            vertices.append((_meet(before, after, v, radius),
                             (before[0] + after[0], before[1] + after[1])))
    return rows, vertices
