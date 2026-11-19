"""Small pure-python geometry helpers (no numpy needed, hot paths are tiny)."""
from __future__ import annotations

import math
from typing import Sequence


def point_in_polygon(x: float, y: float, poly: Sequence[tuple[float, float]]) -> bool:
    """Ray-casting test; works for non-convex polygons."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            x_int = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < x_int:
                inside = not inside
        j = i
    return inside


def distance_to_polygon_edge(x: float, y: float, poly: Sequence[tuple[float, float]]) -> float:
    """Shortest distance from (x, y) to any polygon edge."""
    best = float("inf")
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        seg_len2 = dx * dx + dy * dy or 1e-12
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg_len2))
        px, py = ax + t * dx, ay + t * dy
        best = min(best, math.hypot(x - px, y - py))
    return best


def dist(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(bx - ax, by - ay)


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
