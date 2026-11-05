"""Conservative visibility-graph travel estimates over configured rectangular exclusions.

This is the tactical estimate, not a replacement for the live Nav2 costmap.
The executor checks the current Nav2 path before every new navigation goal.
"""

from functools import lru_cache
import heapq
import math

from .config import Config, Zone
from .model import Vec2


class TacticalMap:
    def __init__(self, config: Config):
        self.config = config
        r = config.robot_radius
        self.rectangles = tuple((a - r, b - r, c + r, d + r)
                                for a, b, c, d in config.obstacles)
        eps = 1e-4
        self.corners = tuple(
            Vec2(x, y) for a, b, c, d in self.rectangles
            for x, y in ((a - eps, b - eps), (a - eps, d + eps),
                         (c + eps, b - eps), (c + eps, d + eps))
            if self.legal(Vec2(x, y))
        )

    def legal(self, p: Vec2) -> bool:
        a, b, c, d = self.config.bounds
        r = self.config.robot_radius
        return (p.finite and a + r <= p.x <= c - r and b + r <= p.y <= d - r
                and not any(x0 <= p.x <= x1 and y0 <= p.y <= y1
                            for x0, y0, x1, y1 in self.rectangles))

    def segment_legal(self, start: Vec2, end: Vec2) -> bool:
        if not self.legal(start) or not self.legal(end):
            return False
        dx, dy = end.x - start.x, end.y - start.y
        for a, b, c, d in self.rectangles:
            lo, hi = 0.0, 1.0
            for p, q in ((-dx, start.x - a), (dx, c - start.x),
                         (-dy, start.y - b), (dy, d - start.y)):
                if abs(p) < 1e-12:
                    if q < 0:
                        lo, hi = 1.0, 0.0
                        break
                elif p < 0:
                    lo = max(lo, q / p)
                else:
                    hi = min(hi, q / p)
            if lo <= hi:
                return False
        return True

    @lru_cache(maxsize=8192)
    def route(self, start: Vec2, end: Vec2) -> tuple[Vec2, ...]:
        if not self.legal(start) or not self.legal(end):
            return ()
        if self.segment_legal(start, end):
            return start, end
        nodes = (start, end, *self.corners)
        distance, previous = {0: 0.0}, {}
        queue = [(0.0, 0)]
        while queue:
            cost, i = heapq.heappop(queue)
            if cost > distance[i]:
                continue
            if i == 1:
                route = [end]
                while i != 0:
                    i = previous[i]
                    route.append(nodes[i])
                return tuple(reversed(route))
            for j, target in enumerate(nodes):
                if i == j or not self.segment_legal(nodes[i], target):
                    continue
                candidate = cost + nodes[i].distance(target)
                if candidate < distance.get(j, math.inf):
                    distance[j] = candidate
                    previous[j] = i
                    heapq.heappush(queue, (candidate, j))
        return ()

    def distance(self, start: Vec2, end: Vec2) -> float:
        path = self.route(start, end)
        return sum(a.distance(b) for a, b in zip(path, path[1:])) if path else math.inf

    def zone_route(self, start: Vec2, zone: Zone) -> tuple[Vec2, ...]:
        # An outpost/base center may itself be a solid structure. Its calibrated
        # approach points remain valid evidence of motion toward that objective.
        targets = ([zone.center] if self.legal(zone.center) else
                   [p.position for p in self.config.points if p.zone == zone.name])
        routes = [route for target in targets if (route := self.route(start, target))]
        return min(routes, key=lambda path: sum(a.distance(b) for a, b in zip(path, path[1:])),
                   default=())

    def zone_distance(self, start: Vec2, zone: Zone) -> float:
        path = self.zone_route(start, zone)
        return sum(a.distance(b) for a, b in zip(path, path[1:])) if path else math.inf

    def advance(self, start: Vec2, goal: Vec2, distance: float) -> Vec2:
        path = self.route(start, goal)
        if not path:
            return start
        for a, b in zip(path, path[1:]):
            length = a.distance(b)
            if distance <= length and length > 1e-9:
                fraction = distance / length
                return Vec2(a.x + fraction * (b.x - a.x), a.y + fraction * (b.y - a.y))
            distance -= length
        return goal
