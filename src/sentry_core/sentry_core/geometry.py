"""Conservative tactical geometry over convex obstacle and keep-out polygons.

Exclusions grow by the robot radius and routes use a visibility graph over the
grown corners. Reachability also uses 4-connected components of a fixed grid.
This is the tactical estimate, not a replacement for the live Nav2 costmap; the
executor checks the current Nav2 path before every new navigation goal.
"""

from collections import deque
from functools import lru_cache
import math

import numpy as np

from .config import Config, Zone
from .model import Vec2
from .shapes import area_polygon, inflate

EPS = 1e-4
CHUNK = 4096


def _arrays(rows):
    """Pad half-plane rows into arrays; padding rows (0, 0, 1) never constrain anything."""
    width = max((len(r) for r in rows), default=0)
    normals, offsets = np.zeros((len(rows), width, 2)), np.ones((len(rows), width))
    for i, polygon in enumerate(rows):
        for k, (nx, ny, c) in enumerate(polygon):
            normals[i, k] = nx, ny
            offsets[i, k] = c
    return normals, offsets


def _inside_any(normals, offsets, points: np.ndarray) -> np.ndarray:
    result = np.zeros(len(points), dtype=bool)
    if not len(offsets):
        return result
    for start in range(0, len(points), CHUNK):
        values = np.einsum("pkj,mj->mpk", normals, points[start:start + CHUNK])
        result[start:start + CHUNK] = (values <= offsets).all(axis=2).any(axis=1)
    return result


@lru_cache(maxsize=8)
def _grid(bounds, radius, resolution, rows):
    """Cell centres, free mask and 4-connected labels (-1: blocked), shared by equal maps."""
    a, b, c, d = bounds
    xs = a + (np.arange(max(1, int((c - a) / resolution))) + 0.5) * resolution
    ys = b + (np.arange(max(1, int((d - b) / resolution))) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    points = np.stack([gx.ravel(), gy.ravel()], axis=1)
    x, y = points[:, 0], points[:, 1]
    free = ((a + radius <= x) & (x <= c - radius) & (b + radius <= y) & (y <= d - radius)
            & ~_inside_any(*_arrays(rows), points)).reshape(gx.shape)
    labels = np.full(free.shape, -1, dtype=np.int64)
    width, height = free.shape
    current = 0
    for i, j in zip(*np.nonzero(free)):
        if labels[i, j] >= 0:
            continue
        labels[i, j] = current
        queue = deque([(int(i), int(j))])
        while queue:
            ci, cj = queue.popleft()
            for ni, nj in ((ci + 1, cj), (ci - 1, cj), (ci, cj + 1), (ci, cj - 1)):
                if 0 <= ni < width and 0 <= nj < height and free[ni, nj] and labels[ni, nj] < 0:
                    labels[ni, nj] = current
                    queue.append((ni, nj))
        current += 1
    for array in (xs, ys, free, labels):
        array.flags.writeable = False
    return xs, ys, free, labels


class TacticalMap:
    def __init__(self, config: Config):
        self.config = config
        grown = [inflate(area_polygon(item), config.robot_radius)
                 for item in (*config.obstacles, *config.keepouts)]
        self._rows = tuple(tuple(rows) for rows, _ in grown)
        self._normals, self._offsets = _arrays(self._rows)
        corners = [Vec2(v.x + EPS * nx, v.y + EPS * ny)
                   for _, vertices in grown for v, (nx, ny) in vertices]
        self.corners = tuple(p for p in corners if self.legal(p))
        self._nodes = np.array([[p.x, p.y] for p in self.corners]).reshape(-1, 2)
        n = len(self.corners)
        self._weights = np.full((n, n), np.inf)
        for i in range(n - 1):
            others = self._nodes[i + 1:]
            open_ = ~self._blocked(np.repeat(self._nodes[i:i + 1], len(others), axis=0), others)
            lengths = np.hypot(*(others - self._nodes[i]).T)
            self._weights[i, i + 1:] = np.where(open_, lengths, np.inf)
        self._weights = np.minimum(self._weights, self._weights.T)

    def in_bounds(self, p: Vec2) -> bool:
        a, b, c, d = self.config.bounds
        return p.finite and a <= p.x <= c and b <= p.y <= d

    def legal(self, p: Vec2) -> bool:
        a, b, c, d = self.config.bounds
        r = self.config.robot_radius
        if not (p.finite and a + r <= p.x <= c - r and b + r <= p.y <= d - r):
            return False
        for rows in self._rows:
            if all(nx * p.x + ny * p.y <= offset for nx, ny, offset in rows):
                return False
        return True

    def _blocked(self, starts: np.ndarray, ends: np.ndarray) -> np.ndarray:
        """Vectorized Liang-Barsky/Cyrus-Beck test against every grown polygon."""
        if not len(self._offsets) or not len(starts):
            return np.zeros(len(starts), dtype=bool)
        direction = ends - starts
        nd = np.einsum("pkj,mj->mpk", self._normals, direction)
        q = self._offsets[None] - np.einsum("pkj,mj->mpk", self._normals, starts)
        parallel = np.abs(nd) < 1e-12
        with np.errstate(divide="ignore", invalid="ignore"):
            ratio = np.where(parallel, 0.0, q / np.where(parallel, 1.0, nd))
        lo = np.where(~parallel & (nd < 0), ratio, 0.0).max(axis=2)
        hi = np.where(~parallel & (nd > 0), ratio, 1.0).min(axis=2)
        outside = (parallel & (q < 0)).any(axis=2)
        return ((lo <= hi) & ~outside).any(axis=1)

    def segment_legal(self, start: Vec2, end: Vec2) -> bool:
        if not self.legal(start) or not self.legal(end):
            return False
        dx, dy = end.x - start.x, end.y - start.y
        for rows in self._rows:
            lo, hi = 0.0, 1.0
            for nx, ny, offset in rows:
                p, q = nx * dx + ny * dy, offset - nx * start.x - ny * start.y
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

    @lru_cache(maxsize=4096)
    def _visible(self, p: Vec2) -> np.ndarray:
        """Which graph corners a legal point sees through legal straight segments.

        Memoized per exact point: every route, batch distance and RL waypoint query of a
        cycle tests the same goals against every polygon, and fixed goals repeat across
        cycles. The shared result is read-only.
        """
        if not len(self._nodes):
            seen = np.zeros(0, dtype=bool)
        else:
            origin = np.repeat(np.array([[p.x, p.y]]), len(self._nodes), axis=0)
            seen = ~self._blocked(origin, self._nodes)
        seen.flags.writeable = False
        return seen

    @lru_cache(maxsize=512)
    def _tree(self, root: Vec2):
        """Dijkstra over the corner graph: distance to ``root`` and next hop (-1: root)."""
        n = len(self.corners)
        distance = np.full(n, np.inf)
        hop = np.full(n, -1, dtype=np.int64)
        if not n:
            return distance, hop
        seen = self._visible(root)
        distance[seen] = np.hypot(*(self._nodes[seen] - (root.x, root.y)).T)
        done = np.zeros(n, dtype=bool)
        for _ in range(n):
            pending = np.where(done, np.inf, distance)
            i = int(np.argmin(pending))
            if not math.isfinite(pending[i]):
                break
            done[i] = True
            through = distance[i] + self._weights[i]
            better = ~done & (through < distance)
            distance[better] = through[better]
            hop[better] = i
        return distance, hop

    def _via(self, point: Vec2, tree) -> tuple[float, int]:
        distance, _ = tree
        if not len(distance):
            return math.inf, -1
        total = np.where(self._visible(point),
                         np.hypot(*(self._nodes - (point.x, point.y)).T) + distance, np.inf)
        j = int(np.argmin(total))
        return float(total[j]), j

    @lru_cache(maxsize=8192)
    def route(self, start: Vec2, end: Vec2) -> tuple[Vec2, ...]:
        if not self.legal(start) or not self.legal(end):
            return ()
        if self.segment_legal(start, end):
            return start, end
        tree = self._tree(end)
        length, j = self._via(start, tree)
        if not math.isfinite(length):
            return ()
        path = [start]
        while j != -1:
            path.append(self.corners[j])
            j = int(tree[1][j])
        path.append(end)
        return tuple(path)

    def distance(self, start: Vec2, end: Vec2) -> float:
        path = self.route(start, end)
        return sum(a.distance(b) for a, b in zip(path, path[1:])) if path else math.inf

    def distances(self, start: Vec2, goals) -> list[float]:
        """Path lengths from one start to many goals with a single graph search."""
        if not self.legal(start):
            return [math.inf for _ in goals]
        result = []
        for goal in goals:
            if not self.legal(goal):
                result.append(math.inf)
            elif self.segment_legal(start, goal):
                result.append(start.distance(goal))
            else:
                result.append(self._via(goal, self._tree(start))[0])
        return result

    def path_from(self, start: Vec2, goal: Vec2) -> tuple[Vec2, ...]:
        """Like ``route`` but searched from ``start``; cheap for many goals of one start."""
        if not self.legal(start) or not self.legal(goal):
            return ()
        if self.segment_legal(start, goal):
            return start, goal
        tree = self._tree(start)
        length, j = self._via(goal, tree)
        if not math.isfinite(length):
            return ()
        path = [goal]
        while j != -1:
            path.append(self.corners[j])
            j = int(tree[1][j])
        path.append(start)
        return tuple(reversed(path))

    @lru_cache(maxsize=256)
    def zone_targets(self, zone: Zone) -> tuple[Vec2, ...]:
        # A base/outpost centre may be a solid structure. Calibrated approach points
        # remain valid evidence; otherwise use the nearest legal position.
        if self.legal(zone.center):
            return (zone.center,)
        points = tuple(p.position for p in self.config.points if p.zone == zone.name)
        if points:
            return points
        projected = self.nearest_legal(zone.center)
        return (projected,) if projected is not None else ()

    def zone_route(self, start: Vec2, zone: Zone) -> tuple[Vec2, ...]:
        routes = [route for target in self.zone_targets(zone) if (route := self.route(start, target))]
        return min(routes, key=lambda path: sum(a.distance(b) for a, b in zip(path, path[1:])),
                   default=())

    def zone_distance(self, start: Vec2, zone: Zone) -> float:
        path = self.zone_route(start, zone)
        return sum(a.distance(b) for a, b in zip(path, path[1:])) if path else math.inf

    def observed_zone_route(self, pos: Vec2, zone: Zone) -> tuple[Vec2, ...]:
        start = self.origin(pos)
        return self.zone_route(start, zone) if start is not None else ()

    def observed_zone_distance(self, pos: Vec2, zone: Zone) -> float:
        start = self.origin(pos)
        return (pos.distance(start) + self.zone_distance(start, zone)
                if start is not None else math.inf)

    @staticmethod
    def _walk(path, distance: float) -> Vec2:
        for a, b in zip(path, path[1:]):
            length = a.distance(b)
            if distance <= length and length > 1e-9:
                fraction = distance / length
                return Vec2(a.x + fraction * (b.x - a.x), a.y + fraction * (b.y - a.y))
            distance -= length
        return path[-1]

    def advance(self, start: Vec2, goal: Vec2, distance: float) -> Vec2:
        path = self.route(start, goal)
        return self._walk(path, distance) if path else start

    def advance_from(self, start: Vec2, goal: Vec2, distance: float) -> Vec2:
        path = self.path_from(start, goal)
        return self._walk(path, distance) if path else start

    @property
    def _grid(self):
        c = self.config
        return _grid(c.bounds, c.robot_radius, c.grid_resolution, self._rows)

    @lru_cache(maxsize=4096)
    def component(self, p: Vec2) -> int:
        """Grid component of a legal point; -1 when illegal or not linked to a free cell."""
        if not self.legal(p):
            return -1
        xs, ys, free, labels = self._grid
        a, b = self.config.bounds[:2]
        i = int((p.x - a) // self.config.grid_resolution)
        j = int((p.y - b) // self.config.grid_resolution)
        best = None
        for ni in (i, i - 1, i + 1):
            for nj in (j, j - 1, j + 1):
                if 0 <= ni < len(xs) and 0 <= nj < len(ys) and free[ni, nj]:
                    centre = Vec2(float(xs[ni]), float(ys[nj]))
                    gap = p.distance(centre)
                    if (best is None or gap < best[0]) and self.segment_legal(p, centre):
                        best = gap, int(labels[ni, nj])
        return best[1] if best else -1

    def reachable(self, start: Vec2, goal: Vec2) -> bool:
        label = self.component(start)
        return label >= 0 and label == self.component(goal)

    def nearest_legal(self, p: Vec2, anchor: Vec2 | None = None) -> Vec2 | None:
        """``p`` itself when legal (and linked to ``anchor``), else the nearest free cell centre."""
        if not p.finite:
            return None
        label = None if anchor is None else self.component(anchor)
        if label is not None and label < 0:
            return None
        if self.legal(p) and (label is None or self.component(p) == label):
            return p
        xs, ys, free, labels = self._grid
        ii, jj = np.nonzero(free if label is None else labels == label)
        if not len(ii):
            return None
        k = int(np.argmin((xs[ii] - p.x) ** 2 + (ys[jj] - p.y) ** 2))
        return Vec2(float(xs[ii[k]]), float(ys[jj[k]]))

    @lru_cache(maxsize=4096)
    def origin(self, p: Vec2) -> Vec2 | None:
        """Observed robot start: ``p`` when legal and on the grid, else nearest free cell."""
        if not self.in_bounds(p):
            return None
        if self.legal(p) and self.component(p) >= 0:
            return p
        xs, ys, free, _ = self._grid
        ii, jj = np.nonzero(free)
        if not len(ii):
            return None
        k = int(np.argmin((xs[ii] - p.x) ** 2 + (ys[jj] - p.y) ** 2))
        return Vec2(float(xs[ii[k]]), float(ys[jj[k]]))
