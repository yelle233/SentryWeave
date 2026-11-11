"""Field semantic files: where structures, zones, obstacles and rule keep-outs are.

A field file only describes the field; it never whitelists goals. With
``mirror: auto`` the file is written from the red side and every position is
reflected through the field centre when the robot plays BLUE.
"""

from pathlib import Path

import yaml

from .config import Area, Config, Point, Structure, Zone
from .model import STRUCTURE_IDS, IntentType, TaskType, Vec2
from .shapes import area_polygon, ccw, centroid, mirror_point, mirror_yaw, rectangle

FIELD_KEYS = {"field_version", "season", "site", "source", "mirror", "bounds", "obstacles",
              "keepouts", "structures", "supply", "zones", "points"}


def _vec(value) -> Vec2:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"expected [x, y], got {value!r}")
    return Vec2(float(value[0]), float(value[1]))


def _polygon(values) -> tuple[Vec2, ...]:
    """Vertices without repeated neighbours; a closed ring loses its closing vertex."""
    points = []
    for p in map(_vec, values):
        if not points or p != points[-1]:
            points.append(p)
    if len(points) > 1 and points[-1] == points[0]:
        points.pop()
    return tuple(points)


def _area(item, index: int, prefix: str) -> Area:
    if isinstance(item, (list, tuple)):
        return Area(f"{prefix}_{index}", rectangle(*(float(v) for v in item)))
    if not isinstance(item, dict) or not item.get("id") or ("rect" in item) == ("polygon" in item):
        raise ValueError(f"{prefix} {index} needs an id and exactly one of rect/polygon")
    if "rect" in item:
        return Area(str(item["id"]), rectangle(*(float(v) for v in item["rect"])))
    polygon = _polygon(item["polygon"])
    return Area(str(item["id"]), ccw(polygon) if polygon else polygon)


def _zone(item: dict, intent: IntentType | None = None) -> Zone:
    polygon = _polygon(item.get("polygon", ()))
    if polygon:
        center = _vec(item["center"]) if "center" in item else centroid(polygon)
        radius = float(item.get("radius", max(center.distance(p) for p in polygon)))
    else:
        center, radius = _vec(item["center"]), float(item["radius"])
    return Zone(str(item["id"]), center, radius, intent or IntentType[item["intent"]],
                float(item.get("importance", 0.7)), polygon, str(item.get("structure", "")))


def parse_field(raw: dict) -> dict:
    """Config keyword arguments for a field mapping, before any mirroring."""
    if not isinstance(raw, dict) or raw.get("field_version") != 1:
        raise ValueError("unsupported field_version")
    if set(raw) - FIELD_KEYS:
        raise ValueError(f"unknown field keys: {set(raw) - FIELD_KEYS}")
    for key in ("season", "bounds", "structures", "supply", "zones"):
        if key not in raw:
            raise ValueError(f"field file needs {key}")
    structures = tuple(Structure(str(name), _vec(value["xy"]), float(value.get("yaw", 0.0)))
                       for name, value in raw["structures"].items())
    zones = [_zone(item) for item in raw["zones"]]
    zones.append(_zone(raw["supply"], IntentType.RETREAT))
    points = tuple(Point(str(p["id"]), str(p["zone"]), _vec(p["xy"]), float(p.get("yaw", 0.0)),
                         tuple(TaskType[t] for t in p["tasks"])) for p in raw.get("points") or ())
    return {
        "season": str(raw["season"]), "site": str(raw.get("site", raw["season"])),
        "mirror": str(raw.get("mirror", "none")), "bounds": tuple(float(v) for v in raw["bounds"]),
        "obstacles": tuple(_area(a, i, "obstacle") for i, a in enumerate(raw.get("obstacles") or ())),
        "keepouts": tuple(_area(a, i, "keepout") for i, a in enumerate(raw.get("keepouts") or ())),
        "structures": structures, "zones": tuple(zones), "points": points,
    }


def mirror_field(values: dict) -> dict:
    """Reflect every position of parsed field values through the field centre."""
    bounds = values["bounds"]

    def point(p):
        return mirror_point(p, bounds)

    def area(a):
        return Area(a.name, tuple(point(p) for p in a.polygon))

    return {
        **values, "mirrored": True,
        "obstacles": tuple(area(a) for a in values["obstacles"]),
        "keepouts": tuple(area(a) for a in values["keepouts"]),
        "structures": tuple(Structure(s.name, point(s.position), mirror_yaw(s.yaw))
                            for s in values["structures"]),
        "zones": tuple(Zone(z.name, point(z.center), z.radius, z.intent, z.importance,
                            tuple(point(p) for p in z.polygon), z.structure)
                       for z in values["zones"]),
        "points": tuple(Point(p.name, p.zone, point(p.position), mirror_yaw(p.yaw), p.tasks)
                        for p in values["points"]),
    }


def read_field(field, base: Path, own_team: str) -> dict:
    """Parse an inline field mapping or a field file path relative to ``base``."""
    if isinstance(field, str):
        field = yaml.safe_load((Path(base) / field).read_text(encoding="utf-8"))
    values = parse_field(field)
    if values["mirror"] == "auto" and own_team == "BLUE":
        values = mirror_field(values)
    return values


def field_problems(config: Config, reachability: bool = False) -> list[str]:
    """Semantic field checks; ``reachability`` adds the slower grid connectivity check."""
    problems = []
    x0, y0, x1, y1 = config.bounds
    names = {s.name for s in config.structures}
    if config.season:
        problems += [f"missing structure {name}" for name in STRUCTURE_IDS if name not in names]
    for structure in config.structures:
        if not (x0 <= structure.position.x <= x1 and y0 <= structure.position.y <= y1):
            problems.append(f"structure {structure.name} is outside the bounds")
    if config.mirror == "auto":
        tolerance = config.mirror_tolerance
        polygons = [area_polygon(a) for a in config.obstacles]
        for item, polygon in zip(config.obstacles, polygons):
            image = [mirror_point(p, config.bounds) for p in polygon]
            if not any(len(other) == len(image) and all(
                    min(p.distance(q) for q in other) <= tolerance for p in image)
                    for other in polygons):
                problems.append(f"obstacle {getattr(item, 'name', item)} has no mirror image")
        for own, enemy in (("own_base", "enemy_base"), ("own_outpost", "enemy_outpost")):
            a, b = config.structure(own), config.structure(enemy)
            if a and b and mirror_point(a.position, config.bounds).distance(b.position) > tolerance:
                problems.append(f"{enemy} is not the mirror image of {own}")
    if reachability:
        from .geometry import TacticalMap

        arena = TacticalMap(config)
        anchor = None
        for zone in config.zones:
            targets = arena.zone_targets(zone)
            if not targets:
                problems.append(f"zone {zone.name} has no legal position")
                continue
            if not zone.contains(targets[0]) and zone.polygon:
                problems.append(f"zone {zone.name} has no legal position inside its polygon")
            anchor = anchor or targets[0]
            if not arena.reachable(anchor, targets[0]):
                problems.append(f"zone {zone.name} is not reachable from {config.zones[0].name}")
        for point in config.points:
            if anchor and not arena.reachable(anchor, point.position):
                problems.append(f"point {point.name} is not reachable")
    return problems
