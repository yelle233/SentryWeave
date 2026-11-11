"""Validated, immutable tactics settings and the field semantics they reference.

Schema 1 keeps the original flat file. Schema 2 references a field file
(``field_version: 1``) with structures, zones, obstacles and rule keep-outs.
"""

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

import yaml

from .model import TEAM_COLORS, IntentType, TaskType, Vec2
from .shapes import area_polygon, contains, inflate, is_convex


@dataclass(frozen=True)
class Area:
    """Convex obstacle or rule keep-out polygon in the map frame."""

    name: str
    polygon: tuple[Vec2, ...]


@dataclass(frozen=True)
class Structure:
    """Base, outpost or field-defined structure position (may lie inside an obstacle)."""

    name: str
    position: Vec2
    yaw: float = 0.0


@dataclass(frozen=True)
class Zone:
    name: str
    center: Vec2
    radius: float
    intent: IntentType
    importance: float
    polygon: tuple[Vec2, ...] = ()
    structure: str = ""

    def contains(self, p: Vec2) -> bool:
        if self.polygon:
            return contains(self.polygon, p)
        return self.center.distance(p) <= self.radius


@dataclass(frozen=True)
class Point:
    name: str
    zone: str
    position: Vec2
    yaw: float
    tasks: tuple[TaskType, ...]


@dataclass(frozen=True)
class Config:
    frame: str
    self_id: int
    site: str
    bounds: tuple[float, float, float, float]
    # Physical obstacles: [x0, y0, x1, y1] rectangles or convex Area polygons.
    obstacles: tuple[tuple[float, float, float, float] | Area, ...]
    robot_radius: float
    zones: tuple[Zone, ...]
    points: tuple[Point, ...]
    weights: tuple[float, ...]
    observed_age: float = 0.3
    predicted_age: float = 0.9
    self_timeout: float = 0.5
    referee_timeout: float = 2.0
    retention: float = 5.0
    max_speed: float = 6.0
    jump_slack: float = 0.6
    history_sec: float = 3.0
    min_hold: float = 3.0
    switch_margin: float = 0.08
    confirm_cycles: int = 3
    cooldown: float = 5.0
    task_lease: float = 2.0
    intent_threshold: float = 0.35
    critical_hp: float = 0.25
    recovered_hp: float = 0.7
    urgent_threat: float = 0.72
    ally_clearance: float = 0.85
    nominal_speed: float = 1.5
    supply_enabled: bool = True
    max_tracks: int = 32
    own_team: str = "RED"
    game_timeout: float = 2.0
    robot_status_timeout: float = 2.0
    structure_timeout: float = 5.0
    # Rule-prohibited areas for this robot (convex Area polygons).
    keepouts: tuple[Area, ...] = ()
    grid_resolution: float = 0.1
    structures: tuple[Structure, ...] = ()
    season: str = ""
    mirror: str = "none"
    mirrored: bool = False
    mirror_tolerance: float = 0.05

    def structure(self, name: str) -> Structure | None:
        return next((s for s in self.structures if s.name == name), None)

    @property
    def map_fingerprint(self) -> str:
        values = {key: asdict(self)[key] for key in
                  ("frame", "bounds", "obstacles", "keepouts", "structures", "robot_radius",
                   "zones", "points")}
        return hashlib.sha256(json.dumps(values, sort_keys=True).encode()).hexdigest()


def validate_config(config: Config) -> Config:
    if len(config.weights) != 6 or not all(
        math.isfinite(v) and v >= 0 for v in config.weights
    ) or abs(sum(config.weights) - 1) > 1e-6:
        raise ValueError("utility weights must contain six finite nonnegative values summing to 1")
    x0, y0, x1, y1 = config.bounds
    if not all(math.isfinite(v) for v in config.bounds) or not (x0 < x1 and y0 < y1):
        raise ValueError("invalid map bounds")
    if not config.frame or not 1 <= config.self_id <= 65535:
        raise ValueError("invalid frame or self_id")
    if config.own_team not in TEAM_COLORS:
        raise ValueError("own_team must be RED or BLUE")
    for name in ("robot_radius", "observed_age", "predicted_age", "self_timeout",
                 "referee_timeout", "retention", "max_speed", "history_sec", "cooldown",
                 "task_lease", "nominal_speed", "ally_clearance", "game_timeout",
                 "robot_status_timeout", "structure_timeout"):
        if not math.isfinite(getattr(config, name)) or getattr(config, name) <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if not config.observed_age < config.predicted_age < config.retention:
        raise ValueError("observed_age < predicted_age < retention is required")
    if config.min_hold < 0 or not math.isfinite(config.min_hold):
        raise ValueError("min_hold must be finite and nonnegative")
    for name in ("switch_margin", "intent_threshold", "critical_hp",
                 "recovered_hp", "urgent_threat", "jump_slack"):
        value = getattr(config, name)
        if not math.isfinite(value) or not 0 <= value <= 1:
            raise ValueError(f"invalid {name}")
    if config.critical_hp >= config.recovered_hp:
        raise ValueError("critical_hp must be below recovered_hp")
    if config.confirm_cycles < 1 or config.max_tracks < 1:
        raise ValueError("invalid positive count")
    names = {z.name for z in config.zones}
    if len(names) != len(config.zones) or not names:
        raise ValueError("zone names must be unique and nonempty")
    roles = {z.intent for z in config.zones}
    if not {IntentType.ATTACK_OUTPOST, IntentType.DEFEND_BASE,
            IntentType.CONTEST_AREA, IntentType.RETREAT} <= roles:
        raise ValueError("map needs attack, defense, contest and recovery zones")
    if not math.isfinite(config.grid_resolution) or not 0.02 <= config.grid_resolution <= 1.0:
        raise ValueError("grid_resolution must be in [0.02, 1.0] m")
    if config.mirror not in ("auto", "none") or not (
            math.isfinite(config.mirror_tolerance) and config.mirror_tolerance > 0):
        raise ValueError("mirror must be auto or none with a positive mirror_tolerance")
    structure_names = [s.name for s in config.structures]
    if len(set(structure_names)) != len(structure_names) or any(
            not s.name or not s.position.finite or not math.isfinite(s.yaw)
            for s in config.structures):
        raise ValueError("structure names must be unique with finite poses")
    for item in (*config.obstacles, *config.keepouts):
        if isinstance(item, Area):
            if (not item.name or len(item.polygon) < 3 or not all(p.finite for p in item.polygon)
                    or not is_convex(item.polygon)):
                raise ValueError(f"area {item.name or '?'} must be a convex polygon with distinct "
                                 "vertices; split concave areas into convex parts")
        elif (len(item) != 4 or not all(math.isfinite(v) for v in item)
                or item[0] >= item[2] or item[1] >= item[3]):
            raise ValueError("invalid obstacle rectangle")
    exclusions = [inflate(area_polygon(item), config.robot_radius)[0]
                  for item in (*config.obstacles, *config.keepouts)]
    for zone in config.zones:
        if (not zone.center.finite or not x0 <= zone.center.x <= x1
                or not y0 <= zone.center.y <= y1 or not math.isfinite(zone.radius)
                or zone.radius <= 0 or not 0 <= zone.importance <= 1
                or (zone.polygon and (len(zone.polygon) < 3
                                      or not all(p.finite for p in zone.polygon)))
                or (zone.structure and zone.structure not in structure_names)):
            raise ValueError(f"invalid zone {zone.name}")
    point_names = set()
    for point in config.points:
        if (point.name in point_names or not point.name or point.zone not in names
                or not point.position.finite or not math.isfinite(point.yaw)
                or not x0 < point.position.x < x1 or not y0 < point.position.y < y1
                or not point.tasks):
            raise ValueError(f"invalid tactical point {point.name}")
        if any(all(nx * point.position.x + ny * point.position.y <= c for nx, ny, c in rows)
               for rows in exclusions):
            raise ValueError(f"tactical point {point.name} lies in an obstacle")
        point_names.add(point.name)
    return config


# Official referee ids of each team's sentry, used when a tactics file omits self_id.
SENTRY_IDS = {"RED": 7, "BLUE": 107}


def sentry_id(team: str, self_id=None) -> int:
    """An explicit self_id wins; an invalid team is reported by validate_config."""
    return SENTRY_IDS.get(team, 7) if self_id is None else int(self_id)


def _parameters(raw: dict) -> dict:
    parameters = raw.get("parameters", {}) or {}
    if not isinstance(parameters, dict):
        raise ValueError("parameters must be a mapping")
    return parameters


def load_config(path: str | Path, own_team: str | None = None, check: bool = True) -> Config:
    path = Path(path)
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema_version") not in (1, 2):
        raise ValueError("unsupported tactics schema_version")
    team = str(own_team or raw.get("own_team", "RED")).upper()
    if raw["schema_version"] == 2:
        from .field import field_problems, read_field

        allowed = {"schema_version", "frame", "self_id", "own_team", "field", "robot_radius",
                   "weights", "parameters"}
        if set(raw) - allowed:
            raise ValueError(f"unknown tactics keys: {set(raw) - allowed}")
        config = validate_config(Config(
            frame=raw["frame"], self_id=sentry_id(team, raw.get("self_id")),
            robot_radius=float(raw["robot_radius"]), weights=tuple(raw["weights"]),
            own_team=team, **read_field(raw["field"], path.parent, team), **_parameters(raw),
        ))
        problems = field_problems(config) if check else []
        if problems:
            raise ValueError("field check failed: " + "; ".join(problems))
        return config
    allowed = {"schema_version", "frame", "self_id", "site", "bounds", "obstacles",
               "robot_radius", "zones", "points", "weights", "parameters", "own_team"}
    if set(raw) - allowed:
        raise ValueError(f"unknown tactics keys: {set(raw) - allowed}")
    zones = tuple(Zone(z["id"], Vec2(*z["center"]), float(z["radius"]),
                       IntentType[z["intent"]], float(z["importance"])) for z in raw["zones"])
    points = tuple(Point(p["id"], p["zone"], Vec2(*p["xy"]), float(p.get("yaw", 0)),
                         tuple(TaskType[t] for t in p["tasks"])) for p in raw["points"])
    return validate_config(Config(
        frame=raw["frame"], self_id=sentry_id(team, raw.get("self_id")),
        site=raw["site"],
        bounds=tuple(raw["bounds"]), obstacles=tuple(tuple(r) for r in raw["obstacles"]),
        robot_radius=float(raw["robot_radius"]), zones=zones, points=points,
        weights=tuple(raw["weights"]), own_team=team, **_parameters(raw),
    ))
