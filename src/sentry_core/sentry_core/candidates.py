"""Per-cycle candidate goals generated from field semantics and the current situation.

Field files never whitelist goals. Generated positions (intercept, defense,
support, contest, recovery) are projected onto the nearest legal position in the
robot's own connected region; preferred field points are added unchanged.
"""

from dataclasses import dataclass
import math

from .config import Config, Zone
from .geometry import TacticalMap
from .model import IntentType, TaskType, Vec2, World

MIN_INTERCEPT_THREAT = 0.18
TRAVEL_SCALE = 25.0
DUPLICATE_DISTANCE = 0.25
# The RL sub-goal never becomes a candidate; this names its cooldown and rejections.
SUBGOAL_ID = "rl_subgoal"


@dataclass(frozen=True)
class Goal:
    goal_id: str
    kind: TaskType
    zone_id: str
    position: Vec2
    yaw: float
    source: str
    target_id: int | None = None


def utility(features, weights) -> float:
    """Weighted desirability; risk, travel and switching count against a candidate."""
    desirability = (*features[:3], *(1 - x for x in features[3:]))
    return sum(w * f for w, f in zip(weights, desirability))


def _unit(dx: float, dy: float) -> tuple[float, float]:
    norm = math.hypot(dx, dy)
    return (dx / norm, dy / norm) if norm > 1e-6 else (1.0, 0.0)


def _turn(u: tuple[float, float], angle: float) -> tuple[float, float]:
    c, s = math.cos(angle), math.sin(angle)
    return u[0] * c - u[1] * s, u[0] * s + u[1] * c


class GoalGenerator:
    def __init__(self, config: Config, arena: TacticalMap):
        self.config, self.arena = config, arena
        x0, y0, x1, y1 = config.bounds
        self.center = Vec2((x0 + x1) / 2, (y0 + y1) / 2)
        self.zones = {zone.name: zone for zone in config.zones}

    def anchor(self, zone: Zone) -> Vec2:
        """The structure a zone protects or targets; the zone centre without one."""
        structure = self.config.structure(zone.structure) if zone.structure else None
        return structure.position if structure is not None else zone.center

    def generate(self, world: World, intents, threats, preferred_only=False) -> list[Goal]:
        goals = [Goal(p.name, kind, p.zone, p.position, p.yaw, "preferred")
                 for p in self.config.points for kind in p.tasks]
        if preferred_only and goals:
            return goals
        origin = self.arena.origin(world.self_track.position) if world.self_track else None
        if origin is None:
            return [g for g in goals if self.arena.legal(g.position)]
        for goal in (*self._defense(world, threats), *self._support(world, intents),
                     *self._areas()):
            position = self.arena.nearest_legal(goal.position, anchor=origin)
            if position is None or (goal.source == "contest"
                                    and not self.zones[goal.zone_id].contains(position)
                                    and position.distance(goal.position) > 1.0):
                continue
            if any(g.kind == goal.kind and g.zone_id == goal.zone_id
                   and g.target_id == goal.target_id
                   and g.position.distance(position) < DUPLICATE_DISTANCE for g in goals):
                continue
            goals.append(Goal(goal.goal_id, goal.kind, goal.zone_id, position, goal.yaw,
                              goal.source, goal.target_id))
        return goals

    def _defense(self, world: World, threats):
        config = self.config
        enemies = {t.robot_id: t for t in world.enemies if t.position.finite}
        threat_by_zone = {t.zone_id: t for t in threats}
        for zone in config.zones:
            if zone.intent != IntentType.DEFEND_BASE:
                continue
            base = self.anchor(zone)
            threat = threat_by_zone.get(zone.name)
            enemy = enemies.get(threat.enemy_id) if threat and threat.enemy_id is not None else None
            if enemy is not None and threat.score >= MIN_INTERCEPT_THREAT:
                u = _unit(enemy.position.x - base.x, enemy.position.y - base.y)
                reach = min(config.intercept_distance, 0.6 * base.distance(enemy.position))
                yield Goal(f"intercept:{zone.name}:{enemy.robot_id}", TaskType.INTERCEPT, zone.name,
                           Vec2(base.x + u[0] * reach, base.y + u[1] * reach),
                           math.atan2(u[1], u[0]), "intercept", enemy.robot_id)
            facing = enemy.position if enemy is not None else self.center
            u = _unit(facing.x - base.x, facing.y - base.y)
            for k, angle in enumerate((0.0, config.defense_spread, -config.defense_spread)):
                d = _turn(u, angle)
                yield Goal(f"defense:{zone.name}:{k}", TaskType.DEFEND, zone.name,
                           Vec2(base.x + d[0] * config.defense_radius,
                                base.y + d[1] * config.defense_radius),
                           math.atan2(u[1], u[0]), "defense")

    def _support(self, world: World, intents):
        config = self.config
        allies = {t.robot_id: t for t in world.allies if t.usable}
        for intent in intents:
            zone, ally = self.zones.get(intent.zone_id), allies.get(intent.robot_id)
            if (not intent.valid or ally is None or zone is None
                    or zone.intent not in (IntentType.ATTACK_OUTPOST, IntentType.CONTEST_AREA)):
                continue
            target = self.anchor(zone)
            if ally.velocity.norm > 0.15:
                ahead = _unit(ally.velocity.x, ally.velocity.y)
            else:
                ahead = _unit(target.x - ally.position.x, target.y - ally.position.y)
            side, yaw = (-ahead[1], ahead[0]), math.atan2(ahead[1], ahead[0])
            for k, sign in enumerate((1.0, -1.0)):
                yield Goal(f"support:{zone.name}:{ally.robot_id}:{k}", TaskType.SUPPORT, zone.name,
                           Vec2(ally.position.x - ahead[0] * config.support_back
                                + sign * side[0] * config.support_side,
                                ally.position.y - ahead[1] * config.support_back
                                + sign * side[1] * config.support_side),
                           yaw, "support", ally.robot_id)
            back = _unit(ally.position.x - target.x, ally.position.y - target.y)
            yield Goal(f"support:{zone.name}:{ally.robot_id}:2", TaskType.SUPPORT, zone.name,
                       Vec2(target.x + back[0] * config.support_standoff,
                            target.y + back[1] * config.support_standoff),
                       math.atan2(-back[1], -back[0]), "support", ally.robot_id)

    def _areas(self):
        for zone in self.config.zones:
            if zone.intent == IntentType.CONTEST_AREA:
                for k, p in enumerate(self._samples(zone)):
                    yield Goal(f"contest:{zone.name}:{k}", TaskType.SUPPORT, zone.name, p,
                               math.atan2(self.center.y - p.y, self.center.x - p.x), "contest")
            elif zone.intent == IntentType.RETREAT:
                yield Goal(f"recovery:{zone.name}", TaskType.RECOVER, zone.name, zone.center, 0.0,
                           "recovery")

    def _samples(self, zone: Zone) -> list[Vec2]:
        n, c = self.config.contest_samples, zone.center
        if zone.polygon:
            corners = [zone.polygon[(k * len(zone.polygon)) // n] for k in range(n)]
            ring = [Vec2((c.x + p.x) / 2, (c.y + p.y) / 2) for p in corners]
        else:
            ring = [Vec2(c.x + 0.5 * zone.radius * math.cos(2 * math.pi * k / n),
                         c.y + 0.5 * zone.radius * math.sin(2 * math.pi * k / n))
                    for k in range(n)]
        return [c, *ring]
