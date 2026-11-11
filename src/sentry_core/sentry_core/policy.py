"""Constrained utility policy and hysteresis, shared by ROS and closed-loop training."""

from dataclasses import replace
import math
import uuid

from .config import Config, validate_config
from .geometry import TacticalMap
from .inputs import view_inputs
from .intent import IntentEstimator
from .model import Candidate, Decision, IntentType, Phase, Task, TaskType, Threat, World


FEATURES = ("strategic", "cooperation", "urgency", "risk", "travel", "switching")


def safety_reason(world: World, config: Config, arena: TacticalMap) -> str | None:
    track = world.self_track
    if world.frame != config.frame:
        return "FRAME_MISMATCH"
    if (track is None or not track.usable
            or not -0.05 <= world.stamp - track.observed_at <= config.self_timeout
            or not arena.legal(track.position)):
        return "LOCALIZATION_INVALID"
    info = view_inputs(world, config)
    if info.phase == Phase.UNKNOWN:
        return "GAME_STATE_UNKNOWN"
    if info.phase != Phase.RUNNING:
        return "GAME_NOT_RUNNING"
    if info.own_team is not None and info.own_team != config.own_team:
        return "TEAM_MISMATCH"
    if info.self_alive is False or (info.self_hp is not None and info.self_hp <= 0):
        return "ROBOT_DISABLED"
    return None


class DecisionEngine:
    def __init__(self, config: Config, model=None, session_id: str | None = None,
                 baseline: bool = False, hysteresis: bool = True, goal_prior=None):
        self.config = validate_config(config)
        self.arena = TacticalMap(config)
        self.estimator = IntentEstimator(config, self.arena, model)
        self.session_id = session_id or uuid.uuid4().hex
        self.baseline, self.hysteresis = baseline, hysteresis
        self.goal_prior = goal_prior
        self.current: Task | None = None
        self._next_id = 0
        self._last_time: float | None = None
        self._epoch: int | None = None
        self._challenger = None
        self._confirmations = 0
        self._cooldowns: dict[str, float] = {}
        self._recovering = False

    def feedback(self, session_id: str, task_id: int, state: str, now: float) -> None:
        if (self.current is None or self.current.identity != (session_id, task_id)
                or not math.isfinite(now)):
            return
        if state in ("FAILED", "REJECTED"):
            self._cooldowns[self.current.goal_id] = now + self.config.cooldown
        elif state == "CANCELED":
            # A watchdog can cancel between planner ticks. Resuming requires a new
            # task identity, otherwise the executor correctly treats it as a replay.
            self.current = None

    def _unsafe(self, world: World) -> str | None:
        return safety_reason(world, self.config, self.arena)

    def _threats(self, world: World) -> tuple[Threat, ...]:
        result = []
        for zone in self.config.zones:
            if zone.intent != IntentType.DEFEND_BASE:
                continue
            best = Threat(zone.name, 0.0, None, None)
            for enemy in world.enemies:
                if not enemy.usable:
                    continue
                route = self.arena.zone_route(enemy.position, zone)
                distance = self.arena.zone_distance(enemy.position, zone)
                if not route:
                    continue
                speed = max(0.7, enemy.velocity.norm)
                eta = max(0., distance - zone.radius) / speed
                dx, dy = route[1].x - enemy.position.x, route[1].y - enemy.position.y
                denom = math.hypot(dx, dy) * enemy.velocity.norm
                approach = max(0., (dx * enemy.velocity.x + dy * enemy.velocity.y) / denom
                               ) if denom > 1e-6 else 0.5
                factor = 1.0 if distance <= zone.radius else 0.55 + 0.45 * approach
                score = min(1., enemy.confidence * zone.importance * math.exp(-eta / 8) * factor)
                if score > best.score:
                    best = Threat(zone.name, score, enemy.robot_id, eta)
            result.append(best)
        return tuple(result)

    def _candidates(self, world, intents, threats):
        config = self.config
        unsafe = self._unsafe(world)
        hold = Candidate(TaskType.HOLD, "", None, "", None, 0., (0.,) * 6,
                         0., 1., bool(unsafe), unsafe or "NO_FEASIBLE_TASK")
        if unsafe:
            return [hold], [unsafe]
        info, own = view_inputs(world, config), world.self_track
        own_hp, own_ammo = info.self_hp, info.self_ammo
        hp = own_hp if own_hp is not None else 0.5
        can_supply = config.supply_enabled and info.supply_available is True
        need_supply = ((own_hp is not None and own_hp < config.critical_hp)
                       or (own_ammo is not None and own_ammo < 0.1))
        if self._recovering and own_hp is not None and own_hp >= config.recovered_hp:
            self._recovering = False
        self._recovering = can_supply and (need_supply or self._recovering)
        results, rejected = [hold], []
        zones = {z.name: z for z in config.zones}
        threat_by_zone = {t.zone_id: t for t in threats}
        allies = {t.robot_id: t for t in world.allies if t.usable}
        for point in config.points:
            if self._cooldowns.get(point.name, 0) > world.stamp:
                rejected.append(f"{point.name}:FAILURE_COOLDOWN")
                continue
            path_length = self.arena.distance(own.position, point.position)
            if not math.isfinite(path_length):
                rejected.append(f"{point.name}:UNREACHABLE")
                continue
            clearance = min((t.position.distance(point.position) for t in allies.values()),
                            default=10.)
            if clearance < config.ally_clearance:
                rejected.append(f"{point.name}:ALLY_OCCUPIED")
                continue
            zone = zones[point.zone]
            threat = threat_by_zone.get(point.zone, Threat(point.zone, 0., None, None))
            coverage = sum(t.position.distance(zone.center) < zone.radius for t in allies.values())
            for kind in point.tasks:
                target = None
                confidence, urgent = 1.0, False
                value, cooperation, urgency, risk = 0., 0., 0., (1 - hp) * 0.3
                reason = ""
                if kind == TaskType.DEFEND:
                    value = 0.42 + (0.12 if info.structures["own_outpost"] is False else 0.)
                    cooperation = 0.18 / (1 + coverage)
                    urgency, reason = 0.15 + 0.8 * threat.score, "AREA_UNCOVERED"
                elif kind == TaskType.INTERCEPT:
                    if self.baseline or threat.score < 0.18 or hp < config.critical_hp:
                        continue
                    value, cooperation = zone.importance, 1 / (1 + coverage)
                    urgency, target = threat.score, threat.enemy_id
                    confidence = threat.score
                    urgent = threat.score >= config.urgent_threat
                    risk += 0.12
                    reason = "BASE_THREAT"
                elif kind == TaskType.SUPPORT:
                    if own_hp is None or hp < 0.35 or own_ammo is None or own_ammo < 0.1:
                        rejected.append(f"{point.name}:RESOURCES_UNCERTAIN_OR_LOW")
                        continue
                    if (zone.intent == IntentType.ATTACK_OUTPOST
                            and info.structures["enemy_outpost"] is not True):
                        rejected.append(f"{point.name}:OUTPOST_NOT_CONFIRMED_ALIVE")
                        continue
                    supporting = [i for i in intents if i.valid and i.zone_id == zone.name
                                  and i.robot_id in allies]
                    if self.baseline and zone.intent == IntentType.ATTACK_OUTPOST:
                        confidence, cooperation = 1., 0.7
                    elif supporting:
                        intent = max(supporting, key=lambda i: i.confidence)
                        target, confidence = intent.robot_id, intent.confidence
                        cooperation = confidence * min(1., clearance / 3.)
                    else:
                        continue
                    value, urgency, risk = zone.importance, 0.5, risk + 0.1
                    reason = "TEAM_SUPPORT_HIGH" if not self.baseline else "FIXED_OUTPOST_RULE"
                elif kind == TaskType.RECOVER:
                    if not can_supply or not (self._recovering
                                              or (own_hp is not None and own_hp < 0.6)
                                              or (own_ammo is not None and own_ammo < 0.2)):
                        continue
                    value, cooperation = 0.8, 0.4
                    urgency = max(1 - hp, 1 - (own_ammo if own_ammo is not None else 1))
                    urgent, reason = self._recovering, "LOW_RESOURCES"
                key = kind, point.zone, target, point.name
                switching = float(self.current is not None and self.current.key != key)
                features = (value, cooperation, urgency, min(1., risk),
                            min(1., path_length / 25.), switching)
                desirability = (*features[:3], *(1 - x for x in features[3:]))
                utility = sum(w * f for w, f in zip(config.weights, desirability))
                results.append(Candidate(kind, point.zone, target, point.name, point.position,
                                         point.yaw, features, utility, confidence, urgent, reason))
        # Resource recovery is a hard team-configured survival constraint, not a model weight.
        if self._recovering:
            results = [c for c in results if c.kind in (TaskType.HOLD, TaskType.RECOVER)]
        return results, rejected

    def _select(self, candidates, now):
        current = next((c for c in candidates if self.current and c.key == self.current.key), None)
        urgent = [c for c in candidates if c.urgent]
        best = max(urgent or candidates, key=lambda c: (c.utility, c.goal_id))
        reason = "INITIAL"
        chosen = best
        if self.current and current is None:
            reason = "CURRENT_INVALID"
        elif current is not None:
            if best.key == current.key:
                chosen, reason = current, "KEEP_CURRENT"
            elif urgent:
                reason = "URGENT_PREEMPT"
            elif not self.hysteresis:
                reason = "HYSTERESIS_DISABLED"
            elif now - self.current.created_at < self.config.min_hold:
                chosen, reason = current, "MIN_HOLD"
            elif best.utility < current.utility + self.config.switch_margin:
                chosen, reason = current, "SWITCH_MARGIN"
            else:
                self._confirmations = self._confirmations + 1 if self._challenger == best.key else 1
                self._challenger = best.key
                if self._confirmations < self.config.confirm_cycles:
                    chosen, reason = current, "CONFIRMING"
                else:
                    reason = "CONFIRMED"
        if reason != "CONFIRMING":
            self._confirmations, self._challenger = 0, None
        return chosen, reason

    def step(self, world: World) -> Decision:
        if not math.isfinite(world.stamp) or world.stamp < 0:
            raise ValueError("invalid world timestamp")
        if (self._last_time is not None and world.stamp < self._last_time
                or self._epoch is not None and world.epoch != self._epoch):
            self.estimator.reset()
            self._cooldowns.clear()
            self.current = None
            self.session_id = uuid.uuid4().hex
            self._confirmations, self._challenger = 0, None
            self._recovering = False
            if self.goal_prior:
                self.goal_prior.reset()
        self._last_time, self._epoch = world.stamp, world.epoch
        self._cooldowns = {k: v for k, v in self._cooldowns.items() if v > world.stamp}
        intents = self.estimator.estimate(world)
        threats = self._threats(world)
        candidates, rejected = self._candidates(world, intents, threats)
        if self.goal_prior:
            candidates = self.goal_prior.rank(world, candidates, self.arena)
        chosen, arbitration = self._select(candidates, world.stamp)
        if self.current is None or self.current.key != chosen.key:
            self._next_id += 1
            self.current = Task(
                self.session_id, self._next_id, world.seq, world.stamp,
                world.stamp + self.config.task_lease, chosen.kind, chosen.zone_id,
                chosen.target_id, chosen.goal_id, chosen.goal, chosen.yaw,
                chosen.utility, chosen.confidence, chosen.urgent, chosen.reason,
            )
        else:
            self.current = replace(self.current, world_seq=world.seq,
                                   valid_until=world.stamp + self.config.task_lease,
                                   utility=chosen.utility, confidence=chosen.confidence,
                                   urgent=chosen.urgent, reason=chosen.reason)
        return Decision(self.current, intents, threats, tuple(candidates), tuple(rejected),
                        arbitration, world.mode,
                        dict(self.goal_prior.info) if self.goal_prior else {})
