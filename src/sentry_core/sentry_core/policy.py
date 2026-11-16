"""Constrained utility policy and hysteresis, shared by ROS and closed-loop training."""

import copy
from dataclasses import replace
import math
import uuid

from .candidates import (
    MIN_INTERCEPT_THREAT, SUBGOAL_ID, TRAVEL_SCALE, Goal, GoalGenerator, utility,
)
from .config import Config, validate_config
from .geometry import TacticalMap
from .inputs import RL_CATEGORIES, assess, view_inputs
from .intent import IntentEstimator
from .model import (
    Candidate, Decision, IntentType, Phase, Task, TaskType, Team, Threat, TrackStatus, World,
)


FEATURES = ("strategic", "cooperation", "urgency", "risk", "travel", "switching")


def safety_reason(world: World, config: Config, arena: TacticalMap) -> str | None:
    track = world.self_track
    if world.frame != config.frame:
        return "FRAME_MISMATCH"
    if (track is None or not track.usable
            or not -0.05 <= world.stamp - track.observed_at <= config.self_timeout
            or arena.origin(track.position) is None):
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


def _down(info, robot_id: int) -> bool:
    status = info.robot(Team.ALLY, robot_id)
    return status is not None and status.known_alive is False


class DecisionEngine:
    def __init__(self, config: Config, model=None, session_id: str | None = None,
                 baseline: bool = False, hysteresis: bool = True, goal_prior=None,
                 model_fallbacks: dict | None = None):
        self.config = validate_config(config)
        self.arena = TacticalMap(config)
        self.estimator = IntentEstimator(config, self.arena, model)
        self.generator = GoalGenerator(self.config, self.arena)
        self.session_id = session_id or uuid.uuid4().hex
        self.baseline, self.hysteresis = baseline, hysteresis
        self.goal_prior = goal_prior
        # Configured artifacts running in rule mode and why (learned.load_decision_models);
        # every decision repeats them in policy_info["model_fallbacks"].
        self.model_fallbacks = copy.deepcopy(model_fallbacks or {})
        self.current: Task | None = None
        self._next_id = 0
        self._last_time: float | None = None
        self._epoch: int | None = None
        self._challenger = None
        self._confirmations = 0
        # When the current candidate key was first held; a GOAL_MOVED re-issue keeps it.
        self._held_since: float | None = None
        # Whether the current task's goal is an RL sub-goal refining its rule goal.
        self._refined = False
        self._cooldowns: dict[str, float] = {}
        self._recovering = False
        self._degraded: list[str] = []

    def feedback(self, session_id: str, task_id: int, state: str, now: float) -> None:
        if (self.current is None or self.current.identity != (session_id, task_id)
                or not math.isfinite(now)):
            return
        if state in ("FAILED", "REJECTED") and self._refined:
            # The RL sub-goal failed, not the rule goal it refined: cool the sub-goal down
            # and resume the task at its rule goal, which needs a new task identity.
            self._cooldowns[SUBGOAL_ID] = now + self.config.cooldown
            self.current = None
        elif state in ("FAILED", "REJECTED"):
            self._cooldowns[self.current.goal_id] = now + self.config.cooldown
        elif state == "CANCELED":
            # A watchdog can cancel between planner ticks. Resuming requires a new
            # task identity, otherwise the executor correctly treats it as a replay.
            self.current = None

    def _unsafe(self, world: World) -> str | None:
        return safety_reason(world, self.config, self.arena)

    def _threats(self, world: World, view=None) -> tuple[Threat, ...]:
        config = self.config
        view = view or view_inputs(world, config)
        fade = max(1e-6, config.retention - config.predicted_age)
        result = []
        for zone in config.zones:
            if zone.intent != IntentType.DEFEND_BASE:
                continue
            best, best_priority, top = Threat(zone.name, 0.0, None, None), 0.0, 0.0
            for enemy in world.enemies:
                status = view.robot(Team.ENEMY, enemy.robot_id)
                if status is not None and status.known_alive is False:
                    continue
                if enemy.usable:
                    weight, speed = enemy.confidence, max(0.7, enemy.velocity.norm)
                elif enemy.status == TrackStatus.STALE and enemy.position.finite:
                    # Missing radar data: keep the last known position, fading until dropped.
                    weight = config.stale_enemy_weight * max(
                        0., 1 - (enemy.age - config.predicted_age) / fade)
                    speed = 0.7
                else:
                    continue
                route = self.arena.observed_zone_route(enemy.position, zone)
                distance = self.arena.observed_zone_distance(enemy.position, zone)
                if not route or weight <= 0:
                    continue
                eta = max(0., distance - zone.radius) / speed
                approach = 0.5
                if enemy.usable:
                    dx, dy = route[1].x - enemy.position.x, route[1].y - enemy.position.y
                    denom = math.hypot(dx, dy) * enemy.velocity.norm
                    if denom > 1e-6:
                        approach = max(0., (dx * enemy.velocity.x + dy * enemy.velocity.y) / denom)
                inside = (distance <= zone.radius
                          or bool(zone.polygon) and zone.contains(enemy.position))
                factor = 1.0 if inside else 0.55 + 0.45 * approach
                score = min(1., weight * zone.importance * math.exp(-eta / 8) * factor)
                top = max(top, score)
                ratio = status.hp_ratio if status is not None else None
                # Health re-orders targets only when known; unknown is neither low nor full.
                priority = score + (config.enemy_low_hp_bonus * (1 - ratio)
                                    if ratio is not None else 0.)
                if priority > best_priority:
                    best, best_priority = Threat(zone.name, score, enemy.robot_id, eta), priority
            if best.enemy_id is not None:
                enemy = next(t for t in world.enemies if t.robot_id == best.enemy_id)
                if not enemy.usable:
                    self._degraded.append(f"STALE_ENEMY:{best.enemy_id}")
                status = view.robot(Team.ENEMY, best.enemy_id)
                if status is None or status.hp_ratio is None:
                    self._degraded.append(f"ENEMY_HP_UNKNOWN:{best.enemy_id}")
            # Health only picks the target (and its ETA). The zone's threat, and so its
            # urgency, is always the largest per-enemy threat.
            result.append(replace(best, score=top))
        return tuple(result)

    def _candidates(self, world, intents, threats, report=None):
        config = self.config
        unsafe = self._unsafe(world)
        hold = Candidate(TaskType.HOLD, "", None, "", None, 0., (0.,) * 6,
                         0., 1., bool(unsafe), unsafe or "NO_FEASIBLE_TASK")
        if unsafe:
            return [hold], [unsafe]
        info, own = view_inputs(world, config), world.self_track
        report = report or assess(world, config, info)
        own_hp, own_ammo = info.self_hp, info.self_ammo
        can_supply = config.supply_enabled and info.supply_available is True
        need_supply = ((own_hp is not None and own_hp < config.critical_hp)
                       or (own_ammo is not None and own_ammo < 0.1))
        if self._recovering and own_hp is not None and own_hp >= config.recovered_hp:
            self._recovering = False
        self._recovering = can_supply and (need_supply or self._recovering)
        results, rejected = [hold], []
        allies = {t.robot_id: t for t in world.allies if t.usable}
        origin = self.arena.origin(own.position)
        extra = own.position.distance(origin)
        goals = self.generator.generate(world, intents, threats, preferred_only=self.baseline)
        lengths = [extra + d for d in self.arena.distances(origin, [g.position for g in goals])]
        own_region = self.arena.component(origin)
        context = {"info": info, "intents": intents, "allies": allies, "can_supply": can_supply,
                   "threats": {t.zone_id: t for t in threats},
                   "completeness": report.completeness}
        for goal, path_length in zip(goals, lengths):
            if self._cooldowns.get(goal.goal_id, 0) > world.stamp:
                rejected.append(f"{goal.goal_id}:FAILURE_COOLDOWN")
                continue
            if not math.isfinite(path_length) or (
                    goal.source == "preferred"
                    and self.arena.component(goal.position) != own_region):
                rejected.append(f"{goal.goal_id}:UNREACHABLE")
                continue
            clearance = min((t.position.distance(goal.position) for t in allies.values()),
                            default=10.)
            if clearance < config.ally_clearance:
                rejected.append(f"{goal.goal_id}:ALLY_OCCUPIED")
                continue
            candidate = self._score(goal, path_length, clearance, context, rejected)
            if candidate is not None:
                results.append(candidate)
        # Resource recovery is a hard team-configured survival constraint, not a model weight.
        if self._recovering:
            results = [c for c in results if c.kind in (TaskType.HOLD, TaskType.RECOVER)]
        # A preferred point with several tasks is filtered once per task; log it once.
        return results, list(dict.fromkeys(rejected))

    def _score(self, goal: Goal, path_length, clearance, context, rejected):
        config, info = self.config, context["info"]
        zone = next(z for z in config.zones if z.name == goal.zone_id)
        threat = context["threats"].get(zone.name, Threat(zone.name, 0., None, None))
        allies = context["allies"]
        coverage = sum(zone.contains(t.position) for t in allies.values())
        own_hp, own_ammo = info.self_hp, info.self_ammo
        hp = own_hp if own_hp is not None else 0.5
        target, confidence, urgent, reason = None, 1.0, False, ""
        value, cooperation, urgency, risk = 0., 0., 0., (1 - hp) * 0.3
        kind = goal.kind
        completeness = context["completeness"]
        if kind == TaskType.DEFEND:
            value = 0.42 + (0.12 if info.structures["own_outpost"] is False else 0.)
            cooperation = 0.18 / (1 + coverage)
            urgency, reason = 0.15 + 0.8 * threat.score, "AREA_UNCOVERED"
            boost = config.missing_enemy_defense_boost * (1 - completeness["enemies"])
            if zone.intent == IntentType.DEFEND_BASE and boost > 0:
                # Unknown enemy positions raise the priority of the key defense areas.
                urgency = min(1., urgency + boost)
                self._degraded.append(f"ENEMY_POSITIONS_MISSING:defense+{boost:.2f}")
        elif kind == TaskType.INTERCEPT:
            if self.baseline or threat.score < MIN_INTERCEPT_THREAT or hp < config.critical_hp:
                return None
            value, cooperation = zone.importance, 1 / (1 + coverage)
            urgency, target, confidence = threat.score, threat.enemy_id, threat.score
            urgent = threat.score >= config.urgent_threat
            risk += 0.12
            reason = "BASE_THREAT"
        elif kind == TaskType.SUPPORT:
            if own_hp is None or hp < 0.35 or own_ammo is None or own_ammo < 0.1:
                rejected.append(f"{goal.goal_id}:RESOURCES_UNCERTAIN_OR_LOW")
                return None
            attack = 1.0
            if zone.intent == IntentType.ATTACK_OUTPOST:
                structure = zone.structure or "enemy_outpost"
                alive = info.structures.get(structure)
                if alive is False:
                    rejected.append(f"{goal.goal_id}:TARGET_DESTROYED")
                    return None
                if alive is None:
                    # Alive but unconfirmed: still a valid target, with less attack weight.
                    attack = config.unconfirmed_attack_factor
                    self._degraded.append(f"STRUCTURE_UNCONFIRMED:{structure}")
            if goal.target_id is not None and _down(info, goal.target_id):
                rejected.append(f"{goal.goal_id}:ALLY_DOWN")
                return None
            supporting = [i for i in context["intents"] if i.valid and i.zone_id == zone.name
                          and i.robot_id in allies and not _down(info, i.robot_id)
                          and (goal.target_id is None or i.robot_id == goal.target_id)]
            if self.baseline and zone.intent == IntentType.ATTACK_OUTPOST:
                confidence, cooperation = 1., 0.7
            elif supporting:
                intent = max(supporting, key=lambda i: i.confidence)
                target, confidence = intent.robot_id, intent.confidence
                cooperation = confidence * min(1., clearance / 3.)
            else:
                return None
            scale = 0.5 + 0.5 * completeness["allies"]
            if scale < 1:
                self._degraded.append(f"ALLIES_INCOMPLETE:support*{scale:.2f}")
            value, urgency, risk = zone.importance * attack, 0.5, risk + 0.1
            cooperation *= scale
            reason = "TEAM_SUPPORT_HIGH" if not self.baseline else "FIXED_OUTPOST_RULE"
        elif kind == TaskType.RECOVER:
            if not context["can_supply"] or not (
                    self._recovering or (own_hp is not None and own_hp < 0.6)
                    or (own_ammo is not None and own_ammo < 0.2)):
                return None
            value, cooperation = 0.8, 0.4
            urgency = max(1 - hp, 1 - (own_ammo if own_ammo is not None else 1))
            urgent, reason = self._recovering, "LOW_RESOURCES"
        key = kind, zone.name, target, goal.goal_id
        switching = float(self.current is not None and self.current.key != key)
        features = (value, cooperation, urgency, min(1., risk),
                    min(1., path_length / TRAVEL_SCALE), switching)
        return Candidate(kind, zone.name, target, goal.goal_id, goal.position, goal.yaw, features,
                         utility(features, config.weights), confidence, urgent, reason)

    def _screen_subgoal(self, world, subgoal, rejected):
        """A model sub-goal still obeys failure cooldowns and teammate clearance."""
        if self._cooldowns.get(SUBGOAL_ID, 0) > world.stamp:
            reason = "FAILURE_COOLDOWN"
        elif any(t.usable and t.position.distance(subgoal) < self.config.ally_clearance
                 for t in world.allies):
            reason = "ALLY_OCCUPIED"
        else:
            return None
        rejected.append(f"{SUBGOAL_ID}:{reason}")
        return reason

    def _refine(self, world, chosen, rejected):
        """The RL sub-goal may refine the goal of the task the rules chose, never the task.

        It must lie in the task's zone or within ``goal_update_distance`` of the rule
        goal, be reachable and pass ``_screen_subgoal``. ``policy_info`` records whether
        it refined the chosen task this cycle (``subgoal_refined``) and why not otherwise
        (``subgoal_reason``). As for any goal, the task is only re-issued at a refined goal
        that moved more than ``goal_update_distance``.
        """
        current = self.current
        kept = (current.goal if current is not None and self._refined
                and current.key == chosen.key else None)
        subgoal, reason = self.goal_prior.subgoal_for(chosen, kept)
        if subgoal is not None:
            zone = next(z for z in self.config.zones if z.name == chosen.zone_id)
            origin = self.arena.origin(world.self_track.position) if world.self_track else None
            if not (zone.contains(subgoal)
                    or subgoal.distance(chosen.goal) <= self.config.goal_update_distance):
                subgoal, reason = None, "OUTSIDE_TASK_AREA"
            elif (origin is None
                  or not math.isfinite(self.arena.distances(origin, [subgoal])[0])):
                subgoal, reason = None, "UNREACHABLE"
            elif screened := self._screen_subgoal(world, subgoal, rejected):
                subgoal, reason = None, screened
        info = self.goal_prior.info
        info.update(subgoal_refined=subgoal is not None, subgoal_reason=reason)
        if subgoal is not None:
            info.update(subgoal=[subgoal.x, subgoal.y], subgoal_base=chosen.goal_id)
        return subgoal

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
            elif self.current.kind == TaskType.HOLD and self.current.urgent:
                # Only a safety reason makes HOLD urgent (_candidates). Once safe, that HOLD
                # gets no hold time, margin or confirmation: the best task starts at once,
                # and its own minimum hold with it.
                reason = "SAFETY_RECOVERED"
            elif not self.hysteresis:
                reason = "HYSTERESIS_DISABLED"
            elif now - self._held_since < self.config.min_hold:
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
        self._degraded = []
        view = view_inputs(world, self.config)
        report = assess(world, self.config, view)
        intents = self.estimator.estimate(world)
        threats = self._threats(world, view)
        candidates, rejected = self._candidates(world, intents, threats, report)
        missing_info = {"completeness": {k: round(v, 3) for k, v in report.completeness.items()},
                        "missing": list(report.missing)}
        if self.goal_prior:
            low = [c for c in RL_CATEGORIES
                   if report.completeness[c] < self.config.completeness_threshold]
            scale = (min(report.completeness[c] for c in RL_CATEGORIES)
                     / self.config.completeness_threshold if low else 1.0)
            reason = "INFO_INCOMPLETE:" + ",".join(low) if low else ""
            # A reduced blend re-weights the rule ranking; at 0 the model is only recorded
            # (SHADOW), so it cannot refine the chosen task's goal either.
            candidates = self.goal_prior.rank(world, candidates, self.arena, scale, reason)
            missing_info["rl_blend"] = {"configured": self.goal_prior.blend,
                                        "effective": self.goal_prior.blend * scale,
                                        "reason": reason}
            if low:
                self._degraded.append(f"RL_BLEND_REDUCED:{reason}")
        missing_info["degraded"] = list(dict.fromkeys(self._degraded))
        chosen, arbitration = self._select(candidates, world.stamp)
        subgoal = self._refine(world, chosen, rejected) if self.goal_prior else None
        if subgoal is not None:
            # Only the goal moves: kind, zone, target and goal id stay the rules' choice.
            chosen = replace(chosen, goal=subgoal)
        moved = (self.current is not None and self.current.key == chosen.key
                 and chosen.goal is not None and self.current.goal is not None
                 and chosen.goal.distance(self.current.goal) > self.config.goal_update_distance)
        if moved:
            arbitration = "GOAL_MOVED"
        fresh = self.current is None or self.current.key != chosen.key
        if fresh:
            # A moved goal gets a new task identity but keeps this hold start.
            self._held_since = world.stamp
        if fresh or moved:
            self._next_id += 1
            self.current = Task(
                self.session_id, self._next_id, world.seq, world.stamp,
                world.stamp + self.config.task_lease, chosen.kind, chosen.zone_id,
                chosen.target_id, chosen.goal_id, chosen.goal, chosen.yaw,
                chosen.utility, chosen.confidence, chosen.urgent, chosen.reason,
            )
            self._refined = subgoal is not None
        else:
            self.current = replace(self.current, world_seq=world.seq,
                                   valid_until=world.stamp + self.config.task_lease,
                                   utility=chosen.utility, confidence=chosen.confidence,
                                   urgent=chosen.urgent, reason=chosen.reason)
        policy_info = dict(self.goal_prior.info) if self.goal_prior else {}
        if self.model_fallbacks:
            policy_info["model_fallbacks"] = copy.deepcopy(self.model_fallbacks)
        return Decision(self.current, intents, threats, tuple(candidates), tuple(rejected),
                        arbitration, world.mode, policy_info, missing_info)
