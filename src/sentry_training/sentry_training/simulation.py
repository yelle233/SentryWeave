"""Fast tactical simulator: causal noisy observations, actual kinematic navigation.

Reward is computed from positions, coverage and resources, never from the name
of the selected task. The physics and referee are deliberately simplified.
Gazebo/Nav2 is a separate validation layer.
"""

from dataclasses import asdict
import math
from pathlib import Path
import time

import numpy as np

from sentry_core.config import Config
from sentry_core.execution import Executor
from sentry_core.geometry import TacticalMap
from sentry_core.model import IntentType, Observation, Phase, Referee, TaskType, Team, Vec2
from sentry_core.policy import DecisionEngine, safety_reason
from sentry_core.serde import json_line, world_to_dict
from sentry_core.tracking import WorldModel


SCENARIOS = ("team_push", "base_raid", "conflict", "ambiguous", "dropout", "blocked",
             "phase_change", "low_hp", "localization_loss", "id_jump")


class Scenario:
    """Reusable scenario driver for offline simulation and ROS sensor injection."""

    def __init__(self, config: Config, name: str, seed: int):
        if name not in SCENARIOS:
            raise ValueError(f"unknown scenario {name}; choose from {SCENARIOS}")
        self.config, self.name = config, name
        self.arena = TacticalMap(config)
        self.rng = np.random.default_rng(seed)
        self.base = next(z.center for z in config.zones if z.intent == IntentType.DEFEND_BASE)
        self.outpost = next(z.center for z in config.zones if z.intent == IntentType.ATTACK_OUTPOST)
        self.supply = next(z.center for z in config.zones if z.intent == IntentType.RETREAT)
        defense = next(p.position for p in config.points if TaskType.DEFEND in p.tasks)
        self.ally = self.arena.advance(defense, self.outpost, 1.5)
        self.enemy = self.outpost
        self.self_start = self.arena.advance(self.base, defense, 1.0)
        self.ally_speed = float(self.rng.uniform(0.9, 1.4))
        self.enemy_speed = float(self.rng.uniform(0.8, 1.3))
        self.self_speed = config.nominal_speed * float(self.rng.uniform(0.85, 1.1))
        self.hp = 0.9
        self._hp_event = False
        self._id_event = False
        self.has_ally = name != "base_raid"
        self.has_enemy = name in ("base_raid", "conflict")

    def observe(self, elapsed: float, dt: float, self_position: Vec2):
        now = 1.0 + elapsed
        old_ally, old_enemy = self.ally, self.enemy
        if self.name == "ambiguous":
            a, b, c, d = self.config.bounds
            point = Vec2((a + c) * 0.6, d - 1.3)
            if self.arena.legal(point):
                self.ally = point
        else:
            if self.name == "id_jump" and elapsed >= 10 and not self._id_event:
                self.ally = self.supply
                self._id_event = True
            self.ally = self.arena.advance(self.ally, self.outpost, self.ally_speed * dt)
        if self.has_enemy and (self.name == "base_raid" or elapsed >= 8):
            self.enemy = self.arena.advance(self.enemy, self.base, self.enemy_speed * dt)
        if self.name == "low_hp" and elapsed >= 8 and not self._hp_event:
            self.hp, self._hp_event = 0.18, True
        if self.has_enemy and self_position.distance(self.enemy) < 3:
            self.hp = max(0.01, self.hp - 0.006 * dt)
        if self.config.supply_enabled and self_position.distance(self.supply) < 0.8:
            self.hp = min(1., self.hp + 0.1 * dt)
        phase = Phase.FINISHED if self.name == "phase_change" and elapsed >= 24 else Phase.RUNNING
        referee = Referee(now, phase, self.hp, 0.85, True,
                          not (self.name == "phase_change" and elapsed >= 12),
                          self.config.supply_enabled, max(0., 420 - elapsed))
        observations = []
        if not (self.name == "localization_loss" and 10 <= elapsed < 16):
            observations.append(Observation(self.config.self_id, Team.ALLY, self_position, now,
                                            velocity=Vec2(0., 0.)))
        for enabled, robot, team, position, old in (
            (self.has_ally, 1, Team.ALLY, self.ally, old_ally),
            (self.has_enemy, 11, Team.ENEMY, self.enemy, old_enemy),
        ):
            if (not enabled or self.rng.random() < 0.04
                    or self.name == "dropout" and elapsed >= 12):
                continue
            delay = float(self.rng.uniform(0., 0.12))
            velocity = Vec2((position.x - old.x) / dt, (position.y - old.y) / dt)
            observation = Vec2(position.x - velocity.x * delay + float(self.rng.normal(0, 0.025)),
                               position.y - velocity.y * delay + float(self.rng.normal(0, 0.025)))
            if self.arena.legal(observation):
                observations.append(Observation(robot, team, observation, now - delay,
                                                float(self.rng.uniform(0.88, 1.0))))
        return observations, referee

    def coverage(self, position: Vec2, referee: Referee):
        support_needed = float(self.has_ally and self.ally.distance(self.outpost) < 6
                               and referee.enemy_outpost is True and referee.phase == Phase.RUNNING)
        enemy_distance = self.arena.distance(self.enemy, self.base)
        defense_needed = (max(0., 1 - enemy_distance / 9) if self.has_enemy else 0.)
        support_points = [p.position for p in self.config.points
                          if TaskType.SUPPORT in p.tasks
                          and next(z for z in self.config.zones if z.name == p.zone).intent
                          == IntentType.ATTACK_OUTPOST]
        defense_points = [p.position for p in self.config.points
                          if TaskType.DEFEND in p.tasks or TaskType.INTERCEPT in p.tasks]
        support_score = math.exp(-min(position.distance(p) for p in support_points) / 3)
        defense_score = math.exp(-min(position.distance(p) for p in defense_points) / 3)
        if position.distance(self.ally) < self.config.ally_clearance:
            support_score = 0.
        return support_needed, support_score, defense_needed, defense_score


def rollout(config: Config, scenario: str, seed: int, duration=30., dt=0.5,
            policy="rule", model=None, trace_path=None, goal_prior=None):
    if duration <= 0 or dt <= 0:
        raise ValueError("duration and dt must be positive")
    if policy not in ("fixed", "rule", "learned", "no_hysteresis", "rl"):
        raise ValueError("unknown policy")
    if policy == "learned" and model is None:
        raise ValueError("learned policy requires an intent model")
    if (policy == "rl") != (goal_prior is not None):
        raise ValueError("rl policy requires a goal prior and its own evaluation label")
    scene = Scenario(config, scenario, seed)
    tracker = WorldModel(config)
    engine = DecisionEngine(config, model if policy in ("learned", "rl") else None,
                            session_id="eval", baseline=policy == "fixed",
                            hysteresis=policy != "no_hysteresis", goal_prior=goal_prior)
    executor = Executor(engine.arena)
    position = scene.self_start
    previous_position = position
    previous_identity = None
    task_sequence, runtimes = [], []
    switches = illegal = goals = failures = arrivals = unsafe_motion = stale_support = 0
    distance = reward = support_time = supported = defense_time = defended = 0.
    response_start = response_delay = None
    blocked = False
    rl_used = rl_rank_changes = 0
    rl_fallbacks = {}
    steps = int(math.ceil(duration / dt))
    trace = None
    if trace_path:
        Path(trace_path).parent.mkdir(parents=True, exist_ok=True)
        trace = open(trace_path, "w", encoding="utf-8")
    try:
        for index in range(steps):
            elapsed, now = index * dt, 1.0 + index * dt
            # Radar/localization run at >=10 Hz independently of the 2 Hz planner.
            # Sampling sensors only once per decision created artificial track loss.
            subdivisions = max(1, math.ceil(dt / 0.1)) if index else 1
            sensor_dt = dt / subdivisions if index else 0.1
            travelled_last = engine.arena.distance(previous_position, position)
            for sample in range(subdivisions):
                sample_elapsed = elapsed - dt + (sample + 1) * sensor_dt if index else elapsed
                sample_position = engine.arena.advance(
                    previous_position, position, travelled_last * (sample + 1) / subdivisions
                )
                observations, referee = scene.observe(sample_elapsed, sensor_dt, sample_position)
                for observation in observations:
                    tracker.observe(observation, 1.0 + sample_elapsed)
                tracker.update_referee(referee)
            world = tracker.snapshot(now)
            start = time.perf_counter()
            decision = engine.step(world)
            runtimes.append((time.perf_counter() - start) * 1000)
            task = decision.task
            if goal_prior is not None:
                if decision.policy_info.get("used", False):
                    rl_used += 1
                    pool = [c for c in decision.candidates if c.urgent] or decision.candidates
                    original = max(pool, key=lambda c: (c.rule_utility if c.rule_utility is not None
                                                        else c.utility, c.goal_id))
                    ranked = max(pool, key=lambda c: (c.utility, c.goal_id))
                    rl_rank_changes += int(original.key != ranked.key)
                else:
                    reason = decision.policy_info.get("status", "UNKNOWN")
                    rl_fallbacks[reason] = rl_fallbacks.get(reason, 0) + 1
            changed = task.identity != previous_identity
            if changed:
                switches += int(previous_identity is not None)
                goals += int(task.goal is not None)
                task_sequence.append(f"{task.kind.name}:{task.zone_id}:{task.goal_id}")
                previous_identity = task.identity
            executor.safety(safety_reason(world, config, engine.arena) is None, now, now)
            executor.submit(task, now, now)
            executor.tick(now, now)
            feedback = []

            def dispatch():
                nonlocal blocked, failures, arrivals
                for _ in range(20):
                    commands = executor.drain()
                    if not commands:
                        return
                    for command in commands:
                        if command.kind == "PLAN":
                            reachable = bool(engine.arena.route(position, command.task.goal))
                            if scenario == "blocked" and command.task.kind == TaskType.SUPPORT and not blocked:
                                reachable, blocked = False, True
                            executor.planned(command.token, reachable, now, now)
                        elif command.kind == "SEND":
                            executor.accepted(command.token, True, now, now)
                        elif command.kind == "CANCEL":
                            executor.cancel_ack(command.token, True, now, now)
                            executor.result(command.token, "CANCELED", now, now)
                        elif command.kind == "FEEDBACK" and command.task:
                            feedback.append({"session_id": command.task.session_id,
                                             "task_id": command.task.task_id, "state": command.state,
                                             "stamp": now, "reason": command.reason})
                            engine.feedback(command.task.session_id, command.task.task_id,
                                            command.state, now)
                            failures += int(command.state == "FAILED")
                            arrivals += int(command.state == "SUCCEEDED")
                raise RuntimeError("executor command loop did not settle")

            dispatch()
            if task.goal is not None and not engine.arena.legal(task.goal):
                illegal += 1
            old = position
            previous_position = old
            if executor.motion_allowed(now, now):
                position = engine.arena.advance(position, executor.current.goal, scene.self_speed * dt)
                if position.distance(executor.current.goal) < 0.05:
                    executor.result(executor.token, "SUCCEEDED", now, now)
                    dispatch()
            travelled = engine.arena.distance(old, position)
            distance += travelled
            unsafe_motion += int(safety_reason(world, config, engine.arena) is not None and travelled > 1e-6)
            stale_support += int(task.kind == TaskType.SUPPORT and policy != "fixed"
                                 and not any(t.usable and t.robot_id == task.target_id for t in world.allies))
            sn, ss, dn, ds = scene.coverage(position, referee)
            support_time += sn * dt
            supported += sn * ss * dt
            defense_time += dn * dt
            defended += dn * ds * dt
            reward += (2 * sn * ss + 4 * dn * ds + 0.2 * scene.hp
                       - 0.04 * travelled - 0.12 * int(changed)
                       - 5 * illegal - 5 * unsafe_motion)
            if dn > 0.25 and response_start is None:
                response_start = now
            if (response_start is not None and response_delay is None
                    and task.kind in (TaskType.INTERCEPT, TaskType.DEFEND)):
                response_delay = now - response_start
            if trace:
                trace.write(json_line({
                    "schema_version": 1, "episode_id": f"{scenario}-{seed}",
                    "map_fingerprint": config.map_fingerprint,
                    "world": world_to_dict(world), "decision": asdict(decision),
                    "feedback": feedback, "position_after": asdict(position),
                }) + "\n")
    finally:
        if trace:
            trace.close()
    return {
        "scenario": scenario, "seed": seed, "policy": policy, "steps": steps,
        "reward": reward / steps, "switches": switches, "navigation_goals": goals,
        "navigation_arrivals": arrivals, "navigation_failures": failures,
        "illegal_goals": illegal, "unsafe_motion_steps": unsafe_motion,
        "stale_support_steps": stale_support, "distance_travelled": distance,
        "support_opportunity_sec": support_time,
        "support_coverage": supported / support_time if support_time else None,
        "defense_opportunity_sec": defense_time,
        "defense_coverage": defended / defense_time if defense_time else None,
        "threat_response_sec": response_delay, "final_hp": scene.hp,
        "decision_p95_ms": float(np.percentile(runtimes, 95)),
        "task_sequence": task_sequence,
        "rl_used_steps": rl_used, "rl_rank_changes": rl_rank_changes,
        "rl_fallbacks": rl_fallbacks,
    }
