"""RMUC 2027 sentry posture state machine (Table 5-28) and reactive anti-aim chassis spin."""

from dataclasses import dataclass
import math

from .config import Config
from .inputs import InputView
from .model import POSTURES, TaskType, Threat, Vec2, World


@dataclass(frozen=True)
class SpinDecision:
    enabled: bool
    rate_rad_s: float
    reason: str

    def to_dict(self) -> dict:
        return {"enabled": self.enabled, "rate_rad_s": round(self.rate_rad_s, 3),
                "reason": self.reason}


@dataclass(frozen=True)
class PostureDecision:
    posture: str
    enhanced: bool
    cooldown_remaining: float
    posture_time_sec: tuple[float, float, float]
    enhanced_used_sec: tuple[float, float, float]
    reason: str

    def to_dict(self) -> dict:
        return {
            "posture": self.posture,
            "enhanced": self.enhanced,
            "cooldown_remaining": round(self.cooldown_remaining, 3),
            "posture_time_sec": [round(v, 2) for v in self.posture_time_sec],
            "enhanced_used_sec": [round(v, 2) for v in self.enhanced_used_sec],
            "reason": self.reason,
        }


def _nearest_usable_enemy_distance(world: World) -> float:
    own = world.self_track
    if own is None or not own.usable:
        return math.inf
    return min((own.position.distance(e.position) for e in world.enemies if e.usable),
               default=math.inf)


class SpinPolicy:
    """Reactive chassis spin: instant on hit or close combat, alternating sign, never unsafe."""

    def __init__(self, config: Config):
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._sign = 1
        self._last_flip_at: float | None = None
        self._last_seen_hit: float | None = None
        self._hit_recover_latched = False

    def decide(
        self,
        world: World,
        view: InputView,
        task_kind: TaskType,
        task_goal: Vec2 | None,
        task_urgent: bool,
        unsafe: str | None,
    ) -> SpinDecision:
        if unsafe is not None:
            self._hit_recover_latched = False
            return SpinDecision(False, 0.0, f"UNSAFE:{unsafe}")

        now = world.stamp
        hit_stamp = view.self_last_hit_stamp
        hit_age = (
            (now - hit_stamp)
            if (hit_stamp is not None and now >= hit_stamp - 0.05)
            else None
        )
        if hit_age is not None:
            hit_age = max(0.0, hit_age)

        if task_kind == TaskType.RECOVER:
            if hit_age is not None and hit_age <= self.config.spin_recover_hold_sec:
                self._hit_recover_latched = True
        else:
            self._hit_recover_latched = False

        enemy_dist = _nearest_usable_enemy_distance(world)
        own_pos = world.self_track.position if world.self_track else None
        goal_dist = (own_pos.distance(task_goal)
                     if own_pos is not None and task_goal is not None else 0.0)

        reason = ""
        if hit_age is not None and hit_age <= self.config.spin_hold_sec:
            reason = "REACTIVE_HIT"
        elif enemy_dist <= self.config.spin_combat_range:
            reason = "COMBAT_PROXIMITY"
        elif task_kind == TaskType.INTERCEPT and task_urgent:
            reason = "URGENT_INTERCEPT"
        elif task_kind == TaskType.RECOVER and self._hit_recover_latched:
            reason = "HIT_RECOVER"

        if not reason:
            stop_reason = ("UNTHREATENED_TRANSIT"
                           if goal_dist > self.config.transit_distance_m else "CALM")
            return SpinDecision(False, 0.0, stop_reason)

        new_hit = (hit_stamp is not None and (
            self._last_seen_hit is None
            or hit_stamp - self._last_seen_hit >= self.config.spin_hit_flip_min_gap_sec
        ))
        if new_hit and self._last_flip_at is not None and self._last_seen_hit is not None:
            self._sign = -self._sign
            self._last_flip_at = now
        elif self._last_flip_at is None:
            self._last_flip_at = now
        elif now - self._last_flip_at >= self.config.spin_flip_sec:
            self._sign = -self._sign
            self._last_flip_at = now
        if hit_stamp is not None and (self._last_seen_hit is None
                                      or hit_stamp > self._last_seen_hit):
            self._last_seen_hit = hit_stamp

        return SpinDecision(True, float(self._sign * self.config.spin_speed_rad_s), reason)


class PosturePolicy:
    """RMUC 2027 sentry posture state machine (5s cooldown, 180s decay, 3x15s enhanced cap)."""

    def __init__(self, config: Config):
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._posture = "move"
        self._enhanced = False
        self._ready_at = 0.0
        self._posture_time = [0.0, 0.0, 0.0]
        self._enhanced_used = [0.0, 0.0, 0.0]
        self._last_stamp: float | None = None

    def _sync(self, now: float, view: InputView) -> None:
        dt = (max(0.0, now - self._last_stamp)
              if self._last_stamp is not None and now >= self._last_stamp else 0.0)
        self._last_stamp = now
        idx = POSTURES.index(self._posture)
        self._posture_time[idx] += dt
        if self._enhanced:
            self._enhanced_used[idx] = min(
                self.config.enhanced_budget_sec, self._enhanced_used[idx] + dt
            )
            if self._enhanced_used[idx] >= self.config.enhanced_budget_sec - 1e-6:
                self._enhanced = False

        if view.self_posture in POSTURES:
            self._posture = view.self_posture
        if view.self_posture_enhanced is not None:
            self._enhanced = bool(view.self_posture_enhanced)
        if view.self_posture_ready_at is not None:
            self._ready_at = float(view.self_posture_ready_at)
        if view.self_posture_time_sec is not None:
            self._posture_time = [float(x) for x in view.self_posture_time_sec]
        if view.self_enhanced_used_sec is not None:
            self._enhanced_used = [float(x) for x in view.self_enhanced_used_sec]
            cur = POSTURES.index(self._posture)
            if self._enhanced_used[cur] >= self.config.enhanced_budget_sec - 1e-6:
                self._enhanced = False

    def _has_enhanced_budget(self, posture: str) -> bool:
        idx = POSTURES.index(posture)
        left = self.config.enhanced_budget_sec - self._enhanced_used[idx]
        return left > self.config.posture_enhanced_margin_sec

    def _is_decayed(self, posture: str) -> bool:
        return self._posture_time[POSTURES.index(posture)] >= self.config.posture_decay_sec

    def decide(
        self,
        world: World,
        view: InputView,
        task_kind: TaskType,
        task_goal: Vec2 | None,
        task_urgent: bool,
        threats: tuple[Threat, ...],
        unsafe: str | None,
        rl_preference: dict | None = None,
        effective_blend: float = 0.0,
    ) -> PostureDecision:
        now = world.stamp
        self._sync(now, view)
        if unsafe is not None:
            self._enhanced = False
            cooldown = max(0.0, self._ready_at - now)
            return PostureDecision(
                self._posture, False, cooldown, tuple(self._posture_time),
                tuple(self._enhanced_used), "UNSAFE_HOLD",
            )

        cooldown = max(0.0, self._ready_at - now)
        if cooldown > 1e-6:
            return PostureDecision(
                self._posture, self._enhanced, cooldown, tuple(self._posture_time),
                tuple(self._enhanced_used), "COOLDOWN_LOCKED",
            )

        hit_age = ((now - view.self_last_hit_stamp)
                   if view.self_last_hit_stamp is not None
                   and now >= view.self_last_hit_stamp - 0.05 else None)
        recently_hit = (
            hit_age is not None
            and max(0.0, hit_age) <= self.config.posture_hit_window_sec
        )
        hp = view.self_hp
        heat = view.self_heat_ratio
        ammo = view.self_ammo
        can_shoot = ammo is not None and ammo > 0.05
        enemy_dist = _nearest_usable_enemy_distance(world)
        in_combat = enemy_dist <= self.config.posture_attack_range
        own_pos = world.self_track.position if world.self_track else None
        goal_dist = (own_pos.distance(task_goal)
                     if own_pos is not None and task_goal is not None else 0.0)
        max_threat = max((t.score for t in threats), default=0.0)

        # 1. Hard survival override: never overridden by RL.
        critical_retreat = (
            (task_kind == TaskType.RECOVER
             and (recently_hit or (hp is not None and hp < self.config.critical_hp)))
            or (hp is not None and hp < 0.35 and (recently_hit or enemy_dist <= 4.5))
        )
        if critical_retreat:
            want_enh = (recently_hit and (hp is None or hp < 0.45)
                        and self._has_enhanced_budget("defense"))
            return self._apply(now, "defense", want_enh, "SURVIVAL_DEFENSE")

        # 2. Rule posture scores & candidate enhanced trigger.
        scores = {"attack": 0.1, "defense": 0.1, "move": 0.2}
        rule_enh = {"attack": False, "defense": False, "move": False}
        reason = "RESERVE_BUDGET_MOVE"

        if heat is not None and heat >= 0.75:
            scores["attack"] = 0.88
            rule_enh["attack"] = in_combat and can_shoot and heat >= 0.80
            reason = "THERMAL_RELIEF_ATTACK"
        elif goal_dist > self.config.transit_distance_m and not recently_hit and not in_combat:
            scores["move"] = 0.85
            rule_enh["move"] = ((task_urgent or task_kind == TaskType.INTERCEPT)
                                and goal_dist > 7.0)
            reason = "TRANSIT_MOVE"
        elif in_combat and can_shoot:
            if recently_hit and ((hp is not None and hp < 0.55)
                                 or (self._is_decayed("attack")
                                     and not self._is_decayed("defense"))):
                scores["defense"] = 0.82
                scores["attack"] = 0.42
                rule_enh["defense"] = hp is not None and hp < 0.45
                reason = "UNDER_FIRE_DEFENSE"
            else:
                scores["attack"] = 0.84
                scores["defense"] = 0.38
                rule_enh["attack"] = (
                    task_urgent and enemy_dist <= 6.0 and heat is not None and heat >= 0.55
                )
                reason = "COMBAT_ATTACK"
        elif recently_hit or (max_threat >= 0.35 and not self._is_decayed("defense")):
            scores["defense"] = 0.78
            reason = "THREAT_DEFENSE"
        else:
            # Unthreatened patrol/hold stays in "move" to conserve the 180s budgets of
            # "attack" (3x cooling) and "defense" (50% DR) for real engagements.
            scores["move"] = 0.75
            reason = "RESERVE_BUDGET_MOVE"

        rule_choice = max(POSTURES, key=lambda p: (scores[p], p == self._posture))
        blend = min(1.0, max(0.0, float(effective_blend)))
        if rl_preference is not None and blend > 0.0:
            probs = rl_preference.get("probs") or {}
            for name in POSTURES:
                if name in probs:
                    scores[name] = (1.0 - blend) * scores[name] + blend * float(probs[name])

        chosen = max(POSTURES, key=lambda p: (scores[p], p == self._posture))
        if chosen != rule_choice:
            reason = f"RL_BLENDED:{reason}"
        engaged = (recently_hit or in_combat
                   or (chosen == "move" and goal_dist > 7.0 and task_urgent))
        enh_score = float(rule_enh[chosen])
        if rl_preference is not None and blend > 0.0 and "enhanced_prob" in rl_preference:
            enh_score = (1.0 - blend) * enh_score + blend * float(rl_preference["enhanced_prob"])
        want_enh = bool(engaged and enh_score >= 0.5 and self._has_enhanced_budget(chosen))
        return self._apply(now, chosen, want_enh, reason)

    def _apply(
        self, now: float, wanted: str, wanted_enhanced: bool, reason: str
    ) -> PostureDecision:
        if wanted != self._posture or wanted_enhanced != self._enhanced:
            self._posture = wanted
            self._enhanced = wanted_enhanced
            self._ready_at = now + self.config.posture_cooldown_sec
        cooldown = max(0.0, self._ready_at - now)
        return PostureDecision(
            self._posture, self._enhanced, cooldown, tuple(self._posture_time),
            tuple(self._enhanced_used), reason,
        )
