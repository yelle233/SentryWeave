"""Shared feature extraction and causal intent filtering.

The optional learned model implements ``posterior(features, previous)``.
Only new observations advance its posterior; republished worlds do not.
"""

import math

import numpy as np

from .config import Config
from .geometry import TacticalMap
from .model import Intent, IntentType, Track, World


ROLES = (IntentType.ATTACK_OUTPOST, IntentType.DEFEND_BASE,
         IntentType.CONTEST_AREA, IntentType.RETREAT)
FEATURE_NAMES = ("speed", "quality", "history", "velocity_valid") + tuple(
    f"{role.name.lower()}.{feature}" for role in ROLES
    for feature in ("alignment", "progress", "inside", "proximity", "reachable")
)
FEATURE_VERSION = 1


def softmax(logits: np.ndarray) -> np.ndarray:
    values = np.exp(logits - np.max(logits))
    return values / values.sum()


class IntentEstimator:
    def __init__(self, config: Config, arena: TacticalMap, model=None):
        self.config, self.arena, self.model = config, arena, model
        self._memory: dict[int, tuple[int, float, np.ndarray]] = {}

    def reset(self) -> None:
        self._memory.clear()

    def features(self, track: Track) -> tuple[float, ...]:
        span = (track.history[-1].stamp - track.history[0].stamp
                if len(track.history) > 1 else 0.0)
        values = [min(track.velocity.norm / self.config.max_speed, 1.0),
                  track.confidence, min(span / 2.0, 1.0), float(track.velocity_valid)]
        for role in ROLES:
            options = []
            for zone in self.config.zones:
                if zone.intent != role:
                    continue
                path = self.arena.observed_zone_route(track.position, zone)
                distance = self.arena.observed_zone_distance(track.position, zone)
                if not path:
                    options.append((0.5, 0.5, 0.0, 0.0, 0.0))
                    continue
                direction = path[1]
                dx, dy = direction.x - track.position.x, direction.y - track.position.y
                denom = math.hypot(dx, dy) * track.velocity.norm
                alignment = (0.5 + 0.5 * (dx * track.velocity.x + dy * track.velocity.y) / denom
                             if denom > 1e-6 and track.velocity.norm > 0.15 else 0.5)
                progress = 0.5
                if span > 0.1:
                    old = self.arena.observed_zone_distance(track.history[0].position, zone)
                    if math.isfinite(old):
                        progress = np.clip(
                            0.5 + (old - distance) / (2 * self.config.nominal_speed * span),
                            0, 1,
                        )
                inside = float(zone.contains(track.position))
                options.append((alignment, progress, inside, math.exp(-distance / 5.0), 1.0))
            values.extend(max(options, key=lambda f: f[2] + f[3]))
        return tuple(float(v) for v in values)

    def _rule_posterior(self, features: tuple[float, ...]) -> np.ndarray:
        f = np.asarray(features)
        moving = f[0] * self.config.max_speed > 0.15
        scores = np.zeros(6)
        inside_any = max(f[6::5]) > 0
        scores[IntentType.UNKNOWN] = (
            (1.2 if moving or inside_any else 4.5) + 2 * (1 - f[1])
        )
        scores[IntentType.TRANSIT] = 2.2 if moving else 0.2
        for i, role in enumerate(ROLES):
            alignment, progress, inside, proximity, reachable = f[4 + 5 * i:9 + 5 * i]
            scores[role] = ((2.2 * alignment + 2 * progress + 2.5 * inside
                             + 0.5 * proximity) if reachable else -10.0)
        return softmax(scores)

    def estimate(self, world: World) -> tuple[Intent, ...]:
        results = []
        live_ids = {t.robot_id for t in world.allies}
        self._memory = {k: v for k, v in self._memory.items() if k in live_ids}
        for track in world.allies:
            features = self.features(track)
            if (not track.usable or len(track.history) < 2
                    or track.history[-1].stamp - track.history[0].stamp < 0.35):
                self._memory.pop(track.robot_id, None)
                results.append(Intent(track.robot_id, IntentType.UNKNOWN,
                                      (1., 0., 0., 0., 0., 0.), 0.0, features=features))
                continue
            memory = self._memory.get(track.robot_id)
            if memory and memory[0] != track.generation:
                memory = None
            previous = memory[2] if memory else None
            if memory and memory[1] == track.observed_at:
                posterior = memory[2]
            elif self.model is not None:
                posterior = self.model.posterior(features, previous)
            else:
                posterior = self._rule_posterior(features)
                if previous is not None:
                    posterior = 0.55 * previous + 0.45 * posterior
            posterior = posterior / posterior.sum()
            self._memory[track.robot_id] = track.generation, track.observed_at, posterior
            kind = IntentType(int(np.argmax(posterior)))
            first, second = sorted(posterior, reverse=True)[:2]
            confidence = track.confidence * max(0., (first - second) / (1 - second + 1e-9))
            valid = bool(kind in ROLES and confidence >= self.config.intent_threshold)
            zones = [z for z in self.config.zones if z.intent == kind]
            zone = min(zones, key=lambda z: self.arena.observed_zone_distance(track.position, z),
                       default=None)
            results.append(Intent(track.robot_id, kind, tuple(float(p) for p in posterior),
                                  float(confidence), zone.name if zone else "", valid, features))
        return tuple(results)
