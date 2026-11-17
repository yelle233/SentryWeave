"""Causal, role-agnostic navigation and 2027 combat-posture observations.

v1 (73 dims) covers official 2026 SQLite logs. v2 (85 dims) appends 12 strictly
onboard referee regular-link signals (ammo, barrel heat, hit recency, active posture,
switch cooldown and remaining 3x15s enhanced budgets). Enemy HP, enemy gold and
future match outcomes are deliberately absent from both versions.
"""

from dataclasses import asdict, dataclass
import math

import numpy as np

from .model import POSTURES, World


FEATURE_VERSION = 1
FEATURE_NAMES = (
    "ego.x", "ego.y", "ego.vx", "ego.vy", "ego.velocity_valid",
    "ego.hp_ratio", "ego.hp_valid", "time.remaining", "time.valid",
    "own_outpost.alive", "own_outpost.valid", "enemy_outpost.alive", "enemy_outpost.valid",
) + tuple(
    f"{side}.{i}.{field}"
    for side in ("ally", "enemy")
    for i in range(5)
    for field in ("dx", "dy", "vx", "vy", "velocity_valid", "present")
)

FEATURE_VERSION_V2 = 2
FEATURE_NAMES_V2 = FEATURE_NAMES + (
    "ego.ammo_ratio", "ego.ammo_valid",
    "ego.heat_ratio", "ego.heat_valid",
    "ego.hit_recency",
    "ego.posture.attack", "ego.posture.defense", "ego.posture.move",
    "ego.posture.cooldown_norm",
    "ego.enhanced_left.attack", "ego.enhanced_left.defense", "ego.enhanced_left.move",
)


def feature_names_for(version: int) -> tuple[str, ...]:
    if version == FEATURE_VERSION:
        return FEATURE_NAMES
    if version == FEATURE_VERSION_V2:
        return FEATURE_NAMES_V2
    raise ValueError(f"unsupported feature_version {version!r}")


@dataclass(frozen=True)
class NavigationSpec:
    width: float = 28.0
    height: float = 15.0
    origin_x: float = 0.0
    origin_y: float = 0.0
    speed_scale: float = 6.0
    horizon: int = 5
    regulation_seconds: float = 420.0
    hit_horizon_sec: float = 5.0
    posture_cooldown_sec: float = 5.0
    enhanced_budget_sec: float = 15.0

    def validate(self):
        if not all(
            math.isfinite(v) and v > 0
            for v in (
                self.width, self.height, self.speed_scale, self.regulation_seconds,
                self.hit_horizon_sec, self.posture_cooldown_sec, self.enhanced_budget_sec,
            )
        ):
            raise ValueError("navigation dimensions and scales must be finite and positive")
        if not all(math.isfinite(v) for v in (self.origin_x, self.origin_y)):
            raise ValueError("navigation origin must be finite")
        if not isinstance(self.horizon, int) or not 1 <= self.horizon <= 30:
            raise ValueError("navigation horizon must be an integer in [1,30]")
        return self

    def to_dict(self):
        data = asdict(self)
        # Keep canonical v1 keys compact when default v2-only scales are unchanged.
        for key, default in (
            ("hit_horizon_sec", 5.0),
            ("posture_cooldown_sec", 5.0),
            ("enhanced_budget_sec", 15.0),
        ):
            if data[key] == default:
                data.pop(key)
        return data


def encode_world(
    world: World,
    spec: NavigationSpec,
    mirror: bool = False,
    inputs=None,
    feature_version: int = FEATURE_VERSION,
) -> np.ndarray:
    """Encode ``world`` into v1 (73-dim) or v2 (85-dim) causal onboard features."""
    names = feature_names_for(feature_version)
    own = world.self_track
    if own is None or not own.usable:
        raise ValueError("navigation policy requires a valid self pose")
    sign = -1.0 if mirror else 1.0
    px = (own.position.x - spec.origin_x) / spec.width
    py = (own.position.y - spec.origin_y) / spec.height
    if mirror:
        px, py = 1.0 - px, 1.0 - py
    ref = world.referee
    if inputs is not None:
        hp, remaining = inputs.self_hp, inputs.remaining
        outposts = (
            inputs.structures.get("own_outpost"),
            inputs.structures.get("enemy_outpost"),
        )
    else:
        hp = ref.hp if ref else None
        remaining = ref.remaining if ref else None
        outposts = (
            (ref.own_outpost if ref else None),
            (ref.enemy_outpost if ref else None),
        )
    rem_norm = (
        min(1.0, max(0.0, remaining / spec.regulation_seconds))
        if remaining is not None
        else 0.0
    )
    values = [
        px,
        py,
        sign * own.velocity.x / spec.speed_scale if own.velocity_valid else 0.0,
        sign * own.velocity.y / spec.speed_scale if own.velocity_valid else 0.0,
        float(own.velocity_valid),
        hp if hp is not None else 0.0,
        float(hp is not None),
        rem_norm,
        float(remaining is not None),
    ]
    for alive in outposts:
        values.extend((float(alive) if alive is not None else 0.0, float(alive is not None)))
    for tracks in (world.allies, world.enemies):
        nearest = sorted(
            (track for track in tracks if track.usable and track is not own),
            key=lambda t: (own.position.distance(t.position), t.robot_id),
        )[:5]
        for track in nearest:
            values.extend((
                sign * (track.position.x - own.position.x) / spec.width,
                sign * (track.position.y - own.position.y) / spec.height,
                sign * track.velocity.x / spec.speed_scale if track.velocity_valid else 0.0,
                sign * track.velocity.y / spec.speed_scale if track.velocity_valid else 0.0,
                float(track.velocity_valid),
                1.0,
            ))
        values.extend([0.0] * (6 * (5 - len(nearest))))

    if feature_version == FEATURE_VERSION_V2:
        ammo = inputs.self_ammo if inputs is not None else (ref.ammo if ref else None)
        heat = inputs.self_heat_ratio if inputs is not None else None
        hit_stamp = inputs.self_last_hit_stamp if inputs is not None else None
        posture = inputs.self_posture if inputs is not None else None
        ready_at = inputs.self_posture_ready_at if inputs is not None else None
        enhanced_used = inputs.self_enhanced_used_sec if inputs is not None else None

        hit_recency = 0.0
        if hit_stamp is not None and -0.05 <= world.stamp - hit_stamp <= spec.hit_horizon_sec:
            hit_age = max(0.0, world.stamp - hit_stamp)
            hit_recency = min(1.0, max(0.0, 1.0 - hit_age / spec.hit_horizon_sec))
        cooldown_norm = 0.0
        if ready_at is not None and ready_at > world.stamp:
            cooldown_norm = min(
                1.0, max(0.0, (ready_at - world.stamp) / spec.posture_cooldown_sec)
            )
        if enhanced_used is not None and len(enhanced_used) == 3:
            enh_left = [
                min(
                    1.0,
                    max(0.0, (spec.enhanced_budget_sec - float(u)) / spec.enhanced_budget_sec),
                )
                for u in enhanced_used
            ]
        else:
            enh_left = [0.0, 0.0, 0.0]

        values.extend((
            float(ammo) if ammo is not None else 0.0,
            float(ammo is not None),
            float(heat) if heat is not None else 0.0,
            float(heat is not None),
            hit_recency,
            *(1.0 if posture == name else 0.0 for name in POSTURES),
            cooldown_norm,
            *enh_left,
        ))

    result = np.asarray(values, dtype=np.float32)
    if result.shape != (len(names),) or not np.isfinite(result).all():
        raise ValueError("invalid navigation observation")
    return result
