"""Causal, role-agnostic navigation observations shared by SQLite and ROS.

Only position/velocity, ego health, regulation time and outpost validity are
used. Enemy health, economy, future outcome and remaining ammunition inferred
from cumulative shots are deliberately absent.
"""

from dataclasses import asdict, dataclass
import math

import numpy as np

from .model import World


FEATURE_VERSION = 1
FEATURE_NAMES = (
    "ego.x", "ego.y", "ego.vx", "ego.vy", "ego.velocity_valid",
    "ego.hp_ratio", "ego.hp_valid", "time.remaining", "time.valid",
    "own_outpost.alive", "own_outpost.valid", "enemy_outpost.alive", "enemy_outpost.valid",
) + tuple(f"{side}.{i}.{field}" for side in ("ally", "enemy") for i in range(5)
          for field in ("dx", "dy", "vx", "vy", "velocity_valid", "present"))


@dataclass(frozen=True)
class NavigationSpec:
    width: float = 28.
    height: float = 15.
    origin_x: float = 0.
    origin_y: float = 0.
    speed_scale: float = 6.
    horizon: int = 5
    regulation_seconds: float = 420.

    def validate(self):
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.width, self.height, self.speed_scale, self.regulation_seconds)):
            raise ValueError("navigation dimensions and scales must be finite and positive")
        if not all(math.isfinite(v) for v in (self.origin_x, self.origin_y)):
            raise ValueError("navigation origin must be finite")
        if not isinstance(self.horizon, int) or not 1 <= self.horizon <= 30:
            raise ValueError("navigation horizon must be an integer in [1,30]")
        return self

    def to_dict(self):
        return asdict(self)


def encode_world(world: World, spec: NavigationSpec, mirror=False, inputs=None) -> np.ndarray:
    """``inputs`` is an optional InputView; without it the legacy Referee is read."""
    own = world.self_track
    if own is None or not own.usable:
        raise ValueError("navigation policy requires a valid self pose")
    sign = -1. if mirror else 1.
    px = (own.position.x - spec.origin_x) / spec.width
    py = (own.position.y - spec.origin_y) / spec.height
    if mirror:
        px, py = 1. - px, 1. - py
    ref = world.referee
    if inputs is not None:
        hp, remaining = inputs.self_hp, inputs.remaining
        outposts = (inputs.structures.get("own_outpost"), inputs.structures.get("enemy_outpost"))
    else:
        hp = ref.hp if ref else None
        remaining = ref.remaining if ref else None
        outposts = ((ref.own_outpost if ref else None), (ref.enemy_outpost if ref else None))
    values = [
        px, py,
        sign * own.velocity.x / spec.speed_scale if own.velocity_valid else 0.,
        sign * own.velocity.y / spec.speed_scale if own.velocity_valid else 0.,
        float(own.velocity_valid),
        hp if hp is not None else 0., float(hp is not None),
        min(1., max(0., remaining / spec.regulation_seconds)) if remaining is not None else 0.,
        float(remaining is not None),
    ]
    for alive in outposts:
        values.extend((float(alive) if alive is not None else 0., float(alive is not None)))
    for tracks in (world.allies, world.enemies):
        nearest = sorted(
            (track for track in tracks if track.usable and track is not own),
            key=lambda t: (own.position.distance(t.position), t.robot_id),
        )[:5]
        for track in nearest:
            values.extend((
                sign * (track.position.x - own.position.x) / spec.width,
                sign * (track.position.y - own.position.y) / spec.height,
                sign * track.velocity.x / spec.speed_scale if track.velocity_valid else 0.,
                sign * track.velocity.y / spec.speed_scale if track.velocity_valid else 0.,
                float(track.velocity_valid), 1.,
            ))
        values.extend([0.] * (6 * (5 - len(nearest))))
    result = np.asarray(values, dtype=np.float32)
    if result.shape != (len(FEATURE_NAMES),) or not np.isfinite(result).all():
        raise ValueError("invalid navigation observation")
    return result
