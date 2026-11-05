"""Bounded world model. It never refreshes a source observation's timestamp."""

from dataclasses import dataclass
import math

from .config import Config
from .model import Observation, Phase, Referee, Team, Track, TrackStatus, Vec2, World


@dataclass
class _History:
    observations: list[Observation]
    position: Vec2
    velocity: Vec2
    generation: int
    pending_jump: Observation | None = None


class WorldModel:
    def __init__(self, config: Config):
        self.config = config
        self._tracks: dict[tuple[Team, int], _History] = {}
        self._referee: Referee | None = None
        self._last_now: float | None = None
        self._generation = 0
        self._epoch = 0
        self._seq = 0
        self.rejections = 0

    def _clock(self, now: float) -> None:
        if not math.isfinite(now) or now < 0:
            raise ValueError("observation time must be finite and nonnegative")
        if self._last_now is not None and now < self._last_now - 0.001:
            self._tracks.clear()
            self._referee = None
            self._epoch += 1
        self._last_now = now

    def observe(self, obs: Observation, received_at: float) -> bool:
        self._clock(received_at)
        a, b, c, d = self.config.bounds
        valid = (
            1 <= obs.robot_id <= 65535 and obs.team in (Team.ALLY, Team.ENEMY)
            and obs.position.finite and a <= obs.position.x <= c and b <= obs.position.y <= d
            and math.isfinite(obs.stamp) and 0 <= obs.stamp <= received_at + 0.05
            and received_at - obs.stamp <= self.config.predicted_age
            and math.isfinite(obs.confidence) and 0 < obs.confidence <= 1
            and (obs.velocity is None or (obs.velocity.finite
                                         and obs.velocity.norm <= self.config.max_speed))
        )
        if not valid:
            self.rejections += 1
            return False
        key = obs.team, obs.robot_id
        history = self._tracks.get(key)
        if history:
            last = history.observations[-1]
            dt = obs.stamp - last.stamp
            if dt <= 0:
                self.rejections += 1
                return False
            if dt > self.config.predicted_age:
                history = None
            elif last.position.distance(obs.position) > (
                self.config.max_speed * dt + self.config.jump_slack
            ):
                pending = history.pending_jump
                # Require a second observation at the new location before rebuilding ID.
                if (pending is not None and obs.stamp > pending.stamp
                        and pending.position.distance(obs.position)
                        <= self.config.max_speed * (obs.stamp - pending.stamp)
                        + self.config.jump_slack):
                    history = None
                else:
                    history.pending_jump = obs
                    self.rejections += 1
                    return False
        if history is None:
            if key not in self._tracks and len(self._tracks) >= self.config.max_tracks:
                oldest = min(self._tracks, key=lambda k: self._tracks[k].observations[-1].stamp)
                if received_at - self._tracks[oldest].observations[-1].stamp < self.config.retention:
                    self.rejections += 1
                    return False
                del self._tracks[oldest]
            self._generation += 1
            self._tracks[key] = _History([obs], obs.position, Vec2(0.0, 0.0), self._generation)
            return True
        last = history.observations[-1]
        dt = obs.stamp - last.stamp
        predicted = Vec2(history.position.x + history.velocity.x * dt,
                         history.position.y + history.velocity.y * dt)
        rx, ry = obs.position.x - predicted.x, obs.position.y - predicted.y
        history.position = Vec2(predicted.x + 0.8 * rx, predicted.y + 0.8 * ry)
        velocity = (obs.velocity if obs.velocity is not None else
                    Vec2(history.velocity.x + 0.25 * rx / dt,
                         history.velocity.y + 0.25 * ry / dt))
        scale = min(1.0, self.config.max_speed / max(velocity.norm, 1e-9))
        history.velocity = Vec2(velocity.x * scale, velocity.y * scale)
        if obs.team == Team.ALLY and obs.robot_id == self.config.self_id:
            # Localization has already filtered the robot pose. A second trajectory
            # filter can cut corners through a wall or create a fictitious overshoot.
            history.position = obs.position
            if obs.velocity is None:
                vx, vy = (obs.position.x - last.position.x) / dt, (obs.position.y - last.position.y) / dt
                scale = min(1., self.config.max_speed / max(math.hypot(vx, vy), 1e-9))
                history.velocity = Vec2(vx * scale, vy * scale)
        history.pending_jump = None
        history.observations.append(obs)
        history.observations = [o for o in history.observations
                                if obs.stamp - o.stamp <= self.config.history_sec][-128:]
        return True

    def update_referee(self, referee: Referee) -> bool:
        if (not math.isfinite(referee.stamp) or referee.stamp < 0
                or referee.phase not in Phase
                or (self._referee is not None and referee.stamp <= self._referee.stamp)):
            return False
        if any(value is not None and (not math.isfinite(value) or not 0 <= value <= 1)
               for value in (referee.hp, referee.ammo)):
            return False
        if referee.remaining is not None and (
            not math.isfinite(referee.remaining) or referee.remaining < 0
        ):
            return False
        if any(value is not None and not isinstance(value, bool)
               for value in (referee.own_outpost, referee.enemy_outpost, referee.supply_available)):
            return False
        self._referee = referee
        return True

    def snapshot(self, now: float) -> World:
        self._clock(now)
        self._seq += 1
        tracks = []
        for key, history in list(self._tracks.items()):
            observation = history.observations[-1]
            age = max(0.0, now - observation.stamp)
            if age > self.config.retention:
                del self._tracks[key]
                continue
            status = (TrackStatus.OBSERVED if age <= self.config.observed_age else
                      TrackStatus.PREDICTED if age <= self.config.predicted_age else
                      TrackStatus.STALE)
            is_self = observation.team == Team.ALLY and observation.robot_id == self.config.self_id
            horizon = 0.0 if is_self else min(age, self.config.predicted_age)
            position = Vec2(history.position.x + history.velocity.x * horizon,
                            history.position.y + history.velocity.y * horizon)
            tracks.append(Track(
                observation.robot_id, observation.team, position, history.velocity,
                observation.stamp, age,
                observation.confidence * math.exp(-age / self.config.predicted_age),
                status, history.generation, len(history.observations) > 1,
                tuple(history.observations),
            ))
        own = next((t for t in tracks if t.team == Team.ALLY
                    and t.robot_id == self.config.self_id), None)
        allies = tuple(t for t in tracks if t.team == Team.ALLY and t is not own)
        enemies = tuple(t for t in tracks if t.team == Team.ENEMY)
        referee = self._referee
        if referee and not -0.05 <= now - referee.stamp <= self.config.referee_timeout:
            referee = None
        external = (*allies, *enemies)
        if own is None or not own.usable or own.age > self.config.self_timeout:
            mode = "SAFE_STOP"
        elif not any(t.usable for t in external):
            mode = "LOCAL_ONLY"
        elif all(t.usable for t in external):
            mode = "FULL"
        else:
            mode = "PARTIAL"
        return World(self._seq, now, own, allies, enemies, referee, mode,
                     self.config.frame, self._epoch)
