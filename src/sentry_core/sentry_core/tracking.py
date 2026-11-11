"""Bounded world model. It never refreshes a source observation's timestamp."""

from dataclasses import dataclass
import math

from .config import Config
from .model import (
    TEAM_COLORS, GameState, Observation, Phase, Referee, RobotStatus, StructureState, Team, Track,
    TrackStatus, Vec2, World,
)


@dataclass
class _History:
    observations: list[Observation]
    position: Vec2
    velocity: Vec2
    generation: int
    pending_jump: Observation | None = None


class _Fields:
    """Latest valid value of each field, with that value's own source time and source bits."""

    def __init__(self):
        self.values: dict[str, tuple[object, float, int]] = {}

    def merge(self, values: dict, stamp: float, source: int) -> None:
        for name, value in values.items():
            old = self.values.get(name)
            if value is not None and (old is None or stamp >= old[1]):
                self.values[name] = (value, stamp, source)

    def fresh(self, now: float, timeout: float):
        items = {name: item for name, item in self.values.items()
                 if -0.05 <= now - item[1] <= timeout}
        if not items:
            return None
        source = 0
        for _, _, bits in items.values():
            source |= bits
        return ({name: item[0] for name, item in items.items()},
                max(item[1] for item in items.values()), source)


class WorldModel:
    def __init__(self, config: Config):
        self.config = config
        self._tracks: dict[tuple[Team, int], _History] = {}
        self._referee: Referee | None = None
        self._game = _Fields()
        self._robots: dict[int, tuple[Team, _Fields]] = {}
        self._structures: dict[str, _Fields] = {}
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
            self._game = _Fields()
            self._robots.clear()
            self._structures.clear()
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

    def _accept(self, stamp: float, received_at: float, timeout: float, source: int) -> bool:
        return (math.isfinite(stamp) and 0 <= stamp <= received_at + 0.05
                and received_at - stamp <= timeout and isinstance(source, int) and source >= 0)

    def update_game(self, game: GameState, received_at: float) -> bool:
        self._clock(received_at)
        if (not self._accept(game.stamp, received_at, self.config.game_timeout, game.source)
                or game.phase not in Phase
                or (game.own_team is not None and game.own_team not in TEAM_COLORS)
                or (game.remaining is not None
                    and (not math.isfinite(game.remaining) or game.remaining < 0))
                or (game.supply_available is not None
                    and not isinstance(game.supply_available, bool))):
            self.rejections += 1
            return False
        self._game.merge({
            "season": game.season or None, "own_team": game.own_team,
            "phase": game.phase if game.phase != Phase.UNKNOWN else None,
            "remaining": game.remaining, "supply_available": game.supply_available,
        }, game.stamp, game.source)
        return True

    def update_robot(self, status: RobotStatus, received_at: float) -> bool:
        self._clock(received_at)
        numbers = (status.hp, status.max_hp, status.ammo)
        valid = (
            1 <= status.robot_id <= 65535 and status.team in (Team.ALLY, Team.ENEMY)
            and self._accept(status.stamp, received_at, self.config.robot_status_timeout,
                             status.source)
            and all(v is None or math.isfinite(v) for v in numbers)
            and (status.hp is None or status.hp >= 0)
            and (status.max_hp is None or status.max_hp > 0)
            and (status.ammo is None or 0 <= status.ammo <= 1)
            and (status.level is None or (isinstance(status.level, int) and status.level >= 0))
            and (status.alive is None or isinstance(status.alive, bool))
        )
        entry = self._robots.get(status.robot_id)
        if (not valid or (entry is not None and entry[0] != status.team)
                or (entry is None and len(self._robots) >= self.config.max_tracks)):
            self.rejections += 1
            return False
        if entry is None:
            entry = self._robots[status.robot_id] = (status.team, _Fields())
        entry[1].merge({"hp": status.hp, "max_hp": status.max_hp, "alive": status.alive,
                        "level": status.level, "ammo": status.ammo}, status.stamp, status.source)
        return True

    def update_structure(self, state: StructureState, received_at: float) -> bool:
        self._clock(received_at)
        numbers = (state.hp, state.max_hp, state.shield)
        valid = (
            isinstance(state.name, str) and 0 < len(state.name) <= 64
            and self._accept(state.stamp, received_at, self.config.structure_timeout, state.source)
            and all(v is None or (math.isfinite(v) and v >= 0) for v in numbers)
            and (state.max_hp is None or state.max_hp > 0)
            and (state.alive is None or isinstance(state.alive, bool))
        )
        if not valid or (state.name not in self._structures and len(self._structures) >= 16):
            self.rejections += 1
            return False
        self._structures.setdefault(state.name, _Fields()).merge(
            {"hp": state.hp, "max_hp": state.max_hp, "alive": state.alive, "shield": state.shield},
            state.stamp, state.source)
        return True

    def _typed_inputs(self, now: float):
        game = None
        fresh = self._game.fresh(now, self.config.game_timeout)
        if fresh is not None:
            values, stamp, source = fresh
            game = GameState(stamp, values.get("season", ""), values.get("own_team"),
                             values.get("phase", Phase.UNKNOWN), values.get("remaining"),
                             values.get("supply_available"), source)
        robots = {}
        for robot_id, (team, fields) in list(self._robots.items()):
            fresh = fields.fresh(now, self.config.robot_status_timeout)
            if fresh is None:
                del self._robots[robot_id]
                continue
            values, stamp, source = fresh
            robots[robot_id] = RobotStatus(robot_id, team, stamp, source=source, **values)
        structures = {}
        for name, fields in list(self._structures.items()):
            fresh = fields.fresh(now, self.config.structure_timeout)
            if fresh is None:
                del self._structures[name]
                continue
            values, stamp, source = fresh
            structures[name] = StructureState(name, stamp, source=source, **values)
        return game, robots, structures

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
        game, robots, structures = self._typed_inputs(now)
        return World(self._seq, now, own, allies, enemies, referee, mode,
                     self.config.frame, self._epoch, game, robots, structures)
