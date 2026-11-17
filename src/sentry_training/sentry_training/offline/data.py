"""Read-only RMUC SQLite import with match grouping and non-overlapping macro actions."""

from dataclasses import asdict, dataclass, field
import hashlib
import json
import math
from pathlib import Path
import sqlite3

import numpy as np

from sentry_core.model import Phase, Referee, Team, Track, TrackStatus, Vec2, World
from sentry_core.navigation_features import (
    FEATURE_NAMES,
    FEATURE_VERSION,
    FEATURE_VERSION_V2,
    NavigationSpec,
    encode_world,
    feature_names_for,
)


GROUND_ROLES = (1, 2, 3, 4, 7)
SPLITS = ("train", "validation", "test")
REQUIRED = {
    "matches": {"game_id", "赛区", "场次号", "胜方", "时长秒"},
    "timeseries": {"game_id", "时刻秒", "robot_id", "当前血量", "最大血量", "x", "y"},
    "events": {"game_id", "时刻秒", "robot_id", "事件类型", "类别", "数值"},
}


def file_hash(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def readonly(path):
    connection = sqlite3.connect(Path(path).resolve().as_uri() + "?mode=ro", uri=True)
    connection.execute("PRAGMA query_only=ON")
    for table, required in REQUIRED.items():
        columns = {row[1] for row in connection.execute(f'PRAGMA table_info("{table}")')}
        if not required <= columns:
            connection.close()
            raise ValueError(f"{table} missing columns: {required - columns}")
    return connection


@dataclass(frozen=True)
class RewardWeights:
    enemy_damage: float = 0.25
    own_damage: float = 0.25
    enemy_outpost: float = 3.
    own_outpost: float = 4.
    enemy_base: float = 5.
    own_base: float = 6.
    own_health: float = 0.2
    death: float = 1.
    win: float = 5.


@dataclass(frozen=True)
class BuildOptions:
    seed: int = 29
    gamma: float = 0.99
    train_fraction: float = 0.7
    validation_fraction: float = 0.15
    roles: tuple[int, ...] = (3, 4, 7)
    spec: NavigationSpec = field(default_factory=NavigationSpec)
    reward: RewardWeights = field(default_factory=RewardWeights)

    def validate(self):
        self.spec.validate()
        if (not 0 < self.gamma < 1 or not 0 < self.train_fraction < 1
                or not 0 < self.validation_fraction < 1
                or self.train_fraction + self.validation_fraction >= 1
                or not self.roles or not set(self.roles) <= set(GROUND_ROLES)):
            raise ValueError("invalid dataset build options")
        if not all(math.isfinite(v) and v >= 0 for v in asdict(self.reward).values()):
            raise ValueError("reward weights must be finite and nonnegative")
        return self


class Game:
    def __init__(self, metadata, rows, events, spec):
        self.game_id, self.region, self.match_no, self.winner, self.duration = metadata
        self.group = f"{self.region}/{self.match_no}"
        self.spec = spec
        self.raw_rows = len(rows)
        self.max_time = max((int(round(row[0])) for row in rows), default=0)
        self.T = min(int(self.duration), self.max_time)
        self.complete = self.max_time >= self.duration
        self.series = {}
        self.events = events
        self.audit = {"invalid_positions": 0, "jump_links": 0}
        seen = set()
        for time, rid, hp, maximum, x, y in rows:
            t = int(round(time))
            if not math.isfinite(time) or abs(t - time) > 1e-3:
                raise ValueError(f"non-integral state timestamp in game {self.game_id}")
            if not 0 <= t <= self.T:
                continue
            if (t, rid) in seen:
                raise ValueError(f"duplicate robot/timestamp in game {self.game_id}: {(rid, t)}")
            seen.add((t, rid))
            values = self.series.setdefault(rid, np.full((self.T + 1, 4), np.nan))
            values[t] = [np.nan if v is None else float(v) for v in (x, y, hp, maximum)]
        self.valid, self.velocity, self.velocity_valid = {}, {}, {}
        for rid, values in self.series.items():
            x, y = values[:, 0], values[:, 1]
            known = (np.isfinite(x) & np.isfinite(y) & ((np.abs(x) > 1e-6) | (np.abs(y) > 1e-6))
                     & (x >= 0) & (x <= spec.width) & (y >= 0) & (y <= spec.height))
            valid_velocity = np.zeros(self.T + 1, dtype=bool)
            delta = np.diff(values[:, :2], axis=0)
            links = known[1:] & known[:-1] & (np.linalg.norm(delta, axis=1) <= spec.speed_scale + 0.05)
            valid_velocity[1:] = links
            velocity = np.zeros((self.T + 1, 2))
            velocity[1:] = np.where(links[:, None], delta, 0.)
            self.valid[rid], self.velocity[rid], self.velocity_valid[rid] = known, velocity, valid_velocity
            if rid % 100 in GROUND_ROLES:
                self.audit["invalid_positions"] += int((~known[1:]).sum())
                self.audit["jump_links"] += int((known[1:] & known[:-1] & ~links).sum())
        self._world_cache = {}
        self._obs_cache = {}

    def _track(self, t, rid, side):
        if rid not in self.series or not self.valid[rid][t]:
            return None
        values = self.series[rid][t]
        team = Team.ALLY if rid // 100 == side else Team.ENEMY
        return Track(rid, team, Vec2(*values[:2]), Vec2(*self.velocity[rid][t]),
                     float(t), 0., 1., TrackStatus.OBSERVED, 1, bool(self.velocity_valid[rid][t]))

    def _hp(self, t, rid):
        if rid not in self.series:
            return None
        value = self.series[rid][t, 2]
        return float(value) if np.isfinite(value) and value >= 0 else None

    def world(self, t, ego):
        if (t, ego) in self._world_cache:
            return self._world_cache[t, ego]
        side = ego // 100
        own = self._track(t, ego, side)
        tracks = [track for rid in self.series if rid != ego and rid % 100 in GROUND_ROLES
                  if (track := self._track(t, rid, side)) is not None]
        hp, maximum = (self.series[ego][t, 2:] if ego in self.series else (np.nan, np.nan))
        fraction = float(np.clip(hp / maximum, 0, 1)) if np.isfinite(hp + maximum) and maximum > 0 else None
        own_out, enemy_out = self._hp(t, 11 + 100 * side), self._hp(t, 11 + 100 * (1 - side))
        referee = Referee(
            float(t), Phase.RUNNING, fraction, None, None if own_out is None else own_out > 0,
            None if enemy_out is None else enemy_out > 0, None,
            max(0., self.spec.regulation_seconds - t),
        )
        world = World(t, float(t), own, tuple(v for v in tracks if v.team == Team.ALLY),
                      tuple(v for v in tracks if v.team == Team.ENEMY), referee,
                      "FULL" if own is not None else "SAFE_STOP")
        self._world_cache[t, ego] = world
        return world

    def observation(self, t, ego):
        if (t, ego) not in self._obs_cache:
            self._obs_cache[t, ego] = encode_world(self.world(t, ego), self.spec, mirror=ego >= 100)
        return self._obs_cache[t, ego]

    def reward_steps(self, ego, weights):
        reward = np.zeros(self.T + 1, dtype=np.float64)
        side = ego // 100
        for time, rid, category, value in self.events:
            if value is None or not math.isfinite(value) or value >= 0 or rid is None:
                continue
            t = int(math.ceil(time - 1e-8))
            if not 1 <= t <= self.T or rid % 100 not in GROUND_ROLES:
                continue
            damage = -float(value) / 400.
            if rid // 100 == side:
                reward[t] -= weights.own_damage * damage
            elif rid // 100 == 1 - side and category != "判罚":
                reward[t] += weights.enemy_damage * damage

        def drops(rid):
            if rid not in self.series:
                return np.zeros(self.T)
            hp = self.series[rid][:, 2]
            valid = np.isfinite(hp[:-1]) & np.isfinite(hp[1:])
            return np.where(valid, np.maximum(0., hp[:-1] - hp[1:]), 0.)

        for role, scale, own_weight, enemy_weight in (
            (11, 1500., weights.own_outpost, weights.enemy_outpost),
            (10, 5000., weights.own_base, weights.enemy_base),
        ):
            reward[1:] -= own_weight * drops(role + 100 * side) / scale
            reward[1:] += enemy_weight * drops(role + 100 * (1 - side)) / scale
        reward[1:] -= weights.own_health * drops(ego) / 400.
        if ego in self.series:
            hp = self.series[ego][:, 2]
            reward[1:] -= weights.death * ((hp[:-1] > 0) & (hp[1:] == 0))
        if self.complete and self.winner in ("红", "蓝"):
            reward[self.T] += weights.win * (1 if self.winner == ("蓝" if side else "红") else -1)
        return reward

    def episodes(self, ego, options):
        if ego not in self.series:
            return
        hp = self.series[ego][:, 2]
        known = self.valid[ego]
        links = (self.velocity_valid[ego][1:] & (hp[:-1] > 0) & np.isfinite(hp[1:]))
        reward = self.reward_steps(ego, options.reward)
        start = 1
        while start < self.T:
            if not known[start] or not hp[start] > 0:
                start += 1
                continue
            end = start
            while end < self.T and links[end]:
                end += 1
                if hp[end] <= 0:
                    break
            if end == start:
                start += 1
                continue
            records = []
            t = start
            while t < end:
                nxt = min(t + options.spec.horizon, end)
                duration = nxt - t
                obs, following = self.observation(t, ego), self.observation(nxt, ego)
                action = following[:2] - obs[:2]
                terminal = bool(hp[nxt] <= 0 or nxt == self.T and self.complete)
                truncated = nxt == end and not terminal
                discounted = float(np.dot(
                    options.gamma ** np.arange(duration), reward[t + 1:nxt + 1]))
                records.append((obs, following, action, discounted,
                                0. if terminal else options.gamma ** duration,
                                terminal, truncated, duration, t))
                t = nxt
            yield records, {"game_id": int(self.game_id), "group": self.group, "robot_id": int(ego),
                            "role": int(ego % 100), "camp": "蓝" if ego >= 100 else "红",
                            "first_time": start, "last_time": end}
            start = end + 1


def _load_game(connection, game_id, spec):
    metadata = connection.execute(
        'SELECT game_id,"赛区","场次号","胜方","时长秒" FROM matches WHERE game_id=?', (int(game_id),)
    ).fetchone()
    if metadata is None:
        raise ValueError(f"unknown game {game_id}")
    rows = connection.execute(
        'SELECT "时刻秒",robot_id,"当前血量","最大血量",x,y FROM timeseries '
        'WHERE game_id=? ORDER BY "时刻秒",robot_id', (int(game_id),)
    ).fetchall()
    events = connection.execute(
        'SELECT "时刻秒",robot_id,"类别","数值" FROM events WHERE game_id=? AND "事件类型"=?',
        (int(game_id), "受击"),
    ).fetchall()
    return Game(metadata, rows, events, spec)


def load_game(path, game_id, spec):
    connection = readonly(path)
    try:
        return _load_game(connection, game_id, spec)
    finally:
        connection.close()


ARRAY_NAMES = ("obs", "next_obs", "actions", "rewards", "discounts", "terminated",
               "truncated", "durations", "times")


def build_dataset(db_path, output, options=BuildOptions(), progress=None):
    options.validate()
    output = Path(output)
    if (output / "metadata.json").exists():
        raise FileExistsError(f"dataset already exists: {output}")
    output.mkdir(parents=True, exist_ok=True)
    connection = readonly(db_path)
    try:
        matches = connection.execute(
            'SELECT game_id,"赛区","场次号" FROM matches ORDER BY game_id').fetchall()
        groups = sorted({f"{region}/{match}" for _, region, match in matches})
        if len(groups) < 5:
            raise ValueError("need at least five independent match groups")
        np.random.default_rng(options.seed).shuffle(groups)
        a, b = int(len(groups) * options.train_fraction), int(len(groups) * options.validation_fraction)
        split_groups = dict(zip(SPLITS, (groups[:a], groups[a:a + b], groups[a + b:])))
        membership = {group: split for split, subset in split_groups.items() for group in subset}
        episodes, records = {s: [] for s in SPLITS}, {s: [] for s in SPLITS}
        counts = {name: connection.execute(f'SELECT COUNT(*) FROM "{name}"').fetchone()[0]
                  for name in REQUIRED}
        audit = {"invalid_positions": 0, "jump_links": 0, "terminal_transitions": 0,
                 "truncated_transitions": 0}
        for number, (game_id, region, match) in enumerate(matches):
            game = _load_game(connection, game_id, options.spec)
            split = membership[f"{region}/{match}"]
            for key in ("invalid_positions", "jump_links"):
                audit[key] += game.audit[key]
            for role in options.roles:
                for ego in (role, role + 100):
                    for rows, metadata in game.episodes(ego, options):
                        metadata.update(start=len(records[split]), length=len(rows),
                                        episode_id=len(episodes[split]))
                        episodes[split].append(metadata)
                        records[split].extend(rows)
                        audit["terminal_transitions"] += sum(row[5] for row in rows)
                        audit["truncated_transitions"] += sum(row[6] for row in rows)
            if progress and (number + 1) % 25 == 0:
                progress(f"Processed {number + 1}/{len(matches)} games")
    finally:
        connection.close()
    metadata = {
        "schema_version": 1, "feature_version": FEATURE_VERSION, "feature_names": list(FEATURE_NAMES),
        "spec": options.spec.to_dict(), "build_options": asdict(options),
        "source": {"filename": Path(db_path).name, "sha256": file_hash(db_path), **counts},
        "split_groups": split_groups, "splits": {}, "audit": audit,
        "action_definition": "non-overlapping future displacement / field dimensions",
        "limitations": ["global ground-robot positions assumed available",
                        "future endpoints are proxy actions, not recorded control commands",
                        "missing endpoints censor transitions, including some deaths"],
    }
    for split in SPLITS:
        if not records[split]:
            raise ValueError(f"no valid transitions in {split}")
        arrays = {}
        for column, name in enumerate(ARRAY_NAMES):
            arrays[name] = np.asarray([row[column] for row in records[split]], dtype=np.float32)
        arrays["episode_ids"] = np.concatenate([
            np.full(ep["length"], ep["episode_id"], dtype=np.int64) for ep in episodes[split]])
        arrays["roles"] = np.concatenate([
            np.full(ep["length"], ep["role"], dtype=np.int64) for ep in episodes[split]])
        path = output / f"{split}.npz"
        np.savez_compressed(path, **arrays)
        (output / f"{split}.episodes.json").write_text(
            json.dumps(episodes[split], ensure_ascii=False, separators=(",", ":")))
        metadata["splits"][split] = {
            "transitions": len(records[split]), "episodes": len(episodes[split]),
            "sha256": file_hash(path),
            "roles": {str(role): int((arrays["roles"] == role).sum()) for role in options.roles},
        }
    canonical = json.dumps(metadata, ensure_ascii=False, sort_keys=True).encode()
    metadata["dataset_id"] = hashlib.sha256(canonical).hexdigest()
    (output / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    return metadata


class Replay:
    def __init__(self, arrays, episodes, metadata):
        self.arrays, self.episodes, self.metadata = arrays, episodes, metadata

    def __getitem__(self, key):
        return self.arrays[key]

    def __len__(self):
        return len(self.arrays["actions"])

    def indices(self, roles=(3, 4)):
        return np.flatnonzero(np.isin(self["roles"], roles))


def read_split(directory, split, verify=True):
    if split not in SPLITS:
        raise ValueError("unknown split")
    directory = Path(directory)
    metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
    schema = metadata.get("schema_version")
    feat_ver = metadata.get("feature_version")
    if (schema, feat_ver) not in ((1, FEATURE_VERSION), (2, FEATURE_VERSION_V2)):
        raise ValueError("offline dataset feature schema mismatch")
    expected_names = list(feature_names_for(feat_ver))
    if metadata.get("feature_names") != expected_names:
        raise ValueError("offline dataset feature schema mismatch")
    action_dim = 2 if schema == 1 else 6
    path = directory / f"{split}.npz"
    if verify and file_hash(path) != metadata["splits"][split]["sha256"]:
        raise ValueError("offline dataset checksum mismatch")
    with np.load(path, allow_pickle=False) as data:
        arrays = {name: data[name] for name in (*ARRAY_NAMES, "episode_ids", "roles")}
    n = len(arrays["actions"])
    if (
        arrays["obs"].shape != (n, len(expected_names))
        or arrays["next_obs"].shape != arrays["obs"].shape
        or arrays["actions"].shape != (n, action_dim)
        or any(len(a) != n or not np.isfinite(a).all() for a in arrays.values())
        or not ((arrays["discounts"] >= 0) & (arrays["discounts"] <= 1)).all()
    ):
        raise ValueError("invalid offline transition arrays")
    episodes = json.loads((directory / f"{split}.episodes.json").read_text(encoding="utf-8"))
    cursor = 0
    for episode in episodes:
        if episode["start"] != cursor or episode["length"] < 1:
            raise ValueError("invalid episode boundaries")
        cursor += episode["length"]
    if cursor != n:
        raise ValueError("episode lengths do not cover data")
    return Replay(arrays, episodes, metadata)
