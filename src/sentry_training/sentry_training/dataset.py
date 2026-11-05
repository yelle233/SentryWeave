"""Sequence datasets with group-level splits, synthetic trajectories and real-log labels."""

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from sentry_core.config import Config
from sentry_core.geometry import TacticalMap
from sentry_core.intent import FEATURE_NAMES, IntentEstimator
from sentry_core.model import IntentType, Observation, Team, Vec2
from sentry_core.serde import json_line, world_from_dict
from sentry_core.tracking import WorldModel


@dataclass(frozen=True)
class Sequence:
    episode_id: str
    group_id: str
    source: str
    map_fingerprint: str
    features: np.ndarray
    labels: np.ndarray
    times: np.ndarray

    def validate(self):
        n = len(self.labels)
        if (n < 2 or self.features.shape != (n, len(FEATURE_NAMES))
                or self.times.shape != (n,) or not np.isfinite(self.features).all()
                or not np.isfinite(self.times).all() or not (np.diff(self.times) > 0).all()
                or not ((self.labels >= 0) & (self.labels < 6)).all()
                or not self.episode_id or not self.group_id):
            raise ValueError(f"invalid sequence {self.episode_id}")
        return self


def _random_position(rng, arena: TacticalMap, away_from_zones=False):
    a, b, c, d = arena.config.bounds
    for _ in range(10000):
        point = Vec2(float(rng.uniform(a + 0.6, c - 0.6)),
                     float(rng.uniform(b + 0.6, d - 0.6)))
        if arena.legal(point) and (not away_from_zones or all(
            point.distance(z.center) > z.radius * 1.1 for z in arena.config.zones
        )):
            return point
    raise ValueError("map has insufficient legal sampling area")


def generate_dataset(config: Config, episodes_per_class=40, seed=11, steps=80) -> list[Sequence]:
    if episodes_per_class < 5 or steps < 10:
        raise ValueError("use at least 5 episodes per class and 10 steps")
    rng = np.random.default_rng(seed)
    arena = TacticalMap(config)
    sequences = []
    for repeat in range(episodes_per_class):
        for kind in IntentType:
            tracker = WorldModel(config)
            estimator = IntentEstimator(config, arena)
            zones = [z for z in config.zones if z.intent == kind]
            position = _random_position(rng, arena, away_from_zones=True)
            goal = zones[0].center if zones else _random_position(rng, arena, away_from_zones=True)
            speed = float(rng.uniform(0.7, 2.0))
            noise = float(rng.uniform(0.005, 0.065))
            features, labels, times = [], [], []
            for step in range(steps):
                now = 1.0 + step * 0.2
                label = kind
                if kind != IntentType.UNKNOWN:
                    if kind == IntentType.TRANSIT and (position.distance(goal) < 0.5 or step % 12 == 0):
                        goal = _random_position(rng, arena, away_from_zones=True)
                    # Some attacking sequences include a genuine retreat transition.
                    if kind == IntentType.ATTACK_OUTPOST and repeat % 3 == 0 and step > steps * 0.7:
                        goal = next(z.center for z in config.zones if z.intent == IntentType.RETREAT)
                        label = IntentType.RETREAT
                    position = arena.advance(position, goal, speed * 0.2)
                observed = Vec2(position.x + float(rng.normal(0, noise)),
                                position.y + float(rng.normal(0, noise)))
                if rng.random() < 0.04 or not arena.legal(observed):
                    continue
                tracker.observe(Observation(1, Team.ALLY, observed, now,
                                            float(rng.uniform(0.8, 1.0))), now)
                track = tracker.snapshot(now).allies[0]
                features.append(estimator.features(track))
                span = track.history[-1].stamp - track.history[0].stamp
                labels.append(int(label if span >= 0.35 else IntentType.UNKNOWN))
                times.append(now)
            sequences.append(Sequence(
                f"synthetic-{seed}-{repeat}-{kind.name}", f"synthetic-{seed}-{repeat}",
                "synthetic", config.map_fingerprint, np.asarray(features, dtype=np.float64),
                np.asarray(labels, dtype=np.int64), np.asarray(times, dtype=np.float64),
            ).validate())
    return sequences


def split_sequences(sequences, seed=17):
    for sequence in sequences:
        sequence.validate()
    ids = [s.episode_id for s in sequences]
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate episode IDs would leak data across splits")
    groups = sorted({s.group_id for s in sequences})
    if len(groups) < 5:
        raise ValueError("need at least five independent match/trajectory groups")
    np.random.default_rng(seed).shuffle(groups)
    n = len(groups)
    n_train = max(1, int(n * 0.6))
    n_validation = max(1, int(n * 0.2))
    partitions = (set(groups[:n_train]), set(groups[n_train:n_train + n_validation]),
                  set(groups[n_train + n_validation:]))
    return tuple([s for s in sequences if s.group_id in part] for part in partitions)


def write_dataset(path, sequences):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        for sequence in sequences:
            sequence.validate()
            stream.write(json_line({
                "schema_version": 1, "feature_names": FEATURE_NAMES,
                "episode_id": sequence.episode_id, "group_id": sequence.group_id,
                "source": sequence.source, "map_fingerprint": sequence.map_fingerprint,
                "features": sequence.features.tolist(), "labels": sequence.labels.tolist(),
                "times": sequence.times.tolist(),
            }) + "\n")


def read_dataset(path):
    sequences = []
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            raw = json.loads(line)
            if raw.get("schema_version") != 1 or raw.get("feature_names") != list(FEATURE_NAMES):
                raise ValueError("dataset feature schema mismatch")
            labels = np.asarray(raw["labels"])
            if labels.dtype.kind not in "iu":
                raise ValueError("labels must be integer class IDs")
            sequences.append(Sequence(
                raw["episode_id"], raw["group_id"], raw["source"], raw["map_fingerprint"],
                np.asarray(raw["features"], dtype=np.float64), labels,
                np.asarray(raw["times"], dtype=np.float64),
            ).validate())
    return sequences


def extract_labeled(config, trace_path, annotations_path):
    """Use explicit human labels; never silently treat unlabeled samples as UNKNOWN."""
    annotations = json.loads(Path(annotations_path).read_text(encoding="utf-8"))
    if not isinstance(annotations, list):
        raise ValueError("annotations must be a list of labeled time intervals")
    for item in annotations:
        if item["end"] <= item["start"] or item["label"] not in IntentType.__members__:
            raise ValueError("invalid annotation interval or label")
    estimator = IntentEstimator(config, TacticalMap(config))
    samples = {}
    with open(trace_path, encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            world = world_from_dict(record["world"])
            episode = record["episode_id"]
            for track in world.allies:
                matching = [a for a in annotations if a["episode_id"] == episode
                            and a["robot_id"] == track.robot_id
                            and a["start"] <= track.observed_at < a["end"]]
                if len(matching) > 1:
                    raise ValueError("overlapping annotation intervals")
                if not matching or not track.usable:
                    continue
                key = episode, track.robot_id, track.generation
                rows = samples.setdefault(key, {})
                rows[track.observed_at] = (
                    estimator.features(track), int(IntentType[matching[0]["label"]])
                )
    output = []
    for (episode, robot, generation), rows in samples.items():
        segments = [[]]
        for stamp, values in sorted(rows.items()):
            if segments[-1] and stamp - segments[-1][-1][0] > config.predicted_age:
                segments.append([])
            segments[-1].append((stamp, *values))
        for part, segment in enumerate(segments):
            if len(segment) < 2:
                continue
            stamps, features, labels = zip(*segment)
            output.append(Sequence(
                f"{episode}:{robot}:{generation}:{part}", episode, "human_labeled_replay",
                config.map_fingerprint, np.asarray(features), np.asarray(labels),
                np.asarray(stamps),
            ).validate())
    if not output:
        raise ValueError("no labeled trajectories overlap this trace")
    return output
