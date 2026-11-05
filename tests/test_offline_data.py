"""Public SQLite/feature contracts, including temporal and split leakage regressions."""

from dataclasses import replace
import json
import sqlite3
import importlib.util

import numpy as np
import pytest


@pytest.fixture
def official_db(tmp_path):
    path = tmp_path / "official-format.sqlite"
    connection = sqlite3.connect(path)
    connection.executescript("""
        CREATE TABLE matches (
            game_id INT, 赛区 TEXT, 场次号 INT, 局号 INT, 胜方 TEXT, 时长秒 INT);
        CREATE TABLE timeseries (
            game_id INT, 时刻秒 REAL, robot_id INT, 当前血量 REAL, 最大血量 REAL,
            x REAL, y REAL);
        CREATE TABLE events (
            game_id INT, 时刻秒 REAL, robot_id INT, 事件类型 TEXT, 类别 TEXT, 数值 REAL);
        CREATE INDEX game_ts ON timeseries(game_id);
        CREATE INDEX game_ev ON events(game_id);
    """)
    for match in range(10):
        for round_id in (1, 2):
            game = match * 10 + round_id
            connection.execute("INSERT INTO matches VALUES (?,?,?,?,?,?)",
                               (game, "test", match, round_id, "红", 31))
            for t in range(1, 32):
                for role in (1, 2, 3, 4, 6, 7, 10, 11):
                    for blue in (False, True):
                        rid = role + 100 * blue
                        hp = 1500 if role == 11 else 5000 if role == 10 else 400
                        x, y = (2 + role * 0.2 + t * 0.12, 3 + role * 0.2)
                        if blue:
                            x, y = 28 - x, 15 - y
                        if role in (10, 11):
                            x = y = 0.
                        connection.execute("INSERT INTO timeseries VALUES (?,?,?,?,?,?,?)",
                                           (game, t, rid, hp, hp, x, y))
            connection.execute("INSERT INTO events VALUES (?,?,?,?,?,?)",
                               (game, 3., 103, "受击", "17mm", -40.))
    connection.commit()
    connection.close()
    return path


def test_feature_contract_and_side_symmetry(world_factory):
    from sentry_core.navigation_features import NavigationSpec, encode_world, FEATURE_NAMES
    from sentry_core.model import Vec2

    spec = NavigationSpec(width=20, height=12)
    world = world_factory(allies=((1, 6, 3, 1, 0.2),), enemies=((11, 15, 7, -1, 0),))
    features = encode_world(world, spec)
    assert features.shape == (73,) and len(FEATURE_NAMES) == 73

    def mirror(track):
        return replace(track, position=Vec2(20 - track.position.x, 12 - track.position.y),
                       velocity=Vec2(-track.velocity.x, -track.velocity.y))
    blue = replace(world, self_track=mirror(world.self_track),
                   allies=tuple(mirror(t) for t in world.allies),
                   enemies=tuple(mirror(t) for t in world.enemies))
    np.testing.assert_allclose(features, encode_world(blue, spec, mirror=True), atol=1e-6)


def test_unknown_fields_and_stale_tracks_do_not_become_known(world_factory):
    from sentry_core.navigation_features import NavigationSpec, encode_world, FEATURE_NAMES
    from sentry_core.model import TrackStatus

    world = world_factory(hp=None, enemy_outpost=None, allies=((1, 6, 3, 1, 0),))
    world = replace(world, allies=(replace(world.allies[0], status=TrackStatus.STALE),))
    x = encode_world(world, NavigationSpec(width=20, height=12))
    assert x[FEATURE_NAMES.index("ego.hp_valid")] == 0
    assert x[FEATURE_NAMES.index("enemy_outpost.valid")] == 0
    assert x[FEATURE_NAMES.index("ally.0.present")] == 0


def test_sqlite_split_groups_all_rounds_sides_and_agents_together(official_db, tmp_path):
    from sentry_training.offline.data import BuildOptions, build_dataset, read_split

    output = tmp_path / "data"
    metadata = build_dataset(official_db, output, BuildOptions(seed=19))
    groups = [set(metadata["split_groups"][s]) for s in ("train", "validation", "test")]
    assert not groups[0] & groups[1] and not groups[0] & groups[2] and not groups[1] & groups[2]
    owner = {}
    for split in ("train", "validation", "test"):
        data = read_split(output, split)
        assert np.isfinite(data["obs"]).all()
        for episode in data.episodes:
            assert owner.setdefault(episode["group"], split) == split
    assert metadata["source"]["matches"] == 20
    assert metadata["spec"]["horizon"] == 5


def test_macro_transition_uses_matching_next_state_and_discount(official_db, tmp_path):
    from sentry_training.offline.data import BuildOptions, build_dataset, read_split

    out = tmp_path / "data"
    build_dataset(official_db, out, BuildOptions(gamma=0.9))
    data = read_split(out, "train")
    i = next(i for i in range(len(data)) if data["durations"][i] == 5 and not data["terminated"][i])
    assert data["discounts"][i] == pytest.approx(0.9 ** 5)
    np.testing.assert_allclose(
        data["actions"][i], data["next_obs"][i, :2] - data["obs"][i, :2], atol=1e-6)
    assert data["actions"][i, 0] == pytest.approx(0.6 / 28, abs=1e-6)


def test_missing_blue_position_is_masked_before_mirroring(official_db):
    from sentry_training.offline.data import load_game
    from sentry_core.navigation_features import NavigationSpec, encode_world, FEATURE_NAMES

    connection = sqlite3.connect(official_db)
    connection.execute("UPDATE timeseries SET x=0,y=0 WHERE robot_id=101 AND 时刻秒=6")
    connection.commit()
    connection.close()
    game = load_game(official_db, 1, NavigationSpec())
    world = game.world(6, 103)
    assert 101 not in {track.robot_id for track in world.allies}
    x = encode_world(world, NavigationSpec(), mirror=True)
    assert sum(x[FEATURE_NAMES.index(f"ally.{i}.present")] for i in range(5)) == 3


def test_reward_reads_negative_damage_and_has_no_winner_in_observations(official_db):
    from sentry_training.offline.data import load_game, RewardWeights
    from sentry_core.navigation_features import NavigationSpec, encode_world

    game = load_game(official_db, 1, NavigationSpec())
    weights = RewardWeights(enemy_damage=1., own_damage=1., win=0.)
    red, blue = game.reward_steps(3, weights), game.reward_steps(103, weights)
    assert red[3] == pytest.approx(0.1)
    assert blue[3] == pytest.approx(-0.1)
    before = encode_world(game.world(6, 3), NavigationSpec())
    game.winner = "蓝"
    np.testing.assert_array_equal(before, encode_world(game.world(6, 3), NavigationSpec()))


def test_tracking_gap_does_not_join_future_goal_or_zero_bootstrap(official_db, tmp_path):
    from sentry_training.offline.data import BuildOptions, build_dataset, read_split

    connection = sqlite3.connect(official_db)
    connection.execute("UPDATE timeseries SET x=0,y=0 WHERE robot_id=3 AND 时刻秒=13")
    connection.commit()
    connection.close()
    out = tmp_path / "data"
    build_dataset(official_db, out, BuildOptions())
    seen_truncation = False
    for split in ("train", "validation", "test"):
        data = read_split(out, split)
        for episode in data.episodes:
            if episode["robot_id"] != 3:
                continue
            for i in range(episode["start"], episode["start"] + episode["length"]):
                assert not (data["times"][i] < 13 < data["times"][i] + data["durations"][i])
                if data["truncated"][i]:
                    seen_truncation = True
                    assert not data["terminated"][i]
                    assert data["discounts"][i] > 0
    assert seen_truncation


def test_regulation_clock_does_not_reveal_actual_early_end(official_db):
    from sentry_core.navigation_features import NavigationSpec, encode_world, FEATURE_NAMES
    from sentry_training.offline.data import load_game

    game = load_game(official_db, 1, NavigationSpec())
    x = encode_world(game.world(6, 3), NavigationSpec())
    assert x[FEATURE_NAMES.index("time.remaining")] == pytest.approx(414 / 420)


def test_dataset_hash_and_schema_are_checked(official_db, tmp_path):
    from sentry_training.offline.data import BuildOptions, build_dataset, read_split

    out = tmp_path / "data"
    build_dataset(official_db, out, BuildOptions())
    metadata = json.loads((out / "metadata.json").read_text())
    metadata["feature_names"][0] = "future_winner"
    (out / "metadata.json").write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match="feature"):
        read_split(out, "train")


@pytest.mark.skipif(importlib.util.find_spec("torch") is None, reason="optional torch training")
@pytest.mark.parametrize("algorithm", ["bc", "iql", "dt"])
def test_training_exports_without_accessing_the_test_split(official_db, tmp_path, algorithm):
    from sentry_training.offline.data import BuildOptions, build_dataset
    from sentry_training.offline.train import TrainingOptions, train
    from sentry_core.rl_policy import NumpyNavigationPolicy

    dataset, run = tmp_path / "data", tmp_path / algorithm
    build_dataset(official_db, dataset, BuildOptions())
    test_bytes = (dataset / "test.npz").read_bytes()
    (dataset / "test.npz").unlink()  # Model/checkpoint selection must not read this.
    options = TrainingOptions(algorithm=algorithm, steps=20, batch_size=16, eval_every=10,
                              hidden=16, dt_width=16, dt_heads=2, dt_layers=1,
                              context=3, device="cpu", seed=2)
    report = train(dataset, run, options)
    assert report["selection_split"] == "validation"
    assert report["optimizer_steps"] == 20
    assert (run / "network.safetensors").is_file()
    model = NumpyNavigationPolicy.load(run / "actor.json")
    assert model.payload["algorithm"] == algorithm
    assert np.isfinite(model.predict(np.zeros((1, 73), np.float32))).all()
    (dataset / "test.npz").write_bytes(test_bytes)
    from sentry_training.offline.evaluate import evaluate_actor
    report = evaluate_actor(dataset, run / "actor.json", device="cpu")
    assert report["split"] == "test"
    assert report["domains"]["infantry"]["samples"] > 0
    assert report["domains"]["sentry_transfer"]["samples"] > 0
    assert report["future_test_return_used"] is False
