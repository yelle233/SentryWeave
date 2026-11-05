import json
from dataclasses import replace

import numpy as np
import pytest


def test_grouped_split_and_model_train_load_round_trip(config, tmp_path):
    from sentry_core.learned import IntentModel
    from sentry_training.dataset import generate_dataset, split_sequences
    from sentry_training.fit import train_intent

    sequences = generate_dataset(config, episodes_per_class=8, seed=19, steps=40)
    splits = split_sequences(sequences, seed=7)
    groups = [{s.group_id for s in split} for split in splits]
    assert not groups[0] & groups[1] and not groups[0] & groups[2] and not groups[1] & groups[2]
    path = tmp_path / "intent.json"
    report = train_intent(sequences, config, path, seed=7)
    assert 0 <= report["test"]["accuracy"] <= 1
    assert report["test"]["accuracy"] > 0.45  # Better than six-class chance on unseen trajectories.
    model = IntentModel.load(path, config)
    posterior = model.posterior(sequences[0].features[-1], None)
    assert posterior.shape == (6,)
    assert posterior.sum() == pytest.approx(1)
    assert np.all(np.isfinite(posterior))
    changed = replace(config, robot_radius=config.robot_radius + 0.01)
    with pytest.raises(ValueError, match="map"):
        IntentModel.load(path, changed)


def test_model_rejects_corrupt_parameters(config, tmp_path):
    from sentry_core.learned import IntentModel
    from sentry_training.dataset import generate_dataset
    from sentry_training.fit import train_intent

    path = tmp_path / "intent.json"
    train_intent(generate_dataset(config, 6, seed=1, steps=20), config, path)
    value = json.loads(path.read_text())
    value["transition"][0][0] = -1
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match="transition"):
        IntentModel.load(path, config)


def test_dataset_roundtrip_retains_whole_sequences(config, tmp_path):
    from sentry_training.dataset import generate_dataset, read_dataset, write_dataset

    data = generate_dataset(config, 6, seed=1, steps=15)
    path = tmp_path / "data.jsonl"
    write_dataset(path, data)
    decoded = read_dataset(path)
    assert [s.episode_id for s in decoded] == [s.episode_id for s in data]
    assert np.array_equal(decoded[-1].features, data[-1].features)


def test_closed_loop_is_deterministic_and_uses_positions_for_reward(config):
    from sentry_training.simulation import rollout

    first = rollout(config, "conflict", 42, duration=12)
    second = rollout(config, "conflict", 42, duration=12)
    assert first["task_sequence"] == second["task_sequence"]
    assert first["reward"] == second["reward"]
    assert first["illegal_goals"] == 0
    assert first["distance_travelled"] > 0
    assert first["steps"] == 24


def test_cem_returns_valid_weights_and_separate_validation_seeds(config):
    from sentry_core.learned import load_utility
    from sentry_training.fit import tune_utility

    result = tune_utility(config, iterations=1, population=4,
                          scenarios=("team_push", "base_raid"), duration=8, seed=3)
    assert sum(result["weights"]) == pytest.approx(1)
    assert set(result["train_seeds"]).isdisjoint(result["validation_seeds"])
    assert load_utility(result, config).weights == tuple(result["weights"])


def test_replay_uses_the_same_decision_pipeline(config, tmp_path):
    from sentry_training.evaluate import replay
    from sentry_training.simulation import rollout

    path = tmp_path / "episode.jsonl"
    original = rollout(config, "team_push", 7, duration=10, trace_path=path)
    replayed = replay(config, path)
    assert replayed["task_sequence"] == original["task_sequence"]


def test_trained_weights_do_not_change_the_control_policy(config, tmp_path):
    from sentry_core.learned import IntentModel
    from sentry_training.dataset import generate_dataset
    from sentry_training.evaluate import evaluate
    from sentry_training.fit import train_intent
    from sentry_training.simulation import rollout

    path = tmp_path / "intent.json"
    train_intent(generate_dataset(config, 6, seed=1, steps=20), config, path)
    model = IntentModel.load(path, config)
    tuned = replace(config, weights=(0.8, 0.04, 0.04, 0.04, 0.04, 0.04))
    report = evaluate(config, ("fixed", "learned"), (901,), ("team_push",),
                      duration=10, model=model, learned_config=tuned)
    baseline = rollout(config, "team_push", 901, duration=10, policy="fixed")
    expected = rollout(tuned, "team_push", 901, duration=10, policy="learned", model=model)
    assert report["runs"][0]["reward"] == baseline["reward"]
    assert report["runs"][1]["reward"] == expected["reward"]


def test_online_feedback_is_replayed_before_its_next_decision(config, world_factory, tmp_path):
    from dataclasses import asdict
    from sentry_core.policy import DecisionEngine
    from sentry_core.serde import json_line, world_to_dict
    from sentry_training.evaluate import replay

    engine = DecisionEngine(config, session_id="logged")
    first_world = world_factory()
    first = engine.step(first_world)
    engine.feedback("logged", first.task.task_id, "FAILED", 10.2)
    second_world = world_factory(now=10.5)
    second = engine.step(second_world)
    records = [
        {"schema_version": 1, "world": world_to_dict(first_world),
         "decision": asdict(first), "feedback_before": []},
        {"schema_version": 1, "world": world_to_dict(second_world),
         "decision": asdict(second), "feedback_before": [
             {"session_id": "logged", "task_id": first.task.task_id, "state": "FAILED", "stamp": 10.2}
         ]},
    ]
    path = tmp_path / "online.jsonl"
    path.write_text("\n".join(json_line(r) for r in records) + "\n")
    report = replay(config, path)
    assert len(report["task_sequence"]) == 2
    assert report["task_sequence"][-1].endswith(second.task.goal_id)


def test_sensor_updates_are_faster_than_tactical_reassessment(config, tmp_path):
    from sentry_training.simulation import rollout

    path = tmp_path / "sensors.jsonl"
    rollout(config, "team_push", 7, duration=5, trace_path=path)
    last = json.loads(path.read_text().splitlines()[-1])
    assert len(last["world"]["allies"][0]["history"]) >= 15


def test_real_trace_extraction_only_uses_explicitly_labeled_source_times(config, tmp_path):
    from sentry_training.dataset import extract_labeled
    from sentry_training.simulation import rollout

    trace, annotations = tmp_path / "trace.jsonl", tmp_path / "labels.json"
    rollout(config, "team_push", 7, duration=8, trace_path=trace)
    annotations.write_text(json.dumps([
        {"episode_id": "team_push-7", "robot_id": 1, "start": 2., "end": 5.,
         "label": "ATTACK_OUTPOST"}
    ]))
    sequences = extract_labeled(config, trace, annotations)
    assert sequences
    for sequence in sequences:
        assert sequence.source == "human_labeled_replay"
        assert sequence.group_id == "team_push-7"
        assert (sequence.labels == 1).all()
        assert (sequence.times >= 2).all() and (sequence.times < 5).all()


def test_replay_translates_feedback_sessions_after_clock_rewind(config, world_factory, tmp_path):
    from dataclasses import asdict
    from sentry_core.policy import DecisionEngine
    from sentry_core.serde import json_line, world_to_dict
    from sentry_training.evaluate import replay

    engine = DecisionEngine(config, session_id="before-rewind")
    records = []
    for now, epoch in ((10., 0), (1., 1)):
        world = replace(world_factory(now=now), epoch=epoch)
        decision = engine.step(world)
        records.append({"schema_version": 1, "world": world_to_dict(world),
                        "decision": asdict(decision), "feedback_before": []})
    feedback = {"session_id": decision.task.session_id, "task_id": decision.task.task_id,
                "state": "FAILED", "stamp": 1.2}
    engine.feedback(feedback["session_id"], feedback["task_id"], "FAILED", 1.2)
    world = replace(world_factory(now=1.5), epoch=1)
    expected = engine.step(world)
    records.append({"schema_version": 1, "world": world_to_dict(world),
                    "decision": asdict(expected), "feedback_before": [feedback]})
    path = tmp_path / "rewound.jsonl"
    path.write_text("\n".join(json_line(r) for r in records) + "\n")
    assert replay(config, path)["task_sequence"][-1].endswith(expected.task.goal_id)
