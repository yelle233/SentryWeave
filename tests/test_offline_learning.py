"""Algorithm and export tests. Optional torch dependency never blocks core/ROS collection."""

import importlib.util
import json

import numpy as np
import pytest


pytestmark = pytest.mark.skipif(importlib.util.find_spec("torch") is None,
                                reason="requires the optional offline RL environment")


def test_iql_uses_macro_discount_and_stops_target_gradients():
    import torch
    from sentry_training.offline.networks import IQLAgent

    torch.set_num_threads(1)
    agent = IQLAgent(4, hidden=16, reward_scale=1.)
    with torch.no_grad():
        for parameter in agent.value.parameters():
            parameter.zero_()
        agent.value.layers[-1].bias.fill_(2.)
    batch = {"obs": torch.zeros(2, 4), "next_obs": torch.zeros(2, 4),
             "actions": torch.zeros(2, 2), "rewards": torch.tensor([0.3, 0.3]),
             "discounts": torch.tensor([0.9 ** 5, 0.])}
    np.testing.assert_allclose(agent.td_target(batch).numpy(), [0.3 + 2 * 0.9 ** 5, 0.3], rtol=1e-6)
    losses, _ = agent.losses(batch)
    sum(losses.values()).backward()
    assert all(p.grad is None for p in agent.target_q1.parameters())
    assert all(p.grad is None for p in agent.target_q2.parameters())
    assert all(torch.isfinite(p.grad).all() for p in agent.actor.parameters() if p.grad is not None)


def test_iql_learns_reward_preference_beyond_behavior_cloning():
    import torch
    from sentry_training.offline.networks import GaussianActor, IQLAgent

    torch.set_num_threads(1)
    torch.manual_seed(8)
    obs = torch.zeros(100, 4)
    actions = torch.full((100, 2), -0.6)
    actions[70:] = 0.6
    rewards = torch.zeros(100)
    rewards[70:] = 2.
    batch = {"obs": obs, "actions": actions, "next_obs": obs,
             "rewards": rewards, "discounts": torch.zeros(100)}
    bc = GaussianActor(4, hidden=32)
    optimizer = torch.optim.Adam(bc.parameters(), lr=0.003)
    for _ in range(200):
        loss = -bc.log_prob(obs, actions).mean()
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
    agent = IQLAgent(4, hidden=32, beta=3., reward_scale=1.)
    agent.actor.load_state_dict(bc.state_dict())
    optimizer = torch.optim.Adam([p for p in agent.parameters() if p.requires_grad], lr=0.003)
    for _ in range(600):
        losses, _ = agent.losses(batch)
        optimizer.zero_grad()
        sum(losses.values()).backward()
        torch.nn.utils.clip_grad_norm_(agent.parameters(), 5.)
        optimizer.step()
        agent.update_target()
    with torch.no_grad():
        imitation = bc.action(obs[:1])[0, 0].item()
        learned = agent.actor.action(obs[:1])[0, 0].item()
    assert learned > 0.35 and learned > imitation + 0.4, (imitation, learned)


def test_decision_transformer_cannot_read_current_action_or_future_tokens():
    import torch
    from sentry_training.offline.networks import DecisionTransformer

    torch.manual_seed(3)
    model = DecisionTransformer(73, width=32, heads=2, layers=2, context=4, dropout=0.).eval()
    obs, actions = torch.randn(2, 4, 73), torch.randn(2, 4, 2).tanh()
    returns, times, mask = torch.randn(2, 4, 1), torch.arange(4).repeat(2, 1), torch.ones(2, 4).bool()
    with torch.no_grad():
        original = model.action(obs, actions, returns, times, mask)
        altered = obs.clone()
        altered[:, 1:] += 100
        different_returns = returns.clone()
        different_returns[:, 1:] -= 100
        predicted = model.action(altered, -actions, different_returns, times, mask)
    torch.testing.assert_close(original[:, 0], predicted[:, 0], atol=1e-6, rtol=1e-6)


def test_dt_padding_is_finite_and_cannot_change_valid_predictions():
    import torch
    from sentry_training.offline.networks import DecisionTransformer

    torch.manual_seed(3)
    model = DecisionTransformer(73, width=32, heads=2, layers=1, context=4, dropout=0.).eval()
    obs, actions = torch.randn(1, 4, 73), torch.randn(1, 4, 2).tanh()
    rtg, times = torch.randn(1, 4, 1), torch.arange(4)[None]
    mask = torch.tensor([[False, False, True, True]])
    with torch.no_grad():
        full = model.action(obs, actions, rtg, times, mask)
        short = model.action(obs[:, 2:], actions[:, 2:], rtg[:, 2:], times[:, 2:], mask[:, 2:])
    assert torch.isfinite(full).all()
    torch.testing.assert_close(full[:, 2:], short, atol=1e-6, rtol=1e-6)


@pytest.mark.parametrize("algorithm", ["bc", "dt"])
def test_numpy_export_matches_torch_and_rejects_corrupt_weights(tmp_path, algorithm):
    import torch
    from sentry_core.navigation_features import NavigationSpec
    from sentry_core.rl_policy import NumpyNavigationPolicy
    from sentry_training.offline.export import export_actor
    from sentry_training.offline.networks import GaussianActor, DecisionTransformer

    torch.manual_seed(3)
    if algorithm == "bc":
        network = GaussianActor(73, hidden=32).eval()
    else:
        network = DecisionTransformer(73, width=32, heads=2, layers=2, context=4, dropout=0.).eval()
    path = tmp_path / "actor.json"
    export_actor(network, path, algorithm, NavigationSpec(), {"dataset_id": "unit-test"}, target_return=2.)
    portable = NumpyNavigationPolicy.load(path)
    obs = np.random.default_rng(9).uniform(-1, 1, (3, 73)).astype(np.float32)
    actions = np.zeros((3, 2), dtype=np.float32)
    rtg = np.ones((3, 1), dtype=np.float32) * 2
    times = np.array([1, 6, 11], dtype=np.int64)
    with torch.no_grad():
        if algorithm == "bc":
            expected = network.action(torch.from_numpy(obs[-1:])).numpy()[0]
        else:
            expected = network.action(torch.from_numpy(obs[None]), torch.from_numpy(actions[None]),
                                      torch.from_numpy(rtg[None]), torch.from_numpy(times[None]),
                                      torch.ones(1, 3).bool()).numpy()[0, -1]
    np.testing.assert_allclose(portable.predict(obs, actions, rtg, times), expected, atol=3e-5, rtol=3e-5)
    payload = json.loads(path.read_text())
    key = next(k for k in payload["parameters"] if k.endswith("weight"))
    payload["parameters"][key] = [[float("nan")]]
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError):
        NumpyNavigationPolicy.load(path)


def test_dt_deployable_evaluation_does_not_condition_on_test_future_rewards():
    import torch
    from sentry_training.offline.data import Replay
    from sentry_training.offline.networks import DecisionTransformer
    from sentry_training.offline.train import TensorReplay, predict_torch

    torch.manual_seed(3)
    n = 5
    arrays = {"obs": np.zeros((n, 73), np.float32), "next_obs": np.zeros((n, 73), np.float32),
              "actions": np.zeros((n, 2), np.float32), "rewards": np.zeros(n, np.float32),
              "discounts": np.full(n, .95, np.float32), "times": np.arange(n, dtype=np.float32),
              "episode_ids": np.zeros(n, np.int64)}
    episode = [{"start": 0, "length": n}]
    model = DecisionTransformer(73, width=16, heads=2, layers=1, context=3, dropout=0.).eval()
    initial = TensorReplay(Replay(arrays, episode, {}), "cpu")
    arrays = {**arrays, "rewards": np.full(n, 999., np.float32)}
    altered = TensorReplay(Replay(arrays, episode, {}), "cpu")
    ids = torch.arange(n)
    first, _ = predict_torch(model, initial, ids, 3, 1., 2.)
    second, _ = predict_torch(model, altered, ids, 3, 1., 2.)
    np.testing.assert_array_equal(first, second)


def test_fqe_recovers_a_known_terminal_return():
    import torch
    from sentry_training.offline.data import Replay
    from sentry_training.offline.evaluate import fit_fqe
    from sentry_training.offline.networks import GaussianActor

    torch.set_num_threads(1)
    n = 64
    data = Replay({"obs": np.zeros((n, 4), np.float32),
                   "next_obs": np.zeros((n, 4), np.float32),
                   "actions": np.zeros((n, 2), np.float32),
                   "rewards": np.full(n, 2., np.float32),
                   "discounts": np.zeros(n, np.float32),
                   "roles": np.full(n, 3, np.int64)}, [], {})
    actor = GaussianActor(4, hidden=16)
    for parameter in actor.parameters():
        parameter.data.zero_()
    critic, report = fit_fqe(data, actor, device="cpu", steps=300, hidden=16, lr=0.01)
    with torch.no_grad():
        value = critic(torch.zeros(1, 4), torch.zeros(1, 2)).item()
    assert value == pytest.approx(2., abs=0.1)
    assert report["td_mse"] < 0.02


def test_velocity_baseline_uses_only_observable_horizon_information():
    from sentry_core.navigation_features import NavigationSpec
    from sentry_training.offline.evaluate import constant_velocity_actions

    obs = np.zeros((2, 73), np.float32)
    obs[:, 2] = 0.1  # 0.6 m/s
    obs[:, 4] = 1.
    obs[0, 7], obs[0, 8] = 2 / 420, 1.
    prediction = constant_velocity_actions(obs, NavigationSpec())
    np.testing.assert_allclose(prediction[:, 0], [1.2 / 28, 3. / 28], rtol=1e-6)


@pytest.mark.parametrize("algorithm", ["bc", "dt"])
def test_v2_joint_numpy_export_matches_torch_across_all_6_outputs(tmp_path, algorithm):
    import torch
    from sentry_core.navigation_features import NavigationSpec
    from sentry_core.rl_policy import NumpyNavigationPolicy
    from sentry_training.offline.export import export_actor
    from sentry_training.offline.networks import DecisionTransformer, GaussianActor

    torch.manual_seed(11)
    if algorithm == "bc":
        network = GaussianActor(85, hidden=32, action_dim=6).eval()
    else:
        network = DecisionTransformer(
            85, width=32, heads=2, layers=2, context=4, dropout=0.0, action_dim=6
        ).eval()
    path = tmp_path / f"actor_v2_{algorithm}.json"
    export_actor(
        network, path, algorithm, NavigationSpec(), {"dataset_id": "v2-unit"}, target_return=1.5
    )
    portable = NumpyNavigationPolicy.load(path)
    assert portable.schema_version == 2 and portable.action_dim == 6
    obs = np.random.default_rng(13).uniform(-1, 1, (3, 85)).astype(np.float32)
    actions = np.zeros((3, 6), dtype=np.float32)
    actions[:, 4] = 1.0  # past "move" one-hot
    rtg = np.full((3, 1), 1.5, dtype=np.float32)
    times = np.array([0, 5, 10], dtype=np.int64)
    with torch.no_grad():
        if algorithm == "bc":
            expected = network.action(torch.from_numpy(obs[-1:])).numpy()[0]
        else:
            expected = network.action(
                torch.from_numpy(obs[None]),
                torch.from_numpy(actions[None]),
                torch.from_numpy(rtg[None]),
                torch.from_numpy(times[None]),
                torch.ones(1, 3, dtype=torch.bool),
            ).numpy()[0, -1]
    actual = portable.predict(obs, actions, rtg, times)
    assert actual.shape == (6,) and np.all(np.abs(actual[:2]) <= 1.0)
    np.testing.assert_allclose(actual, expected, atol=3e-5, rtol=3e-5)


def test_v2_iql_learns_high_reward_posture_and_navigation_jointly():
    import torch
    from sentry_training.offline.networks import GaussianActor, IQLAgent

    torch.set_num_threads(1)
    torch.manual_seed(21)
    obs = torch.zeros(120, 85)
    # Low-reward majority: move (-0.5, 0) + posture "move" [0, 0, 1] + enhanced=0
    actions = torch.zeros(120, 6)
    actions[:85, 0] = -0.5
    actions[:85, 4] = 1.0
    # High-reward minority: move (+0.5, 0) + posture "attack" [1, 0, 0] + enhanced=1
    actions[85:, 0] = 0.5
    actions[85:, 2] = 1.0
    actions[85:, 5] = 1.0
    rewards = torch.zeros(120)
    rewards[85:] = 3.0
    batch = {
        "obs": obs, "actions": actions, "next_obs": obs,
        "rewards": rewards, "discounts": torch.zeros(120),
    }
    bc = GaussianActor(85, hidden=32, action_dim=6)
    opt_bc = torch.optim.Adam(bc.parameters(), lr=3e-3)
    for _ in range(180):
        loss = -bc.log_prob(obs, actions).mean()
        opt_bc.zero_grad()
        loss.backward()
        opt_bc.step()

    agent = IQLAgent(85, hidden=32, beta=3.5, reward_scale=1.0, action_dim=6)
    agent.actor.load_state_dict(bc.state_dict())
    opt_iql = torch.optim.Adam([p for p in agent.parameters() if p.requires_grad], lr=3e-3)
    for _ in range(500):
        losses, _ = agent.losses(batch)
        opt_iql.zero_grad()
        sum(losses.values()).backward()
        torch.nn.utils.clip_grad_norm_(agent.parameters(), 5.0)
        opt_iql.step()
        agent.update_target()

    with torch.no_grad():
        pred = agent.actor.action(obs[:1])[0]
    assert pred[0].item() > 0.25
    assert int(pred[2:5].argmax().item()) == 0  # "attack" wins over majority "move"
    assert pred[5].item() > 0.0  # enhanced logit > 0 (prob > 0.5)


def test_v2_dataset_train_warm_start_evaluate_and_fqe_end_to_end(tmp_path):
    from sentry_core.navigation_features import (
        FEATURE_NAMES_V2,
        FEATURE_VERSION_V2,
        NavigationSpec,
    )
    from sentry_training.offline.data import SPLITS, file_hash, read_split
    from sentry_training.offline.evaluate import evaluate_actor, evaluate_fqe
    from sentry_training.offline.train import TrainingOptions, train

    dataset = tmp_path / "dataset_v2"
    dataset.mkdir()
    rng = np.random.default_rng(31)
    spec = NavigationSpec()
    meta = {
        "schema_version": 2,
        "feature_version": FEATURE_VERSION_V2,
        "feature_names": list(FEATURE_NAMES_V2),
        "spec": spec.to_dict(),
        "source": {"filename": "arena2027.jsonl", "sha256": "0" * 64},
        "splits": {},
        "dataset_id": "v2-e2e-dataset",
    }
    for split in SPLITS:
        n = 16
        obs = rng.uniform(0.0, 1.0, (n, 85)).astype(np.float32)
        next_obs = obs.copy()
        actions = np.zeros((n, 6), dtype=np.float32)
        actions[:, :2] = rng.uniform(-0.2, 0.2, (n, 2)).astype(np.float32)
        actions[:8, 3] = 1.0
        actions[8:, 2] = 1.0
        actions[8:, 5] = 1.0
        terminated = np.zeros(n, dtype=np.float32)
        terminated[[7, 15]] = 1.0
        discounts = np.where(terminated > 0.5, 0.0, 0.95).astype(np.float32)
        arrays = {
            "obs": obs,
            "next_obs": next_obs,
            "actions": actions,
            "rewards": rng.normal(0.5, 0.2, n).astype(np.float32),
            "discounts": discounts,
            "terminated": terminated,
            "truncated": np.zeros(n, dtype=np.float32),
            "durations": np.full(n, 5.0, dtype=np.float32),
            "times": np.tile(np.arange(0, 40, 5, dtype=np.float32), 2),
            "episode_ids": np.repeat([0, 1], 8).astype(np.int64),
            "roles": np.full(n, 7, dtype=np.int64),
        }
        npz_path = dataset / f"{split}.npz"
        np.savez_compressed(npz_path, **arrays)
        episodes = [
            {"episode_id": 0, "start": 0, "length": 8, "group": f"{split}/0", "role": 7},
            {"episode_id": 1, "start": 8, "length": 8, "group": f"{split}/1", "role": 7},
        ]
        (dataset / f"{split}.episodes.json").write_text(json.dumps(episodes), encoding="utf-8")
        meta["splits"][split] = {
            "transitions": n,
            "episodes": 2,
            "sha256": file_hash(npz_path),
            "roles": {"7": n},
        }
    (dataset / "metadata.json").write_text(json.dumps(meta), encoding="utf-8")

    loaded = read_split(dataset, "train")
    assert loaded["obs"].shape == (16, 85) and loaded["actions"].shape == (16, 6)

    bc_dir = tmp_path / "bc_v2"
    bc_opts = TrainingOptions(
        algorithm="bc", steps=10, batch_size=8, eval_every=5, hidden=16, roles=(7,), device="cpu"
    )
    bc_report = train(dataset, bc_dir, bc_opts)
    assert "posture_accuracy" in bc_report["validation"]
    assert "enhanced_accuracy" in bc_report["validation"]

    iql_dir = tmp_path / "iql_v2"
    iql_opts = TrainingOptions(
        algorithm="iql", steps=10, batch_size=8, eval_every=5, hidden=16, roles=(7,), device="cpu"
    )
    iql_report = train(dataset, iql_dir, iql_opts, initial_actor=bc_dir / "actor.json")
    assert iql_report["validation"]["posture_accuracy"] >= 0.0

    eval_report = evaluate_actor(dataset, iql_dir / "actor.json", device="cpu")
    assert "posture_accuracy" in eval_report["domains"]["sentry_transfer"]
    assert "allies_missing" in eval_report["feature_occlusion"]

    fqe_report = evaluate_fqe(
        dataset, iql_dir / "actor.json", steps=20, seeds=(7,), device="cpu"
    )
    assert fqe_report["fragments"] == 2
    assert np.isfinite(fqe_report["runs"][0]["estimated_fragment_start_value"])


