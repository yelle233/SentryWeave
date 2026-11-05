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
