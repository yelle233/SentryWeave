"""Held-out action diagnostics and explicitly limited, same-domain FQE."""

import copy
import json
from pathlib import Path
import time

import numpy as np
import torch
from torch.nn import functional as F

from sentry_core.rl_policy import NumpyNavigationPolicy
from .data import read_split, file_hash
from .networks import Critic, DecisionTransformer, GaussianActor
from .train import TensorReplay, discounted_returns, navigation_metrics, predict_torch


def load_actor(path, device="cpu"):
    portable = NumpyNavigationPolicy.load(path)
    architecture = portable.architecture
    if architecture["type"] == "mlp":
        actor = GaussianActor(73, architecture["hidden"])
    else:
        actor = DecisionTransformer(
            73, architecture["width"], architecture["heads"], architecture["layers"],
            architecture["context"], dropout=0., max_timestep=architecture["max_timestep"],
        )
    actor.load_state_dict({name: torch.as_tensor(value, dtype=torch.float32)
                           for name, value in portable.parameters.items()}, strict=True)
    return actor.to(device).eval().requires_grad_(False), portable


def clustered_interval(values, groups, seed=1729, replicates=1000):
    """Resample whole matches, keeping correlated rounds/sides/agents together."""
    groups, inverse = np.unique(np.asarray(groups), return_inverse=True)
    totals = np.bincount(inverse, weights=values, minlength=len(groups))
    counts = np.bincount(inverse, minlength=len(groups))
    rng = np.random.default_rng(seed)
    draw = rng.integers(len(groups), size=(replicates, len(groups)))
    means = totals[draw].sum(axis=1) / counts[draw].sum(axis=1)
    return [float(v) for v in np.percentile(means, [2.5, 97.5])]


def _group_labels(data):
    group = np.asarray([episode["group"] for episode in data.episodes])
    return group[data["episode_ids"]]


def constant_velocity_actions(observations, spec):
    # A future death or telemetry interruption can shorten a stored transition,
    # but that duration is not known when making a prediction.
    available = np.where(observations[:, 8] > 0.5,
                         np.clip(observations[:, 7], 0, 1) * spec.regulation_seconds,
                         spec.horizon)
    horizon = np.minimum(spec.horizon, available)
    velocity = observations[:, 2:4] * spec.speed_scale * (observations[:, 4:5] > 0.5)
    return np.clip(velocity * horizon[:, None] / np.array([spec.width, spec.height]), -1., 1.)


def evaluate_actor(dataset, actor_path, device="auto", split="test", output=None):
    torch.set_num_threads(1)
    if device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    data = read_split(dataset, split)
    actor, portable = load_actor(actor_path, device)
    if portable.payload.get("provenance", {}).get("dataset_id") != data.metadata["dataset_id"]:
        raise ValueError("actor and evaluation dataset IDs differ")
    tensors = TensorReplay(data, device)
    ids = torch.arange(len(data), device=device)
    started = time.perf_counter()
    prediction, nll = predict_torch(actor, tensors, ids, portable.architecture.get("context", 1),
                                     portable.return_scale, portable.target_return)
    elapsed = time.perf_counter() - started
    extent = np.array([portable.spec.width, portable.spec.height])
    errors = np.linalg.norm((prediction - data["actions"]) * extent, axis=1)
    velocity_baseline = constant_velocity_actions(data["obs"], portable.spec)
    groups = _group_labels(data)
    report = {
        "algorithm": portable.payload["algorithm"],
        "seed": portable.payload.get("provenance", {}).get("seed"),
        "actor": str(actor_path), "actor_sha256": file_hash(actor_path),
        "dataset_id": data.metadata["dataset_id"], "source": data.metadata["source"],
        "split": split, "future_test_return_used": False,
        "dt_history": "past logged completed macro actions; no current/future action attention",
        "target_return": portable.target_return, "batched_prediction_seconds": elapsed,
        "domains": {},
    }
    training_roles = set(portable.payload.get("provenance", {}).get("training_roles", (3, 4)))
    for name, roles in (("infantry", (3, 4)), ("sentry_transfer", (7,))):
        selected = data.indices(roles)
        if not len(selected):
            continue
        metrics = navigation_metrics(prediction[selected], data["actions"][selected],
                                     data["obs"][selected], *extent, nll[selected])
        metrics["match_groups"] = len(np.unique(groups[selected]))
        metrics["endpoint_cluster_ci95_m"] = clustered_interval(errors[selected], groups[selected])
        metrics["cross_role"] = not set(roles) <= training_roles
        metrics["zero_displacement_baseline"] = navigation_metrics(
            np.zeros_like(prediction[selected]), data["actions"][selected], data["obs"][selected],
            *extent,
        )
        metrics["constant_velocity_baseline"] = navigation_metrics(
            velocity_baseline[selected], data["actions"][selected], data["obs"][selected], *extent)
        report["domains"][name] = metrics
    sample = np.random.default_rng(1729).choice(len(data), min(2048, len(data)), replace=False)
    sample_ids = torch.as_tensor(sample, device=device)
    original = tensors.values["obs"]
    occlusion = {}
    for name, start, end in (("allies_missing", 13, 43), ("enemies_missing", 43, 73)):
        changed = original.clone()
        changed[:, start:end] = 0.
        tensors.values["obs"] = changed
        masked, _ = predict_torch(actor, tensors, sample_ids, portable.architecture.get("context", 1),
                                  portable.return_scale, portable.target_return)
        masked_error = np.linalg.norm((masked - data["actions"][sample]) * extent, axis=1)
        occlusion[name] = {
            "samples": len(sample),
            "mean_proposal_change_m": float(np.linalg.norm((masked - prediction[sample]) * extent, axis=1).mean()),
            "endpoint_error_change_m": float((masked_error - errors[sample]).mean()),
        }
    tensors.values["obs"] = original
    report["feature_occlusion"] = occlusion
    report["limitations"] = [
        "Endpoints are trajectory-derived action proxies, not recorded navigation commands.",
        "Prediction error and FQE are not measured match win rates.",
        "Ground-robot global positions are privileged unless legally provided to the real robot.",
        "DT reports use past logged actions; online projected-policy history is a different distribution.",
        "Feature occlusion measures input sensitivity, not the causal value of cooperation.",
    ]
    if output:
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        np.savez_compressed(output.with_suffix(".predictions.npz"), prediction=prediction, errors=errors)
    return report


def compare(dataset, actors, output, device="auto"):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    reports, errors = [], {}
    for actor_path in actors:
        name = Path(actor_path).parent.name
        report_file = output / f"{name}.json"
        report = evaluate_actor(dataset, actor_path, device, output=report_file)
        reports.append(report)
        with np.load(report_file.with_suffix(".predictions.npz"), allow_pickle=False) as arrays:
            errors[report["algorithm"], report["seed"]] = arrays["errors"]
    data = read_split(dataset, "test")
    groups = _group_labels(data)
    aggregate, paired = {}, []
    for algorithm in sorted({r["algorithm"] for r in reports}):
        aggregate[algorithm] = {}
        for domain in ("infantry", "sentry_transfer"):
            rows = [r["domains"][domain] for r in reports
                    if r["algorithm"] == algorithm and domain in r["domains"]]
            if not rows:
                continue
            aggregate[algorithm][domain] = {
                metric: {"mean": float(np.mean([r[metric] for r in rows])),
                         "seed_std": float(np.std([r[metric] for r in rows], ddof=1)) if len(rows) > 1 else 0.}
                for metric in ("endpoint_error_m", "moving_endpoint_error_m",
                               "stationary_endpoint_error_m", "balanced_endpoint_error_m",
                               "outside_field_fraction")
            }
            aggregate[algorithm][domain]["seeds"] = len(rows)
    for (algorithm, seed), error in errors.items():
        if algorithm == "bc" or ("bc", seed) not in errors:
            continue
        difference = error - errors["bc", seed]
        for domain, roles in (("infantry", (3, 4)), ("sentry_transfer", (7,))):
            selected = data.indices(roles)
            if len(selected):
                paired.append({
                    "algorithm": algorithm, "seed": seed, "domain": domain,
                    "endpoint_error_minus_bc_m": float(difference[selected].mean()),
                    "match_cluster_ci95_m": clustered_interval(difference[selected], groups[selected]),
                })
    result = {"schema_version": 1, "dataset_id": data.metadata["dataset_id"],
              "aggregate": aggregate, "paired_vs_bc": paired, "runs": reports,
              "interpretation": "Lower endpoint error means closer imitation of recorded movement, not higher win rate."}
    (output / "comparison.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    lines = ["# 官方日志独立测试", "", "全部测试组在训练前固定；结果为导航终点误差，不是胜率。",
             "", "| 算法 | 域 | Seed 数 | 平均终点误差 m | 移动样本误差 m | 越界比例 |",
             "|---|---|---:|---:|---:|---:|"]
    for algorithm, domains in aggregate.items():
        for domain, values in domains.items():
            lines.append(f"| {algorithm} | {domain} | {values['seeds']} | "
                         f"{values['endpoint_error_m']['mean']:.4f} | "
                         f"{values['moving_endpoint_error_m']['mean']:.4f} | "
                         f"{values['outside_field_fraction']['mean']:.4%} |")
    (output / "comparison.md").write_text("\n".join(lines) + "\n")
    return result


def fit_fqe(replay, actor, device="cpu", steps=8000, hidden=128, lr=3e-4,
            seed=17, roles=(3, 4), batch_size=512):
    if not isinstance(actor, GaussianActor):
        raise ValueError("Markov FQE cannot evaluate a history-conditioned DT policy")
    if steps < 1 or not 0 < lr < 1:
        raise ValueError("invalid FQE settings")
    torch.manual_seed(seed)
    torch.set_num_threads(1)
    actor = actor.to(device).eval().requires_grad_(False)
    selected = replay.indices(roles)
    if not len(selected):
        raise ValueError("no FQE training samples")
    arrays = {name: torch.as_tensor(replay[name][selected], device=device)
              for name in ("obs", "actions", "next_obs", "rewards", "discounts")}
    critic = Critic(arrays["obs"].shape[1], hidden).to(device)
    target = copy.deepcopy(critic).requires_grad_(False)
    optimizer = torch.optim.Adam(critic.parameters(), lr=lr)
    with torch.no_grad():
        next_actions = actor.action(arrays["next_obs"])
    started = time.perf_counter()
    for _ in range(steps):
        ids = torch.randint(len(selected), (batch_size,), device=device)
        with torch.no_grad():
            y = arrays["rewards"][ids] + arrays["discounts"][ids] * target(
                arrays["next_obs"][ids], next_actions[ids])
        loss = F.mse_loss(critic(arrays["obs"][ids], arrays["actions"][ids]), y)
        if not torch.isfinite(loss):
            raise FloatingPointError("nonfinite FQE loss")
        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(critic.parameters(), 5.)
        optimizer.step()
        with torch.no_grad():
            for p, q in zip(critic.parameters(), target.parameters()):
                q.lerp_(p, 0.01)
    return critic.eval(), {"td_mse": float(loss.detach().cpu()),
                           "steps": steps, "seed": seed, "seconds": time.perf_counter() - started}


def evaluate_fqe(dataset, actor_path, steps=8000, seeds=(17, 23), device="auto", output=None):
    if device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    actor, portable = load_actor(actor_path, device)
    if not isinstance(actor, GaussianActor):
        raise ValueError("DT needs a history-state evaluator; this FQE accepts BC/IQL only")
    training, test = read_split(dataset, "train"), read_split(dataset, "test")
    if portable.payload.get("provenance", {}).get("dataset_id") != training.metadata["dataset_id"]:
        raise ValueError("FQE dataset mismatch")
    roles = tuple(portable.payload["provenance"]["training_roles"])
    episodes = [ep for ep in test.episodes if ep["role"] in roles]
    starts = np.asarray([ep["start"] for ep in episodes])
    if not len(starts):
        raise ValueError("no same-domain FQE evaluation episodes")
    start_obs = torch.as_tensor(test["obs"][starts], device=device)
    runs = []
    for seed in seeds:
        critic, report = fit_fqe(training, actor, device=device, steps=steps, seed=seed, roles=roles)
        with torch.no_grad():
            value = critic(start_obs, actor.action(start_obs)).cpu().numpy()
        report.update(estimated_fragment_start_value=float(value.mean()),
                      std_across_fragments=float(value.std()))
        runs.append(report)
    complete = np.asarray([bool(test["terminated"][ep["start"] + ep["length"] - 1]) for ep in episodes])
    logged = discounted_returns(test)[starts]
    result = {
        "algorithm": portable.payload["algorithm"], "actor": str(actor_path),
        "roles": roles, "fit_split": "train", "evaluation_split": "test",
        "policy_variant": "raw deterministic actor, before legal-candidate projection",
        "initial_states": "first observation of each held-out life/telemetry fragment",
        "fragments": len(starts), "complete_fragments": int(complete.sum()),
        "logged_observed_fragment_return": float(logged.mean()),
        "logged_complete_fragment_return": float(logged[complete].mean()) if complete.any() else None,
        "runs": runs,
        "limitations": [
            "Partial observations, censoring and off-support actions bias FQE.",
            "Compare to same-domain BC as a diagnostic; subtraction does not prove bias removal.",
            "This is not a win-rate estimate and does not evaluate the projected ROS controller.",
        ],
    }
    if output:
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    return result
