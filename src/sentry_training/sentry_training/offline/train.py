"""Deterministic offline training. Checkpoint selection never opens the test split."""

from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import platform
import time

import numpy as np
import torch
from safetensors.torch import save_file

from sentry_core.navigation_features import NavigationSpec
from .data import read_split
from .export import export_actor
from .networks import DecisionTransformer, GaussianActor, IQLAgent


@dataclass(frozen=True)
class TrainingOptions:
    algorithm: str = "iql"
    seed: int = 7
    steps: int = 30000
    batch_size: int = 512
    eval_every: int = 1000
    lr: float = 3e-4
    hidden: int = 128
    dt_width: int = 64
    dt_heads: int = 4
    dt_layers: int = 2
    context: int = 8
    roles: tuple[int, ...] = (3, 4)
    device: str = "auto"
    expectile: float = 0.7
    beta: float = 3.
    observation_dropout: float = 0.03
    target_quantile: float = 0.9
    validation_limit: int = 20000

    def validate(self):
        if self.algorithm not in ("bc", "iql", "dt"):
            raise ValueError("choose bc, iql or dt")
        if min(self.steps, self.batch_size, self.eval_every, self.hidden, self.context) < 1:
            raise ValueError("training sizes must be positive")
        if (not 0 < self.lr < 1 or not 0 <= self.observation_dropout < 0.5
                or not 0 < self.target_quantile < 1 or not self.roles):
            raise ValueError("invalid training options")
        return self


def discounted_returns(data):
    returns = np.zeros(len(data), dtype=np.float32)
    for episode in data.episodes:
        value = 0.
        for i in range(episode["start"] + episode["length"] - 1, episode["start"] - 1, -1):
            value = float(data["rewards"][i]) + float(data["discounts"][i]) * value
            returns[i] = value
    return returns


class TensorReplay:
    def __init__(self, replay, device):
        self.replay = replay
        self.values = {
            name: torch.as_tensor(replay[name], device=device)
            for name in ("obs", "next_obs", "actions", "rewards", "discounts", "times")
        }
        starts = np.asarray([e["start"] for e in replay.episodes])
        self.starts = torch.as_tensor(starts[replay["episode_ids"]], device=device)
        self.returns = torch.as_tensor(discounted_returns(replay), device=device)
        self.device = device

    def transition(self, indices):
        return {key: values[indices] for key, values in self.values.items()}

    def context(self, indices, length, return_scale, target_return=None):
        take = indices[:, None] + torch.arange(-length + 1, 1, device=self.device)[None]
        first = self.starts[indices, None]
        valid = take >= first
        take = torch.maximum(take, first)
        rtg = (self.returns[take] / return_scale if target_return is None else
               torch.full(take.shape, target_return / return_scale, device=self.device))
        return (self.values["obs"][take], self.values["actions"][take], rtg[..., None],
                self.values["times"][take].long(), valid)


def augment_observations(obs, probability):
    """Mask complete observable channels, never change labels or leak reward information."""
    if probability <= 0:
        return obs
    result = obs.clone()
    batch = len(result)
    channels = [(5, 2), (7, 2), (9, 2), (11, 2)]
    if obs.shape[-1] == 85:
        channels.extend(((73, 2), (75, 2), (78, 7)))
    for start, length in channels:
        dropped = torch.rand(batch, device=obs.device) < probability
        result[dropped, ..., start:start + length] = 0
    for start in (13, 43):
        slots = result[..., start:start + 30].reshape(*result.shape[:-1], 5, 6)
        dropped = torch.rand(batch, 5, device=obs.device) < probability
        if obs.ndim == 3:
            dropped = dropped[:, None, :]
        slots.masked_fill_(dropped[..., None], 0.)
        # Keep "k-th nearest available robot" semantics when a channel disappears.
        rank = torch.arange(5, device=obs.device) + (slots[..., 5] == 0) * 5
        order = rank.argsort(dim=-1)[..., None].expand(*slots.shape)
        result[..., start:start + 30] = slots.gather(-2, order).reshape(*result.shape[:-1], 30)
    return result


@torch.no_grad()
def predict_torch(actor, tensors, indices, context, return_scale, target_return, batch_size=1024):
    actor.eval()
    predictions, nll = [], []
    dt = isinstance(actor, DecisionTransformer)
    for start in range(0, len(indices), batch_size if not dt else min(batch_size, 256)):
        chunk = indices[start:start + (batch_size if not dt else min(batch_size, 256))]
        if dt:
            args = tensors.context(chunk, context, return_scale, target_return)
            predictions.append(actor.action(*args)[:, -1].cpu().numpy())
            nll.append(-actor.log_prob(*args)[:, -1].cpu().numpy())
        else:
            obs, actions = tensors.values["obs"][chunk], tensors.values["actions"][chunk]
            predictions.append(actor.action(obs).cpu().numpy())
            nll.append(-actor.log_prob(obs, actions).cpu().numpy())
    return np.concatenate(predictions), np.concatenate(nll)


def navigation_metrics(predicted, target, obs, width, height, nll=None):
    pred_xy, tgt_xy = predicted[:, :2], target[:, :2]
    delta = (pred_xy - tgt_xy) * np.array([width, height])
    distance = np.linalg.norm(delta, axis=1)
    magnitude = np.linalg.norm(tgt_xy * np.array([width, height]), axis=1)
    moving = magnitude > 0.5
    moving_error = float(distance[moving].mean()) if moving.any() else float(distance.mean())
    stationary_error = float(distance[~moving].mean()) if (~moving).any() else moving_error
    proposed = obs[:, :2] + pred_xy
    metrics = {
        "samples": len(distance), "endpoint_error_m": float(distance.mean()),
        "endpoint_p90_m": float(np.percentile(distance, 90)),
        "moving_endpoint_error_m": moving_error,
        "stationary_endpoint_error_m": stationary_error,
        "balanced_endpoint_error_m": 0.5 * (moving_error + stationary_error),
        "moving_fraction": float(moving.mean()),
        "outside_field_fraction": float(((proposed < 0) | (proposed > 1)).any(axis=1).mean()),
        "nll": float(nll.mean()) if nll is not None else None,
    }
    if predicted.shape[1] == 6 and target.shape[1] == 6:
        metrics["posture_accuracy"] = float(
            (predicted[:, 2:5].argmax(axis=1) == target[:, 2:5].argmax(axis=1)).mean()
        )
        metrics["enhanced_accuracy"] = float(
            ((predicted[:, 5] > 0.0) == (target[:, 5] >= 0.5)).mean()
        )
    return metrics


def _warm_start(actor, path):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    arch = payload["architecture"]
    if (
        arch.get("type") != "mlp"
        or arch.get("hidden") != actor.hidden
        or arch.get("action_dim", 2) != actor.action_dim
    ):
        raise ValueError("BC warm start architecture does not match IQL")
    actor.load_state_dict(
        {key: torch.as_tensor(value, dtype=torch.float32)
         for key, value in payload["parameters"].items()},
        strict=True,
    )


def train(dataset, output, options=TrainingOptions(), initial_actor=None, progress=None):
    options.validate()
    os.environ.setdefault("CUBLAS_WORKSPACE_CONFIG", ":4096:8")
    output = Path(output)
    if (output / "report.json").exists():
        raise FileExistsError(f"training run already exists: {output}")
    output.mkdir(parents=True, exist_ok=True)
    np.random.seed(options.seed)
    torch.manual_seed(options.seed)
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    device = ("cuda" if torch.cuda.is_available() else "cpu") if options.device == "auto" else options.device
    training, validation = read_split(dataset, "train"), read_split(dataset, "validation")
    spec = NavigationSpec(**training.metadata["spec"])
    selected = training.indices(options.roles)
    validation_indices = validation.indices(options.roles)
    if not len(selected) or not len(validation_indices):
        raise ValueError("selected roles have no training or validation samples")
    if options.validation_limit and len(validation_indices) > options.validation_limit:
        validation_indices = np.random.default_rng(options.seed).choice(
            validation_indices, options.validation_limit, replace=False)
    replay, valid_replay = TensorReplay(training, device), TensorReplay(validation, device)
    eligible = torch.as_tensor(selected, dtype=torch.long, device=device)
    validate_ids = torch.as_tensor(validation_indices, dtype=torch.long, device=device)
    all_returns = discounted_returns(training)[selected]
    target_return = float(np.quantile(all_returns, options.target_quantile))
    return_scale = max(1., float(all_returns.std()))
    reward_scale = 1 / max(0.1, float(training["rewards"][selected].std()))
    obs_dim = training["obs"].shape[1]
    action_dim = training["actions"].shape[1]
    if options.algorithm == "iql":
        network = IQLAgent(
            obs_dim, options.hidden, expectile=options.expectile, beta=options.beta,
            reward_scale=reward_scale, action_dim=action_dim,
        )
        actor = network.actor
        if initial_actor:
            _warm_start(actor, initial_actor)
    elif options.algorithm == "bc":
        network = actor = GaussianActor(obs_dim, options.hidden, action_dim=action_dim)
    else:
        network = actor = DecisionTransformer(
            obs_dim, options.dt_width, options.dt_heads,
            options.dt_layers, options.context, action_dim=action_dim,
        )
    network.to(device)
    optimizer = torch.optim.AdamW(
        (p for p in network.parameters() if p.requires_grad), lr=options.lr, weight_decay=1e-4)
    provenance = {
        "dataset_id": training.metadata["dataset_id"], "source_sha256": training.metadata["source"]["sha256"],
        "training_roles": list(options.roles), "seed": options.seed,
        "python": platform.python_version(), "torch": torch.__version__, "numpy": np.__version__,
        "device": device, "gpu": torch.cuda.get_device_name(0) if device.startswith("cuda") else None,
        "data_kind": "SQLite match logs", "selection_split": "validation",
    }
    config = {**asdict(options), "actual_device": device, "reward_scale": reward_scale,
              "target_return": target_return, "return_scale": return_scale,
              "initial_actor": str(initial_actor) if initial_actor else None, "provenance": provenance}
    (output / "config.json").write_text(json.dumps(config, indent=2) + "\n")
    before = [p.detach().clone() for p in actor.parameters()]
    best = math.inf
    best_step, best_metrics = 0, None
    started = time.perf_counter()
    with (output / "metrics.jsonl").open("w") as log:
        for step in range(1, options.steps + 1):
            network.train()
            indices = eligible[torch.randint(len(eligible), (options.batch_size,), device=device)]
            if options.algorithm == "dt":
                obs, actions, returns, times, mask = replay.context(indices, options.context, return_scale)
                obs = augment_observations(obs, options.observation_dropout)
                log_probability = actor.log_prob(obs, actions, returns, times, mask)
                loss = -(log_probability * mask).sum() / mask.sum()
                losses, metrics = {"actor": loss}, {}
            else:
                batch = replay.transition(indices)
                batch["obs"] = augment_observations(batch["obs"], options.observation_dropout)
                batch["next_obs"] = augment_observations(batch["next_obs"], options.observation_dropout)
                if options.algorithm == "iql":
                    losses, metrics = network.losses(batch)
                else:
                    losses, metrics = {"actor": -actor.log_prob(batch["obs"], batch["actions"]).mean()}, {}
                loss = sum(losses.values())
            if not torch.isfinite(loss):
                raise FloatingPointError(f"nonfinite training loss at update {step}")
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            norm = torch.nn.utils.clip_grad_norm_(network.parameters(), 1.)
            if not torch.isfinite(norm):
                raise FloatingPointError(f"nonfinite gradient at update {step}")
            optimizer.step()
            if options.algorithm == "iql":
                network.update_target()
            if step == 1 or step % 200 == 0 or step == options.steps:
                row = {"step": step, "seconds": time.perf_counter() - started,
                       **{f"loss_{k}": float(v.detach().cpu()) for k, v in losses.items()},
                       **{k: float(v.detach().cpu()) for k, v in metrics.items()}}
                log.write(json.dumps(row, allow_nan=False) + "\n")
                log.flush()
            if step % options.eval_every == 0 or step == options.steps:
                predictions, nll = predict_torch(actor, valid_replay, validate_ids, options.context,
                                                 return_scale, target_return)
                values = navigation_metrics(predictions, validation["actions"][validation_indices],
                                            validation["obs"][validation_indices], spec.width, spec.height, nll)
                score = values["balanced_endpoint_error_m"]
                if score < best:
                    best, best_step, best_metrics = score, step, values
                    save_file({k: v.detach().cpu().contiguous() for k, v in network.state_dict().items()},
                              output / "network.safetensors")
                    export_actor(actor, output / "actor.json", options.algorithm, spec,
                                 {**provenance, "optimizer_step": step}, target_return, return_scale)
                log.write(json.dumps({"step": step, "validation": values, "best_step": best_step}) + "\n")
                log.flush()
                if progress:
                    progress(f"{options.algorithm} step={step} validation_balanced_m={score:.4f} best={best:.4f}")
        export_actor(actor, output / "actor_last.json", options.algorithm, spec,
                     {**provenance, "optimizer_step": options.steps}, target_return, return_scale)
    changed = sum(float((p.detach() - old).square().sum().cpu()) for p, old in zip(actor.parameters(), before))
    report = {
        "algorithm": options.algorithm, "optimizer_steps": options.steps, "best_step": best_step,
        "selection_split": "validation", "selection_metric": "balanced_endpoint_error_m",
        "validation": best_metrics, "training_samples": len(selected),
        "validation_samples": len(validation_indices), "actor_parameter_change_l2": math.sqrt(changed),
        "seconds": time.perf_counter() - started, **provenance,
        "dt_condition": "training observed-segment RTG; evaluation fixed training quantile"
                        if options.algorithm == "dt" else None,
        "test_split_read": False,
    }
    (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    return report
