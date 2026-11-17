"""Export inference-only, validated JSON; full training tensors use safetensors."""

import json
from pathlib import Path

from sentry_core.navigation_features import (
    FEATURE_NAMES,
    FEATURE_NAMES_V2,
    FEATURE_VERSION,
    FEATURE_VERSION_V2,
)
from sentry_core.rl_policy import NumpyNavigationPolicy
from .networks import DecisionTransformer, GaussianActor


def export_actor(
    network, path, algorithm, spec, provenance, target_return=0.0, return_scale=1.0
):
    is_v2 = getattr(network, "action_dim", 2) == 6
    schema_version = 2 if is_v2 else 1
    feat_ver = FEATURE_VERSION_V2 if is_v2 else FEATURE_VERSION
    feat_names = list(FEATURE_NAMES_V2 if is_v2 else FEATURE_NAMES)
    if isinstance(network, GaussianActor):
        architecture = {"type": "mlp", "hidden": network.hidden}
    elif isinstance(network, DecisionTransformer):
        architecture = {
            "type": "dt",
            "width": network.width,
            "heads": network.heads,
            "layers": network.depth,
            "context": network.context,
            "max_timestep": network.max_timestep,
        }
    else:
        raise TypeError("unsupported actor architecture")
    if is_v2:
        architecture["action_dim"] = 6
    payload = {
        "artifact_type": "sentry_navigation_policy",
        "schema_version": schema_version,
        "feature_version": feat_ver,
        "feature_names": feat_names,
        "algorithm": algorithm,
        "spec": spec.to_dict(),
        "architecture": architecture,
        "parameters": {
            name: tensor.detach().cpu().float().tolist()
            for name, tensor in network.state_dict().items()
        },
        "target_return": float(target_return),
        "return_scale": float(return_scale),
        "provenance": provenance,
    }
    NumpyNavigationPolicy(payload)  # Reject incompatible exports before writing artifact.
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    return payload
