"""Export inference-only, validated JSON; full training tensors use safetensors."""

import json
from pathlib import Path

from sentry_core.navigation_features import FEATURE_NAMES, FEATURE_VERSION
from sentry_core.rl_policy import NumpyNavigationPolicy
from .networks import DecisionTransformer, GaussianActor


def export_actor(network, path, algorithm, spec, provenance, target_return=0., return_scale=1.):
    if isinstance(network, GaussianActor):
        architecture = {"type": "mlp", "hidden": network.hidden}
    elif isinstance(network, DecisionTransformer):
        architecture = {"type": "dt", "width": network.width, "heads": network.heads,
                        "layers": network.depth, "context": network.context,
                        "max_timestep": network.max_timestep}
    else:
        raise TypeError("unsupported actor architecture")
    payload = {
        "artifact_type": "sentry_navigation_policy", "schema_version": 1,
        "feature_version": FEATURE_VERSION, "feature_names": list(FEATURE_NAMES),
        "algorithm": algorithm, "spec": spec.to_dict(), "architecture": architecture,
        "parameters": {name: tensor.detach().cpu().float().tolist()
                       for name, tensor in network.state_dict().items()},
        "target_return": float(target_return), "return_scale": float(return_scale),
        "provenance": provenance,
    }
    NumpyNavigationPolicy(payload)  # Reject incompatible exports before writing a deployment artifact.
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n")
    return payload
