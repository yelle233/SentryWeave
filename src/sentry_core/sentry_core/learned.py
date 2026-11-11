"""Portable JSON model inference. No pickle or training-only code is loaded on a robot."""

from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from .config import Config, validate_config
from .intent import FEATURE_NAMES, FEATURE_VERSION, softmax
from .model import IntentType


class MapFingerprintMismatch(ValueError):
    """An artifact was fitted on another field (``Config.map_fingerprint`` differs)."""

    def __init__(self, message: str, model_fingerprint, field_fingerprint: str):
        super().__init__(message)
        self.model_fingerprint, self.field_fingerprint = model_fingerprint, field_fingerprint


def _check_fingerprint(payload: dict, config: Config, message: str) -> None:
    if payload.get("map_fingerprint") != config.map_fingerprint:
        raise MapFingerprintMismatch(message, payload.get("map_fingerprint"),
                                     config.map_fingerprint)


class IntentModel:
    def __init__(self, payload: dict, config: Config):
        if payload.get("artifact_type") != "sentry_intent_hmm" or payload.get("schema_version") != 1:
            raise ValueError("unsupported intent model schema")
        if (payload.get("feature_version") != FEATURE_VERSION
                or payload.get("feature_names") != list(FEATURE_NAMES)
                or payload.get("labels") != [v.name for v in IntentType]):
            raise ValueError("intent feature or label contract mismatch")
        _check_fingerprint(payload, config,
                           "intent model map fingerprint mismatch; retrain for this map")
        self.payload = payload
        shapes = {"means": (6, len(FEATURE_NAMES)), "variances": (6, len(FEATURE_NAMES)),
                  "transition": (6, 6), "prior": (6,)}
        for name, shape in shapes.items():
            value = np.asarray(payload[name], dtype=np.float64)
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError(f"invalid {name} shape or values")
            setattr(self, name, value)
        if (self.variances < 1e-6).any():
            raise ValueError("variances must be positive")
        if (self.transition <= 0).any() or not np.allclose(self.transition.sum(axis=1), 1):
            raise ValueError("transition must be a positive row-stochastic matrix")
        if (self.prior <= 0).any() or not np.isclose(self.prior.sum(), 1):
            raise ValueError("prior must be a positive distribution")
        self.temperature = float(payload["temperature"])
        if not np.isfinite(self.temperature) or not 0.1 <= self.temperature <= 100:
            raise ValueError("invalid temperature")

    @classmethod
    def load(cls, path: str | Path, config: Config) -> "IntentModel":
        with open(path, encoding="utf-8") as stream:
            return cls(json.load(stream), config)

    def posterior(self, features, previous: np.ndarray | None) -> np.ndarray:
        x = np.asarray(features, dtype=np.float64)
        if x.shape != (len(FEATURE_NAMES),) or not np.isfinite(x).all():
            raise ValueError("invalid intent features")
        likelihood = -0.5 * np.sum(np.log(2 * np.pi * self.variances)
                                   + (x - self.means) ** 2 / self.variances, axis=1)
        prior = self.prior if previous is None else np.asarray(previous) @ self.transition
        return softmax(likelihood / self.temperature + np.log(np.maximum(prior, 1e-12)))


def load_utility(payload_or_path: dict | str | Path, config: Config) -> Config:
    if isinstance(payload_or_path, dict):
        payload = payload_or_path
    else:
        with open(payload_or_path, encoding="utf-8") as stream:
            payload = json.load(stream)
    if payload.get("artifact_type") != "sentry_utility" or payload.get("schema_version") != 1:
        raise ValueError("unsupported utility model schema")
    _check_fingerprint(payload, config,
                       "utility model map fingerprint mismatch; retrain for this map")
    return validate_config(replace(config, weights=tuple(payload["weights"])))


# What each optional decision artifact is replaced with when it was fitted on another field.
RULE_FALLBACKS = {"intent_model": "rule intent estimator",
                  "utility_model": "utility weights of the tactics file"}


def load_decision_models(config: Config, intent_path=None, utility_path=None):
    """Optional intent and utility artifacts for online decisions.

    An artifact fitted on another field (map fingerprint mismatch, for example one
    trained before the field file changed) does not stop the decision: that model falls
    back to rule mode, the rule intent estimator or the utility weights of the tactics
    file. ``fallbacks`` maps ``intent_model``/``utility_model`` to the reason, for the
    node log and ``Decision.policy_info["model_fallbacks"]``. A missing or malformed
    artifact still raises. Returns ``(config, intent model or None, fallbacks)``.
    """
    fallbacks = {}

    def fall_back(name, path, error):
        fallbacks[name] = {
            "mode": "rule", "reason": "MAP_FINGERPRINT_MISMATCH", "path": str(path),
            "model_map_fingerprint": error.model_fingerprint,
            "field_map_fingerprint": error.field_fingerprint,
            "detail": (f"{error} (model {str(error.model_fingerprint)[:12]}, field "
                       f"{error.field_fingerprint[:12]}); using the {RULE_FALLBACKS[name]}"),
        }

    if utility_path:
        try:
            config = load_utility(utility_path, config)
        except MapFingerprintMismatch as error:
            fall_back("utility_model", utility_path, error)
    model = None
    if intent_path:
        try:
            model = IntentModel.load(intent_path, config)
        except MapFingerprintMismatch as error:
            fall_back("intent_model", intent_path, error)
    return config, model, fallbacks
