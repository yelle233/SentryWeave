"""Intent/utility artifacts fitted on another field fall back to rule mode instead of failing."""

import json

import pytest

from conftest import ROOT

SEASON = ROOT / "src/sentry_bringup/config/tactics_rmuc2027.yaml"
TRAINED_WEIGHTS = (0.3, 0.2, 0.2, 0.1, 0.1, 0.1)


def fitted_models(directory, config, weights=TRAINED_WEIGHTS):
    """Minimal valid intent and utility artifacts fitted on ``config``'s field."""
    from sentry_core.intent import FEATURE_NAMES, FEATURE_VERSION
    from sentry_core.model import IntentType

    width = len(FEATURE_NAMES)
    payloads = {
        "intent.json": {
            "artifact_type": "sentry_intent_hmm", "schema_version": 1,
            "feature_version": FEATURE_VERSION, "feature_names": list(FEATURE_NAMES),
            "labels": [v.name for v in IntentType], "map_fingerprint": config.map_fingerprint,
            "means": [[0.5] * width] * 6, "variances": [[0.1] * width] * 6,
            "transition": [[1 / 6] * 6] * 6, "prior": [1 / 6] * 6, "temperature": 1.0,
        },
        "utility.json": {"artifact_type": "sentry_utility", "schema_version": 1,
                         "map_fingerprint": config.map_fingerprint, "weights": list(weights)},
    }
    directory.mkdir(parents=True, exist_ok=True)
    for name, payload in payloads.items():
        (directory / name).write_text(json.dumps(payload), encoding="utf-8")
    return directory / "intent.json", directory / "utility.json"


def test_models_fitted_on_the_old_field_fall_back_to_rule_mode(tmp_path, config):
    from sentry_core.config import load_config
    from sentry_core.learned import load_decision_models
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine
    from benchmark_decision import scenario

    # Trained on the demo field, then the 2027 season field became the default.
    intent, utility = fitted_models(tmp_path, config)
    season = load_config(SEASON)
    loaded, model, fallbacks = load_decision_models(season, intent, utility)
    assert model is None and loaded == season  # rule intents, tactics-file weights
    assert set(fallbacks) == {"intent_model", "utility_model"}
    for name, path in (("intent_model", intent), ("utility_model", utility)):
        assert fallbacks[name]["mode"] == "rule"
        assert fallbacks[name]["reason"] == "MAP_FINGERPRINT_MISMATCH"
        assert fallbacks[name]["path"] == str(path)
        assert fallbacks[name]["model_map_fingerprint"] == config.map_fingerprint
        assert fallbacks[name]["field_map_fingerprint"] == season.map_fingerprint
        assert "retrain" in fallbacks[name]["detail"]
    expected = json.loads(json.dumps(fallbacks))  # also proves it fits a JSON trace line
    engine = DecisionEngine(loaded, model, session_id="fallback", model_fallbacks=fallbacks)
    worlds = list(scenario(loaded, 4))
    for world in worlds[:3]:
        decision = engine.step(world)
        # Every decision (and so every trace record) says which model is not used and why.
        assert decision.policy_info["model_fallbacks"] == expected
    assert decision.task.kind != TaskType.HOLD
    # Editing one decision's record changes neither the engine nor later records.
    decision.policy_info["model_fallbacks"]["intent_model"]["mode"] = "edited"
    assert engine.step(worlds[3]).policy_info["model_fallbacks"] == expected


def test_only_the_mismatched_model_falls_back(tmp_path, config, world_factory):
    from sentry_core.config import load_config
    from sentry_core.learned import IntentModel, load_decision_models
    from sentry_core.policy import DecisionEngine

    season = load_config(SEASON)
    demo_intent, demo_utility = fitted_models(tmp_path / "demo", config)
    _, season_utility = fitted_models(tmp_path / "season", season)
    loaded, model, fallbacks = load_decision_models(season, demo_intent, season_utility)
    assert model is None and set(fallbacks) == {"intent_model"}
    assert loaded.weights == TRAINED_WEIGHTS
    # On the field they were fitted on, both are used and nothing is reported.
    same, used, none = load_decision_models(config, demo_intent, demo_utility)
    assert isinstance(used, IntentModel) and same.weights == TRAINED_WEIGHTS and none == {}
    decision = DecisionEngine(same, used, model_fallbacks=none).step(world_factory())
    assert "model_fallbacks" not in decision.policy_info


def test_broken_models_still_fail_and_strict_loaders_still_reject_other_fields(tmp_path,
                                                                                 config):
    from sentry_core.config import load_config
    from sentry_core.learned import (
        IntentModel, MapFingerprintMismatch, load_decision_models, load_utility,
    )

    intent, utility = fitted_models(tmp_path, config)
    season = load_config(SEASON)
    # Training and evaluation tools keep rejecting artifacts of another field.
    with pytest.raises(MapFingerprintMismatch, match="retrain"):
        IntentModel.load(intent, season)
    with pytest.raises(MapFingerprintMismatch, match="map fingerprint"):
        load_utility(utility, season)
    # Only a field mismatch falls back: a wrong or missing file is a configuration error.
    broken = tmp_path / "broken.json"
    broken.write_text(json.dumps({**json.loads(utility.read_text()), "artifact_type": "x"}))
    with pytest.raises(ValueError, match="unsupported utility model schema"):
        load_decision_models(season, None, broken)
    with pytest.raises(OSError):
        load_decision_models(season, tmp_path / "missing.json")
