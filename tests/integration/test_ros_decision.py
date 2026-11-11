"""Decision node with trained artifacts; skipped when the overlay is not sourced."""

import importlib.util
import json

import pytest

try:
    ROS_AVAILABLE = all(importlib.util.find_spec(name) is not None
                        for name in ("rclpy", "sentry_interfaces.msg"))
except ModuleNotFoundError:
    ROS_AVAILABLE = False
pytestmark = [
    pytest.mark.ros,
    pytest.mark.skipif(not ROS_AVAILABLE, reason="requires a sourced Jazzy workspace"),
]


def running_world(config, now):
    """A running game with this sentry localized at a preferred point at ``now``."""
    from sentry_core.model import GameState, Observation, Phase, RobotStatus, Team
    from sentry_core.tracking import WorldModel

    model, position = WorldModel(config), config.points[0].position
    for stamp in (now - 0.4, now - 0.2, now):
        model.observe(Observation(config.self_id, Team.ALLY, position, stamp), stamp)
    model.update_game(GameState(now, config.season, config.own_team, Phase.RUNNING, 300.0, True),
                      now)
    model.update_robot(RobotStatus(config.self_id, Team.ALLY, now, 300.0, 400.0, True, ammo=0.8,
                                   source=1), now)
    return model.snapshot(now)


def test_decision_node_runs_models_of_another_field_in_rule_mode(tmp_path, config, capfd):
    import rclpy
    from rclpy.parameter import Parameter
    from sentry_core.config import load_config
    from sentry_ros.common import seconds
    from sentry_ros.decision_node import DecisionNode
    from test_model_fallback import SEASON, fitted_models

    # Artifacts fitted on the demo field must not stop the node on the 2027 season field.
    intent, utility = fitted_models(tmp_path, config)
    trace = tmp_path / "trace.jsonl"
    season = load_config(SEASON)
    rclpy.init()
    try:
        node = DecisionNode(parameter_overrides=[
            Parameter("config_file", value=str(SEASON)),
            Parameter("intent_model", value=str(intent)),
            Parameter("utility_model", value=str(utility)),
            Parameter("trace_path", value=str(trace)),
        ])
        try:
            assert node.engine.estimator.model is None
            assert node.config.weights == season.weights
            log = capfd.readouterr().err
            for name in ("intent_model", "utility_model"):
                assert f"{name} falls back to rule mode (MAP_FINGERPRINT_MISMATCH)" in log, log
            node.world = running_world(season, seconds(node))
            node._decide()
            record = json.loads(trace.read_text(encoding="utf-8").splitlines()[-1])
            fallbacks = record["decision"]["policy_info"]["model_fallbacks"]
            assert {name: value["reason"] for name, value in fallbacks.items()} == {
                "intent_model": "MAP_FINGERPRINT_MISMATCH",
                "utility_model": "MAP_FINGERPRINT_MISMATCH"}
            assert record["decision"]["task"]["kind"] != 0  # not HOLD
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
