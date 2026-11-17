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


def test_rl_mirror_is_auto_true_or_false(tmp_path):
    import rclpy
    from rclpy.exceptions import InvalidParameterTypeException
    from rclpy.parameter import Parameter
    from sentry_ros.decision_node import DecisionNode
    from test_model_fallback import SEASON
    from test_rl_policy import actor_file
    from test_ros_inputs import launch_argument_default

    # auto, the launch default, follows the field's mirror setting (the 2027 field mirrors
    # for BLUE); true and false force mirrored or unmirrored RL features on either side.
    assert launch_argument_default("rl_mirror", "decision.launch.py", "robot.launch.py") == "auto"
    actor = actor_file(tmp_path, width=28., height=15.)

    def decision(team, *mirror):
        return DecisionNode(parameter_overrides=[
            Parameter("config_file", value=str(SEASON)), Parameter("own_team", value=team),
            Parameter("rl_model", value=str(actor)),
            *(Parameter("rl_mirror", value=value) for value in mirror)])

    rclpy.init()
    try:
        for team, mirror, expected in (("BLUE", (), True), ("RED", ("auto",), False),
                                       ("BLUE", ("false",), False), ("RED", ("true",), True)):
            node = decision(team, *mirror)
            try:
                assert node.get_parameter("rl_mirror").value == (mirror[0] if mirror else "auto")
                assert node.engine.goal_prior.mirror is expected, (team, mirror)
            finally:
                node.destroy_node()
        with pytest.raises(ValueError, match="rl_mirror must be auto, true or false"):
            decision("BLUE", "yes")
        # Booleans are refused instead of being read as "follow the field".
        with pytest.raises(InvalidParameterTypeException):
            decision("BLUE", False)
    finally:
        rclpy.shutdown()


def test_goal_scope_launch_argument_and_node_parameter(tmp_path):
    import rclpy
    import yaml
    from rclpy.exceptions import InvalidParameterTypeException
    from rclpy.parameter import Parameter
    from sentry_ros.decision_node import DecisionNode
    from test_model_fallback import SEASON
    from test_ros_inputs import launch_argument_default

    assert launch_argument_default(
        "goal_scope", "decision.launch.py", "navigation.launch.py", "robot.launch.py"
    ) == ""

    raw = yaml.safe_load(SEASON.read_text(encoding="utf-8"))
    raw.update(goal_scope="own_half_and_highland", field=str(SEASON.parent / raw["field"]))
    scoped_yaml = tmp_path / "tactics_scoped.yaml"
    scoped_yaml.write_text(yaml.safe_dump(raw), encoding="utf-8")

    def make_node(cfg_path, *scope):
        return DecisionNode(parameter_overrides=[
            Parameter("config_file", value=str(cfg_path)),
            *(Parameter("goal_scope", value=v) for v in scope),
        ])

    rclpy.init()
    try:
        for cfg_path, override, expected in (
            (SEASON, (), "full"),
            (scoped_yaml, (), "own_half_and_highland"),
            (scoped_yaml, ("",), "own_half_and_highland"),
            (scoped_yaml, ("own_half",), "own_half"),
            (SEASON, ("own_half_and_highland",), "own_half_and_highland"),
        ):
            node = make_node(cfg_path, *override)
            try:
                assert node.config.goal_scope == expected
            finally:
                node.destroy_node()
        with pytest.raises(ValueError, match="goal_scope"):
            make_node(SEASON, "invalid_scope")
        with pytest.raises(InvalidParameterTypeException):
            make_node(SEASON, False)
    finally:
        rclpy.shutdown()


def test_decision_node_publishes_sentry_cmd_and_spin_and_guard_gates_spin(tmp_path):
    import time
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import Float32
    from sentry_core.config import load_config
    from sentry_core.model import GameState, Observation, Phase, RobotStatus, Team
    from sentry_core.tracking import WorldModel
    from sentry_interfaces.msg import MotionPermit, SentryRefereeCmd, TacticalTask
    from sentry_ros.codec import stamp, world_to_msg
    from sentry_ros.common import LATEST, seconds
    from sentry_ros.decision_node import DecisionNode
    from sentry_ros.guard_node import GuardNode
    from test_model_fallback import SEASON

    season = load_config(SEASON)
    rclpy.init()
    try:
        decision = DecisionNode(parameter_overrides=[Parameter("config_file", value=str(SEASON))])
        guard = GuardNode(parameter_overrides=[
            Parameter("inject_spin_into_cmd_vel", value=True),
            Parameter("max_spin_angular", value=8.0),
        ])
        probe = Node("posture_spin_probe")
        executor = SingleThreadedExecutor()
        for n in (decision, guard, probe):
            executor.add_node(n)
        tasks, cmds, spins, cmd_vels = [], [], [], []
        probe.create_subscription(TacticalTask, "tactical/task", tasks.append, LATEST)
        probe.create_subscription(SentryRefereeCmd, "referee/sentry_cmd", cmds.append, LATEST)
        probe.create_subscription(Float32, "chassis/spin", spins.append, LATEST)
        probe.create_subscription(Twist, "cmd_vel", cmd_vels.append, LATEST)
        permits = probe.create_publisher(MotionPermit, "motion/permit", LATEST)

        now = seconds(decision)
        model = WorldModel(season)
        pos = season.points[0].position
        for t in (now - 0.4, now - 0.2, now):
            model.observe(Observation(season.self_id, Team.ALLY, pos, t), t)
        model.update_game(GameState(now, "rmuc2027", "RED", Phase.RUNNING, 300.0, True), now)
        model.update_robot(
            RobotStatus(
                season.self_id, Team.ALLY, now, hp=340.0, max_hp=400.0, alive=True, ammo=0.8,
                source=1, heat_ratio=0.85, last_hit_stamp=now, hit_armor_id=0,
            ),
            now,
        )
        # Immediate _world reaction on hit without waiting for the 2 Hz timer:
        decision._world(world_to_msg(model.snapshot(now)))
        deadline = time.monotonic() + 3.0
        while (not cmds or not spins) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)

        assert cmds and cmds[-1].posture == "attack"
        assert cmds[-1].raw_sentry_cmd == (1 << 21)
        assert spins and abs(spins[-1].data) == pytest.approx(season.spin_speed_rad_s)
        assert tasks and tasks[-1].spin_enabled is True

        # Without MotionPermit.allowed=True, GuardNode MUST output 0 angular.z!
        guard._tick()
        for _ in range(5):
            executor.spin_once(timeout_sec=0.02)
        assert cmd_vels and cmd_vels[-1].angular.z == 0.0

        # Grant MotionPermit -> GuardNode injects clamped spin angular.z; revoke -> immediate 0.0!
        permit = MotionPermit()
        permit.header.stamp = stamp(seconds(probe))
        permit.executor_session, permit.sequence, permit.allowed = "exec-1", 1, True
        permits.publish(permit)
        for _ in range(8):
            executor.spin_once(timeout_sec=0.02)
        guard._tick()
        for _ in range(5):
            executor.spin_once(timeout_sec=0.02)
        assert abs(cmd_vels[-1].angular.z) == pytest.approx(season.spin_speed_rad_s)

        permit.header.stamp = stamp(seconds(probe))
        permit.sequence, permit.allowed = 2, False
        permits.publish(permit)
        for _ in range(8):
            executor.spin_once(timeout_sec=0.02)
        guard._tick()
        for _ in range(5):
            executor.spin_once(timeout_sec=0.02)
        assert cmd_vels[-1].angular.z == 0.0
    finally:
        for n in (probe, guard, decision):
            n.destroy_node()
        rclpy.shutdown()

