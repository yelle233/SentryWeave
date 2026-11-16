"""Typed input topics through the real world node; skipped when the overlay is not sourced."""

from dataclasses import replace
import importlib.util
import time

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


def test_codec_round_trips_typed_inputs(world_factory):
    from sentry_core.model import GameState, Phase, RobotStatus, StructureState, Team
    from sentry_ros.codec import world_from_msg, world_to_msg

    world = replace(
        world_factory(hp=None),
        game=GameState(10.0, "rmuc2027", "BLUE", Phase.RUNNING, None, True, 1),
        robots={7: RobotStatus(7, Team.ALLY, 10.0, 200.0, 400.0, True, 3, 0.5, 1),
                103: RobotStatus(103, Team.ENEMY, 9.5, alive=False, source=2)},
        structures={"enemy_outpost": StructureState("enemy_outpost", 9.5, 750.0, 1500.0, True,
                                                    None, 1)},
    )
    assert world_from_msg(world_to_msg(world)) == world


def test_robot_status_hp_and_max_hp_are_valid_independently(world_factory):
    from sentry_core.model import RobotStatus, Team
    from sentry_interfaces import msg as wire
    from sentry_ros.codec import (
        robot_status_from_msg, robot_status_to_msg, world_from_msg, world_to_msg,
    )

    # A publisher that knows hp but not max_hp keeps hp (0 still means not alive); without
    # max_hp the ratio stays unknown, never 0 or full. max_hp alone round-trips as well.
    robots = {7: RobotStatus(7, Team.ALLY, 10.0, hp=150.0, source=1),
              103: RobotStatus(103, Team.ENEMY, 10.0, hp=0.0, source=2),
              104: RobotStatus(104, Team.ENEMY, 10.0, max_hp=400.0, source=2),
              1: RobotStatus(1, Team.ALLY, 10.0, 100.0, 200.0, source=1)}
    for status in robots.values():
        message = robot_status_to_msg(status)
        assert (message.hp_valid, message.max_hp_valid) == (
            status.hp is not None, status.max_hp is not None)
        assert robot_status_from_msg(message) == status
    world = replace(world_factory(), robots=robots)
    decoded = world_from_msg(world_to_msg(world))
    assert decoded == world
    assert {i: (s.hp, s.hp_ratio, s.known_alive) for i, s in decoded.robots.items()} == {
        7: (150.0, None, True), 103: (0.0, None, False), 104: (None, None, None),
        1: (100.0, 0.5, True)}
    # A bridge that only fills hp: an unflagged max_hp is unknown, not 0.
    only_hp = robot_status_from_msg(wire.RobotStatus(robot_id=7, team=wire.RobotStatus.ALLY,
                                                     hp=80.0, hp_valid=True))
    assert (only_hp.hp, only_hp.max_hp, only_hp.hp_ratio) == (80.0, None, None)


def test_world_node_publishes_typed_topics_and_typed_values_win(config):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sentry_interfaces.msg import (
        GameStatus, RefereeState, RobotStatusArray, StructureStatus, WorldState,
    )
    from sentry_core.inputs import view_inputs
    from sentry_core.model import (
        GameState, Phase, Referee, RobotStatus, StructureState, Team,
    )
    from sentry_ros.codec import (
        game_to_msg, referee_to_msg, robot_status_to_msg, stamp, structure_to_msg, world_from_msg,
    )
    from sentry_ros.world_node import WorldNode
    from conftest import ROOT

    rclpy.init()
    namespace = "typed_inputs_test"
    node = Node("typed_input_publisher", namespace=namespace)
    runtime = WorldNode(namespace=namespace, parameter_overrides=[
        Parameter("config_file", value=str(ROOT / "src/sentry_bringup/config/tactics.yaml")),
        Parameter("self_from_tf", value=False),
    ])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(runtime)
    poses = node.create_publisher(PoseWithCovarianceStamped, "self_pose", 10)
    referees = node.create_publisher(RefereeState, "referee/state", 1)
    games = node.create_publisher(GameStatus, "game/status", 1)
    robots = node.create_publisher(RobotStatusArray, "robots/status", 10)
    structures = node.create_publisher(StructureStatus, "structures/status", 10)
    received = []
    node.create_subscription(WorldState, "world/state", received.append, 10)

    def publish():
        now = node.get_clock().now().nanoseconds / 1e9
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id, pose.header.stamp = "map", stamp(now)
        pose.pose.pose.position.x, pose.pose.pose.position.y = 3.0, 6.0
        pose.pose.pose.orientation.w = 1.0
        poses.publish(pose)
        referees.publish(referee_to_msg(Referee(now, Phase.READY, 0.9, 0.9, True, True, True)))
        games.publish(game_to_msg(GameState(now, "rmuc2027", "RED", Phase.RUNNING, 300.0, True)))
        array = RobotStatusArray()
        array.header.stamp = stamp(now)
        array.robots = [robot_status_to_msg(RobotStatus(7, Team.ALLY, now, 200.0, 400.0, True,
                                                        source=1))]
        robots.publish(array)
        states = StructureStatus()
        states.header.stamp = stamp(now)
        states.structures = [structure_to_msg(StructureState("enemy_outpost", now, alive=False,
                                                             source=1))]
        structures.publish(states)

    world = None
    try:
        deadline = time.monotonic() + 10
        while world is None and time.monotonic() < deadline:
            publish()
            for _ in range(5):
                executor.spin_once(timeout_sec=0.02)
            for message in received:
                decoded = world_from_msg(message)
                if (decoded.self_track and decoded.game and 7 in decoded.robots
                        and "enemy_outpost" in decoded.structures and decoded.referee):
                    world = decoded
        assert world is not None, "world/state never carried the typed inputs"
        view = view_inputs(world, config)
        assert world.referee.phase == Phase.READY
        assert view.phase == Phase.RUNNING
        assert view.self_hp == pytest.approx(0.5)
        assert view.structures["enemy_outpost"] is False
    finally:
        executor.shutdown()
        runtime.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def launch_argument_default(argument, *names):
    """The default of ``argument``, which must be the same in each named bringup launch file."""
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    from conftest import ROOT

    defaults = set()
    for name in names:
        spec = importlib.util.spec_from_file_location(
            name.replace(".", "_"), ROOT / "src/sentry_bringup/launch" / name)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        declared = next(entity for entity in module.generate_launch_description().entities
                        if isinstance(entity, DeclareLaunchArgument)
                        and entity.name == argument)
        defaults.add(perform_substitutions(LaunchContext(), declared.default_value))
    assert len(defaults) == 1, defaults
    return defaults.pop()


def launch_default_config_file():
    """The config_file default shared by decision.launch.py and navigation.launch.py."""
    from pathlib import Path

    return Path(launch_argument_default("config_file", "decision.launch.py",
                                        "navigation.launch.py"))


def test_nodes_default_to_the_launch_default_config_file():
    import rclpy
    from rclpy.node import Node
    from sentry_core.config import load_config
    from sentry_ros.common import read_config

    # A node started without the launch files (ros2 run) loads the same RMUC 2027 season
    # file as decision.launch.py and navigation.launch.py, not the demo tactics.
    tactics = launch_default_config_file()
    assert tactics.name == "tactics_rmuc2027.yaml"
    rclpy.init()
    try:
        node = Node("default_config_test")
        try:
            config = read_config(node)
            assert node.get_parameter("config_file").value == str(tactics)
            assert config == load_config(tactics) and config.season == "rmuc2027"
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_own_team_parameter_keeps_self_id_derivation(tmp_path):
    import rclpy
    import yaml
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sentry_ros.common import read_config

    # The launch default is the RMUC 2027 season file, which omits self_id like every bundled
    # tactics file; an explicit copy keeps its relative field file reachable.
    tactics = launch_default_config_file()
    assert tactics.name == "tactics_rmuc2027.yaml"
    raw = yaml.safe_load(tactics.read_text(encoding="utf-8"))
    assert "self_id" not in raw
    raw.update(self_id=7, field=str(tactics.parent / raw["field"]))
    explicit = tmp_path / "tactics.yaml"
    explicit.write_text(yaml.safe_dump(raw), encoding="utf-8")
    rclpy.init()
    try:
        for path, team, expected in ((tactics, "", ("RED", 7)), (tactics, "blue", ("BLUE", 107)),
                                     (explicit, "BLUE", ("BLUE", 7))):
            node = Node("own_team_test", parameter_overrides=[
                Parameter("config_file", value=str(path)), Parameter("own_team", value=team),
            ])
            try:
                config = read_config(node)
                assert (config.own_team, config.self_id) == expected
            finally:
                node.destroy_node()
    finally:
        rclpy.shutdown()


def test_decision_launch_exposes_executor_guard_and_precheck_switches():
    import importlib.util
    from launch import LaunchContext
    from launch_ros.actions import Node as LaunchNode
    from sentry_ros.executor_node import ExecutorNode
    import rclpy
    from rclpy.parameter import Parameter
    from conftest import ROOT

    assert launch_argument_default("enable_executor", "decision.launch.py") == "true"
    assert launch_argument_default("enable_velocity_guard", "decision.launch.py") == "true"
    assert launch_argument_default("check_path_before_navigate", "decision.launch.py") == "true"

    spec = importlib.util.spec_from_file_location(
        "decision_launch", ROOT / "src/sentry_bringup/launch/decision.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    nodes = [e for e in description.entities if isinstance(e, LaunchNode)]
    assert len(nodes) == 4

    def enabled_count(overrides):
        ctx = LaunchContext()
        for k, v in {"enable_executor": "true", "enable_velocity_guard": "true",
                     **overrides}.items():
            ctx.launch_configurations[k] = v
        return sum(1 for n in nodes if n.condition is None or n.condition.evaluate(ctx))

    assert enabled_count({}) == 4
    assert enabled_count({"enable_executor": "false"}) == 3
    assert enabled_count({"enable_executor": "false", "enable_velocity_guard": "false"}) == 2

    rclpy.init()
    try:
        node = ExecutorNode(parameter_overrides=[
            Parameter("check_path_before_navigate", value=False)])
        try:
            assert node.machine.check_path_before_navigate is False
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
