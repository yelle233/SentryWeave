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


def test_own_team_parameter_keeps_self_id_derivation(tmp_path):
    import rclpy
    import yaml
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sentry_ros.common import read_config
    from conftest import ROOT

    tactics = ROOT / "src/sentry_bringup/config/tactics.yaml"
    raw = yaml.safe_load(tactics.read_text(encoding="utf-8"))
    del raw["self_id"]
    derived = tmp_path / "tactics.yaml"
    derived.write_text(yaml.safe_dump(raw), encoding="utf-8")
    rclpy.init()
    try:
        for path, team, expected in ((derived, "", ("RED", 7)), (derived, "blue", ("BLUE", 107)),
                                     (tactics, "BLUE", ("BLUE", 7))):
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
