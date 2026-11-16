from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sentry_core.config import load_config


LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE)
# Robot/structure arrays may come from several bridges (referee, radar, team link).
STATUS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE)


def read_config(node):
    # Same default as decision.launch.py and navigation.launch.py, also for `ros2 run`.
    try:
        default = str(Path(get_package_share_directory("sentry_bringup"))
                      / "config/tactics_rmuc2027.yaml")
    except PackageNotFoundError:
        default = ""
    path = node.declare_parameter("config_file", default).value
    own_team = node.declare_parameter("own_team", "").value
    if not path:
        raise ValueError("config_file is required")
    return load_config(path, own_team or None)


def seconds(node):
    return node.get_clock().now().nanoseconds / 1e9


def spin_node(factory, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
