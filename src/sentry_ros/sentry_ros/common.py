from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sentry_core.config import load_config


LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE)


def read_config(node):
    try:
        default = str(Path(get_package_share_directory("sentry_bringup")) / "config/tactics.yaml")
    except PackageNotFoundError:
        default = ""
    path = node.declare_parameter("config_file", default).value
    if not path:
        raise ValueError("config_file is required")
    return load_config(path)


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
