"""Simulation-only ground-truth localization; never launch this node on a real robot."""

import copy
import math

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from .codec import to_seconds, yaw
from .common import spin_node


class LocalizationNode(Node):
    def __init__(self, **kwargs):
        super().__init__("sim_localization", **kwargs)
        self.frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.world_to_map = self.declare_parameter("world_to_map", [0.0, 0.0, 0.0]).value
        if len(self.world_to_map) != 3 or not all(math.isfinite(v) for v in self.world_to_map):
            raise ValueError("world_to_map must contain finite x, y, yaw")
        self.truth, self.odometry = None, None
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, "self_pose", qos_profile_sensor_data)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, "ground_truth/odom", self._truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self._odom, qos_profile_sensor_data)

    def _truth(self, message):
        self.truth = message
        self._publish()

    def _odom(self, message):
        self.odometry = message
        self._publish()

    def _publish(self):
        if self.truth is None or self.odometry is None:
            return
        if abs(to_seconds(self.truth.header.stamp) - to_seconds(self.odometry.header.stamp)) > 0.05:
            return
        truth, odom = copy.deepcopy(self.truth.pose.pose), self.odometry.pose.pose
        tx, ty, rotation = self.world_to_map
        x, y = truth.position.x, truth.position.y
        truth.position.x = tx + math.cos(rotation) * x - math.sin(rotation) * y
        truth.position.y = ty + math.sin(rotation) * x + math.cos(rotation) * y
        absolute_yaw = yaw(truth.orientation) + rotation
        truth.orientation.x = truth.orientation.y = 0.
        truth.orientation.z, truth.orientation.w = math.sin(absolute_yaw / 2), math.cos(absolute_yaw / 2)
        angle = yaw(truth.orientation) - yaw(odom.orientation)
        transform = TransformStamped()
        transform.header.stamp = self.truth.header.stamp
        transform.header.frame_id, transform.child_frame_id = self.frame, self.odom_frame
        transform.transform.translation.x = (truth.position.x - math.cos(angle) * odom.position.x
                                              + math.sin(angle) * odom.position.y)
        transform.transform.translation.y = (truth.position.y - math.sin(angle) * odom.position.x
                                              - math.cos(angle) * odom.position.y)
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(angle / 2), math.cos(angle / 2)
        self.broadcaster.sendTransform(transform)
        message = PoseWithCovarianceStamped()
        message.header.stamp, message.header.frame_id = self.truth.header.stamp, self.frame
        message.pose = copy.deepcopy(self.truth.pose)
        message.pose.pose = truth
        self.publisher.publish(message)


def main(args=None):
    spin_node(LocalizationNode, args)
