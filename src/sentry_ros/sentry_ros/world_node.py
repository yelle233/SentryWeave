"""Acquisition-time TF conversion, bounded tracking and referee validity."""

import copy
from dataclasses import replace
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener
from sentry_interfaces.msg import ObservationArray, RefereeState, WorldState

from sentry_core.model import Observation, Team, Vec2
from sentry_core.tracking import WorldModel
from .codec import observation_from_msg, referee_from_msg, to_seconds, world_to_msg
from .common import LATEST, read_config, seconds, spin_node


class WorldNode(Node):
    def __init__(self, **kwargs):
        super().__init__("world_model", **kwargs)
        self.config = read_config(self)
        self.model = WorldModel(self.config)
        self.from_tf = self.declare_parameter("self_from_tf", True).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.require_health = self.declare_parameter("require_localization_health", False).value
        self.health, self.health_at = False, -1e30
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(WorldState, "world/state", LATEST)
        self.create_subscription(ObservationArray, "perception/tracks", self._tracks, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "self_pose", self._self_pose, qos_profile_sensor_data)
        self.create_subscription(RefereeState, "referee/state", self._referee, LATEST)
        self.create_subscription(Bool, "localization_ok", self._health, LATEST)
        self.create_timer(0.1, self._tick)

    def _health(self, message):
        self.health, self.health_at = message.data, time.monotonic()

    def _convert(self, message):
        if not message.header.frame_id:
            raise ValueError("observation frame is empty")
        if message.header.frame_id == self.config.frame:
            return message
        transform = self.buffer.lookup_transform(
            self.config.frame, message.header.frame_id, Time.from_msg(message.header.stamp)
        )
        result = copy.deepcopy(message)
        result.pose.pose = do_transform_pose(message.pose.pose, transform)
        result.header.frame_id = self.config.frame
        # Do not rotate a moving-frame velocity as if it were an inertial velocity.
        # The core will estimate velocity from converted position history instead.
        if hasattr(result, "velocity_valid"):
            result.velocity_valid = False
        return result

    def _tracks(self, message):
        now = seconds(self)
        for item in message.observations:
            if item.team == int(Team.ALLY) and item.robot_id == self.config.self_id:
                continue  # External radar cannot replace the localization authority.
            try:
                converted = self._convert(item)
                self.model.observe(observation_from_msg(converted), now)
            except (ValueError, TransformException) as error:
                self.get_logger().warning(f"Dropped track: {error}", throttle_duration_sec=2.)

    def _self_pose(self, message):
        if self.from_tf:
            return
        try:
            converted = self._convert(message)
            self.model.observe(Observation(
                self.config.self_id, Team.ALLY,
                Vec2(converted.pose.pose.position.x, converted.pose.pose.position.y),
                to_seconds(converted.header.stamp), source_mask=2,
            ), seconds(self))
        except (ValueError, TransformException) as error:
            self.get_logger().warning(f"Dropped self pose: {error}", throttle_duration_sec=2.)

    def _referee(self, message):
        try:
            value = referee_from_msg(message)
            now = seconds(self)
            if -0.05 <= now - value.stamp <= self.config.referee_timeout:
                self.model.update_referee(value)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.)

    def _tick(self):
        now = seconds(self)
        if self.from_tf:
            try:
                transform = self.buffer.lookup_transform(self.config.frame, self.base_frame, Time())
                observed = to_seconds(transform.header.stamp)
                if observed > 0:
                    self.model.observe(Observation(
                        self.config.self_id, Team.ALLY,
                        Vec2(transform.transform.translation.x, transform.transform.translation.y),
                        observed, source_mask=2,
                    ), now)
            except (ValueError, TransformException):
                pass
        world = self.model.snapshot(now)
        if self.require_health and (not self.health or time.monotonic() - self.health_at > 0.5):
            world = replace(world, self_track=None, mode="SAFE_STOP")
        self.publisher.publish(world_to_msg(world))


def main(args=None):
    spin_node(WorldNode, args)
