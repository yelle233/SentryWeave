"""Scenario sensor/referee injection; never publishes a chassis command."""

from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from sentry_interfaces.msg import ObservationArray, RefereeState

from sentry_core.model import Vec2
from sentry_training.simulation import Scenario
from .codec import observation_to_msg, referee_to_msg, stamp
from .common import LATEST, read_config, seconds, spin_node


class ScenarioNode(Node):
    def __init__(self, **kwargs):
        super().__init__("scenario", **kwargs)
        self.config = read_config(self)
        self.name = self.declare_parameter("scenario", "conflict").value
        self.seed = self.declare_parameter("seed", 101).value
        self.scene = Scenario(self.config, self.name, self.seed)
        self.tracks = self.create_publisher(ObservationArray, "perception/tracks", qos_profile_sensor_data)
        self.referee = self.create_publisher(RefereeState, "referee/state", LATEST)
        self.health = self.create_publisher(Bool, "localization_ok", LATEST)
        self.create_subscription(PoseWithCovarianceStamped, "self_pose", self._pose,
                                 qos_profile_sensor_data)
        self.position = self.scene.self_start
        self.started = self.last = None
        self.create_timer(0.1, self._tick)

    def _pose(self, message):
        if message.header.frame_id == self.config.frame:
            self.position = Vec2(message.pose.pose.position.x, message.pose.pose.position.y)

    def _tick(self):
        now = seconds(self)
        if now <= 0:
            return
        if self.started is None or self.last is not None and now < self.last:
            self.started = now
            self.scene = Scenario(self.config, self.name, self.seed)
        dt = min(0.2, max(0.01, now - self.last)) if self.last is not None else 0.1
        self.last = now
        observations, referee = self.scene.observe(now - self.started, dt, self.position)
        # Scenario internally uses 1+elapsed; translate to the live ROS clock once.
        offset = self.started - 1.0
        from dataclasses import replace
        message = ObservationArray()
        message.header.frame_id, message.header.stamp = self.config.frame, stamp(now)
        message.observations = [
            observation_to_msg(replace(o, stamp=o.stamp + offset), self.config.frame)
            for o in observations if o.robot_id != self.config.self_id
        ]
        self.tracks.publish(message)
        self.referee.publish(referee_to_msg(replace(referee, stamp=now), self.config.frame))
        self.health.publish(Bool(data=not (
            self.name == "localization_loss" and 10 <= now - self.started < 16
        )))


def main(args=None):
    spin_node(ScenarioNode, args)
