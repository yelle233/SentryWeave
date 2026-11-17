"""Steady-clock velocity watchdog, the final software publisher to the chassis."""

import time

from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Float32
from sentry_interfaces.msg import MotionPermit

from sentry_core.execution import VelocityGate
from .codec import to_seconds
from .common import LATEST, seconds, spin_node


class GuardNode(Node):
    def __init__(self, **kwargs):
        super().__init__("velocity_guard", **kwargs)
        inject_spin = self.declare_parameter("inject_spin_into_cmd_vel", False).value
        max_spin = self.declare_parameter("max_spin_angular", 8.0).value
        spin_timeout = self.declare_parameter("spin_timeout", 1.0).value
        self.gate = VelocityGate(
            command_timeout=self.declare_parameter("command_timeout", 0.25).value,
            permit_timeout=self.declare_parameter("permit_timeout", 0.3).value,
            max_linear=self.declare_parameter("max_linear", 1.5).value,
            max_angular=self.declare_parameter("max_angular", 2.0).value,
            inject_spin=inject_spin,
            max_spin_angular=max_spin,
            spin_timeout=spin_timeout,
        )
        self.publisher = self.create_publisher(Twist, "cmd_vel", LATEST)
        self.create_subscription(Twist, "nav_cmd_vel", self._command, LATEST)
        self.create_subscription(MotionPermit, "motion/permit", self._permit, LATEST)
        if self.gate.inject_spin:
            self.create_subscription(Float32, "chassis/spin", self._spin, LATEST)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self._tick, clock=self.clock)
        self.session, self.sequence = None, 0
        self.retired = set()

    def _command(self, message):
        self.gate.command(message.linear.x, message.linear.y, message.angular.z, time.monotonic())

    def _spin(self, message):
        self.gate.spin(message.data, time.monotonic())

    def _permit(self, message):
        now = time.monotonic()
        if (not message.executor_session or message.executor_session in self.retired
                or not -0.05 <= seconds(self) - to_seconds(message.header.stamp) <= 0.3):
            return
        if message.executor_session != self.session:
            if self.session:
                self.retired.add(self.session)
                self.gate.permit(False, now)
            if len(self.retired) > 1024:
                self.gate.permit(False, now)
                return
            self.session, self.sequence = message.executor_session, 0
        if message.sequence <= self.sequence:
            return
        self.sequence = message.sequence
        self.gate.permit(message.allowed, now)

    def _tick(self):
        vx, vy, omega = self.gate.output(time.monotonic())
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = vx, vy, omega
        self.publisher.publish(message)


def main(args=None):
    spin_node(GuardNode, args)
