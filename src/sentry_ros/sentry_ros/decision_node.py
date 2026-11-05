"""World snapshots -> explainable leased tasks, with optional trained artifacts."""

from dataclasses import asdict
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import time

from rclpy.node import Node
from std_msgs.msg import String
from sentry_interfaces.msg import (
    TaskFeedback, TacticalTask, TeammateIntentArray, ThreatState, WorldState,
)

from sentry_core.learned import IntentModel, load_utility
from sentry_core.model import IntentType, Phase
from sentry_core.policy import DecisionEngine
from sentry_core.rl_policy import GoalPrior
from sentry_core.serde import json_line, world_to_dict
from .codec import (
    FEEDBACK_STATES, intents_to_msg, task_to_msg, threats_to_msg, world_from_msg,
)
from .common import LATEST, read_config, seconds, spin_node


class DecisionNode(Node):
    def __init__(self, **kwargs):
        super().__init__("decision", **kwargs)
        config = read_config(self)
        utility = self.declare_parameter("utility_model", "").value
        model_path = self.declare_parameter("intent_model", "").value
        rl_path = self.declare_parameter("rl_model", "").value
        rl_blend = self.declare_parameter("rl_blend", 0.25).value
        rl_mirror = self.declare_parameter("rl_mirror", False).value
        allow_transfer = self.declare_parameter("rl_allow_transfer", False).value
        if utility:
            config = load_utility(utility, config)
        model = IntentModel.load(model_path, config) if model_path else None
        prior = GoalPrior.load(rl_path, config, rl_blend, rl_mirror, allow_transfer) if rl_path else None
        self.engine = DecisionEngine(config, model, goal_prior=prior)
        self.config = config
        self.world = None
        self.last_evaluated = None
        self.last_critical = False
        self.pending_feedback = []
        self.tasks = self.create_publisher(TacticalTask, "tactical/task", LATEST)
        self.intents = self.create_publisher(TeammateIntentArray, "team/intents", LATEST)
        self.threats = self.create_publisher(ThreatState, "threat/state", LATEST)
        self.traces = self.create_publisher(String, "tactical/trace", LATEST)
        self.create_subscription(WorldState, "world/state", self._world, LATEST)
        self.create_subscription(TaskFeedback, "tactical/feedback", self._feedback, 20)
        rate = self.declare_parameter("decision_rate", 2.0).value
        if rate <= 0:
            raise ValueError("decision_rate must be positive")
        self.create_timer(1. / rate, self._decide)
        trace = self.declare_parameter("trace_path", "").value
        self.trace_handler = None
        if trace:
            Path(trace).parent.mkdir(parents=True, exist_ok=True)
            self.trace_handler = RotatingFileHandler(trace, maxBytes=20_000_000, backupCount=3,
                                                     encoding="utf-8")
            self.trace_handler.setFormatter(logging.Formatter("%(message)s"))
        algorithm = prior.model.payload["algorithm"] if prior else "utility"
        self.get_logger().info(
            f"Decision ready: intent={'HMM' if model else 'rule'}, goal={algorithm}; map={config.site}")

    def _world(self, message):
        try:
            world = world_from_msg(message)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.)
            return
        self.world = world
        bases = [z for z in self.config.zones if z.intent == IntentType.DEFEND_BASE]
        critical = (world.self_track is None or world.referee is None
                    or world.referee.phase != Phase.RUNNING
                    or world.referee.hp is not None and world.referee.hp < self.config.critical_hp
                    or any(t.usable and t.position.distance(z.center) <= z.radius
                           for t in world.enemies for z in bases))
        if critical != self.last_critical:
            self.last_critical = critical
            self._decide()

    def _feedback(self, message):
        if message.execution_state >= len(FEEDBACK_STATES):
            return
        now = seconds(self)
        state = FEEDBACK_STATES[message.execution_state]
        self.engine.feedback(message.session_id, message.task_id, state, now)
        self.pending_feedback.append({"session_id": message.session_id, "task_id": message.task_id,
                                      "state": state, "stamp": now})
        self.pending_feedback = self.pending_feedback[-50:]

    def _decide(self):
        world = self.world
        if world is None or self.last_evaluated == (world.epoch, world.seq, world.stamp):
            return
        now = seconds(self)
        if not -0.05 <= now - world.stamp <= self.config.self_timeout:
            return  # No invented freshness; executor's independent watchdog revokes motion.
        self.last_evaluated = world.epoch, world.seq, world.stamp
        started = time.perf_counter()
        decision = self.engine.step(world)
        self.tasks.publish(task_to_msg(decision.task, self.config.frame, now))
        self.intents.publish(intents_to_msg(decision.intents, self.config.frame, now))
        self.threats.publish(threats_to_msg(decision.threats, self.config.frame, now))
        record = {
            "schema_version": 1, "episode_id": self.engine.session_id,
            "map_fingerprint": self.config.map_fingerprint,
            "world": world_to_dict(world), "decision": asdict(decision),
            "feedback_before": self.pending_feedback,
            "decision_ms": (time.perf_counter() - started) * 1000,
        }
        self.pending_feedback = []
        line = json_line(record)
        self.traces.publish(String(data=line))
        if self.trace_handler:
            self.trace_handler.emit(logging.LogRecord("trace", logging.INFO, "", 0, line, (), None))

    def destroy_node(self):
        if self.trace_handler:
            self.trace_handler.close()
        return super().destroy_node()


def main(args=None):
    spin_node(DecisionNode, args)
