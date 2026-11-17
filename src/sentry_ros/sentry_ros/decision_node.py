"""World snapshots -> explainable leased tasks, with optional trained artifacts."""

from dataclasses import asdict
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import time

from rclpy.node import Node
from std_msgs.msg import Float32, String
from sentry_interfaces.msg import (
    SentryRefereeCmd, TaskFeedback, TacticalTask, TeammateIntentArray, ThreatState, WorldState,
)

from sentry_core.inputs import view_inputs
from sentry_core.learned import load_decision_models
from sentry_core.model import IntentType, Phase
from sentry_core.policy import DecisionEngine, defense_structure_destroyed
from sentry_core.rl_policy import GoalPrior, mirror_setting
from sentry_core.serde import json_line, world_to_dict
from .codec import (
    FEEDBACK_STATES, intents_to_msg, sentry_referee_cmd_to_msg, task_to_msg, threats_to_msg,
    world_from_msg,
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
        # auto follows the field (a mirrored BLUE field mirrors the RL features); true and
        # false force mirrored or unmirrored features. Other values stop the node.
        rl_mirror = mirror_setting(self.declare_parameter("rl_mirror", "auto").value)
        allow_transfer = self.declare_parameter("rl_allow_transfer", False).value
        # An intent/utility artifact fitted on another field runs in rule mode instead of
        # stopping the node; the reason is logged here and in every decision's policy_info.
        config, model, fallbacks = load_decision_models(config, model_path, utility)
        for name, fallback in fallbacks.items():
            self.get_logger().warning(
                f"{name} falls back to rule mode ({fallback['reason']}): {fallback['path']}: "
                f"{fallback['detail']}")
        prior = (GoalPrior.load(rl_path, config, rl_blend, rl_mirror, allow_transfer)
                 if rl_path else None)
        self.engine = DecisionEngine(config, model, goal_prior=prior, model_fallbacks=fallbacks)
        self.config = config
        self.world = None
        self.last_evaluated = None
        self.last_critical = False
        self.last_hit_stamp = None
        self.pending_feedback = []
        self.tasks = self.create_publisher(TacticalTask, "tactical/task", LATEST)
        self.referee_cmds = self.create_publisher(SentryRefereeCmd, "referee/sentry_cmd", LATEST)
        self.spins = self.create_publisher(Float32, "chassis/spin", LATEST)
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
        weights = "trained" if utility and "utility_model" not in fallbacks else "tactics"
        self.get_logger().info(
            f"Decision ready: intent={'HMM' if model else 'rule'}, weights={weights}, "
            f"goal={algorithm}; map={config.site}")

    def _world(self, message):
        try:
            world = world_from_msg(message)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.)
            return
        self.world = world
        info = view_inputs(world, self.config)
        bases = [z for z in self.config.zones
                 if z.intent == IntentType.DEFEND_BASE
                 and not defense_structure_destroyed(z, info.structures)]
        critical = (world.self_track is None or info.phase != Phase.RUNNING
                    or info.self_hp is not None and info.self_hp < self.config.critical_hp
                    or any(t.usable and z.contains(t.position)
                           for t in world.enemies for z in bases))
        hit_changed = (info.self_last_hit_stamp is not None
                       and info.self_last_hit_stamp != self.last_hit_stamp)
        self.last_hit_stamp = info.self_last_hit_stamp
        if critical != self.last_critical or hit_changed:
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
        self.referee_cmds.publish(
            sentry_referee_cmd_to_msg(decision.task, self.config.frame, now)
        )
        spin_val = decision.task.spin_rate_rad_s if decision.task.spin_enabled else 0.0
        self.spins.publish(Float32(data=float(spin_val)))
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
