"""Only owner of Nav2 actions; the production entry point uses a single-threaded executor."""

import time
import uuid
from collections import deque

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from std_srvs.srv import Trigger
from unique_identifier_msgs.msg import UUID
from sentry_interfaces.msg import MotionPermit, TacticalTask, TaskFeedback, WorldState

from sentry_core.execution import Executor
from sentry_core.geometry import TacticalMap
from sentry_core.model import Vec2
from sentry_core.policy import safety_reason
from .codec import FEEDBACK_STATES, pose, stamp, task_from_msg, world_from_msg
from .common import LATEST, read_config, seconds, spin_node


class ExecutorNode(Node):
    def __init__(self, **kwargs):
        super().__init__("executor", **kwargs)
        self.config = read_config(self)
        self.arena = TacticalMap(self.config)
        timeout = self.declare_parameter("action_timeout", 2.0).value
        safety_timeout = self.declare_parameter("safety_timeout", 0.75).value
        precheck = self.declare_parameter("check_path_before_navigate", True).value
        self.machine = Executor(self.arena, action_timeout=timeout,
                                safety_timeout=safety_timeout,
                                check_path_before_navigate=precheck)
        self.planner_id = self.declare_parameter("planner_id", "GridBased").value
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.cancel_all = self.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal")
        self.old_results = self.create_client(
            NavigateToPose.Impl.GetResultService, "navigate_to_pose/_action/get_result"
        )
        self.startup_state = "WAITING"
        self.startup_deadline = float("inf")
        self.startup_goals = set()
        self.startup_known = set()
        self.startup_task = None
        self.startup_world = None
        self.reported_active = {}
        self.nav_ids = {}
        self.retired_goals = set()
        self.retired_order = deque()
        self.create_subscription(
            GoalStatusArray, "navigate_to_pose/_action/status", self._status,
            qos_profile_action_status_default,
        )
        self.nav_handles, self.plan_handles = {}, {}
        self.cancel_plans = set()
        self.feedback_pub = self.create_publisher(TaskFeedback, "tactical/feedback", 20)
        self.permit_pub = self.create_publisher(MotionPermit, "motion/permit", LATEST)
        self.create_subscription(TacticalTask, "tactical/task", self._task, LATEST)
        self.create_subscription(WorldState, "world/state", self._world, LATEST)
        self.create_service(Trigger, "executor/reset_fault", self._reset)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self._tick, clock=self.steady_clock)
        self.executor_session, self.permit_sequence = uuid.uuid4().hex, 0
        self._last_world = None

    def _world(self, message):
        if self.startup_state != "READY":
            self.startup_world = message
            return
        now, steady = seconds(self), time.monotonic()
        try:
            world = world_from_msg(message)
            identity = world.epoch, world.seq, world.stamp
            if identity == self._last_world:
                return
            self._last_world = identity
            healthy = (-0.05 <= now - world.stamp <= self.config.self_timeout
                       and safety_reason(world, self.config, self.arena) is None)
        except ValueError:
            healthy = False
        self.machine.safety(healthy, now, steady)
        self._dispatch()

    def _task(self, message):
        if self.startup_state != "READY":
            self.startup_task = message
            return
        try:
            self.machine.submit(task_from_msg(message, self.config.frame), seconds(self),
                                time.monotonic())
        except ValueError as error:
            rejected = TaskFeedback()
            rejected.header.stamp, rejected.header.frame_id = stamp(seconds(self)), self.config.frame
            rejected.session_id, rejected.task_id = message.session_id, message.task_id
            rejected.execution_state, rejected.reason_code = TaskFeedback.REJECTED, str(error)
            self.feedback_pub.publish(rejected)
        self._dispatch()

    def _reset(self, _, response):
        if self.startup_state != "READY":
            response.success = False
            response.message = "Startup ownership unresolved; restore Nav2 and restart the executor"
            return response
        response.success = self.machine.reset()
        response.message = ("Fault reset; awaiting a fresh task heartbeat" if response.success
                            else "Unresolved navigation or unhealthy localization; motion stays disabled")
        return response

    def _tick(self):
        now, steady = seconds(self), time.monotonic()
        self._startup_tick(now, steady)
        self.machine.tick(now, steady)
        self._dispatch()
        self.permit_sequence += 1
        permit = MotionPermit()
        permit.header.stamp, permit.header.frame_id = stamp(now), self.config.frame
        permit.executor_session, permit.sequence = self.executor_session, self.permit_sequence
        active = set(self.reported_active) - self.retired_goals
        exclusive = self.nav_ids.get(self.machine.token) in active and len(active) == 1
        permit.allowed = (self.startup_state == "READY" and exclusive
                          and self.machine.motion_allowed(now, steady))
        permit.reason = self.machine.reason or (
            self.machine.state if self.startup_state == "READY" else f"STARTUP_{self.startup_state}"
        )
        self.permit_pub.publish(permit)

    def _startup_tick(self, now, steady):
        if self.startup_state in ("CANCELING", "DRAINING") and steady > self.startup_deadline:
            self._startup_failed("TIMEOUT")
        elif (self.startup_state == "WAITING" and self.cancel_all.service_is_ready()
              and self.old_results.service_is_ready()):
            # Zero UUID and zero stamp request cancellation of every pre-existing
            # goal in this exclusively-owned navigation namespace.
            self.startup_state = "CANCELING"
            self.startup_deadline = steady + self.machine.action_timeout
            self.cancel_all.call_async(CancelGoal.Request()).add_done_callback(self._startup_canceled)

    def _startup_canceled(self, future):
        if self.startup_state != "CANCELING":
            return
        try:
            response = future.result()
            if response.return_code != CancelGoal.Response.ERROR_NONE:
                self._startup_failed(f"CANCEL_REJECTED_{response.return_code}")
                return
            # CancelGoal may return an empty list for goals already CANCELING.
            # Merge the transient-local action status snapshot before draining.
            goals = {bytes(info.goal_id.uuid): info for info in response.goals_canceling}
            goals.update(self.reported_active)
            if not goals:
                self._startup_ready()
                return
            self.startup_state = "DRAINING"
            for info in goals.values():
                self._await_old_goal(info)
        except Exception as error:
            self._startup_failed(f"CANCEL_ERROR:{error}")

    def _await_old_goal(self, info):
        identity = bytes(info.goal_id.uuid)
        if identity in self.startup_known or identity in self.retired_goals:
            return
        self.startup_known.add(identity)
        self.startup_goals.add(identity)
        request = NavigateToPose.Impl.GetResultService.Request()
        request.goal_id = info.goal_id
        self.old_results.call_async(request).add_done_callback(
            lambda f: self._startup_result(identity, f)
        )

    def _status(self, message):
        self.reported_active = {
            bytes(status.goal_info.goal_id.uuid): status.goal_info
            for status in message.status_list
            if status.status in (
                GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING
            )
        }
        if self.startup_state == "DRAINING":
            for info in self.reported_active.values():
                self._await_old_goal(info)
        elif self.startup_state == "READY":
            foreign = set(self.reported_active) - set(self.nav_ids.values()) - self.retired_goals
            if foreign:
                self._startup_failed("FOREIGN_GOAL")
                self._dispatch()

    def _retire(self, identity):
        if identity is not None and identity not in self.retired_goals:
            self.retired_goals.add(identity)
            self.retired_order.append(identity)
            if len(self.retired_order) > 4096:
                self.retired_goals.remove(self.retired_order.popleft())

    def _startup_result(self, identity, future):
        if self.startup_state != "DRAINING":
            return
        try:
            result = future.result()
            if result.status not in (
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED
            ):
                self._startup_failed("UNCONFIRMED_OLD_GOAL")
                return
            self.startup_goals.discard(identity)
            self._retire(identity)
            if not self.startup_goals:
                self._startup_ready()
        except Exception as error:
            self._startup_failed(f"RESULT_ERROR:{error}")

    def _startup_ready(self):
        self.startup_state = "READY"
        pending_world, self.startup_world = self.startup_world, None
        pending_task, self.startup_task = self.startup_task, None
        if pending_world is not None:
            self._world(pending_world)
        if pending_task is not None:
            self._task(pending_task)

    def _startup_failed(self, reason):
        self.startup_state = "FAULT"
        reason = f"STARTUP_OWNERSHIP_{reason}"
        self.machine.trip(reason, seconds(self), time.monotonic())
        self.get_logger().error(reason)

    def _dispatch(self):
        # A synchronous refusal can enqueue another command. Async operations never block here.
        for _ in range(32):
            commands = self.machine.drain()
            if not commands:
                return
            for command in commands:
                now, steady = seconds(self), time.monotonic()
                if command.kind == "FEEDBACK" and command.task:
                    message = TaskFeedback()
                    message.header.stamp, message.header.frame_id = stamp(now), self.config.frame
                    message.session_id, message.task_id = command.task.identity
                    message.execution_state = FEEDBACK_STATES.index(command.state)
                    message.reason_code = command.reason
                    self.feedback_pub.publish(message)
                elif command.kind == "PLAN":
                    if not self.planner.server_is_ready():
                        self.machine.planned(command.token, False, now, steady)
                        continue
                    goal = ComputePathToPose.Goal()
                    goal.goal = pose(command.task.goal, command.task.yaw, self.config.frame, now)
                    goal.planner_id, goal.use_start = self.planner_id, False
                    future = self.planner.send_goal_async(goal)
                    future.add_done_callback(lambda f, c=command: self._plan_accepted(c, f))
                elif command.kind == "CANCEL_PLAN":
                    handle = self.plan_handles.get(command.token)
                    if handle:
                        handle.cancel_goal_async()
                    else:
                        self.cancel_plans.add(command.token)
                elif command.kind == "SEND":
                    if not self.nav.server_is_ready():
                        self.machine.accepted(command.token, False, now, steady)
                        continue
                    goal = NavigateToPose.Goal()
                    goal.pose = pose(command.task.goal, command.task.yaw, self.config.frame, now)
                    identifier = UUID(uuid=list(uuid.uuid4().bytes))
                    self.nav_ids[command.token] = bytes(identifier.uuid)
                    future = self.nav.send_goal_async(
                        goal, feedback_callback=lambda f, c=command: self._nav_feedback(c, f),
                        goal_uuid=identifier,
                    )
                    future.add_done_callback(lambda f, c=command: self._nav_accepted(c, f))
                elif command.kind == "CANCEL":
                    handle = self.nav_handles.get(command.token)
                    if handle:
                        future = handle.cancel_goal_async()
                        future.add_done_callback(lambda f, token=command.token: self._cancel_done(token, f))
                    else:
                        self.machine.trip("MISSING_OWNED_GOAL_HANDLE", now, steady)
        self.machine.trip("COMMAND_LOOP", seconds(self), time.monotonic())

    def _plan_accepted(self, command, future):
        try:
            handle = future.result()
        except Exception as error:
            self.get_logger().error(f"Path request error: {error}")
            self.machine.planned(command.token, False, seconds(self), time.monotonic())
            self._dispatch()
            return
        if not handle.accepted:
            self.cancel_plans.discard(command.token)
            self.machine.planned(command.token, False, seconds(self), time.monotonic())
        else:
            self.plan_handles[command.token] = handle
            result = handle.get_result_async()
            result.add_done_callback(lambda f: self._plan_done(command, f))
            if (command.token in self.cancel_plans or command.token != self.machine.token
                    or self.machine.state != "PLANNING"):
                self.cancel_plans.discard(command.token)
                handle.cancel_goal_async()
        self._dispatch()

    def _plan_done(self, command, future):
        self.plan_handles.pop(command.token, None)
        reachable = False
        try:
            response = future.result()
            path = response.result.path
            points = [Vec2(p.pose.position.x, p.pose.position.y) for p in path.poses]
            reachable = bool(
                response.status == GoalStatus.STATUS_SUCCEEDED
                and response.result.error_code == 0 and path.header.frame_id == self.config.frame
                and points and points[-1].distance(command.task.goal) < 0.6
                and all(self.arena.legal(p) for p in points)
                and all(self.arena.segment_legal(a, b) for a, b in zip(points, points[1:]))
            )
        except Exception as error:
            self.get_logger().warning(f"Path result error: {error}")
        self.machine.planned(command.token, reachable, seconds(self), time.monotonic())
        self._dispatch()

    def _nav_accepted(self, command, future):
        try:
            handle = future.result()
        except Exception as error:
            # Transport failure does not prove that the server rejected the goal.
            self.machine.trip(f"GOAL_RESPONSE_ERROR:{error}", seconds(self), time.monotonic())
            self._dispatch()
            return
        if handle.accepted:
            self.nav_handles[command.token] = handle
            handle.get_result_async().add_done_callback(
                lambda f, token=command.token: self._nav_done(token, f)
            )
        else:
            self._retire(self.nav_ids.pop(command.token, None))
        self.machine.accepted(command.token, handle.accepted, seconds(self), time.monotonic())
        self._dispatch()

    def _nav_feedback(self, command, feedback):
        if (command.token != self.machine.token or not self.machine.current
                or self.machine.state != "RUNNING"):
            return
        message = TaskFeedback()
        message.header.stamp, message.header.frame_id = stamp(seconds(self)), self.config.frame
        message.session_id, message.task_id = command.task.identity
        message.execution_state = TaskFeedback.RUNNING
        message.remaining_distance_m = feedback.feedback.distance_remaining
        message.remaining_distance_valid = True
        message.reason_code = "NAV_PROGRESS"
        self.feedback_pub.publish(message)

    def _cancel_done(self, token, future):
        try:
            response = future.result()
            accepted = (bool(response.goals_canceling)
                        or response.return_code == CancelGoal.Response.ERROR_GOAL_TERMINATED)
        except Exception:
            accepted = False
        self.machine.cancel_ack(token, accepted, seconds(self), time.monotonic())
        self._dispatch()

    def _nav_done(self, token, future):
        if token != self.machine.token or token not in self.nav_handles:
            return
        try:
            response = future.result()
        except Exception as error:
            self.machine.trip(f"NAV_RESULT_ERROR:{error}", seconds(self), time.monotonic())
            self._dispatch()
            return
        if response.status not in (
            GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED
        ):
            self.machine.trip("NON_TERMINAL_NAV_RESULT", seconds(self), time.monotonic())
            self._dispatch()
            return
        self.nav_handles.pop(token, None)
        self._retire(self.nav_ids.pop(token, None))
        state = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                 GoalStatus.STATUS_CANCELED: "CANCELED"}.get(response.status, "FAILED")
        self.machine.result(token, state, seconds(self), time.monotonic())
        self._dispatch()


def main(args=None):
    spin_node(ExecutorNode, args)
