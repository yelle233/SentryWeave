"""Event-driven single-owner navigation protocol, independent of rclpy.

Commands are drained by the ROS adapter. Every asynchronous callback carries a
generation token. A cancel response never releases navigation ownership; only
a terminal result (or a rejected initial goal) does.
"""

from dataclasses import dataclass
import math

from .config import goal_in_scope
from .geometry import TacticalMap
from .model import POSTURES, Task, TaskType


@dataclass(frozen=True)
class Command:
    kind: str
    token: int = 0
    task: Task | None = None
    state: str = ""
    reason: str = ""


class Executor:
    def __init__(self, arena: TacticalMap, action_timeout: float = 2.0,
                 safety_timeout: float = 0.75,
                 check_path_before_navigate: bool = True):
        if not all(math.isfinite(v) and v > 0 for v in (action_timeout, safety_timeout)):
            raise ValueError("executor timeouts must be positive")
        self.arena = arena
        self.action_timeout, self.safety_timeout = action_timeout, safety_timeout
        self.check_path_before_navigate = bool(check_path_before_navigate)
        self.state = "IDLE"
        self.reason = ""
        self.desired: Task | None = None
        self.current: Task | None = None
        self.token = 0
        self._commands: list[Command] = []
        self._deadline = math.inf
        self._healthy = False
        self._safety_at = -math.inf
        self._last_ros: float | None = None
        self._pending_navigation = False
        self._accepted_navigation = False
        self._session: str | None = None
        self._last_id = 0
        self._retired: set[str] = set()
        self._completed: tuple[str, int] | None = None

    def drain(self) -> list[Command]:
        commands, self._commands = self._commands, []
        return commands

    def _emit(self, kind: str, state="", reason="", task=None) -> None:
        self._commands.append(Command(kind, self.token, task or self.current, state, reason))

    def _fresh_safety(self, steady: float) -> bool:
        return self._healthy and 0 <= steady - self._safety_at <= self.safety_timeout

    def safety(self, healthy: bool, ros: float, steady: float) -> None:
        self._healthy, self._safety_at = healthy, steady
        if not healthy:
            if self.current and self.desired and self.current.identity == self.desired.identity:
                self.desired = None
            self._interrupt(ros, steady, "SAFETY_INTERLOCK")

    def motion_allowed(self, ros: float, steady: float) -> bool:
        return bool(
            self.state == "RUNNING" and self._fresh_safety(steady)
            and self.current and self.desired
            and self.current.identity == self.desired.identity
            and self.current.created_at <= ros + 0.05 < self.current.valid_until
        )

    def _valid_task(self, task: Task, ros: float) -> bool:
        if (not task.session_id or len(task.session_id) > 128 or task.task_id < 1
                or not all(math.isfinite(x) for x in (
                    task.created_at, task.valid_until, task.yaw, task.utility, task.confidence))
                or task.created_at > ros + 0.05 or task.valid_until <= ros
                or task.valid_until > ros + 2 * self.arena.config.task_lease
                or task.valid_until <= task.created_at):
            return False
        if (task.posture not in POSTURES
                or not isinstance(task.posture_enhanced, bool)
                or not isinstance(task.spin_enabled, bool)
                or isinstance(task.spin_rate_rad_s, bool)
                or not math.isfinite(task.spin_rate_rad_s)):
            return False
        if task.kind == TaskType.HOLD:
            return task.goal is None
        return (task.kind in TaskType and task.goal is not None
                and self.arena.legal(task.goal)
                and goal_in_scope(self.arena.config, task.goal))

    @staticmethod
    def _same_definition(a: Task, b: Task) -> bool:
        return a.key == b.key and a.goal == b.goal and abs(a.yaw - b.yaw) < 1e-6

    def submit(self, task: Task, ros: float, steady: float) -> None:
        if not self._valid_task(task, ros):
            self._emit("FEEDBACK", "REJECTED", "INVALID_OR_EXPIRED", task)
            return
        if task.session_id in self._retired:
            self._emit("FEEDBACK", "REJECTED", "RETIRED_SESSION", task)
            return
        if task.session_id == self._session and task.task_id < self._last_id:
            self._emit("FEEDBACK", "REJECTED", "OLD_TASK", task)
            return
        if task.session_id == self._session and task.task_id == self._last_id:
            old = self.desired or self.current
            if old is not None and old.identity == task.identity:
                if not self._same_definition(old, task):
                    self._emit("FEEDBACK", "REJECTED", "IDENTITY_MUTATED", task)
                    return
                if task.valid_until < old.valid_until:
                    return
                if self.desired is not None:
                    self.desired = task
                if self.current and self.current.identity == task.identity:
                    self.current = task
            return
        if task.session_id != self._session:
            if self._session is not None:
                self._retired.add(self._session)
            # Never discard replay protection silently in a long-running process.
            if len(self._retired) > 1024:
                self.trip("EXCESSIVE_SESSION_RESTARTS", ros, steady)
                return
            self._session = task.session_id
        self._last_id = task.task_id
        self.desired = task
        if self.state == "FAULT":
            self._emit("FEEDBACK", "REJECTED", "FAULT_LATCHED", task)
            return
        if self.state in ("IDLE", "ARRIVED"):
            self.current = None
            self.state = "IDLE"
            self._begin(ros, steady)
        else:
            self._interrupt(ros, steady, "TASK_REPLACED")

    def _begin(self, ros: float, steady: float) -> None:
        task = self.desired
        if (self.state == "FAULT" or self._pending_navigation or task is None
                or task.identity == self._completed or not self._fresh_safety(steady)
                or task.valid_until <= ros):
            return
        self.current = task
        self._emit("FEEDBACK", "ACCEPTED", "TASK_ACCEPTED")
        if task.kind == TaskType.HOLD:
            self.state, self._completed = "IDLE", task.identity
            self._emit("FEEDBACK", "IDLE", task.reason or "HOLD")
            return
        self.token += 1
        if not self.check_path_before_navigate:
            self.state = "SENDING"
            self._pending_navigation = True
            self._deadline = steady + self.action_timeout
            self._emit("SEND")
            return
        self.state = "PLANNING"
        self._deadline = steady + self.action_timeout
        self._emit("PLAN")

    def _interrupt(self, ros: float, steady: float, reason: str) -> None:
        if self.state == "PLANNING":
            self._emit("CANCEL_PLAN")
            self._emit("FEEDBACK", "CANCELED", reason)
            self.current = None
            self.state = "IDLE"
            self._begin(ros, steady)
        elif self.state == "RUNNING":
            self.state = "CANCELING"
            self._deadline = steady + self.action_timeout
            self._emit("FEEDBACK", "CANCELING", reason)
            self._emit("CANCEL")
        elif self.state == "ARRIVED":
            self.current = None
            self.state = "IDLE"
            self._begin(ros, steady)
        # SENDING waits for its goal response, then cancels the accepted handle.
        # CANCELING waits for a terminal result, even if another replacement arrives.

    def planned(self, token: int, reachable: bool, ros: float, steady: float) -> None:
        if token != self.token or self.state != "PLANNING":
            return
        if not reachable:
            self._finish("FAILED", "NO_LEGAL_PATH", ros, steady)
        elif (not self._fresh_safety(steady) or not self.desired
              or self.current.identity != self.desired.identity
              or self.current.valid_until <= ros):
            self._finish("CANCELED", "PLAN_OBSOLETE", ros, steady)
        else:
            self.state = "SENDING"
            self._pending_navigation = True
            self._deadline = steady + self.action_timeout
            self._emit("SEND")

    def accepted(self, token: int, accepted: bool, ros: float, steady: float) -> None:
        if token != self.token or not self._pending_navigation:
            return
        if not accepted:
            self._pending_navigation = False
            self._finish("REJECTED", "NAV_GOAL_REJECTED", ros, steady)
            return
        self._accepted_navigation = True
        if self.state == "FAULT":
            self._emit("CANCEL")
            return
        self.state = "RUNNING"
        if (not self.desired or self.current.identity != self.desired.identity
                or self.current.valid_until <= ros or not self._fresh_safety(steady)):
            self._interrupt(ros, steady, "ACCEPTED_OBSOLETE_GOAL")
        else:
            self._emit("FEEDBACK", "RUNNING", "NAV_RUNNING")

    def cancel_ack(self, token: int, accepted: bool, ros: float, steady: float) -> None:
        if token != self.token or not self._pending_navigation:
            return
        if not accepted:
            self.trip("CANCEL_REJECTED", ros, steady)
        # An accepted cancellation is not a terminal action result.

    def result(self, token: int, state: str, ros: float, steady: float) -> None:
        if token != self.token or not self._pending_navigation:
            return
        if state not in ("SUCCEEDED", "CANCELED", "FAILED"):
            self.trip("INVALID_NAV_RESULT", ros, steady)
            return
        self._pending_navigation = False
        self._accepted_navigation = False
        if (state == "SUCCEEDED" and self.state != "FAULT" and self.desired
                and self.desired.identity == self.current.identity
                and self.current.valid_until > ros and self._fresh_safety(steady)):
            self.state = "ARRIVED"
            self._completed = self.current.identity
            self._emit("FEEDBACK", "SUCCEEDED", "AT_TACTICAL_POINT")
            return
        self._finish(state, f"NAV_{state}", ros, steady)

    def _finish(self, state: str, reason: str, ros: float, steady: float) -> None:
        self._emit("FEEDBACK", state, reason)
        if self.current:
            self._completed = self.current.identity
        self.current = None
        if self.state != "FAULT":
            self.state = "IDLE"
            self._begin(ros, steady)

    def trip(self, reason: str, ros: float, steady: float) -> None:
        if self.state == "FAULT":
            return
        old_state = self.state
        self.state, self.reason = "FAULT", reason
        self._emit("FEEDBACK", "FAILED", reason)
        if self._accepted_navigation and old_state != "CANCELING":
            self._emit("CANCEL")
        elif old_state == "PLANNING":
            self._emit("CANCEL_PLAN")
            self.current = None

    def reset(self) -> bool:
        if self._pending_navigation or not self._healthy:
            return False
        self.state, self.reason = "IDLE", ""
        self.current = self.desired = None
        self._last_id = 0
        self._completed = None
        return True

    def tick(self, ros: float, steady: float) -> None:
        if not math.isfinite(ros) or not math.isfinite(steady):
            self.trip("INVALID_CLOCK", ros, steady)
            return
        if self._last_ros is not None and ros < self._last_ros - 0.001:
            self.trip("CLOCK_REWIND", ros, steady)
        self._last_ros = ros
        if self.state in ("PLANNING", "SENDING", "CANCELING") and steady > self._deadline:
            self.trip(f"{self.state}_TIMEOUT", ros, steady)
        if not self._fresh_safety(steady):
            self.safety(False, ros, steady)
        if self.desired and self.desired.valid_until <= ros:
            self.desired = None
            self._interrupt(ros, steady, "TASK_LEASE_EXPIRED")
        if self.state == "IDLE":
            self._begin(ros, steady)


class VelocityGate:
    """Final software velocity gate; does not replace the chassis hardware watchdog."""

    def __init__(self, command_timeout=0.25, permit_timeout=0.3,
                 max_linear=2.0, max_angular=2.0):
        if not all(math.isfinite(v) and v > 0 for v in (
            command_timeout, permit_timeout, max_linear, max_angular
        )):
            raise ValueError("velocity gate limits and timeouts must be finite and positive")
        self.command_timeout, self.permit_timeout = command_timeout, permit_timeout
        self.max_linear, self.max_angular = max_linear, max_angular
        self._command_at = self._permit_at = -math.inf
        self._allowed = False
        self._velocity = (0., 0., 0.)

    def permit(self, allowed: bool, now: float) -> None:
        if not allowed or bool(allowed) != self._allowed:
            self._velocity = (0., 0., 0.)
            self._command_at = -math.inf
        self._allowed, self._permit_at = bool(allowed), now

    def command(self, vx: float, vy: float, yaw_rate: float, now: float) -> None:
        if not self._allowed:
            return
        self._command_at = now
        if not all(math.isfinite(v) for v in (vx, vy, yaw_rate)):
            self._velocity = (0., 0., 0.)
            return
        scale = min(1., self.max_linear / max(math.hypot(vx, vy), 1e-9))
        self._velocity = (vx * scale, vy * scale,
                          max(-self.max_angular, min(self.max_angular, yaw_rate)))

    def output(self, now: float) -> tuple[float, float, float]:
        if (not self._allowed or not 0 <= now - self._permit_at <= self.permit_timeout
                or not 0 <= now - self._command_at <= self.command_timeout):
            return 0., 0., 0.
        return self._velocity
