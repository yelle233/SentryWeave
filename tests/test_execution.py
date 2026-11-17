from dataclasses import replace

import pytest


@pytest.fixture
def rig(config):
    from sentry_core.execution import Executor
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Task, TaskType, Vec2

    executor = Executor(TacticalMap(config))
    executor.safety(True, 10.0, 100.0)
    task = Task("session-a", 1, 1, 10.0, 12.0, TaskType.DEFEND,
                "own_base", goal_id="defense_south", goal=Vec2(4, 4))
    return executor, task


def start(executor, task):
    executor.submit(task, 10.0, 100.0)
    commands = executor.drain()
    plan = next(c for c in commands if c.kind == "PLAN")
    executor.planned(plan.token, True, 10.01, 100.01)
    send = next(c for c in executor.drain() if c.kind == "SEND")
    executor.accepted(send.token, True, 10.02, 100.02)
    executor.drain()
    return send.token


def test_renewing_a_running_task_does_not_send_another_goal(rig):
    executor, task = rig
    token = start(executor, task)
    executor.submit(replace(task, valid_until=12.5), 10.5, 100.5)
    assert not any(c.kind in ("PLAN", "SEND", "CANCEL") for c in executor.drain())
    assert executor.token == token
    assert executor.state == "RUNNING"


def test_cancel_ack_does_not_mean_old_goal_is_finished(rig):
    executor, task = rig
    token = start(executor, task)
    newer = replace(task, task_id=2, goal_id="defense_north",
                    goal=replace(task.goal, y=8), created_at=10.1)
    executor.submit(newer, 10.1, 100.1)
    assert any(c.kind == "CANCEL" for c in executor.drain())
    executor.cancel_ack(token, True, 10.2, 100.2)
    assert executor.state == "CANCELING"
    assert not any(c.kind in ("PLAN", "SEND") for c in executor.drain())
    executor.result(token, "CANCELED", 10.3, 100.3)
    assert any(c.kind == "PLAN" and c.task.task_id == 2 for c in executor.drain())


def test_preemption_while_waiting_for_acceptance_cancels_late_goal(rig):
    executor, task = rig
    executor.submit(task, 10.0, 100.0)
    token = next(c.token for c in executor.drain() if c.kind == "PLAN")
    executor.planned(token, True, 10.01, 100.01)
    executor.drain()
    executor.submit(replace(task, task_id=2), 10.1, 100.1)
    assert not any(c.kind == "SEND" for c in executor.drain())
    executor.accepted(token, True, 10.2, 100.2)
    assert any(c.kind == "CANCEL" and c.token == token for c in executor.drain())
    assert not executor.motion_allowed(10.2, 100.2)


def test_cancel_timeout_latches_fault_and_blocks_new_goals(rig):
    executor, task = rig
    token = start(executor, task)
    executor.submit(replace(task, task_id=2), 10.1, 100.1)
    executor.drain()
    executor.safety(True, 10.2, 103.0)
    executor.tick(10.2, 103.0)
    assert executor.state == "FAULT"
    assert not executor.reset()
    assert not executor.motion_allowed(10.2, 103.0)
    executor.result(token, "CANCELED", 10.3, 103.1)
    assert executor.state == "FAULT"
    assert executor.reset()


def test_late_acceptance_after_timeout_is_still_canceled(rig):
    executor, task = rig
    executor.submit(task, 10.0, 100.0)
    token = next(c.token for c in executor.drain() if c.kind == "PLAN")
    executor.planned(token, True, 10.01, 100.01)
    executor.drain()
    executor.tick(10.1, 104.0)
    assert executor.state == "FAULT"
    executor.accepted(token, True, 10.2, 104.1)
    assert any(c.kind == "CANCEL" for c in executor.drain())
    assert not executor.reset()


def test_arrived_task_is_stationed_and_never_resent_on_heartbeat(rig):
    executor, task = rig
    token = start(executor, task)
    executor.result(token, "SUCCEEDED", 10.1, 100.1)
    executor.drain()
    executor.submit(replace(task, valid_until=12.5), 10.5, 100.5)
    assert executor.state == "ARRIVED"
    assert not executor.motion_allowed(10.5, 100.5)
    assert not any(c.kind == "SEND" for c in executor.drain())


def test_new_session_preempts_and_retired_session_cannot_return(rig):
    executor, task = rig
    token = start(executor, task)
    new = replace(task, session_id="session-b")
    executor.submit(new, 10.1, 100.1)
    executor.result(token, "CANCELED", 10.2, 100.2)
    executor.drain()
    executor.submit(replace(task, task_id=999), 10.3, 100.3)
    assert executor.desired.identity == new.identity
    assert not any(c.kind == "SEND" for c in executor.drain())


def test_expired_or_mutated_identity_is_rejected(rig):
    executor, task = rig
    start(executor, task)
    mutated = replace(task, goal=replace(task.goal, y=8))
    executor.submit(mutated, 10.1, 100.1)
    assert any(c.state == "REJECTED" for c in executor.drain())
    expired = replace(task, task_id=2, valid_until=10.0)
    executor.submit(expired, 10.2, 100.2)
    assert executor.desired == task
    assert any(c.state == "REJECTED" for c in executor.drain())


def test_stale_result_cannot_complete_replacement(rig):
    executor, task = rig
    token = start(executor, task)
    executor.submit(replace(task, task_id=2), 10.1, 100.1)
    executor.result(token, "CANCELED", 10.2, 100.2)
    second = next(c for c in executor.drain() if c.kind == "PLAN")
    executor.result(token, "SUCCEEDED", 10.3, 100.3)
    assert executor.token == second.token
    assert executor.state == "PLANNING"


def test_hold_cancels_motion_and_requires_no_target_pose(rig):
    from sentry_core.model import TaskType

    executor, task = rig
    token = start(executor, task)
    hold = replace(task, task_id=2, kind=TaskType.HOLD, goal=None, goal_id="")
    executor.submit(hold, 10.1, 100.1)
    assert not executor.motion_allowed(10.1, 100.1)
    executor.result(token, "CANCELED", 10.2, 100.2)
    assert executor.state == "IDLE"
    assert not any(c.kind == "PLAN" and c.task.kind == TaskType.HOLD for c in executor.drain())


def test_wall_clock_watchdog_stops_even_when_ros_clock_pauses(rig):
    executor, task = rig
    start(executor, task)
    assert executor.motion_allowed(10.02, 100.02)
    executor.tick(10.02, 101.0)
    assert not executor.motion_allowed(10.02, 101.0)
    assert any(c.kind == "CANCEL" for c in executor.drain())


def test_gate_rejects_nan_and_requires_fresh_permission_and_command():
    from sentry_core.execution import VelocityGate

    gate = VelocityGate()
    gate.command(1, 0, 0, 100)
    assert gate.output(100) == (0., 0., 0.)
    gate.permit(True, 100)
    gate.command(1, 0, 0, 100.01)
    assert gate.output(100.1)[0] == 1
    assert gate.output(101) == (0., 0., 0.)
    gate.permit(True, 102)
    gate.command(float("nan"), 0, 0, 102)
    assert gate.output(102) == (0., 0., 0.)


def test_gate_does_not_release_commands_received_while_permission_was_off():
    from sentry_core.execution import VelocityGate

    gate = VelocityGate()
    gate.permit(False, 10)
    gate.command(1, 0, 0, 10.01)
    gate.permit(True, 10.02)
    assert gate.output(10.03) == (0., 0., 0.)
    gate.command(0.5, 0, 0, 10.04)
    assert gate.output(10.05) == (0.5, 0., 0.)


def test_nonfinite_watchdog_configuration_is_rejected(config):
    from sentry_core.execution import Executor, VelocityGate
    from sentry_core.geometry import TacticalMap

    with pytest.raises(ValueError):
        Executor(TacticalMap(config), action_timeout=float("nan"))
    with pytest.raises(ValueError):
        VelocityGate(permit_timeout=float("inf"))


def test_task_waiting_for_first_healthy_world_is_not_lost(rig):
    executor, task = rig
    executor.safety(False, 10., 100.)
    executor.submit(task, 10., 100.)
    executor.safety(False, 10.1, 100.1)
    executor.tick(10.1, 100.1)
    assert not any(c.kind == "PLAN" for c in executor.drain())
    executor.safety(True, 10.2, 100.2)
    executor.tick(10.2, 100.2)
    assert any(c.kind == "PLAN" and c.task.identity == task.identity for c in executor.drain())


def test_new_task_survives_temporary_interlock_while_old_goal_cancels(rig):
    executor, task = rig
    token = start(executor, task)
    replacement = replace(task, task_id=2)
    executor.submit(replacement, 10.1, 100.1)
    executor.safety(False, 10.2, 100.2)
    executor.result(token, "CANCELED", 10.3, 100.3)
    executor.drain()
    executor.safety(True, 10.4, 100.4)
    executor.tick(10.4, 100.4)
    assert any(c.kind == "PLAN" and c.task.task_id == 2 for c in executor.drain())


def test_executor_can_skip_path_precheck_while_keeping_goal_legality(config):
    from sentry_core.execution import Executor
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Task, TaskType, Vec2

    arena = TacticalMap(config)
    goal = config.points[0].position
    machine = Executor(arena, check_path_before_navigate=False)
    machine.safety(True, 10.0, 100.0)
    legal_task = Task("s1", 1, 1, 10.0, 12.0, TaskType.DEFEND, config.points[0].zone,
                      None, "p0", goal, 0.0, 0.7, 0.9, False, "AREA_UNCOVERED")
    machine.submit(legal_task, 10.0, 100.0)
    kinds = [c.kind for c in machine.drain()]
    assert kinds == ["FEEDBACK", "SEND"]
    assert machine.state == "SENDING"
    machine.accepted(machine.token, True, 10.1, 100.1)
    assert machine.state == "RUNNING" and machine.motion_allowed(10.1, 100.1)

    # Illegal goals inside obstacles or outside bounds are still rejected immediately.
    illegal_machine = Executor(arena, check_path_before_navigate=False)
    illegal_machine.safety(True, 10.0, 100.0)
    bad = Task("s1", 2, 1, 10.0, 12.0, TaskType.DEFEND, config.points[0].zone,
               None, "bad", Vec2(-1.0, -1.0), 0.0, 0.7, 0.9, False, "AREA_UNCOVERED")
    illegal_machine.submit(bad, 10.0, 100.0)
    commands = illegal_machine.drain()
    assert [(c.kind, c.state) for c in commands] == [("FEEDBACK", "REJECTED")]


def test_executor_rejects_tasks_outside_configured_goal_scope(config):
    from sentry_core.execution import Executor
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Task, TaskType, Vec2

    scoped = replace(config, goal_scope="own_half")
    machine = Executor(TacticalMap(scoped))
    machine.safety(True, 10.0, 100.0)
    out_of_scope = Task("s1", 1, 1, 10.0, 12.0, TaskType.SUPPORT, "enemy_outpost",
                        1, "support_north", Vec2(15.0, 8.0), 0.0, 0.8, 0.9, False,
                        "TEAM_SUPPORT_HIGH")
    machine.submit(out_of_scope, 10.0, 100.0)
    assert [(c.kind, c.state) for c in machine.drain()] == [("FEEDBACK", "REJECTED")]
