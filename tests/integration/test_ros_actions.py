"""True rclpy Action integration tests; skipped when the overlay is not sourced."""

import importlib.util
import threading
import time

import pytest

try:
    ROS_AVAILABLE = all(importlib.util.find_spec(name) is not None
                        for name in ("rclpy", "nav2_msgs", "sentry_interfaces.msg"))
except ModuleNotFoundError:
    ROS_AVAILABLE = False
pytestmark = [
    pytest.mark.ros,
    pytest.mark.skipif(not ROS_AVAILABLE, reason="requires a sourced Jazzy workspace with Nav2"),
]


def test_ros_codec_preserves_source_time_unknown_fields_and_history(world_factory):
    from sentry_ros.codec import world_from_msg, world_to_msg

    world = world_factory(hp=None, enemy_outpost=None, allies=((1, 14, 4, 1.4, 0.4),))
    restored = world_from_msg(world_to_msg(world))
    assert restored == world


def test_jazzy_accepts_the_holonomic_controller_configuration(tmp_path):
    import os
    import signal
    import subprocess
    import rclpy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from conftest import ROOT

    log_path = tmp_path / "omni-controller.log"
    environment = {**os.environ, "ROS_DOMAIN_ID": "82"}
    with log_path.open("w") as log:
        process = subprocess.Popen([
            "ros2", "run", "nav2_controller", "controller_server", "--ros-args",
            "--params-file", str(ROOT / "src/sentry_bringup/config/nav2.yaml"),
            "--params-file", str(ROOT / "src/sentry_bringup/config/omni_overlay.yaml"),
        ], env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init(domain_id=82)
        node = rclpy.create_node("omni_configuration_test")
        try:
            client = node.create_client(ChangeState, "/controller_server/change_state")
            assert client.wait_for_service(timeout_sec=10), log_path.read_text()
            request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_CONFIGURE
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=15)
            assert future.done() and future.result().success, log_path.read_text()
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


@pytest.mark.parametrize("orphaned_goal", [None, "executing", "canceling"])
def test_real_nav_action_waits_for_terminal_cancel_before_replacement(config, orphaned_goal):
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from rclpy.action import ActionClient, ActionServer, CancelResponse
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sentry_core.model import Task, TaskType, Vec2
    from sentry_interfaces.msg import TacticalTask, TaskFeedback, WorldState
    from sentry_ros.codec import task_to_msg, world_to_msg
    from sentry_ros.executor_node import ExecutorNode
    from sentry_core.model import Observation, Phase, Referee, Team
    from sentry_core.tracking import WorldModel
    from conftest import ROOT

    rclpy.init()
    namespace = "action_contract_test"
    node = Node("fake_nav2", namespace=namespace)
    group = ReentrantCallbackGroup()
    release_cancel = threading.Event()
    cancel_seen = threading.Event()
    lock = threading.Lock()
    counts = {"active": 0, "maximum": 0, "accepted": 0}
    stop = threading.Event()

    def plan(handle):
        result = ComputePathToPose.Result()
        start = PoseStamped()
        start.header = handle.request.goal.header
        start.pose.position.x, start.pose.position.y = 3., 6.
        start.pose.orientation.w = 1.
        result.path.header = handle.request.goal.header
        result.path.poses = [start, handle.request.goal]
        handle.succeed()
        return result

    def navigate(handle):
        with lock:
            counts["active"] += 1
            counts["accepted"] += 1
            counts["maximum"] = max(counts["maximum"], counts["active"])
        while not stop.is_set():
            if handle.is_cancel_requested:
                cancel_seen.set()
                release_cancel.wait(timeout=1.5)
                with lock:
                    counts["active"] -= 1
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.01)
        with lock:
            counts["active"] -= 1
        handle.abort()
        return NavigateToPose.Result()

    planner = ActionServer(node, ComputePathToPose, "compute_path_to_pose", plan,
                           callback_group=group)
    navigator = ActionServer(node, NavigateToPose, "navigate_to_pose", navigate,
                             cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
    runtime = ExecutorNode(namespace=namespace, parameter_overrides=[
        Parameter("config_file", value=str(ROOT / "src/sentry_bringup/config/tactics.yaml")),
        Parameter("action_timeout", value=3.0),
        Parameter("safety_timeout", value=3.0),
    ])
    worlds = node.create_publisher(WorldState, "world/state", 1)
    tasks = node.create_publisher(TacticalTask, "tactical/task", 1)
    feedback = []
    subscription = node.create_subscription(TaskFeedback, "tactical/feedback",
                                            lambda message: feedback.append(message), 20)
    model = WorldModel(config)
    current = {"id": 1}

    def publish():
        now = node.get_clock().now().nanoseconds / 1e9
        model.observe(Observation(7, Team.ALLY, Vec2(3, 6), now), now)
        model.update_referee(Referee(now, Phase.RUNNING, 0.9, 0.9, True, True, True))
        worlds.publish(world_to_msg(model.snapshot(now)))
        task = Task("integration", current["id"], 1, started, now + 2.0, TaskType.DEFEND,
                    "own_base", goal_id=f"point-{current['id']}",
                    goal=Vec2(4, 4 if current["id"] == 1 else 8))
        tasks.publish(task_to_msg(task, "map", now))

    started = node.get_clock().now().nanoseconds / 1e9
    timer = node.create_timer(0.1, publish, callback_group=MutuallyExclusiveCallbackGroup())
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()

    def wait_for(predicate, seconds=8):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert predicate(), [(f.task_id, f.execution_state, f.reason_code) for f in feedback]

    try:
        orphan_client = None
        if orphaned_goal:
            orphan_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
            assert orphan_client.wait_for_server(timeout_sec=5)
            orphan = NavigateToPose.Goal()
            orphan.pose.header.frame_id = "map"
            orphan.pose.pose.position.x, orphan.pose.pose.position.y = 4., 4.
            orphan.pose.pose.orientation.w = 1.
            orphan_future = orphan_client.send_goal_async(orphan)
            wait_for(orphan_future.done)
            wait_for(lambda: counts["accepted"] == 1)
            if orphaned_goal == "canceling":
                orphan_future.result().cancel_goal_async()
                wait_for(cancel_seen.is_set)
            else:
                release_cancel.set()
        executor.add_node(runtime)
        if orphaned_goal == "canceling":
            wait_for(lambda: runtime.startup_state in ("DRAINING", "READY"))
            assert runtime.startup_state == "DRAINING", "Already-canceling goal was missed"
            assert counts["accepted"] == 1
            release_cancel.set()
        wait_for(lambda: any(f.task_id == 1 and f.execution_state == TaskFeedback.RUNNING
                            for f in feedback))
        assert counts["maximum"] == 1, "An old goal survived executor startup"
        cancel_seen.clear()
        release_cancel.clear()
        current["id"] = 2
        wait_for(cancel_seen.is_set)
        time.sleep(0.15)
        assert counts["accepted"] == (2 if orphaned_goal else 1)
        release_cancel.set()
        wait_for(lambda: any(f.task_id == 2 and f.execution_state == TaskFeedback.RUNNING
                            for f in feedback))
        assert counts["maximum"] == 1
        assert GoalStatus.STATUS_CANCELED == 5
    finally:
        stop.set()
        release_cancel.set()
        timer.cancel()
        executor.shutdown(timeout_sec=3)
        worker.join(timeout=3)
        planner.destroy()
        navigator.destroy()
        if orphan_client:
            orphan_client.destroy()
        node.destroy_subscription(subscription)
        node.destroy_node()
        runtime.destroy_node()
        rclpy.shutdown()
