"""Run against simulation.launch.py with RMJC_GAZEBO_TEST=1."""

import os
import json
from pathlib import Path
import time

import pytest

pytestmark = [
    pytest.mark.gazebo,
    pytest.mark.skipif(os.environ.get("RMJC_GAZEBO_TEST") != "1",
                       reason="requires a running Harmonic + Nav2 stack"),
]


def test_harmonic_sensors_and_autonomous_navigation_complete():
    import rclpy
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from sentry_interfaces.msg import TacticalTask, TaskFeedback, WorldState
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy

    rclpy.init()
    node = rclpy.create_node("rmjc_physics_acceptance", namespace="sentry")
    data = {"scan": 0, "odom": [], "world": [], "arrivals": 0, "faults": [],
            "tasks": {}, "arrived_types": set(), "keepout": None, "rl_used": 0, "rl_algorithms": set()}

    def feedback(message):
        data["arrivals"] += int(message.execution_state == TaskFeedback.SUCCEEDED)
        if message.execution_state == TaskFeedback.SUCCEEDED:
            data["arrived_types"].add(data["tasks"].get((message.session_id, message.task_id)))
        if "TIMEOUT" in message.reason_code or "GOAL_RESPONSE_ERROR" in message.reason_code:
            data["faults"].append(message.reason_code)

    node.create_subscription(LaserScan, "scan",
                             lambda m: data.update(scan=data["scan"] + int(len(m.ranges) > 0)),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, "ground_truth/odom",
                             lambda m: data["odom"].append((m.pose.pose.position.x,
                                                           m.pose.pose.position.y)),
                             qos_profile_sensor_data)
    node.create_subscription(WorldState, "world/state", lambda m: data["world"].append(m), 1)
    node.create_subscription(TaskFeedback, "tactical/feedback", feedback, 20)
    node.create_subscription(TacticalTask, "tactical/task", lambda m:
                             data["tasks"].update({(m.session_id, m.task_id): m.task_type}), 1)
    node.create_subscription(OccupancyGrid, "keepout_mask", lambda m: data.update(keepout=m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def trace(message):
        record = json.loads(message.data)
        info = record["decision"].get("policy_info", {})
        if info.get("used"):
            data["rl_used"] += 1
            data["rl_algorithms"].add(info["algorithm"])
    node.create_subscription(String, "tactical/trace", trace, 1)
    try:
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if TacticalTask.SUPPORT in data["arrived_types"] and len(data["odom"]) > 20:
                break
        assert data["scan"] > 10, {"scans": data["scan"], "odometry": len(data["odom"])}
        assert len(data["odom"]) > 20
        assert any(w.self_valid and w.referee_valid for w in data["world"])
        assert any(w.allies for w in data["world"])
        first = data["odom"][0]
        moved = max(((x - first[0]) ** 2 + (y - first[1]) ** 2) ** 0.5 for x, y in data["odom"])
        assert moved > 0.2, {"moved_m": moved, "arrivals": data["arrivals"]}
        assert data["arrivals"] > 0, "No actual Nav2 navigation succeeded within 90 seconds"
        assert TacticalTask.SUPPORT in data["arrived_types"], "No support task arrived in time"
        assert data["keepout"] is not None and 100 in data["keepout"].data
        assert not data["faults"], data["faults"]
        if os.environ.get("RMJC_RL_EXPECTED") == "1":
            assert data["rl_used"] > 0, "Neural policy never participated in candidate scoring"
        report_path = os.environ.get("RMJC_GAZEBO_REPORT")
        if report_path:
            Path(report_path).write_text(json.dumps({
                "environment": "ROS 2 Jazzy + Gazebo Harmonic + Nav2",
                "scan_messages": data["scan"], "odometry_messages": len(data["odom"]),
                "world_messages": len(data["world"]), "navigation_arrivals": data["arrivals"],
                "displacement_m": moved, "execution_faults": data["faults"],
                "arrived_task_types": sorted(t for t in data["arrived_types"] if t is not None),
                "rl_used_decisions": data["rl_used"], "rl_algorithms": sorted(data["rl_algorithms"]),
            }, indent=2) + "\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()
