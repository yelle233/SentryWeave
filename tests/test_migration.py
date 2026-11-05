from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


def test_fortress_world_plugins_become_harmonic_and_spawn_reference_robot():
    from import_polarbear import migrate_sdf

    source = """<sdf version="1.7"><world name="default">
      <plugin filename="libignition-gazebo-physics-system.so"
              name="ignition::gazebo::systems::Physics"/>
      <model name="arena"><include><uri>model://rmuc_2025</uri></include></model>
    </world></sdf>"""
    converted = migrate_sdf(source, spawn=(3., 6., 0.))
    root = ET.fromstring(converted)
    plugin = root.find("world/plugin")
    assert plugin.get("filename") == "gz-sim-physics-system"
    assert plugin.get("name") == "gz::sim::systems::Physics"
    assert root.find("world/include/uri").text == "model://sentry"
    assert "ignition" not in converted


def test_custom_gazebo_plugins_require_real_port_instead_of_silent_rename():
    from import_polarbear import migrate_sdf

    with pytest.raises(ValueError, match="custom"):
        migrate_sdf('<sdf version="1.7"><world name="w"><plugin filename="librmoss_referee.so" '
                    'name="rmoss::Referee"/></world></sdf>')


def test_nav2_migration_removes_builtin_bt_registration_and_replaces_custom_plugins():
    from import_polarbear import migrate_navigation

    reference = yaml.safe_load((Path(__file__).resolve().parents[1] /
                               "src/sentry_bringup/config/nav2.yaml").read_text())
    source = {
        "bt_navigator": {"ros__parameters": {
            "plugin_lib_names": ["nav2_compute_path_to_pose_action_bt_node"],
            "robot_base_frame": "gimbal_yaw_fake",
            "default_nav_to_pose_bt_xml": "old.xml",
        }},
        "planner_server": {"ros__parameters": {"planner_plugins": ["GridBased"],
            "GridBased": {"plugin": "nav2_theta_star_planner/ThetaStarPlanner"}}},
        "controller_server": {"ros__parameters": {"FollowPath": {
            "plugin": "pb_omni_pid_pursuit_controller::OmniPidPursuitController"}}},
    }
    converted, audit = migrate_navigation(source, reference)
    assert "plugin_lib_names" not in converted["bt_navigator"]["ros__parameters"]
    assert converted["bt_navigator"]["ros__parameters"]["robot_base_frame"] == "base_link"
    assert "default_nav_to_pose_bt_xml" not in converted["bt_navigator"]["ros__parameters"]
    planner = converted["planner_server"]["ros__parameters"]["GridBased"]["plugin"]
    assert planner == "nav2_theta_star_planner::ThetaStarPlanner"
    assert "pb_" not in converted["controller_server"]["ros__parameters"]["FollowPath"]["plugin"]
    assert audit["replaced_modules"]


def test_bridge_message_namespace_migration_keeps_ros_topic_names():
    from import_polarbear import migrate_bridge

    old = [{"ros_topic_name": "odometry", "gz_type_name": "ignition.msgs.Odometry",
            "ros_type_name": "nav_msgs/msg/Odometry", "direction": "GZ_TO_ROS"}]
    new = migrate_bridge(old)
    assert new[0]["gz_type_name"] == "gz.msgs.Odometry"
    assert new[0]["ros_topic_name"] == "odometry"
    assert old[0]["gz_type_name"] == "ignition.msgs.Odometry"


def test_survey_anchor_fit_recovers_rotation_translation_and_rejects_scale_error():
    import math
    from fit_map_transform import fit_transform

    anchors = [
        {"world": [0, 0], "map": [1, 2]},
        {"world": [2, 0], "map": [1, 4]},
        {"world": [0, 3], "map": [-2, 2]},
    ]
    result = fit_transform(anchors)
    assert result["world_to_map"] == pytest.approx([1, 2, math.pi / 2])
    assert result["rmse_m"] < 1e-10
    anchors[-1]["map"] = [-20, 2]
    with pytest.raises(ValueError, match="residual"):
        fit_transform(anchors, max_rmse=0.2)
