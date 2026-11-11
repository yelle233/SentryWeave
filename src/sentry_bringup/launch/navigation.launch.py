from pathlib import Path
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from sentry_core.config import load_config
from sentry_core.keepout import render_keepout


def _nodes(context):
    share = Path(get_package_share_directory("sentry_bringup"))
    namespace = LaunchConfiguration("namespace").perform(context)
    sim = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    tactics = load_config(LaunchConfiguration("config_file").perform(context),
                          LaunchConfiguration("own_team").perform(context) or None)
    mask_directory = tempfile.mkdtemp(prefix="sentryweave-keepout-")
    mask = render_keepout(tactics, LaunchConfiguration("map").perform(context), mask_directory)
    topic_prefix = "/" + namespace.strip("/") if namespace.strip("/") else ""
    params = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration("params_file"), root_key=namespace,
        param_rewrites={"use_sim_time": "true" if sim else "false",
                        "yaml_filename": LaunchConfiguration("map"),
                        "filter_info_topic": topic_prefix + "/costmap_filter_info",
                        "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml":
                            str(share / "behavior_trees/navigate.xml")},
        convert_types=True,
    ), allow_substs=True)
    parameters = [params, {"use_sim_time": sim}]
    overlay = LaunchConfiguration("controller_overlay").perform(context)
    if overlay:
        parameters.append(ParameterFile(RewrittenYaml(
            source_file=overlay, root_key=namespace, param_rewrites={}, convert_types=True,
        ), allow_substs=True))
    tf = [("/tf", "tf"), ("/tf_static", "tf_static")]
    definitions = [
        ("nav2_map_server", "map_server", []),
        ("nav2_planner", "planner_server", []),
        ("nav2_controller", "controller_server", [("cmd_vel", "cmd_vel_raw")]),
        ("nav2_behaviors", "behavior_server", [("cmd_vel", "cmd_vel_raw")]),
        ("nav2_bt_navigator", "bt_navigator", []),
        ("nav2_velocity_smoother", "velocity_smoother", [("cmd_vel", "cmd_vel_raw")]),
        ("nav2_collision_monitor", "collision_monitor", []),
    ]
    nodes = [Node(package=package, executable=name, name=name, namespace=namespace,
                  output="screen", parameters=parameters, remappings=tf + remaps)
             for package, name, remaps in definitions]
    nodes[1:1] = [
        Node(package="nav2_map_server", executable="map_server", name="keepout_mask_server",
             namespace=namespace, output="screen", parameters=[{
                 "use_sim_time": sim, "yaml_filename": str(mask),
                 "topic_name": "keepout_mask", "frame_id": tactics.frame,
             }]),
        Node(package="nav2_map_server", executable="costmap_filter_info_server",
             name="costmap_filter_info_server", namespace=namespace, output="screen",
             parameters=[{"use_sim_time": sim, "type": 0,
                          "filter_info_topic": "costmap_filter_info",
                          "mask_topic": topic_prefix + "/keepout_mask",
                          "base": 0.0, "multiplier": 1.0}]),
    ]
    nodes.append(Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", namespace=namespace, output="screen",
        parameters=[{"use_sim_time": sim, "autostart": True,
                     "node_names": ["map_server", "keepout_mask_server", "costmap_filter_info_server",
                                    *[name for _, name, _ in definitions[1:]]],
                     "bond_timeout": 4.0}],
    ))
    def cleanup(_context):
        shutil.rmtree(mask_directory, ignore_errors=True)
        return []
    nodes.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory("sentry_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="sentry"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("config_file",
                              default_value=str(share / "config/tactics_rmuc2027.yaml")),
        DeclareLaunchArgument("own_team", default_value=""),
        DeclareLaunchArgument("params_file", default_value=str(share / "config/nav2.yaml")),
        DeclareLaunchArgument("controller_overlay", default_value=""),
        OpaqueFunction(function=_nodes),
    ])
