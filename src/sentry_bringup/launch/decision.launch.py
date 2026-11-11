from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("sentry_bringup"))
    ns, sim = LaunchConfiguration("namespace"), LaunchConfiguration("use_sim_time")
    config = LaunchConfiguration("config_file")
    common = {"config_file": config, "own_team": LaunchConfiguration("own_team"),
              "use_sim_time": ParameterValue(sim, value_type=bool)}
    tf = [("/tf", "tf"), ("/tf_static", "tf_static")]
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="sentry"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("config_file",
                              default_value=str(share / "config/tactics_rmuc2027.yaml")),
        DeclareLaunchArgument("own_team", default_value="",
                              description="RED or BLUE; empty uses own_team from config_file"),
        DeclareLaunchArgument("intent_model", default_value=""),
        DeclareLaunchArgument("utility_model", default_value=""),
        DeclareLaunchArgument("rl_model", default_value=""),
        DeclareLaunchArgument("rl_blend", default_value="0.25"),
        DeclareLaunchArgument("rl_mirror", default_value="false"),
        DeclareLaunchArgument("rl_allow_transfer", default_value="false"),
        DeclareLaunchArgument("trace_path", default_value=""),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("self_from_tf", default_value="true"),
        DeclareLaunchArgument("require_localization_health", default_value="false"),
        Node(package="sentry_ros", executable="world_model", namespace=ns, output="screen",
             remappings=tf, parameters=[common, {
                 "base_frame": LaunchConfiguration("base_frame"),
                 "self_from_tf": ParameterValue(LaunchConfiguration("self_from_tf"), value_type=bool),
                 "require_localization_health": ParameterValue(
                     LaunchConfiguration("require_localization_health"), value_type=bool),
             }]),
        Node(package="sentry_ros", executable="decision", namespace=ns, output="screen",
             parameters=[common, {
                 "intent_model": LaunchConfiguration("intent_model"),
                 "utility_model": LaunchConfiguration("utility_model"),
                 "rl_model": LaunchConfiguration("rl_model"),
                 "rl_blend": ParameterValue(LaunchConfiguration("rl_blend"), value_type=float),
                 "rl_mirror": ParameterValue(LaunchConfiguration("rl_mirror"), value_type=bool),
                 "rl_allow_transfer": ParameterValue(LaunchConfiguration("rl_allow_transfer"), value_type=bool),
                 "trace_path": LaunchConfiguration("trace_path"),
             }]),
        Node(package="sentry_ros", executable="executor", namespace=ns, output="screen",
             parameters=[common]),
        Node(package="sentry_ros", executable="velocity_guard", namespace=ns, output="screen",
             parameters=[{"use_sim_time": ParameterValue(sim, value_type=bool)}]),
    ])
