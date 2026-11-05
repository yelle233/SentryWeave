import os
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    bringup = Path(get_package_share_directory("sentry_bringup"))
    gazebo = Path(get_package_share_directory("sentry_gazebo"))
    ns = LaunchConfiguration("namespace").perform(context)
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    config = LaunchConfiguration("config_file").perform(context)
    resource = os.pathsep.join(filter(None, (
        str(gazebo / "models"), LaunchConfiguration("extra_resource_path").perform(context),
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )))
    tf = [("/tf", "tf"), ("/tf_static", "tf_static")]
    common = {"namespace": ns, "use_sim_time": "true", "config_file": config}
    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory("ros_gz_sim"))
                                              / "launch/gz_sim.launch.py")),
            launch_arguments={"gz_args": f'-r {"-s --headless-rendering" if not gui else ""} "{world}"',
                              "gz_version": "8"}.items(),
        ),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gazebo_bridge",
             namespace=ns, output="screen", parameters=[{
                 "config_file": str(gazebo / "config/bridge.yaml"), "use_sim_time": True,
             }]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace=ns, remappings=tf, parameters=[{
                 "use_sim_time": True,
                 "robot_description": (gazebo / "models/sentry/robot.urdf").read_text(),
             }]),
        Node(package="sentry_ros", executable="sim_localization", namespace=ns,
             remappings=tf, parameters=[{"use_sim_time": True, "world_to_map": [
                 float(v) for v in yaml.safe_load(LaunchConfiguration("world_to_map").perform(context))
             ]}], output="screen"),
        Node(package="sentry_ros", executable="scenario", namespace=ns,
             parameters=[{"config_file": config, "use_sim_time": True,
                          "scenario": LaunchConfiguration("scenario").perform(context)}], output="screen"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch/navigation.launch.py")),
            launch_arguments={"namespace": ns, "use_sim_time": "true",
                              "config_file": config,
                              "map": LaunchConfiguration("map").perform(context),
                              "params_file": LaunchConfiguration("params_file").perform(context)}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch/decision.launch.py")),
            launch_arguments={**common, "self_from_tf": "false", "require_localization_health": "true",
                              "intent_model": LaunchConfiguration("intent_model").perform(context),
                              "utility_model": LaunchConfiguration("utility_model").perform(context),
                              "rl_model": LaunchConfiguration("rl_model").perform(context),
                              "rl_blend": LaunchConfiguration("rl_blend").perform(context),
                              "rl_mirror": LaunchConfiguration("rl_mirror").perform(context),
                              "rl_allow_transfer": LaunchConfiguration("rl_allow_transfer").perform(context),
                              "trace_path": LaunchConfiguration("trace_path").perform(context)}.items(),
        ),
    ]


def generate_launch_description():
    bringup = Path(get_package_share_directory("sentry_bringup"))
    gazebo = Path(get_package_share_directory("sentry_gazebo"))
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="sentry"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("world", default_value=str(gazebo / "worlds/arena.sdf")),
        DeclareLaunchArgument("map", default_value=str(gazebo / "maps/demo.yaml")),
        DeclareLaunchArgument("extra_resource_path", default_value=""),
        DeclareLaunchArgument("world_to_map", default_value="[0.0, 0.0, 0.0]"),
        DeclareLaunchArgument("params_file", default_value=str(bringup / "config/nav2.yaml")),
        DeclareLaunchArgument("config_file", default_value=str(bringup / "config/tactics.yaml")),
        DeclareLaunchArgument("scenario", default_value="conflict"),
        DeclareLaunchArgument("intent_model", default_value=""),
        DeclareLaunchArgument("utility_model", default_value=""),
        DeclareLaunchArgument("rl_model", default_value=""),
        DeclareLaunchArgument("rl_blend", default_value="0.25"),
        DeclareLaunchArgument("rl_mirror", default_value="false"),
        DeclareLaunchArgument("rl_allow_transfer", default_value="false"),
        DeclareLaunchArgument("trace_path", default_value=""),
        OpaqueFunction(function=_launch),
    ])
