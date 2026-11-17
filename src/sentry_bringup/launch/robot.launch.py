"""Real-robot bringup: localization, perception and referee drivers are external."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("sentry_bringup"))
    ns, config = LaunchConfiguration("namespace"), LaunchConfiguration("config_file")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="sentry"),
        DeclareLaunchArgument("map", description="Surveyed occupancy map YAML"),
        DeclareLaunchArgument("config_file", description="Calibrated season tactics YAML"),
        DeclareLaunchArgument("own_team", default_value="",
                              description="RED or BLUE; empty uses own_team from config_file"),
        DeclareLaunchArgument(
            "goal_scope", default_value="",
            description="full, own_half_and_highland or own_half; empty uses config_file"),
        DeclareLaunchArgument("params_file", default_value=str(share / "config/nav2.yaml")),
        DeclareLaunchArgument("controller_overlay", default_value=str(share / "config/omni_overlay.yaml")),
        DeclareLaunchArgument("intent_model", default_value=""),
        DeclareLaunchArgument("utility_model", default_value=""),
        DeclareLaunchArgument("rl_model", default_value=""),
        DeclareLaunchArgument("rl_blend", default_value="0.25"),
        DeclareLaunchArgument("rl_mirror", default_value="auto",
                              description="auto follows the field's mirror setting; "
                                          "true/false force mirrored/unmirrored RL features"),
        DeclareLaunchArgument("trace_path", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/navigation.launch.py")),
            launch_arguments={"namespace": ns, "use_sim_time": "false",
                              "config_file": config, "own_team": LaunchConfiguration("own_team"),
                              "goal_scope": LaunchConfiguration("goal_scope"),
                              "map": LaunchConfiguration("map"),
                              "params_file": LaunchConfiguration("params_file"),
                              "controller_overlay": LaunchConfiguration("controller_overlay")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/decision.launch.py")),
            launch_arguments={"namespace": ns, "config_file": config, "use_sim_time": "false",
                              "own_team": LaunchConfiguration("own_team"),
                              "goal_scope": LaunchConfiguration("goal_scope"),
                              "self_from_tf": "true", "require_localization_health": "true",
                              "intent_model": LaunchConfiguration("intent_model"),
                              "utility_model": LaunchConfiguration("utility_model"),
                              "rl_model": LaunchConfiguration("rl_model"),
                              "rl_blend": LaunchConfiguration("rl_blend"),
                              "rl_mirror": LaunchConfiguration("rl_mirror"),
                              "trace_path": LaunchConfiguration("trace_path")}.items(),
        ),
    ])
