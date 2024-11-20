import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Constants
    planner_directory = get_package_share_directory("marvin_bot_description")
    params_file = os.path.join(
        planner_directory, "config", "custom_fn_params", "nav2_fn_params.yaml"
    )
    # params_file = "/home/maaz/ws/src/nav_stack/global_planner/marvin_bot_description/config/smac/smac_params.yaml"
    nav2_directory = get_package_share_directory("nav2_bringup")
    rviz_config = os.path.join(nav2_directory, "rviz", "nav2_default_view.rviz")

    # Declare use_sim_time as launch argument
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",  # Default to False if not provided
        description="Use simulation time if true",
    )

    # Get propagated use_sim_time configuration
    use_sim_time = LaunchConfiguration("use_sim_time")

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    # Nav2 Node
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_directory, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "slam": "True",
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "map": "",
        }.items(),
    )

    return LaunchDescription(
        [
            rviz_node,
            bringup_cmd,
        ]
    )
