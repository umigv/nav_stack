import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def load_global_parameters(config_path):
    """Load global parameters from a yaml file.

    Args:
        config_path (str): Path to the yaml file.

    Returns:
        dict: Global parameters.
    """
    with open(config_path, "r") as file:
        return yaml.safe_load(file)["global_parameters"]


def generate_launch_description():
    planner_directory = get_package_share_directory("marvin_bot_description")
    cv_grid_directory = get_package_share_directory("cv_grid")
    gps_directory = get_package_share_directory("gps_transform")

    # Construct Path to Gloabl Config File
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    config_file_path = os.path.join(launch_dir, "../config/global_config.yaml")

    # Load Global Parameters
    global_params = load_global_parameters(config_file_path)
    use_sim_time = global_params["use_sim_time"]

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planner_directory, "launch", "planner.launch.py")
        ),
        launch_arguments={
            "use_sim_time": str(use_sim_time),
        }.items(),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planner_directory, "launch", "teleop_launch.py")
        ),
        launch_arguments={
            "use_sim_time": str(use_sim_time),
        }.items(),
    )

    cv_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cv_grid_directory, "launch", "cv_grid.launch.py")
        ),
        launch_arguments={
            "use_sim_time": str(use_sim_time),
        }.items(),
    )

    gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gps_directory, "launch", "waypoint.launch.py")
        ),
        launch_arguments={
            "use_sim_time": str(use_sim_time),
        }.items(),
    )

    drive_pub = Node(
        package="drive_state_pub",
        executable="driving_state_publisher",
        name="drive_pub",
        output="screen",
    )

    return LaunchDescription([cv_grid, planner, teleop, drive_pub, gps])
