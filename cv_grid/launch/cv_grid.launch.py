import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    # param_file_path = 'src/nav_stack/cv_grid/config/cv_grid.yaml'
    param_file_path = os.path.join(
        get_package_share_directory("cv_grid"), "params", "cv_grid.yaml"
    )

    # Declare use_sim_time as launch argument
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",  # Default to False if not provided
        description="Use simulation time if true",
    )

    # Get propagated use_sim_time configuration
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            Node(
                package="cv_grid",
                executable="cv_grid",
                name="cv_grid",
                output="screen",
                parameters=[param_file_path, {"use_sim_time": use_sim_time}],
                # remappings=[
                #     ('/cv_grid', '/occupancy_grid')
                # ]
            ),
            Node(
                package="cv_grid",
                executable="cv_grid_transform_publisher",
                name="cv_grid_transform_publisher",
                output="screen",
                parameters=[param_file_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="cv_grid",
                executable="cv_view_transform_publisher",
                name="cv_view_transform_publisher",
                output="screen",
                parameters=[param_file_path, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
