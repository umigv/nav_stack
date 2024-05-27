import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    # param_file_path = 'src/nav_stack/cv_grid/config/cv_grid.yaml'
    param_file_path = os.path.join(
        get_package_share_directory('cv_grid'),
        'params',
        'cv_grid.yaml'
    )

    return LaunchDescription([
        Node(
            package='cv_grid',
            executable='cv_grid',
            name='cv_grid',
            output='screen',
            parameters=[
                param_file_path
            ]
        ),
        Node(
            package='cv_grid',
            executable='cv_grid_transform_publisher',
            name='cv_grid_transform_publisher',
            output='screen',
            parameters=[
                param_file_path
            ]
        ),
        Node(
            package='cv_grid',
            executable='cv_view_transform_publisher',
            name='cv_view_transform_publisher',
            output='screen',
            parameters=[
                param_file_path
            ]
        )
    ])