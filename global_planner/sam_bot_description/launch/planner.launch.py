from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Constants
    planner_directory = get_package_share_directory("sam_bot_description")
    # params_file = os.path.join(planner_directory, 'config', 'custom_fn_params', 'nav2_params.yaml')
    params_file = "/home/maaz/ws/src/nav_stack/global_planner/sam_bot_description/config/smac/smac_params.yaml"
    nav2_directory = get_package_share_directory("nav2_bringup")
    rviz_config = os.path.join(nav2_directory, 'rviz', 'nav2_default_view.rviz')
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # Nav2 Node
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_directory, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': 'True',
            'use_sim_time': 'True',
            'params_file': params_file,
            'map': "",
        }.items(),
    )

    return LaunchDescription([
        rviz_node,
        bringup_cmd,
    ])