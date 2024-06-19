import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    planner_directory = get_package_share_directory("marvin_bot_description")
    cv_grid_directory = get_package_share_directory("cv_grid")
    gps_directory = get_package_share_directory("gps_transform")


    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(planner_directory, 'launch', 'planner.launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
        }.items()        
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(planner_directory, 'launch', 'teleop_launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
        }.items()        
    )

    cv_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cv_grid_directory, 'launch', 'cv_grid.launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
        }.items()
    )

    gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gps_directory, 'launch', 'waypoint.launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
        }.items()
    )
    
    drive_pub = Node(
        package='drive_state_pub',
        executable='driving_state_publisher',
        name='drive_pub',
        output='screen',
    )

    return LaunchDescription([
        cv_grid,
        planner,
        teleop,
        drive_pub,
        gps
    ])