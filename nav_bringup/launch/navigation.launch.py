from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    bringup_share_dir = Path(get_package_share_directory("nav_bringup"))

    with open(bringup_share_dir / "config" / "frames.yaml") as f:
        frames = yaml.safe_load(f)

    course = LaunchConfiguration("course").perform(context)
    waypoints_file = str(bringup_share_dir / "courses" / course / "gps.json")

    return [
        Node(
            package="occupancy_grid_transform",
            executable="occupancy_grid_transform",
            name="occupancy_grid_transform",
            parameters=[
                {"frame_id": frames["odom_frame"]},
            ],
            remappings=[
                ("occupancy_grid", "occupancy_grid/raw"),
                ("transformed_occupancy_grid", "occupancy_grid"),
            ],
        ),
        Node(
            package="autonav_goal_selection",
            executable="autonav_goal_selection",
            name="autonav_goal_selection",
            parameters=[
                {"waypoints_file_path": waypoints_file},
                {"map_frame_id": frames["map_frame"]},
                {"world_frame_id": frames["odom_frame"]},
            ],
            remappings=[
                ("occupancy_grid", "occupancy_grid"),
                ("odom", "odom/local"),
                ("fromLL", "fromLL"),
                ("goal", "goal"),
                ("gps_waypoint", "gps_waypoint"),
            ],
        ),
        Node(
            package="path_planning",
            executable="path_planning",
            name="path_planning",
            parameters=[
                {"frame_id": frames["odom_frame"]},
            ],
            remappings=[
                ("occupancy_grid", "occupancy_grid"),
                ("odom", "odom/local"),
                ("goal", "goal"),
                ("path", "path"),
            ],
        ),
        Node(
            package="path_tracking",
            executable="path_tracking",
            name="path_tracking",
            parameters=[
                {"base_frame_id": frames["base_frame"]},
                {"odom_frame_id": frames["odom_frame"]},
            ],
            remappings=[
                ("odom", "odom/local"),
                ("path", "path"),
                ("nav_cmd_vel", "nav_cmd_vel"),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "course",
                default_value="default",
                description="Course profile in courses/ to load waypoints from",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
