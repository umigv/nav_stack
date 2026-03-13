from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share_dir = Path(get_package_share_directory("nav_bringup"))

    with open(bringup_share_dir / "config" / "frames.yaml") as f:
        frames = yaml.safe_load(f)

    return LaunchDescription(
        [
            Node(
                package="occupancy_grid_transform",
                executable="occupancy_grid_transform",
                name="occupancy_grid_transform",
                parameters=[
                    {"frame_id": frames["odom_frame"]},
                ],
                remappings=[
                    ("occupancy_grid", "occ_grid"),
                    ("transformed_occupancy_grid", "inflated_occupancy_grid"),
                ],
            ),
            Node(
                package="goal_selection",
                executable="goal_selection",
                name="goal_selection",
                remappings=[
                    ("odom", "odom/local"),
                    ("gps_coords", "gps/raw"),
                    ("inflated_occupancy_grid", "inflated_occupancy_grid"),
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
                    ("smoothed_path", "smoothed_path"),
                ],
            ),
        ]
    )
