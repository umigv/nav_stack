from typing import assert_never

from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav_bringup.launch_utils import MODES, Mode, format_mode_description, load_frames


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    mode: Mode = LaunchConfiguration("mode").perform(context)

    occupancy_grid_transform_node = Node(
        package="occupancy_grid_transform",
        executable="occupancy_grid_transform",
        name="occupancy_grid_transform",
        parameters=[
            {"frame_id": frames["odom_frame"]},
        ],
        remappings=[
            ("occupancy_grid", "occupancy_grid/raw"),
            ("transformed_occupancy_grid", "inflated_occupancy_grid"),
        ],
    )

    path_tracking_node = Node(
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
    )

    goal_selection_node = Node(
        package="goal_selection",
        executable="goal_selection",
        name="goal_selection",
        remappings=[
            ("odom", "odom/local"),
            ("gps_coords", "gps/raw"),
            ("inflated_occupancy_grid", "inflated_occupancy_grid"),
            ("path", "path"),
        ],
    )

    match mode:
        case "autonav":
            return [occupancy_grid_transform_node, path_tracking_node, goal_selection_node]
        case "autonav_sim":
            return [occupancy_grid_transform_node, path_tracking_node, goal_selection_node]
        case "self_drive":
            return [occupancy_grid_transform_node, path_tracking_node]
        case "self_drive_sim":
            return [occupancy_grid_transform_node, path_tracking_node]
        case "nav_test":
            return [occupancy_grid_transform_node, path_tracking_node]
        case _:
            assert_never(mode)  # type: ignore[unreachable]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                choices=MODES,
                description=format_mode_description(
                    {
                        "autonav": "occupancy grid transform + path tracking + goal selection",
                        "autonav_sim": "occupancy grid transform + path tracking + goal selection",
                        "self_drive": "occupancy grid transform + path tracking",
                        "self_drive_sim": "occupancy grid transform + path tracking",
                        "nav_test": "occupancy grid transform + path tracking",
                    }
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
