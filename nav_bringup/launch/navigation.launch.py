from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav_bringup.launch_utils import load_frames


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    mode = LaunchConfiguration("mode").perform(context)

    nodes = [
        Node(
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

    if mode.startswith("autonav"):
        nodes.append(
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
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                choices=["autonav", "autonav_sim", "self_drive", "self_drive_sim", "nav_test"],
                description="autonav/autonav_sim: full GPS stack; self_drive/self_drive_sim: EKF local only; nav_test: enc_odom only",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
