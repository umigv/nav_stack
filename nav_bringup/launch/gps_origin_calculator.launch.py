from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="localization",
                executable="gps_origin_calculator",
                output="screen",
                remappings=[("gps", "gps/raw")],
            )
        ]
    )
