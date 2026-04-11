from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav_bringup.launch_utils import MODES, Mode, bringup_share, format_mode_description, load_frames
from typing_extensions import assert_never


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    mode: Mode = LaunchConfiguration("mode").perform(context)
    share = bringup_share()

    imu_node = Node(
        package="vectornav",
        executable="vectornav_node",
        name="vectornav",
        output="screen",
        parameters=[
            f"{share}/config/sensors/imu.yaml",
            {"frame_id": frames["imu_frame"]},
            {"map_frame_id": frames["map_frame"]},
        ],
        remappings=[
            ("vectornav/data", "imu/raw"),
        ],
    )

    gps_node = Node(
        package="ublox_driver",
        executable="ublox_driver",
        name="ublox_driver",
        output="screen",
        parameters=[
            f"{share}/config/sensors/gps.yaml",
            {"ublox_frame_id": frames["gps_frame"]},
        ],
        remappings=[
            ("ublox/gps", "gps/raw"),
        ],
    )

    match mode:
        case "autonav":
            return [imu_node, gps_node]
        case "self_drive":
            return [imu_node]
        case "nav_test":
            return []
        case _:
            assert_never(mode)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                choices=MODES,
                description=format_mode_description(
                    {
                        "autonav": "IMU + GPS",
                        "self_drive": "IMU only",
                        "nav_test": "no sensors",
                    }
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
