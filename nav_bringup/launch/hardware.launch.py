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

    vectornav_node = Node(
        package="vectornav_driver",
        executable="vectornav_driver",
        name="vectornav",
        output="screen",
        parameters=[
            f"{share}/config/sensors/vectornav.yaml",
            {"imuFrameId": frames["imu_frame"]},
            {"insFrameId": frames["base_frame"]},
        ],
        remappings=[
            ("vectornav/raw/imu", "imu/raw"),
            ("vectornav/raw/navsatfix", "gps/raw"),
            ("vectornav/raw/twist_with_covariance_stamped", "ins_vel/raw"),
        ],
    )

    match mode:
        case "autonav":
            return [vectornav_node]
        case "self_drive":
            return [vectornav_node]
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
