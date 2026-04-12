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
        name="vectornav_driver",
        output="screen",
        parameters=[
            f"{share}/config/sensors/vectornav.yaml",
            {"imuFrameId": frames["imu_frame"]},
            {"insFrameId": frames["base_frame"]},
            {"gnssAFrameId": frames["gnss_a_frame"]},
            {"gnssBFrameId": frames["gnss_b_frame"]},
        ],
        remappings=[
            ("vectornav/raw/imu", "imu/raw"),
            ("vectornav/raw/navsatfix", "gps/raw"),
            ("vectornav/raw/twist_with_covariance_stamped", "ins_vel/raw"),
        ],
    )

    # Static transforms for VN-300 sensor geometry (FLU, meters)
    # TODO: replace base_link -> imu_link with URDF once available
    tf_base_to_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", frames["base_frame"],
                   "--child-frame-id", frames["imu_frame"]],
    )
    tf_imu_to_gnss_a = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0.021", "--y", "-0.294481", "--z", "0.0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", frames["imu_frame"],
                   "--child-frame-id", frames["gnss_a_frame"]],
    )
    tf_gnss_a_to_gnss_b = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "-0.518", "--y", "0.585", "--z", "0.0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", frames["gnss_a_frame"],
                   "--child-frame-id", frames["gnss_b_frame"]],
    )

    sensor_transforms = [tf_base_to_imu, tf_imu_to_gnss_a, tf_gnss_a_to_gnss_b]

    match mode:
        case "autonav":
            return [vectornav_node, *sensor_transforms]
        case "self_drive":
            return [vectornav_node, *sensor_transforms]
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
