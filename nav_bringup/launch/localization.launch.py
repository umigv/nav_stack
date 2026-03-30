#from typing import assert_never

from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav_bringup.launch_utils import MODES, Mode, bringup_share, format_mode_description, load_frames, load_gps_file


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    mode: Mode = LaunchConfiguration("mode").perform(context)
    course = LaunchConfiguration("course").perform(context)
    gps_file = load_gps_file(course)
    localization_params = f"{bringup_share()}/config/localization/localization.yaml"

    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[
            localization_params,
            {"map_frame": frames["map_frame"]},
            {"odom_frame": frames["odom_frame"]},
            {"base_link_frame": frames["base_frame"]},
            {"world_frame": frames["odom_frame"]},
        ],
        remappings=[
            ("odometry/filtered", "odom/local"),
        ],
    )

    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[
            localization_params,
            {"map_frame": frames["map_frame"]},
            {"odom_frame": frames["odom_frame"]},
            {"base_link_frame": frames["base_frame"]},
            {"world_frame": frames["map_frame"]},
        ],
        remappings=[
            ("odometry/filtered", "odom/global"),
        ],
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[
            localization_params,
            {"datum": [gps_file["datum"]["latitude"], gps_file["datum"]["longitude"], 0.0]},
        ],
        remappings=[
            ("imu", "imu/raw"),
            ("gps/fix", "gps/raw"),
            ("odometry/filtered", "odom/global"),
            ("odometry/gps", "odom/gps"),
        ],
    )

    enc_odom_node = Node(
        package="localization",
        executable="enc_odom_publisher",
        name="enc_odom_publisher",
        output="screen",
        parameters=[
            {"odom_frame_id": frames["odom_frame"]},
            {"base_frame_id": frames["base_frame"]},
        ],
        remappings=[
            ("enc_vel", "enc_vel/raw"),
            ("odom", "odom/local"),
        ],
    )

    identity_map_odom_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", frames["map_frame"], frames["odom_frame"]],
    )

    match mode:
        case "autonav":
            return [ekf_local_node, navsat_transform_node, ekf_global_node]
        case "autonav_sim":
            return [ekf_local_node, navsat_transform_node, ekf_global_node]
        case "self_drive":
            return [ekf_local_node, identity_map_odom_node]
        case "self_drive_sim":
            return [ekf_local_node, identity_map_odom_node]
        case "nav_test":
            return [enc_odom_node, identity_map_odom_node]
        #case _:
            #assert_never(mode)  # type: ignore[unreachable]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                choices=MODES,
                description=format_mode_description(
                    {
                        "autonav": "EKF local + navsat transform + EKF global",
                        "autonav_sim": "EKF local + navsat transform + EKF global",
                        "self_drive": "EKF local + identity map->odom",
                        "self_drive_sim": "EKF local + identity map->odom",
                        "nav_test": "enc_odom + identity map->odom",
                    }
                ),
            ),
            DeclareLaunchArgument(
                "course",
                default_value="default",
                description="Course profile in courses/ to load GPS datum from (required for autonav, autonav_sim)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
