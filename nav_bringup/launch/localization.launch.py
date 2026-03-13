import json
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    bringup_share_dir = Path(get_package_share_directory("nav_bringup"))
    bringup_share = FindPackageShare("nav_bringup")

    with open(bringup_share_dir / "config" / "frames.yaml") as f:
        frames = yaml.safe_load(f)

    field = LaunchConfiguration("field").perform(context)
    with open(bringup_share_dir / "fields" / field / "gps.json") as f:
        gps_file = json.load(f)

    localization_params = PathJoinSubstitution([bringup_share, "config", "localization", "localization.yaml"])
    use_enc_odom = LaunchConfiguration("use_enc_odom")

    return [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_local",
            output="screen",
            condition=UnlessCondition(use_enc_odom),
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
        ),
        Node(
            package="localization",
            executable="enc_odom_publisher",
            name="enc_odom_publisher",
            output="screen",
            condition=IfCondition(use_enc_odom),
            parameters=[
                {"odom_frame_id": frames["odom_frame"]},
                {"base_frame_id": frames["base_frame"]},
            ],
            remappings=[
                ("enc_vel", "enc_vel/raw"),
                ("odom", "odom/local"),
            ],
        ),
        Node(
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
        ),
        Node(
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
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "field",
                default_value="default",
                description="Field profile in fields/ to load GPS datum from",
            ),
            DeclareLaunchArgument(
                "use_enc_odom",
                default_value="false",
                description="Replace ekf_local with encoder odometry integration",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
