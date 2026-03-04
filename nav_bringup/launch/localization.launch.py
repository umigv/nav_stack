from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav_bringup.global_config import FRAMES, GPS_ORIGIN, MAGNETIC_DECLINATION_RADIANS


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("nav_bringup")
    localization_params = PathJoinSubstitution([bringup_share, "config", "localization.yaml"])
    use_enc_odom = LaunchConfiguration("use_enc_odom")

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        condition=UnlessCondition(use_enc_odom),
        parameters=[
            localization_params,
            {"map_frame": FRAMES["map_frame"]},
            {"odom_frame": FRAMES["odom_frame"]},
            {"base_link_frame": FRAMES["base_frame"]},
            {"world_frame": FRAMES["odom_frame"]},
        ],
        remappings=[
            ("odometry/filtered", "odom/local"),
        ],
    )

    enc_odom = Node(
        package="localization",
        executable="enc_odom_publisher",
        name="enc_odom_publisher",
        output="screen",
        condition=IfCondition(use_enc_odom),
        parameters=[
            {"odom_frame_id": FRAMES["odom_frame"]},
            {"base_frame_id": FRAMES["base_frame"]},
        ],
        remappings=[
            ("enc_vel", "enc_vel/raw"),
            ("odom", "odom/local"),
        ],
    )

    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[
            localization_params,
            {"datum": [GPS_ORIGIN["latitude"], GPS_ORIGIN["longitude"], 0.0]},
            {"magnetic_declination_radians": MAGNETIC_DECLINATION_RADIANS},
        ],
        remappings=[
            ("imu", "imu/raw"),
            ("gps/fix", "gps/raw"),
            ("odometry/filtered", "odom/global"),
            ("odometry/gps", "odom/gps"),
        ],
    )

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[
            localization_params,
            {"map_frame": FRAMES["map_frame"]},
            {"odom_frame": FRAMES["odom_frame"]},
            {"base_link_frame": FRAMES["base_frame"]},
            {"world_frame": FRAMES["map_frame"]},
        ],
        remappings=[
            ("odometry/filtered", "odom/global"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_enc_odom",
                default_value="false",
                description="Replace ekf_local with encoder odometry integration",
            ),
            ekf_local,
            enc_odom,
            navsat,
            ekf_global,
        ]
    )
