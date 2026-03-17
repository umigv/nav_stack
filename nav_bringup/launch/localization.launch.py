from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav_bringup.launch_utils import bringup_share, load_frames, load_gps_file


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    mode = LaunchConfiguration("mode").perform(context)
    localization_params = f"{bringup_share()}/config/localization/localization.yaml"

    if mode == "nav_test":
        return [
            Node(
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
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_odom_publisher",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", frames["map_frame"], frames["odom_frame"]],
            ),
        ]
    elif mode.startswith("autonav"):
        course = LaunchConfiguration("course").perform(context)
        gps_file = load_gps_file(course)

        return [
            Node(
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
    else:
        return [
            Node(
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
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_odom_publisher",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", frames["map_frame"], frames["odom_frame"]],
            ),
        ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                choices=["autonav", "autonav_sim", "self_drive", "self_drive_sim", "nav_test"],
                description="autonav/autonav_sim: full GPS stack; self_drive/self_drive_sim: EKF local only; nav_test: enc_odom only",
            ),
            DeclareLaunchArgument(
                "course",
                default_value="default",
                description="Course profile in courses/ to load GPS datum from (required for autonav, autonav_sim)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
