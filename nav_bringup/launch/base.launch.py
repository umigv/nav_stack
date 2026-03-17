from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("nav_bringup")

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
                description="Course profile in courses/ to load map and GPS datum from (required for autonav, autonav_sim, self_drive_sim)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_share, "launch", "core.launch.py"])),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_share, "launch", "sensors.launch.py"])),
                launch_arguments=[
                    ("mode", LaunchConfiguration("mode")),
                    ("course", LaunchConfiguration("course")),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "localization.launch.py"])
                ),
                launch_arguments=[
                    ("mode", LaunchConfiguration("mode")),
                    ("course", LaunchConfiguration("course")),
                ],
            ),
        ]
    )
