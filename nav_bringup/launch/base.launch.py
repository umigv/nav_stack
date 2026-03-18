from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav_bringup.launch_utils import MODES


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("nav_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                choices=MODES,
                description="Operation mode, passed through to sensors.launch.py and localization.launch.py",
            ),
            DeclareLaunchArgument(
                "course",
                default_value="default",
                description="Course profile, passed through to sensors.launch.py and localization.launch.py (required for autonav, autonav_sim, self_drive_sim)",
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
