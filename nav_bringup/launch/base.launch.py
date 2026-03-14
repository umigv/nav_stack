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
                "simulation",
                default_value="false",
                description="Launch sensor simulator instead of real hardware drivers",
            ),
            DeclareLaunchArgument(
                "use_enc_odom",
                default_value="false",
                description="Replace ekf_local with encoder odometry integration",
            ),
            DeclareLaunchArgument(
                "course",
                default_value="default",
                description="Course profile in courses/ to load map and GPS datum from",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_share, "launch", "core.launch.py"])),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_share, "launch", "sensors.launch.py"])),
                launch_arguments=[
                    ("simulation", LaunchConfiguration("simulation")),
                    ("course", LaunchConfiguration("course")),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "localization.launch.py"])
                ),
                launch_arguments=[
                    ("use_enc_odom", LaunchConfiguration("use_enc_odom")),
                    ("course", LaunchConfiguration("course")),
                ],
            ),
        ]
    )
