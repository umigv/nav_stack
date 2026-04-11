from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav_bringup.launch_utils import bringup_share, load_frames


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    course = LaunchConfiguration("course").perform(context)
    share = bringup_share()

    gps_driver_node = Node(
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

    gps_origin_node = Node(
        package="gps_origin_calculator",
        executable="gps_origin_calculator",
        output="screen",
        parameters=[{"output_file": f"{share}/courses/{course}/gps.json"}],
        remappings=[("gps", "gps/raw")],
    )

    shutdown_on_completion = RegisterEventHandler(
        OnProcessExit(
            target_action=gps_origin_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return [gps_driver_node, gps_origin_node, shutdown_on_completion]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "course",
                default_value="default",
                description="Course profile in courses/ to write the computed GPS datum into.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
