from typing_extensions import assert_never

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
    share = bringup_share()

    imu_node = Node(
        package="vectornav",
        executable="vectornav_node",
        name="vectornav",
        output="screen",
        parameters=[
            f"{share}/config/sensors/imu.yaml",
            {"frame_id": frames["imu_frame"]},
            {"map_frame_id": frames["map_frame"]},
        ],
        remappings=[
            ("vectornav/data", "imu/raw"),
        ],
    )

    gps_node = Node(
        package="gps_publisher",
        executable="gps_publisher",
        name="gps_publisher",
        output="screen",
        parameters=[
            f"{share}/config/sensors/gps.yaml",
            {"gps_frame_id": frames["gps_frame"]},
        ],
        remappings=[
            ("gps", "gps/raw"),
        ],
    )

    sensor_simulator_node = Node(
        package="sensor_simulator",
        executable="sensor_simulator",
        name="sensor_simulator",
        output="screen",
        parameters=[
            f"{share}/config/sensors/sensor_simulator.yaml",
            {"map_frame_id": frames["map_frame"]},
            {"base_frame_id": frames["base_frame"]},
            {"ground_truth_base_frame_id": frames["ground_truth_base_frame"]},
            {"imu_frame_id": frames["imu_frame"]},
            {"gps_frame_id": frames["gps_frame"]},
            {"gps.origin_latitude_deg": gps_file["datum"]["latitude"]},
            {"gps.origin_longitude_deg": gps_file["datum"]["longitude"]},
        ],
        remappings=[
            ("gps", "gps/raw"),
            ("enc_vel", "enc_vel/raw"),
            ("imu", "imu/raw"),
            ("odom/ground_truth", "odom/ground_truth"),
        ],
    )

    occupancy_grid_simulator_node = Node(
        package="occupancy_grid_simulator",
        executable="occupancy_grid_simulator",
        name="occupancy_grid_simulator",
        output="screen",
        parameters=[
            {"map_file_path": f"{share}/courses/{course}/map.json"},
            {"map_frame_id": frames["map_frame"]},
            {"base_frame_id": frames["base_frame"]},
            {"ground_truth_base_frame_id": frames["ground_truth_base_frame"]},
        ],
        remappings=[
            ("odom", "odom/ground_truth"),
            ("occupancy_grid", "occupancy_grid/raw"),
            ("occupancy_grid/ground_truth", "occupancy_grid/ground_truth"),
        ],
    )

    match mode:
        case "autonav":
            return [imu_node, gps_node]
        case "autonav_sim":
            return [sensor_simulator_node, occupancy_grid_simulator_node]
        case "self_drive":
            return [imu_node]
        case "self_drive_sim":
            return [sensor_simulator_node, occupancy_grid_simulator_node]
        case "nav_test":
            return []
        case _:
            assert_never(mode)  # type: ignore[unreachable]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                choices=MODES,
                description=format_mode_description(
                    {
                        "autonav": "IMU + GPS",
                        "autonav_sim": "sensor simulator + occupancy grid simulator",
                        "self_drive": "IMU only",
                        "self_drive_sim": "sensor simulator + occupancy grid simulator",
                        "nav_test": "no sensors",
                    }
                ),
            ),
            DeclareLaunchArgument(
                "course",
                default_value="default",
                description="Course profile in courses/ to load map and GPS datum from (required for autonav_sim, self_drive_sim)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
