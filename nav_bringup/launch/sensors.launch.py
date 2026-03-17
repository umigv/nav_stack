from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav_bringup.launch_utils import bringup_share, load_frames, load_gps_file


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    frames = load_frames()
    mode = LaunchConfiguration("mode").perform(context)

    if mode == "nav_test":
        return []
    elif mode.endswith("_sim"):
        course = LaunchConfiguration("course").perform(context)
        gps_file = load_gps_file(course)

        return [
            Node(
                package="sensor_simulator",
                executable="sensor_simulator",
                name="sensor_simulator",
                output="screen",
                parameters=[
                    f"{bringup_share()}/config/sensors/sensor_simulator.yaml",
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
            ),
            Node(
                package="occupancy_grid_simulator",
                executable="occupancy_grid_simulator",
                name="occupancy_grid_simulator",
                output="screen",
                parameters=[
                    {"map_file_path": f"{bringup_share()}/courses/{course}/map.json"},
                    {"map_frame_id": frames["map_frame"]},
                    {"base_frame_id": frames["base_frame"]},
                    {"ground_truth_base_frame_id": frames["ground_truth_base_frame"]},
                ],
                remappings=[
                    ("odom", "odom/ground_truth"),
                    ("occupancy_grid", "occupancy_grid/raw"),
                    ("occupancy_grid/ground_truth", "occupancy_grid/ground_truth"),
                ],
            ),
        ]
    else:
        return [
            Node(
                package="vectornav",
                executable="vectornav_node",
                name="vectornav",
                output="screen",
                parameters=[
                    f"{bringup_share()}/config/sensors/imu.yaml",
                    {"frame_id": frames["imu_frame"]},
                    {"map_frame_id": frames["map_frame"]},
                ],
                remappings=[
                    ("vectornav/data", "imu/raw"),
                ],
            ),
            Node(
                package="gps_publisher",
                executable="gps_publisher",
                name="gps_publisher",
                output="screen",
                parameters=[
                    f"{bringup_share()}/config/sensors/gps.yaml",
                    {"gps_frame_id": frames["gps_frame"]},
                ],
                remappings=[
                    ("gps", "gps/raw"),
                ],
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
                description="Course profile in courses/ to load map and GPS datum from",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
