from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav_bringup.global_config import FRAMES, GPS_ORIGIN


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("nav_bringup")
    imu_params = PathJoinSubstitution([bringup_share, "config", "imu.yaml"])
    gps_params = PathJoinSubstitution([bringup_share, "config", "gps.yaml"])
    sensor_simulator_params = PathJoinSubstitution([bringup_share, "config", "sensor_simulator.yaml"])
    simulation = LaunchConfiguration("simulation")
    map_file = PathJoinSubstitution([bringup_share, "config", "map.json"])

    vectornav = Node(
        package="vectornav",
        executable="vectornav_node",
        name="vectornav",
        output="screen",
        condition=UnlessCondition(simulation),
        parameters=[
            imu_params,
            {"frame_id": FRAMES["imu_frame"]},
            {"map_frame_id": FRAMES["map_frame"]},
        ],
        remappings=[
            ("vectornav/data", "imu/raw"),
        ],
    )

    gps = Node(
        package="gps_publisher",
        executable="gps_publisher",
        name="gps_publisher",
        output="screen",
        condition=UnlessCondition(simulation),
        parameters=[
            gps_params,
            {"gps_frame_id": FRAMES["gps_frame"]},
        ],
        remappings=[
            ("gps", "gps/raw"),
        ],
    )

    sensor_simulator = Node(
        package="sensor_simulator",
        executable="sensor_simulator",
        name="sensor_simulator",
        output="screen",
        condition=IfCondition(simulation),
        parameters=[
            sensor_simulator_params,
            {
                "map_frame_id": FRAMES["map_frame"],
                "base_frame_id": FRAMES["base_frame"],
                "ground_truth_base_frame_id": FRAMES["ground_truth_base_frame"],
                "imu_frame_id": FRAMES["imu_frame"],
                "gps_frame_id": FRAMES["gps_frame"],
                "gps.origin_latitude_deg": GPS_ORIGIN["latitude"],
                "gps.origin_longitude_deg": GPS_ORIGIN["longitude"],
            },
        ],
        remappings=[
            ("gps", "gps/raw"),
            ("enc_vel", "enc_vel/raw"),
            ("imu", "imu/raw"),
            ("odom/ground_truth", "odom/ground_truth"),
        ],
    )

    occupancy_grid_simulator = Node(
        package="occupancy_grid_simulator",
        executable="occupancy_grid_simulator",
        name="occupancy_grid_simulator",
        output="screen",
        condition=IfCondition(simulation),
        parameters=[
            {
                "map_file_path": map_file,
                "map_frame_id": FRAMES["map_frame"],
                "ground_truth_base_frame_id": FRAMES["ground_truth_base_frame"],
            }
        ],
        remappings=[
            ("odom", "odom/ground_truth"),
            ("occupancy_grid", "occ_grid"),
            ("occupancy_grid/ground_truth", "occupancy_grid/ground_truth"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulation",
                default_value="false",
                description="Launch sensor simulator instead of real hardware drivers",
            ),
            vectornav,
            gps,
            sensor_simulator,
            occupancy_grid_simulator,
        ]
    )
