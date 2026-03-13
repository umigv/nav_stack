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

    imu_params = PathJoinSubstitution([bringup_share, "config", "sensors", "imu.yaml"])
    gps_params = PathJoinSubstitution([bringup_share, "config", "sensors", "gps.yaml"])
    sensor_simulator_params = PathJoinSubstitution([bringup_share, "config", "sensors", "sensor_simulator.yaml"])
    simulation = LaunchConfiguration("simulation")
    map_file = PathJoinSubstitution([bringup_share, "fields", field, "map.json"])

    return [
        Node(
            package="vectornav",
            executable="vectornav_node",
            name="vectornav",
            output="screen",
            condition=UnlessCondition(simulation),
            parameters=[
                imu_params,
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
            condition=UnlessCondition(simulation),
            parameters=[
                gps_params,
                {"gps_frame_id": frames["gps_frame"]},
            ],
            remappings=[
                ("gps", "gps/raw"),
            ],
        ),
        Node(
            package="sensor_simulator",
            executable="sensor_simulator",
            name="sensor_simulator",
            output="screen",
            condition=IfCondition(simulation),
            parameters=[
                sensor_simulator_params,
                {
                    "map_frame_id": frames["map_frame"],
                    "base_frame_id": frames["base_frame"],
                    "ground_truth_base_frame_id": frames["ground_truth_base_frame"],
                    "imu_frame_id": frames["imu_frame"],
                    "gps_frame_id": frames["gps_frame"],
                    "gps.origin_latitude_deg": gps_file["datum"]["latitude"],
                    "gps.origin_longitude_deg": gps_file["datum"]["longitude"],
                },
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
            condition=IfCondition(simulation),
            parameters=[
                {
                    "map_file_path": map_file,
                    "map_frame_id": frames["map_frame"],
                    "ground_truth_base_frame_id": frames["ground_truth_base_frame"],
                }
            ],
            remappings=[
                ("odom", "odom/ground_truth"),
                ("occupancy_grid", "occ_grid"),
                ("occupancy_grid/ground_truth", "occupancy_grid/ground_truth"),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulation",
                default_value="false",
                description="Launch sensor simulator instead of real hardware drivers",
            ),
            DeclareLaunchArgument(
                "field",
                default_value="default",
                description="Field profile in fields/ to load map and GPS datum from",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
