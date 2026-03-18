from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav_bringup.launch_utils import bringup_share, load_frames


def generate_launch_description() -> LaunchDescription:
    frames = load_frames()
    twist_mux_params = f"{bringup_share()}/config/core/twist_mux.yaml"
    urdf = f"{get_package_share_directory('marvin_description')}/urdf/marvin.xacro"

    # fmt: off
    robot_description = ParameterValue(
        Command(
            [
                "xacro ", urdf,
                " base_frame_id:=", frames["base_frame"],
                " imu_name:=", frames["imu_frame"].removesuffix("_link"),
                " gps_name:=", frames["gps_frame"].removesuffix("_link"),
            ]
        ), value_type=str,
    )
    # fmt: on

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
            ),
            Node(
                package="state_machine",
                executable="state_machine",
                name="state_machine",
                output="screen",
                remappings=[
                    ("state", "state"),
                    ("state/set_ramp", "state/set_ramp"),
                    ("state/set_recovery", "state/set_recovery"),
                ],
            ),
            Node(
                package="twist_mux",
                executable="twist_mux",
                name="twist_mux",
                output="screen",
                parameters=[
                    twist_mux_params,
                ],
                remappings=[
                    ("cmd_vel_out", "cmd_vel"),
                ],
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[
                    {"port": 8765},
                    {"address": "0.0.0.0"},
                    {"tls": False},
                    {"topic_whitelist": [".*"]},
                    {"param_whitelist": [".*"]},
                    {"service_whitelist": [".*"]},
                    {"send_buffer_limit": 10000000},
                    {"use_sim_time": False},
                    {"num_threads": 4},
                ],
            ),
        ]
    )
