from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Launches gps_coord_pub node from gps_coord_pub package and 
# waypoint_publisher_node from gps_transform package
def generate_launch_description():
    # Arguments
    waypoints_file_launch_arg = DeclareLaunchArgument(
        name='waypoints_file',
        default_value="waypoints.txt",
        description='Name of waypoints file to load'
    )

    facing_north_launch_arg = DeclareLaunchArgument(
        name='facing_north',
        default_value="True",
        description='Whether the map is facing north'
    )

    goal_tolerance_launch_arg = DeclareLaunchArgument(
        name='goal_tolerance',
        default_value="2.0",
        description='Tolerance for reaching goal'
    )


    # GPS Coord Publisher Node
    gps_coord_publisher_node = Node(
        package="gps_coord_pub",
        executable="gps_coord_pub",
        name="gps_coord_pub_node"
    )

    # Waypoint Publisher Node
    waypoint_publisher_node = Node(
        package="gps_transform",
        executable="waypoint_publisher",
        name="waypoint_publisher_node",
        parameters=[{
            "waypoints_file": LaunchConfiguration('waypoints_file'), 
            "facing_north": LaunchConfiguration('facing_north'),
            "goal_tolerance": LaunchConfiguration('goal_tolerance')
        }]
    )

    args = [
        waypoints_file_launch_arg, 
        facing_north_launch_arg,
        goal_tolerance_launch_arg
    ]

    nodes = [
        gps_coord_publisher_node, 
        waypoint_publisher_node
    ]

    # Launch Description
    return LaunchDescription(
        args + nodes
    )  