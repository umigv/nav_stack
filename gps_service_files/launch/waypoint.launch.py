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

    face_north_launch_arg = DeclareLaunchArgument(
        name='face_north',
        default_value="True",
        description='Whether the map is facing north'
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
            "face_north": LaunchConfiguration('face_north')
        }]
    )

    # Launch Description
    return LaunchDescription([
        waypoints_file_launch_arg,
        face_north_launch_arg,
        gps_coord_publisher_node, 
        waypoint_publisher_node
    ])