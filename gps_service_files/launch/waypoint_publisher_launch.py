from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Launches gps_coord_pub node from gps_coord_pub package and 
# waypoint_publisher_node from gps_transform package

def generate_launch_description():
    # waypoint_publisher configs
    waypoints_file_path_value = LaunchConfiguration('waypoints_file_path')
    facing_north_value = LaunchConfiguration('facing_north')

    # waypoint_publisher args
    waypoints_file_path_arg = DeclareLaunchArgument('waypoints_file_path', 
        default_value = '/home/johnrose/arv_ws/src/nav_stack/gps_service_files/config/waypoints.txt')
    facing_north_arg = DeclareLaunchArgument('facing_north', 
        default_value = 'True')


   # headless_launch_arg = DeclareLaunchArgument(
    #    name='headless',
     #   default_value='False',
      #  description='Show RViz and Gazebo'
    #)

    args = [
        waypoints_file_path_arg, 
        facing_north_arg
    ]

    gps_coord_pub_node = Node(
        package="gps_coord_pub",
        executable="gps_coord_pub",
        name="gps_coord_pub_node"
    )

    waypoint_publisher_node = Node(
        package="gps_transform",
        executable="waypoint_publisher",
        name="waypoint_publisher_node",
        parameters=[{
            "waypoints_file_path": waypoints_file_path_value, 
            "facing_north": facing_north_value
        }]
    )

    nodes = [
        gps_coord_pub_node, 
        waypoint_publisher_node
    ]

    return LaunchDescription(args + nodes)