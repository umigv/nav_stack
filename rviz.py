import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='occupub',
            executable='your_node_executable',
            name='your_node_name',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/path/to/your/rviz/config/file.rviz']  # Specify the path to your RViz configuration file
        )
    ])
