# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterFile

# def generate_launch_description():
#     param_file_path = 'src/nav_stack/cv_grid/config/cv_grid.yaml'

#     return LaunchDescription([
#         Node(
#             package='drivable_area',
#             executable='drivable_area_mqtt',
#             name='drivable_area',
#             output='screen',
#             parameters=[
#                 {'use_sim_time' : True}
#             ],
#             remappings=[
#                 ('/occupancy_grid', "/cv_view")
#             ]
#         )
#     ])


from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    param_file_path = 'src/nav_stack/cv_grid/config/cv_grid.yaml'

    return LaunchDescription([
        Node(
            package='drivable_area',
            executable='drivable_area_test',
            name='drivable_area',
            output='screen',
            parameters=[
                {'use_sim_time' : True}
            ],
            remappings=[
                ('/occupancy_grid', "/cv_view")
            ]
        )
    ])