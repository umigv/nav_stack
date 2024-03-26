#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from robot_navigator import BasicNavigator, NavigationResult

'''
Basic navigation demo to go to pose.
'''
def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set our demo's initial pose
    #  this is for world (-2.01, -0.57) to (1.76, -0.01) 
    # Got a start: -1.94, -0.47,0.01, and a goal: 1.81, 0.07, 0.00
#[component_container_isolated-6] [INFO] [1705270688.368715988] [planner_server]: Got a start: -0.00, -0.00,0.73,0.68, and a goal: 0.00, 0.00, 0.72,0.70
    # this is for house 5.050 0.486  11.50, 1.93
    # 5.050 0.486 3.142
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()

    # initial_pose.pose.position.x = 5.05
    # initial_pose.pose.position.y =  -.03
    # initial_pose.pose.orientation.z = -1.0
    # initial_pose.pose.orientation.w = 0.0
  
  
    initial_pose.pose.position.x = -2.01
    initial_pose.pose.position.y =  -0.59
    initial_pose.pose.position.z =  0.0

    # initial_pose.pose.orientation.z = .73
    # initial_pose.pose.orientation.w = .68
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0


    # initial_pose.pose.position.x = -2.01
    # initial_pose.pose.position.y = -0.57
    # initial_pose.pose.orientation.z = 1.0
    # initial_pose.pose.orientation.w = 1.0



    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
        #  this is for turle wolrd (-2.01, -0.57) to (1.76, -0.01)
# 5.050 0.486 3.142
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose.pose.position.x = 11.50
    # goal_pose.pose.position.y = 1.93
    # goal_pose.pose.orientation.w = 1.0

    # goal_pose.pose.position.x = -1.81
    # goal_pose.pose.position.y = 0.07
    # goal_pose.pose.position.z = 0.00
    # goal_pose.pose.orientation.x = 0.00
    # goal_pose.pose.orientation.y= 0.00
    # goal_pose.pose.orientation.z = .72
    # goal_pose.pose.orientation.w = .70

    goal_pose.pose.position.x = 1.76
    goal_pose.pose.position.y = -0.01
    goal_pose.pose.orientation.w = 1.0

         # Got a start: -1.94, -0.47,0.01, and a goal: 1.81, 0.07, 0.00
#[component_container_isolated-6] [INFO] [1705270688.368715988] [planner_server]: Got a start: -0.00, -0.00,0.73,0.68, and a goal: 0.00, 0.00, 0.72,0.70
    # this is for house 5.050 0.486  11.50, 1.93


    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)
    start_time = time.time()
    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isNavComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            # print('Estimated time of arrival: ' + '{0:.0f}'.format(
            #       Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
            #       + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                print('TIME OUT ERROR, canceling nav')
                navigator.cancelNav()

          

    # Do something depending on the return code
    print("TIME:  --- %s seconds ---" % (time.time() - start_time))
    result = navigator.getResult()
    if result == NavigationResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == NavigationResult.CANCELED:
        print('Goal was canceled!')
    elif result == NavigationResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()