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
    nav = BasicNavigator()

    nav.waitUntilNav2Active() 

    print('CANCELING THE NAV CANCELING THE NAV')
    print('CANCELING THE NAV CANCELING THE NAV')
    button_pressed = True
    if(button_pressed):
        nav.cancelNav()
    print("SAH DUDE BYe")
    


if __name__ == '__main__':
    main()


