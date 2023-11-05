# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import math

from serial import Serial
from pyubx2 import UBXReader


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('gps')

        # Create publisher instance
        self.publisher_ = self.create_publisher(NavSatFix, 'fix', 10)

        # Set up publisher callback on a 1 second timer
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Set up message
        self.fix = NavSatFix()
        self.frame_id = 0

        # Set up GPS connection
        self.stream = Serial('/dev/ttyUSB0', 9600, timeout=3)
        self.ubr = UBXReader(self.stream)
        



    def timer_callback(self):

        (raw_data, parsed_data) = self.ubr.read()
        print(parsed_data)

        # <NMEA(GNRMC, time=22:18:38, status=A, lat=52.62063, NS=N, lon=-2.16012, EW=W, spd=37.84, cog=, date=2021-03-05, mv=, mvEW=, posMode=A)>
        # parsed_data.lat
        # parsed_data.lon

        data = self.gps.getPvt()
        lat = data.rlat * 180 / math.pi
        lon = data.rlon * 180 / math.pi

        # Put information into NavSatFlex msg type
        self.fix.header.stamp = self.get_clock().now().to_msg()
        self.fix.header.frame_id = self.frame_id
        self.fix.status.service = 1
        self.fix.latitude = lat
        self.fix.longitude = lon
        self.fix.altitude = data.alt
        self.fix.position_covariance = [0] * 9
        self.fix.position_covariance_type = 0


        self.publisher_.publish(self.fix)

        # msg = String()
        # msg.data = f'GPS Data: Latitude: {lat}, Longitude: {lon}'
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
