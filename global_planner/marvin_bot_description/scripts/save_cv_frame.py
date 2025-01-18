#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt

class OccupancyGridSaver(Node):
    def __init__(self):
        super().__init__('occupancy_grid_saver')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/cv_view',
            self.occupancy_grid_callback,
            10
        )
        self.saved = False

    def occupancy_grid_callback(self, msg):
        if not self.saved:
            self.saved = True
            self.get_logger().info('Received an OccupancyGrid message. Saving data...')

            # Save data to a file
            with open('occupancy_grid_data.txt', 'w') as f:
                f.write(str(msg))
            self.get_logger().info('Data saved to occupancy_grid_data.txt')

            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))

            plt.imshow(data, cmap='gray', origin='lower')
            plt.title('Occupancy Grid')
            plt.colorbar(label='Occupancy Value')
            plt.show()

            self.get_logger().info('Shutting down node after saving and displaying the data.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridSaver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()