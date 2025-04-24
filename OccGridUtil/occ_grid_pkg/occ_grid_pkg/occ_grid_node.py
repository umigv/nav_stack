import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/test_occ', 10)

        # Load occupancy grid once
        self.load_occupancy_grid()

        # Set up a timer to publish periodically
        self.timer = self.create_timer(1.0, self.publish_occupancy_grid)  # Publish every 1 second

    def load_occupancy_grid(self):
        print("Startign OCC PUB")
        file_path = '/home/umarv/ros2_ws/src/nav_stack/OccGridUtil/occ_grid_pkg/occ_grid_pkg/testout4.txt'
        print("Reading file")
        with open(file_path, 'r') as file:
            lines = file.readlines()

        cleaned_data = [line.replace('[', '').replace(']', '').strip() for line in lines]
        
        matrix_ones = np.array([list(map(int, line.split())) for line in cleaned_data])

        for row in matrix_ones: 
            for i in row: 
                if i != 0 and i != 1:
                    self.get_logger().info('did not see something right ')
                    print(i)

                    
        flipped = np.where(matrix_ones == 1, 0, 1)
        matrix = flipped * 100
        matrix = np.rot90(matrix, k=-1)  # negative k rotates clockwise
        
        self.grid_msg = OccupancyGrid()
        self.grid_msg.header.frame_id = 'map'
        self.grid_msg.info.resolution = 1.0  # Adjust resolution as needed
        self.grid_msg.info.width = matrix.shape[1]
        self.grid_msg.info.height = matrix.shape[0]
        self.grid_msg.info.origin.position.x = 0.0
        self.grid_msg.info.origin.position.y = 0.0
        self.grid_msg.info.origin.position.z = 0.0
        self.grid_msg.info.origin.orientation.w = 1.0
        self.grid_msg.data = matrix.flatten().tolist()

    def publish_occupancy_grid(self):
        self.grid_msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
        self.publisher_.publish(self.grid_msg)
        # self.get_logger().info(self.grid_msg)
        # self.get_logger().info('P1ublished Occupancy Grid')

def main(args=None):
    print("MAIN")
    rclpy.init(args=args)
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
