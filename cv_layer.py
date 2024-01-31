import rclpy
from rclpy.node import Node
from nav2_costmap_2d.layer import CostmapLayer

class SensorsMapSubscriber(Node):

    def __init__(self):
        super().__init__('sensors_map_subscriber') # TODO: FIgure out the real topic
        self.subscription = self.create_subscription(
            String,
            'goofy_ahh_topic',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info('Received: %s' % msg.data)

class CVMapSubscriber(Node):

    def __init__(self):
        super().__init__('cv_map_subscriber') # TODO: Figure out the real topic
        self.subscription = self.create_subscription(
            String,
            'goofy_ahh_topic',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info('Received: %s' % msg.data)

class LayerCV(CostmapLayer):

    def __init__(self):
        super().__init__()

    def on_initialize(self):
        # Create a subscriber to the sensors map topic
        self.sensors_map_subscriber = SensorsMapSubscriber()
        # Create a subscriber to the CV map topic
        self.cv_map_subscriber = CVMapSubscriber()
        # Initialize the costmap layer
        self.initialize('cv_layer', 'master_grid', 'cv_layer', False)

    def update_bounds(self, robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y):
        # Update costmap bounds based on the robot's position and orientation
        # Set the values of min_x, min_y, max_x, and max_y accordingly
        pass

    def update_costs(self, master_grid, min_i, min_j, max_i, max_j):
        # Update the cost values in the specified region of the costmap
        # You can use master_grid to access the underlying costmap and modify its values
        pass


def main():
    rclpy.init()
    node = LayerCV()

    try:
        node.on_initialize()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
