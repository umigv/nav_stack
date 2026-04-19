import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Float64


class LinearSpeed(Node):
    def __init__(self) -> None:
        super().__init__("linear_speed")

        self.create_subscription(TwistWithCovarianceStamped, "enc_vel/raw", self.publish_linear_speed, 10)

        self.linear_speed_publisher = self.create_publisher(Float64, "linear_speed", 10)

    def publish_linear_speed(self, msg: TwistWithCovarianceStamped) -> None:

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        speed = (vx**2 + vy**2 + vz**2) ** 0.5

        self.linear_speed_publisher.publish(Float64(data=speed))


def main() -> None:
    rclpy.init()
    node = LinearSpeed()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
