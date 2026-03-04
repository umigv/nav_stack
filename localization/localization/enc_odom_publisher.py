import nav_utils.config
import rclpy
from geometry_msgs.msg import (
    Pose,
    PoseWithCovariance,
    Transform,
    TransformStamped,
    Twist,
    TwistWithCovariance,
    TwistWithCovarianceStamped,
    Vector3,
)
from nav_msgs.msg import Odometry
from nav_utils.geometry import Point2d, Rotation2d
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")

        self.config: EncOdomPublisherConfig = nav_utils.config.load(self, EncOdomPublisherConfig)

        self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self.enc_vel_callback, 10)

        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.position = Point2d(x=0.0, y=0.0)
        self.rotation = Rotation2d(angle=0.0)
        self.prev_time: Time | None = None

        self.pose_covariance = [0.0] * 36
        self.pose_covariance[0] = self.config.pose_x_variance_m2
        self.pose_covariance[7] = self.config.pose_y_variance_m2
        self.pose_covariance[35] = self.config.pose_yaw_variance_rad2

    def enc_vel_callback(self, msg: TwistWithCovarianceStamped) -> None:
        if msg.header.frame_id != self.config.base_frame_id:
            self.get_logger().warn(f"Dropping enc_vel: frame '{msg.header.frame_id}' != '{self.config.base_frame_id}'")
            return

        current_time = Time.from_msg(msg.header.stamp)
        try:
            if self.prev_time is None:
                return

            dt = (current_time - self.prev_time).nanoseconds * 1e-9
            if dt <= 0.0 or dt > self.config.max_dt_s:
                return

            linear_vel = msg.twist.twist.linear.x
            angular_vel = msg.twist.twist.angular.z

            self.position += Point2d(x=linear_vel * dt, y=0.0).rotate_by(self.rotation)
            self.rotation = Rotation2d(self.rotation.angle + angular_vel * dt)

            stamp = current_time.to_msg()

            self.tf_broadcaster.sendTransform(
                TransformStamped(
                    header=Header(stamp=stamp, frame_id=self.config.odom_frame_id),
                    child_frame_id=self.config.base_frame_id,
                    transform=Transform(
                        translation=Vector3(x=self.position.x, y=self.position.y, z=0.0),
                        rotation=self.rotation.to_ros(),
                    ),
                )
            )

            self.odom_publisher.publish(
                Odometry(
                    header=Header(stamp=stamp, frame_id=self.config.odom_frame_id),
                    child_frame_id=self.config.base_frame_id,
                    pose=PoseWithCovariance(
                        pose=Pose(
                            position=self.position.to_ros(),
                            orientation=self.rotation.to_ros(),
                        ),
                        covariance=self.pose_covariance,
                    ),
                    twist=TwistWithCovariance(
                        twist=Twist(
                            linear=Vector3(x=linear_vel),
                            angular=Vector3(z=angular_vel),
                        ),
                        covariance=msg.twist.covariance,
                    ),
                )
            )
        finally:
            self.prev_time = current_time


def main() -> None:
    rclpy.init()
    node = EncOdomPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
