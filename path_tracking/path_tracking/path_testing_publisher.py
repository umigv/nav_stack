import time

import rclpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from rclpy.node import Node
from rosidl_runtime_py.set_message import set_message_fields


class PathTestingNode(Node):
    def __init__(self):
        super().__init__("path_testing_publisher")
        self.publisher = self.create_publisher(Path, "/path", 10)

        self.odom_subscriber = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.pose = None

        self.set_paths()
        self.current_path_idx = 0
        self.timer = self.create_timer(0.1, self.publish_paths)

    # publish the paths in order
    def publish_paths(self):
        if self.current_path_idx > len(self.paths):
            # no paths to publish
            return

        if self.pose is None:
            self.get_logger().info("Waiting for odom message...", throttle_duration_sec=0.1)
            return

        self.publisher.publish(self.paths[self.current_path_idx])
        if self.check_path_completion(self.paths[self.current_path_idx].poses[-1].pose):
            self.get_logger().info(f"Path {self.current_path_idx} completed!")
            self.current_path_idx += 1

    # get current pose
    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose

    # check if the robot has completed the path
    def check_path_completion(self, target_pose: Pose) -> bool:
        THRESHOLD = 0.1  # max threshold for pure pursuit
        dx = self.pose.position.x - target_pose.position.x
        dy = self.pose.position.y - target_pose.position.y
        dz = self.pose.position.z - target_pose.position.z
        return dx * dx + dy * dy + dz * dz < THRESHOLD * THRESHOLD

    # generate paths at https://umigv.github.io/path-tracer/
    def set_paths(self):
        self.paths = [Path(), Path(), Path()]

        self.paths[0].poses = [
            PoseStamped(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id="odom"),
                pose=Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
            ), 
            PoseStamped(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id="odom"),
                pose=Pose(position=Point(x=1.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
            )
        ]

        self.paths[1].poses = [
            PoseStamped(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id="odom"),
                pose=Pose(position=Point(x=1.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
            ), 
            PoseStamped(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id="odom"),
                pose=Pose(position=Point(x=2.0, y=1.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
            )
        ]

        self.paths[2].poses = [
            PoseStamped(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id="odom"),
                pose=Pose(position=Point(x=2.0, y=1.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
            ),
            PoseStamped(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id="odom"),
                pose=Pose(position=Point(x=2.0, y=2.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
            )
        ]


def main(args=None):
    rclpy.init(args=args)
    node = PathTestingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
