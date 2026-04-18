from __future__ import annotations

import math

import nav_utils.config
import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry, Path
from nav_utils.geometry import Point2d, Pose2d
from nav_utils.math import clamp
from rclpy.node import Node

from .path_tracking_config import PathTrackingConfig


class PathTracking(Node):
    def __init__(self) -> None:
        super().__init__("path_tracking")

        self.config: PathTrackingConfig = nav_utils.config.load(self, PathTrackingConfig)

        self.path_points: list[Point2d] = []
        self.lookahead_fractional_index: float = 0.0
        self.pose: Pose2d | None = None
        self.current_speed: float = 0.0

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(Path, "path", self.path_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, "nav_cmd_vel", 10)

        self.create_timer(self.config.control_period_s, self.control_loop)

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.odom_frame_id:
            self.get_logger().warn(f"Dropping odometry: frame '{msg.header.frame_id}' != '{self.config.odom_frame_id}'")
            return
        if msg.child_frame_id != self.config.base_frame_id:
            self.get_logger().warn(
                f"Dropping odometry: child_frame_id '{msg.child_frame_id}' != '{self.config.base_frame_id}'"
            )
            return

        self.pose = Pose2d.from_ros(msg.pose.pose)
        self.current_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def path_callback(self, path_msg: Path) -> None:
        if path_msg.header.frame_id != self.config.odom_frame_id:
            self.get_logger().warn(
                f"Dropping path: frame '{path_msg.header.frame_id}' != '{self.config.odom_frame_id}'"
            )
            return

        self.get_logger().info("Received a new path from subscription.")
        self.path_points = [Point2d(x=p.pose.position.x, y=p.pose.position.y) for p in path_msg.poses]
        self.lookahead_fractional_index = 0.0

    def compute_lookahead_distance(self) -> float:
        lookahead = self.config.base_lookahead_distance_m + self.config.lookahead_speed_gain * self.current_speed
        return clamp(lookahead, min=self.config.min_lookahead_distance_m, max=self.config.max_lookahead_distance_m)

    def find_lookahead_point(self, lookahead_distance: float) -> Point2d | None:
        assert self.pose is not None
        robot = self.pose.point
        start_segment = int(self.lookahead_fractional_index)

        for i in range(start_segment, len(self.path_points) - 1):
            start = self.path_points[i]
            end = self.path_points[i + 1]
            d = end - start
            f = start - robot

            a = d.dot(d)
            b = 2.0 * f.dot(d)
            c = f.dot(f) - lookahead_distance * lookahead_distance

            discriminant = b * b - 4.0 * a * c
            if discriminant < 0.0:
                continue

            t2 = (-b + math.sqrt(discriminant)) / (2.0 * a)
            t1 = (-b - math.sqrt(discriminant)) / (2.0 * a)

            # Check t2 (far intersection) first to prefer the forward-most point.
            for t in (t2, t1):
                if 0.0 <= t <= 1.0 and i + t >= self.lookahead_fractional_index:
                    self.lookahead_fractional_index = i + t
                    return self.pose.world_to_local(start.lerp(end, t))

        return None

    def find_projected_lookahead(self, lookahead_distance: float) -> Point2d | None:
        assert self.pose is not None
        if len(self.path_points) < 2:
            return None

        def project_robot_onto_path() -> float | None:
            """Return the fractional index of the closest point on the path to the robot, or None."""
            assert self.pose is not None
            robot = self.pose.point

            def projection_onto_segment(i: int) -> tuple[float, float]:
                start, end = self.path_points[i], self.path_points[i + 1]
                segment = end - start
                t = clamp((robot - start).dot(segment) / segment.dot(segment), min=0.0, max=1.0)
                perpendicular_offset = start.lerp(end, t) - robot
                return i + t, perpendicular_offset.mag()

            search_range = range(int(self.lookahead_fractional_index), len(self.path_points) - 1)
            candidates = [projection_onto_segment(i) for i in search_range]
            return min(candidates, key=lambda p: p[1])[0] if candidates else None

        def walk_along_path(start_fractional_index: float, distance: float) -> tuple[Point2d, float]:
            """Walk forward along the path by `distance` meters, or to the end if shorter."""
            segment_index = int(start_fractional_index)
            segment_t = start_fractional_index - segment_index
            remaining = distance
            while segment_index < len(self.path_points) - 1:
                start, end = self.path_points[segment_index], self.path_points[segment_index + 1]
                length = start.distance(end)
                available = length * (1.0 - segment_t)
                if available >= remaining:
                    target_t = segment_t + remaining / length
                    return start.lerp(end, target_t), segment_index + target_t
                remaining -= available
                segment_index += 1
                segment_t = 0.0
            return self.path_points[-1], float(len(self.path_points) - 1)

        projection_index = project_robot_onto_path()
        if projection_index is None:
            return None

        target, _ = walk_along_path(projection_index, lookahead_distance)
        self.lookahead_fractional_index = projection_index
        self.get_logger().warn(
            f"Lookahead miss, projection fallback aiming at fractional index {self.lookahead_fractional_index:.2f}"
        )
        return self.pose.world_to_local(target)

    def control_loop(self) -> None:
        if self.pose is None or not self.path_points:
            return

        lookahead_distance = self.compute_lookahead_distance()
        if (self.path_points[-1] - self.pose.point).mag() < lookahead_distance:
            self.get_logger().info("Reached goal - stopping")
            self.path_points = []
            self.cmd_vel_publisher.publish(Twist())
            return

        lookahead_point = self.find_lookahead_point(lookahead_distance) or self.find_projected_lookahead(lookahead_distance)
        if lookahead_point is None:
            self.get_logger().warn("No valid lookahead point found - stopping")
            self.cmd_vel_publisher.publish(Twist())
            return

        curvature = 2 * lookahead_point.y / (lookahead_point.x**2 + lookahead_point.y**2)
        linear = lookahead_distance * self.config.linear_speed_gain
        angular = linear * curvature
        scale = min(1.0, self.config.max_angular_speed_radps / abs(angular)) if angular != 0.0 else 1.0
        self.cmd_vel_publisher.publish(Twist(linear=Vector3(x=linear * scale), angular=Vector3(z=angular * scale)))


def main() -> None:
    rclpy.init()
    node = PathTracking()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
