from __future__ import annotations

import math

from geometry_msgs.msg import Twist, Vector3
from nav_utils.geometry import Point2d, Pose2d
from nav_utils.math import clamp
from rclpy.impl.rcutils_logger import RcutilsLogger

from .path_tracking_config import PurePursuitConfig


class PurePursuitController:
    def __init__(self, config: PurePursuitConfig, logger: RcutilsLogger) -> None:
        self.config = config
        self.logger = logger
        self.path_points: list[Point2d] = []
        self.lookahead_fractional_index: float = 0.0

    def set_path(self, path: list[Point2d]) -> None:
        self.path_points = path
        self.lookahead_fractional_index = 0.0

    def compute_command(self, pose: Pose2d, speed: float) -> Twist | None:
        if not self.path_points:
            return None

        lookahead_distance = self._compute_lookahead_distance(speed)
        if (self.path_points[-1] - pose.point).mag() < lookahead_distance:
            self.logger.info("Reached goal - stopping")
            self.path_points = []
            return Twist()

        lookahead_point = self._find_lookahead_point(pose, lookahead_distance) or self._find_projected_lookahead(
            pose, lookahead_distance
        )
        if lookahead_point is None:
            self.logger.warn("No valid lookahead point found - stopping")
            return Twist()

        curvature = 2 * lookahead_point.y / (lookahead_point.x**2 + lookahead_point.y**2)
        linear = lookahead_distance * self.config.linear_speed_gain
        angular = linear * curvature
        scale = min(1.0, self.config.max_angular_speed_radps / abs(angular)) if angular != 0.0 else 1.0
        return Twist(linear=Vector3(x=linear * scale), angular=Vector3(z=angular * scale))

    def _compute_lookahead_distance(self, speed: float) -> float:
        lookahead = self.config.base_lookahead_distance_m + self.config.lookahead_speed_gain * speed
        return clamp(lookahead, min=self.config.min_lookahead_distance_m, max=self.config.max_lookahead_distance_m)

    def _find_lookahead_point(self, pose: Pose2d, lookahead_distance: float) -> Point2d | None:
        robot = pose.point
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
                    return pose.world_to_local(start.lerp(end, t))

        return None

    def _find_projected_lookahead(self, pose: Pose2d, lookahead_distance: float) -> Point2d | None:
        if len(self.path_points) < 2:
            return None

        def project_robot_onto_path() -> float | None:
            """Return the fractional index of the closest point on the path to the robot, or None."""
            robot = pose.point

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
        self.logger.warn(
            f"Lookahead miss, projection fallback aiming at fractional index {self.lookahead_fractional_index:.2f}"
        )
        return pose.world_to_local(target)
