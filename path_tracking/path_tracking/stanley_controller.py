from __future__ import annotations

import math

from geometry_msgs.msg import Twist, Vector3
from nav_utils.geometry import Point2d, Pose2d, Rotation2d
from nav_utils.math import clamp
from rclpy.impl.rcutils_logger import RcutilsLogger

from .path_tracking_config import StanleyConfig


class StanleyController:
    def __init__(self, config: StanleyConfig, logger: RcutilsLogger) -> None:
        self.config = config
        self.logger = logger
        self.path_points: list[Point2d] = []
        self.projection_hint: int = 0

    def set_path(self, path: list[Point2d]) -> None:
        self.path_points = path
        self.projection_hint = 0

    def compute_command(self, pose: Pose2d, speed: float) -> Twist | None:
        if len(self.path_points) < 2:
            return None

        front = pose.point + Point2d(pose.rotation.cos, pose.rotation.sin) * self.config.front_offset_m

        if (self.path_points[-1] - front).mag() < self.config.goal_tolerance_m:
            self.logger.info("Reached goal - stopping")
            self.path_points = []
            return Twist()

        projection = self._project_front_onto_path(front)
        if projection is None:
            self.logger.warn("Stanley: no valid projection segment - stopping")
            return Twist()
        segment_index, projected_point = projection

        start = self.path_points[segment_index]
        end = self.path_points[segment_index + 1]
        segment = end - start
        path_heading = Rotation2d(math.atan2(segment.y, segment.x))
        heading_error = (path_heading - pose.rotation).angle

        # Signed cross-track error: positive when the path lies to the left of the front point,
        # so the atan2 term below commands a positive (leftward) steering correction.
        segment_length = segment.mag()
        left_normal = Point2d(-segment.y / segment_length, segment.x / segment_length)
        cte = (projected_point - front).dot(left_normal)

        # Cap speed by the lateral-accel budget: v^2 / r <= a_lat, with r = arclength / heading_change
        # over the lookahead horizon. Slows the robot *before* it reaches an upcoming corner.
        heading_change, arclength = self._lookahead_heading_change(segment_index, projected_point)
        if heading_change > 1e-3 and arclength > 1e-6:
            v_max_curvature = math.sqrt(self.config.max_lateral_accel_mps2 * arclength / heading_change)
            v_ref = min(self.config.target_speed_mps, v_max_curvature)
        else:
            v_ref = self.config.target_speed_mps

        delta = heading_error + math.atan2(self.config.cross_track_gain * cte, v_ref)
        delta = clamp(delta, min=-self.config.max_steer_rad, max=self.config.max_steer_rad)

        # Diff-drive: reuse front_offset as the virtual wheelbase.
        omega = v_ref * math.tan(delta) / self.config.front_offset_m
        omega = clamp(omega, min=-self.config.max_angular_speed_radps, max=self.config.max_angular_speed_radps)

        return Twist(linear=Vector3(x=v_ref), angular=Vector3(z=omega))

    def _lookahead_heading_change(self, segment_index: int, projected_point: Point2d) -> tuple[float, float]:
        """Accumulate absolute path heading change and arclength ahead of `projected_point`.

        Walks forward from the projection until either `curvature_lookahead_m` of arclength has
        been consumed or the path ends. Heading change accrues at each vertex crossed.
        """
        remaining = self.config.curvature_lookahead_m
        arclength = 0.0
        heading_change = 0.0

        # Consume the remainder of the current segment first; if the horizon ends inside it, no vertex is reached.
        end_of_current = self.path_points[segment_index + 1]
        step = (end_of_current - projected_point).mag()
        if step >= remaining:
            return 0.0, remaining
        arclength += step
        remaining -= step

        prev_segment = self.path_points[segment_index + 1] - self.path_points[segment_index]
        i = segment_index + 1
        while i < len(self.path_points) - 1 and remaining > 0.0:
            next_segment = self.path_points[i + 1] - self.path_points[i]
            seg_len = next_segment.mag()
            if seg_len == 0.0:
                i += 1
                continue
            prev_heading = Rotation2d(math.atan2(prev_segment.y, prev_segment.x))
            next_heading = Rotation2d(math.atan2(next_segment.y, next_segment.x))
            heading_change += abs((next_heading - prev_heading).angle)

            step = min(seg_len, remaining)
            arclength += step
            remaining -= step
            prev_segment = next_segment
            i += 1

        return heading_change, arclength

    def _project_front_onto_path(self, front: Point2d) -> tuple[int, Point2d] | None:
        """Find the segment whose closest point to `front` is nearest, starting from the projection hint."""
        best: tuple[int, Point2d, float] | None = None
        for i in range(self.projection_hint, len(self.path_points) - 1):
            start = self.path_points[i]
            end = self.path_points[i + 1]
            segment = end - start
            seg_len_sq = segment.dot(segment)
            if seg_len_sq == 0.0:
                continue
            t = clamp((front - start).dot(segment) / seg_len_sq, min=0.0, max=1.0)
            projected = start.lerp(end, t)
            distance = (projected - front).mag()
            # Prefer the later segment on ties so the projection commits to the next segment at a vertex.
            if best is None or distance <= best[2]:
                best = (i, projected, distance)

        if best is None:
            return None
        self.projection_hint = best[0]
        return best[0], best[1]
