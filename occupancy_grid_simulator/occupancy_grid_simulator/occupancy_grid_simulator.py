from __future__ import annotations

import json
import math

import nav_utils.config
import nav_utils.qos
import numpy as np
import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from nav_utils.geometry import Point2d, Pose2d, Rotation2d
from rclpy.node import Node
from std_msgs.msg import Header

from .occupancy_grid_simulator_config import OccupancyGridSimulatorConfig


class OccupancyGridSimulator(Node):
    OCCUPIED = 100
    FREE = 0

    def __init__(self) -> None:
        super().__init__("occupancy_grid_simulator")
        self.config: OccupancyGridSimulatorConfig = nav_utils.config.load(self, OccupancyGridSimulatorConfig)

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        self.occupancy_grid_publisher = self.create_publisher(OccupancyGrid, "occupancy_grid", 10)
        self.ground_truth_publisher = self.create_publisher(
            OccupancyGrid,
            "occupancy_grid/ground_truth",
            qos_profile=nav_utils.qos.LATCHED,
        )

        self.robot_pose: Pose2d | None = None

        self.load_obstacles_and_configure_grid()
        self.publish_ground_truth_map()
        self.create_timer(self.config.publish_period_s, self.publish_occupancy_grid)

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.map_frame_id:
            self.get_logger().warn(f"Dropping odometry: frame '{msg.header.frame_id}' != '{self.config.map_frame_id}'")
            return
        if msg.child_frame_id != self.config.ground_truth_base_frame_id:
            self.get_logger().warn(
                f"Dropping odometry: child_frame_id '{msg.child_frame_id}' != '{self.config.ground_truth_base_frame_id}'"
            )
            return

        self.robot_pose = Pose2d.from_ros(msg.pose.pose)

    def load_obstacles_and_configure_grid(self) -> None:
        try:
            with open(self.config.map_file_path) as file:
                data = json.load(file)
        except FileNotFoundError:
            self.get_logger().fatal(f"Map file not found: '{self.config.map_file_path}'")
            raise

        self.resolution_m: float = data["resolution_m"]
        self.width_cells: int = math.ceil(self.config.width_m / self.resolution_m)
        self.height_cells: int = math.ceil(self.config.height_m / self.resolution_m)
        self.obstacle_cells: frozenset[tuple[int, int]] = frozenset((x, y) for x, y in data["obstacles"])
        self.lane_line_cells: frozenset[tuple[int, int]] = frozenset(
            (x, y) for x, y in data.get("lane_lines", [])
        )

        self.get_logger().info(
            f"Loaded {len(self.obstacle_cells)} obstacles, {len(self.lane_line_cells)} lane line cells "
            f"at {self.resolution_m} m/cell ({self.width_cells}x{self.height_cells} grid)"
        )

    def publish_ground_truth_map(self) -> None:
        all_cells = self.obstacle_cells | self.lane_line_cells
        if not all_cells:
            return

        min_x = min(x for x, _ in all_cells)
        min_y = min(y for _, y in all_cells)
        max_x = max(x for x, _ in all_cells)
        max_y = max(y for _, y in all_cells)
        map_width_cells = max_x - min_x + 1
        map_height_cells = max_y - min_y + 1

        static_map = np.full((map_height_cells, map_width_cells), self.FREE, dtype=np.int8)
        for x, y in all_cells:
            static_map[y - min_y, x - min_x] = self.OCCUPIED

        self.ground_truth_publisher.publish(
            OccupancyGrid(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.map_frame_id),
                info=MapMetaData(
                    resolution=self.resolution_m,
                    width=map_width_cells,
                    height=map_height_cells,
                    origin=Pose2d(Point2d(x=min_x, y=min_y) * self.resolution_m, Rotation2d(0.0)).to_ros(),
                ),
                data=static_map.flatten().tolist(),
            )
        )

    def publish_occupancy_grid(self) -> None:
        if self.robot_pose is None:
            return

        grid = np.full((self.height_cells, self.width_cells), self.FREE, dtype=np.int8)

        # Collect which local cells map to obstacles and lane lines (world lookup).
        lane_mask = np.zeros((self.height_cells, self.width_cells), dtype=bool)
        for row in range(self.height_cells):
            for col in range(self.width_cells):
                local = Point2d(
                    x=self.config.offset_x_m + (col + 0.5) * self.resolution_m,
                    y=self.config.offset_y_m + (row + 0.5) * self.resolution_m,
                )
                world = self.robot_pose.local_to_world(local)
                ox = math.floor(world.x / self.resolution_m)
                oy = math.floor(world.y / self.resolution_m)
                if (ox, oy) in self.obstacle_cells:
                    grid[row, col] = self.OCCUPIED
                elif (ox, oy) in self.lane_line_cells:
                    lane_mask[row, col] = True

        # Raycast obstacles
        if len(self.obstacle_cells) != 0:
            self._apply_raycasting(grid)

        # Add lane lines in after raycasting to prevent them from occluding obstacles.
        grid[lane_mask] = self.OCCUPIED

        self.occupancy_grid_publisher.publish(
            OccupancyGrid(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.base_frame_id),
                info=MapMetaData(
                    resolution=self.resolution_m,
                    width=self.width_cells,
                    height=self.height_cells,
                    origin=Pose2d(
                        Point2d(x=self.config.offset_x_m, y=self.config.offset_y_m), Rotation2d(0.0)
                    ).to_ros(),
                ),
                data=grid.flatten().tolist(),
            )
        )

    def _apply_raycasting(self, grid: np.ndarray) -> None:
        """Mark all cells behind obstacles (from the robot's perspective) as OCCUPIED."""
        # Robot is at local (0, 0), which is at fractional grid coordinates:
        r_col = -self.config.offset_x_m / self.resolution_m - 0.5
        r_row = -self.config.offset_y_m / self.resolution_m - 0.5

        obs_rows, obs_cols = np.where(grid == self.OCCUPIED)
        if len(obs_rows) == 0:
            return

        # Ray direction from robot to each cell center.
        # dx shape: (1, W, 1), dy shape: (H, 1, 1) — broadcast together to (H, W, N).
        col_idx = np.arange(self.width_cells)
        row_idx = np.arange(self.height_cells)
        dx = col_idx[np.newaxis, :, np.newaxis] + 0.5 - r_col  # (1, W, 1)
        dy = row_idx[:, np.newaxis, np.newaxis] + 0.5 - r_row  # (H, 1, 1)

        # Obstacle grid positions: (1, 1, N)
        oc = obs_cols[np.newaxis, np.newaxis, :].astype(np.float64)
        or_ = obs_rows[np.newaxis, np.newaxis, :].astype(np.float64)

        # For each (target cell, obstacle) pair, find the t-interval on the ray
        # [0=robot, 1=target center] where the ray passes through the obstacle AABB.
        eps = 1e-12
        safe_dx = np.where(np.abs(dx) > eps, dx, 1.0)
        safe_dy = np.where(np.abs(dy) > eps, dy, 1.0)

        tx_a = (oc - r_col) / safe_dx
        tx_b = (oc + 1.0 - r_col) / safe_dx
        ty_a = (or_ - r_row) / safe_dy
        ty_b = (or_ + 1.0 - r_row) / safe_dy

        x_in_range = (r_col >= oc) & (r_col < oc + 1.0)
        y_in_range = (r_row >= or_) & (r_row < or_ + 1.0)

        t_min_x = np.where(np.abs(dx) > eps, np.minimum(tx_a, tx_b), np.where(x_in_range, -np.inf, np.inf))
        t_max_x = np.where(np.abs(dx) > eps, np.maximum(tx_a, tx_b), np.where(x_in_range, np.inf, -np.inf))
        t_min_y = np.where(np.abs(dy) > eps, np.minimum(ty_a, ty_b), np.where(y_in_range, -np.inf, np.inf))
        t_max_y = np.where(np.abs(dy) > eps, np.maximum(ty_a, ty_b), np.where(y_in_range, np.inf, -np.inf))

        t_enter = np.maximum(t_min_x, t_min_y)  # (H, W, N)
        t_exit = np.minimum(t_max_x, t_max_y)   # (H, W, N)

        # Cell is occluded if any obstacle intersects the ray at t ∈ (0, 1).
        blocking = (t_enter < 1.0 - eps) & (t_exit > eps) & (t_enter < t_exit)
        grid[np.any(blocking, axis=2)] = self.OCCUPIED


def main() -> None:
    rclpy.init()
    node = OccupancyGridSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
