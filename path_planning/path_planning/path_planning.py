import nav_utils.config
import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped transform support
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav_utils.geometry import Point2d
from nav_utils.world_occupancy_grid import WorldOccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

from .path_planning_config import PathPlanningConfig
from .path_planning_impl import find_closest_drivable_point, generate_path, interpolate_points, smooth_path

from alert_system_interfaces.msg import Alert
from alert_system_interfaces.srv import CreateAlert, ResolveAlert


class PathPlanner(Node):
    def __init__(self) -> None:
        super().__init__("path_planning")

        self.config: PathPlanningConfig = nav_utils.config.load(self, PathPlanningConfig)

        self.robot_position: Point2d | None = None
        self.grid: WorldOccupancyGrid | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, "occupancy_grid", self.occupancy_grid_callback, 10)
        self.create_subscription(PointStamped, "goal", self.goal_callback, 10)

        self.create_subscription(Alert, "alerts", self.alert_callback, 10)
        self.create_alert_client = self.create_client(CreateAlert, "create_alert")
        self.resolve_alert_client = self.create_client(ResolveAlert, "resolve_alert")

        self.path_publisher = self.create_publisher(Path, "path", 10)

        self.active_alert_id: int | None = None

    def alert_callback(self, msg: Alert): 
        self.get_logger().warn(f"Received alert: {msg.message}")
    
    def create_alert(self, category, severity, source, message):
        request = CreateAlert.Request()
        request.category = category
        request.severity = severity
        request.source = source
        request.message = message
        future = self.create_alert_client.call_async(request)
    
    def resolve_alert(self, alert_id):
        request = ResolveAlert.Request()
        request.alert_id = alert_id
        future = self.resolve_alert_client.call_async(request)

    def odom_callback(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.config.frame_id:
            self.get_logger().error(
                f"Frame ID of odometry ({msg.header.frame_id}) does not match config frame ID ({self.config.frame_id})"
            )
            return

        self.robot_position = Point2d.from_ros(msg.pose.pose.position)

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id != self.config.frame_id:
            self.get_logger().error(
                f"Frame ID of occupancy grid ({msg.header.frame_id}) does not match config frame ID ({self.config.frame_id})"
            )
            return

        self.grid = WorldOccupancyGrid(msg)

    def goal_callback(self, msg: PointStamped) -> None:
        if self.robot_position is None or self.grid is None:
            return

        try:
            msg = self.tf_buffer.transform(msg, self.config.frame_id)
        except Exception as e:
            self.get_logger().error(f"Failed to transform goal to {self.config.frame_id}: {e}")
            self.create_alert("path planning", 2, "path planning", f"Failed to transform goal to {self.config.frame_id}: {e}")
            return

        start = find_closest_drivable_point(self.grid, self.robot_position, self.config.max_search_radius_m)
        if start is None:
            self.get_logger().warn("No drivable area found near robot")
            self.create_alert("path planning", 2, "path planning", "No drivable area found near robot")
            return

        self.get_logger().info(f"Starting from ({start.x:.2f}, {start.y:.2f})")
        path_points = generate_path(self.grid, start, Point2d.from_ros(msg.point))
        if path_points is None:
            self.get_logger().warn("No path found to goal")
            self.create_alert("path planning", 2, "path planning", "No path found to goal")
            return

        bridge = interpolate_points(self.robot_position, start, self.config.interpolation_resolution_m)
        self.get_logger().info(f"Bridge size: {len(bridge)}; Path size: {len(path_points)}")
        full_path = smooth_path(bridge[:-1] + path_points, self.config.spline_smoothing)

        if self.active_alert_id is not None:
            self.resolve_alert(self.active_alert_id)
            self.active_alert_id = None

        header = Header(frame_id=self.config.frame_id, stamp=self.get_clock().now().to_msg())
        self.path_publisher.publish(
            Path(
                header=header,
                poses=[
                    PoseStamped(
                        header=header,
                        pose=Pose(position=point.to_ros(), orientation=Quaternion(w=1.0)),
                    )
                    for point in full_path
                ],
            )
        )


def main() -> None:
    rclpy.init()
    node = PathPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
