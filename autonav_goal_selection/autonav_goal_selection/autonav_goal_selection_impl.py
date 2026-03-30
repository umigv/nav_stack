import math

import numpy as np
from nav_utils.geometry import Point2d, Pose2d
from nav_msgs.msg import MapMetaData, OccupancyGrid
from nav_utils.world_occupancy_grid import WorldOccupancyGrid

from .autonav_goal_selection_config import GoalSelectionParams


def select_goal(
    grid: WorldOccupancyGrid, robot_pose: Pose2d, waypoint: Point2d, params: GoalSelectionParams
) -> Point2d | None:
    """Select the best drivable goal in the occupancy grid.

    Scores every in-bounds drivable cell using a heuristic and returns
    the highest-scoring point. Non-drivable cells are excluded.

    Args:
        grid: World-coordinate occupancy grid.
        robot_pose: Robot pose in world coordinates.
        waypoint: Target waypoint in world coordinates.
        params: Goal selection algorithm parameters.
    Returns:
        The best drivable goal point, or None if no drivable cells exist.
    """


    def heuristic(point: Point2d) -> float:
        if not grid.state(point).is_drivable:
            return math.inf

        # this is rotated to the local
        delta = robot_pose.world_to_local(point)

        # add the zone-based
        x_weight = 0.0
        if delta.x < params.behind_robot_penalty_distance_m:
            x_weight += (params.behind_robot_penalty_distance_m - delta.x) * params.behind_robot_linear_factor
        y_weight = params.lateral_quadratic_factor * (delta.y**2)

        # because we aren't doing this as a full A*, I don't think we need the prior distance as a factor--it should already be accounted for through zone weight.
        # the last term here checks if the point is within 1m of the waypoint, and if so, lowers the priority accordingly.
        return (
            x_weight
            + y_weight
            + grid.state(point).value
            # this is the waypoint priority hole (within 1m)
            - (params.waypoint_proximity_weight * (params.waypoint_proximity_radius_m >= point.distance(waypoint)))
            # waypoint direction priority--adds one lightly weighted term of waypoint distance. just enough to bias the closer side, not enough to even mess with the quadratic.
            + params.waypoint_dist_weight * point.distance(waypoint)
        )

    best_point = min(grid.in_bound_points(Point2d), key=heuristic)
    return best_point if heuristic(best_point) < math.inf  else None

def select_goal_test(
    grid: WorldOccupancyGrid, robot_pose: Pose2d, waypoint: Point2d, params: GoalSelectionParams
) -> tuple[Point2d, list[Point2d]] | tuple[None, None]:
    """Select the best drivable goal in the occupancy grid.

    Scores every in-bounds drivable cell using a heuristic and returns
    the highest-scoring point. Non-drivable cells are excluded.

    Args:
        grid: World-coordinate occupancy grid.
        robot_pose: Robot pose in world coordinates.
        waypoint: Target waypoint in world coordinates.
        params: Goal selection algorithm parameters.
    Returns:
        The best drivable goal point, or None if no drivable cells exist.
    """



    def heuristic(point: Point2d) -> float:
        if not grid.state(point).is_drivable:
            return math.inf

        # this is rotated to the local
        delta = robot_pose.world_to_local(point)

        # add the zone-based
        x_weight = 0.0
        if delta.x < params.behind_robot_penalty_distance_m:
            x_weight += (params.behind_robot_penalty_distance_m - delta.x) * params.behind_robot_linear_factor
        y_weight = params.lateral_quadratic_factor * (delta.y**2)

        # because we aren't doing this as a full A*, I don't think we need the prior distance as a factor--it should already be accounted for through zone weight.
        # the last term here checks if the point is within 1m of the waypoint, and if so, lowers the priority accordingly.
        return (
            x_weight
            + y_weight
            + grid.state(point).value
            # this is the waypoint priority hole (within 1m)
            - (params.waypoint_proximity_weight * (params.waypoint_proximity_radius_m >= point.distance(waypoint)))
            # waypoint direction priority--adds one lightly weighted term of waypoint distance. just enough to bias the closer side, not enough to even mess with the quadratic.
            + params.waypoint_dist_weight * point.distance(waypoint)
        )
    def heuristic_map(point: Point2d) -> float:
        if not grid.state(point).is_drivable:
            return -1
        h = heuristic(point)
        h = max(h, 0)
        h = min(h, 100)
    gs_map = list(map(heuristic_map, grid.in_bound_points(Point2d)))
    if len(gs_map) == 0:
        return (None, None)
    

    best_point = min(grid.in_bound_points(Point2d), key=heuristic)
    return best_point, gs_map if heuristic(best_point) < math.inf  else None



