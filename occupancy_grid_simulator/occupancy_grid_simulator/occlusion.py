from __future__ import annotations

import numpy as np


def bresenham_cells(
    start_col: float,
    start_row: float,
    end_col: int,
    end_row: int,
) -> list[tuple[int, int]]:
    """Return integer grid (col, row) positions on the line from start to end.

    Uses Bresenham's line algorithm. Includes the rounded start cell but excludes
    the end cell. Returns an empty list when the rounded start equals the end.

    Args:
        start_col: Fractional column index of the ray origin (e.g. robot position).
        start_row: Fractional row index of the ray origin.
        end_col: Integer column index of the target cell.
        end_row: Integer row index of the target cell.

    Returns:
        List of (col, row) integer grid cells along the ray, not including (end_col, end_row).
    """
    c, r = round(start_col), round(start_row)
    c1, r1 = end_col, end_row

    dc = abs(c1 - c)
    dr = abs(r1 - r)
    sc = 1 if c1 > c else -1
    sr = 1 if r1 > r else -1
    err = dc - dr

    cells: list[tuple[int, int]] = []
    while (c, r) != (c1, r1):
        cells.append((c, r))
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr

    return cells


def apply_occlusion(
    obstacle_local: np.ndarray,
    robot_col: float,
    robot_row: float,
    free_val: int = 0,
    occupied_val: int = 100,
    unknown_val: int = -1,
) -> np.ndarray:
    """Build an occupancy grid with ray-casting occlusion applied.

    For each cell in the grid, a ray is traced from the robot position to the cell
    using Bresenham's line algorithm. If any obstacle lies on the ray before reaching
    the target cell, the target cell is marked as unknown (occluded). Obstacle cells
    themselves are always marked as occupied regardless of occlusion.

    Args:
        obstacle_local: 2-D boolean array (height x width) where True means the cell
            contains an obstacle in the robot's local grid frame.
        robot_col: Robot's column position in fractional grid coordinates.
        robot_row: Robot's row position in fractional grid coordinates.
        free_val: Value assigned to visible free cells (default 0).
        occupied_val: Value assigned to obstacle cells (default 100).
        unknown_val: Value assigned to occluded cells (default -1).

    Returns:
        int8 numpy array (height x width) with free_val / occupied_val / unknown_val.
    """
    height, width = obstacle_local.shape
    grid = np.full((height, width), free_val, dtype=np.int8)

    for row in range(height):
        for col in range(width):
            if obstacle_local[row, col]:
                grid[row, col] = occupied_val
            else:
                for c, r in bresenham_cells(robot_col, robot_row, col, row):
                    if 0 <= r < height and 0 <= c < width and obstacle_local[r, c]:
                        grid[row, col] = unknown_val
                        break

    return grid
