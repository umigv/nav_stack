from functools import lru_cache

import numpy as np

from .occupancy_grid_transform_config import InflationParams


def add_border(grid: np.ndarray) -> np.ndarray:
    """
    Add a 1-cell occupied border on the top, bottom, and right edges of the grid.

    Sets the top row, bottom row, and right column to 100 (occupied). The left column (y=0, closest
    to the robot) is left untouched. When followed by `inflate_grid`, the border produces a soft
    falloff that discourages planning near grid edges.

    Args:
        grid: 2D occupancy grid of shape `(height, width)`.

    Returns:
        The same grid array, modified in place.
    """
    grid[-1, :] = 100  # top row
    grid[0, :] = 100  # bottom row
    grid[:, -1] = 100  # right column
    return grid


@lru_cache(maxsize=1)
def _make_inflation_kernel(r_hard: int, r_soft: int, decay: float) -> np.ndarray:
    """Precompute a (2*r_soft+1) x (2*r_soft+1) kernel of inflation values."""
    ys, xs = np.mgrid[-r_soft : r_soft + 1, -r_soft : r_soft + 1]
    dists = np.hypot(xs, ys)
    kernel = np.where(
        dists <= r_hard,
        100,
        np.where(dists <= r_soft, np.round(100.0 * decay ** (dists - r_hard)), 0),
    ).astype(np.int8)
    return kernel


def inflate_grid(grid: np.ndarray, params: InflationParams) -> np.ndarray:
    """
    Inflate obstacles in a 2D occupancy grid.

    For each occupied cell (value 100), this expands obstacle values based on the provided InflationParams

    Args:
        grid: 2D occupancy grid of shape `(height, width)` with values in `[-1, 100]` (unknown/free/occupied).
        params: Inflation tuning parameters (hard radius, soft radius, decay).

    Returns:
        A new 2D `np.ndarray` (dtype `int8`) of the same shape containing the inflated occupancy values.
    """
    r_hard = params.inflation_radius_cells
    r_soft = params.inflation_falloff_radius_cells
    decay = params.inflation_decay_factor
    height, width = grid.shape

    kernel = _make_inflation_kernel(r_hard, r_soft, decay)
    output = grid.astype(np.int8, copy=True)

    ys, xs = np.where(grid == 100)
    for y, x in zip(ys.tolist(), xs.tolist(), strict=True):
        y0, y1 = max(0, y - r_soft), min(height, y + r_soft + 1)
        x0, x1 = max(0, x - r_soft), min(width, x + r_soft + 1)

        ky0, ky1 = y0 - (y - r_soft), y1 - (y - r_soft)
        kx0, kx1 = x0 - (x - r_soft), x1 - (x - r_soft)
        np.maximum(output[y0:y1, x0:x1], kernel[ky0:ky1, kx0:kx1], out=output[y0:y1, x0:x1])

    return output
