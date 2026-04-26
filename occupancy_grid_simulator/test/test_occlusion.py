import numpy as np
from occupancy_grid_simulator.occlusion import apply_occlusion, bresenham_cells

# ---------------------------------------------------------------------------
# bresenham_cells
# ---------------------------------------------------------------------------


def test_bresenham_start_equals_end_returns_empty():
    assert bresenham_cells(2, 2, 2, 2) == []


def test_bresenham_fractional_start_rounds_to_end_returns_empty():
    # round(-0.5) == 0 (Python banker's rounding)
    assert bresenham_cells(-0.5, 2.0, 0, 2) == []


def test_bresenham_horizontal_right():
    cells = bresenham_cells(0, 5, 4, 5)
    cols = [c for c, r in cells]
    rows = [r for c, r in cells]
    assert (4, 5) not in cells
    assert cols == sorted(cols)
    assert all(r == 5 for r in rows)
    assert len(cells) == 4


def test_bresenham_horizontal_left():
    cells = bresenham_cells(4, 5, 0, 5)
    cols = [c for c, r in cells]
    assert cols == sorted(cols, reverse=True)
    assert (0, 5) not in cells


def test_bresenham_vertical_up():
    cells = bresenham_cells(3, 5, 3, 0)
    rows = [r for c, r in cells]
    assert rows == sorted(rows, reverse=True)
    assert all(c == 3 for c, r in cells)
    assert (3, 0) not in cells


def test_bresenham_vertical_down():
    cells = bresenham_cells(3, 0, 3, 5)
    rows = [r for c, r in cells]
    assert rows == sorted(rows)
    assert (3, 5) not in cells


def test_bresenham_diagonal():
    cells = bresenham_cells(0.0, 0.0, 3, 3)
    assert (3, 3) not in cells
    assert len(cells) == 3


def test_bresenham_fractional_start():
    # Robot at col=-0.5, row=2 (rounds to col=0, row=2)
    cells = bresenham_cells(-0.5, 2.0, 4, 2)
    cols = [c for c, r in cells]
    assert 0 in cols
    assert 4 not in cols
    assert (4, 2) not in cells


# ---------------------------------------------------------------------------
# apply_occlusion
# ---------------------------------------------------------------------------

FREE = 0
OCCUPIED = 100


def make_obstacles(height: int, width: int, obstacle_rc: list[tuple[int, int]]) -> np.ndarray:
    arr = np.zeros((height, width), dtype=bool)
    for r, c in obstacle_rc:
        arr[r, c] = True
    return arr


def test_no_obstacles_all_free():
    obstacle_local = make_obstacles(5, 5, [])
    grid = apply_occlusion(obstacle_local, robot_col=-0.5, robot_row=2.0)
    assert np.all(grid == FREE)


def test_obstacle_cell_is_occupied():
    obstacle_local = make_obstacles(5, 5, [(2, 2)])
    grid = apply_occlusion(obstacle_local, robot_col=-0.5, robot_row=2.0)
    assert grid[2, 2] == OCCUPIED


def test_cell_in_front_of_obstacle_is_free():
    # Obstacle at col=3, row=2; cell at col=1, row=2 is between robot and obstacle.
    obstacle_local = make_obstacles(5, 5, [(2, 3)])
    grid = apply_occlusion(obstacle_local, robot_col=-0.5, robot_row=2.0)
    assert grid[2, 1] == FREE


def test_cell_directly_behind_obstacle_is_occupied():
    # Robot at frac col=-0.5, row=2 (middle row).
    # Obstacle at col=2, row=2.  Cell at col=4, row=2 is directly behind it.
    obstacle_local = make_obstacles(5, 5, [(2, 2)])
    grid = apply_occlusion(obstacle_local, robot_col=-0.5, robot_row=2.0)
    assert grid[2, 4] == OCCUPIED


def test_cell_adjacent_to_obstacle_not_behind_it_is_free():
    # Obstacle at col=2, row=2. Cell at col=3, row=0 is NOT directly behind obstacle
    # (it is at a different row), and no obstacle blocks the ray to it.
    obstacle_local = make_obstacles(5, 5, [(2, 2)])
    grid = apply_occlusion(obstacle_local, robot_col=-0.5, robot_row=2.0)
    assert grid[0, 3] == FREE


def test_multiple_obstacles_shadow_correct_cells():
    # Two obstacles in the same row. Cells beyond either should be OCCUPIED (occluded).
    obstacle_local = make_obstacles(5, 7, [(2, 1), (2, 3)])
    grid = apply_occlusion(obstacle_local, robot_col=-0.5, robot_row=2.0)
    # Obstacle cells are OCCUPIED
    assert grid[2, 1] == OCCUPIED
    assert grid[2, 3] == OCCUPIED
    # Cell beyond first obstacle (col=2) is occluded by col=1
    assert grid[2, 2] == OCCUPIED
    # Cell beyond second obstacle (col=4,5,6) is occluded by col=3 (or earlier)
    assert grid[2, 4] == OCCUPIED
    assert grid[2, 5] == OCCUPIED
    assert grid[2, 6] == OCCUPIED


def test_returns_int8_array():
    obstacle_local = make_obstacles(4, 4, [(2, 2)])
    grid = apply_occlusion(obstacle_local, robot_col=-0.5, robot_row=1.5)
    assert grid.dtype == np.int8
    assert grid.shape == (4, 4)


def test_robot_inside_grid_occludes_behind():
    # Robot in the middle of the grid (col=2, row=2) in a 5x5 grid.
    # Obstacle at (row=2, col=3). Cell at (row=2, col=4) should be occluded (OCCUPIED).
    obstacle_local = make_obstacles(5, 5, [(2, 3)])
    grid = apply_occlusion(obstacle_local, robot_col=2.0, robot_row=2.0)
    assert grid[2, 3] == OCCUPIED
    assert grid[2, 4] == OCCUPIED
    # Cell at (row=2, col=1) is on the other side; not occluded
    assert grid[2, 1] == FREE
