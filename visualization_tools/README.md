# occupancy_grid_visualization
Converts a `nav_msgs/OccupancyGrid` into a `foxglove_msgs/VoxelGrid` for 3D voxel visualization in Foxglove Studio.

## Subscribed Topics
- `occupancy_grid` (`nav_msgs/OccupancyGrid`) - Occupancy grid to visualize

## Published Topics
- `occupancy_grid/voxels` (`foxglove_msgs/VoxelGrid`) - Voxel representation of the occupancy grid



# gauge_message
Converts `enc_vel/raw` into a `Float64` linear speed

## Subscribed Topics 
- `enc_vel/raw` (`geometry_msgs/TwistWithCovariance`) - directional velocities

## Published Topics
- `linear_speed` (`std_msgs/Float64`) - raw total speed
