cmake_minimum_required(VERSION 3.5)
project(occupancy_grid_saver_cpp)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(occupancy_grid_saver src/occupancy_grid_saver.cpp)

# Link libraries
ament_target_dependencies(occupancy_grid_saver rclcpp nav_msgs)

# Install targets
install(TARGETS
  occupancy_grid_saver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
