cmake_minimum_required(VERSION 3.8)
project(cv_grid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# add_executable(cv_grid src/cv_grid.cpp)
# ament_target_dependencies(
#   cv_grid
#   geometry_msgs
#   std_msgs
#   nav_msgs
#   rclcpp
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )

add_executable(cv_view_transform_publisher src/cv_view_transform_publisher.cpp)
ament_target_dependencies(
  cv_view_transform_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  nav2_costmap_2d
  pluginlib
)

# pluginlib_export_plugin_description_file(nav2_costmap_2d cvStaticLayer.xml)

add_executable(cv_grid_transform_publisher src/cv_grid_transform_publisher.cpp)
ament_target_dependencies(
  cv_grid_transform_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

install(TARGETS
  # cv_grid
  cv_view_transform_publisher
  cv_grid_transform_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/  
  DESTINATION include/
)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# Plugin code:
add_library(cvStaticLayer SHARED
  src/cvStaticLayer.cpp
)

ament_target_dependencies(cvStaticLayer
  nav2_costmap_2d
  pluginlib
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_costmap_2d cvStaticLayer.xml)

install(TARGETS cvStaticLayer
  LIBRARY DESTINATION lib
)