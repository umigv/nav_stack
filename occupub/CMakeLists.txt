cmake_minimum_required(VERSION 3.8)
project(occupub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# include_directories(
#   include
# )

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

add_executable(occupub src/grid_generator.cpp)
target_include_directories(occupub PRIVATE include)
ament_target_dependencies(
  occupub
  rclcpp
  nav_msgs
  tf2_ros
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(merge_costmap_sub src/merge_costmap.cpp)
target_include_directories(merge_costmap_sub PRIVATE include)
ament_target_dependencies(
  merge_costmap_sub
  rclcpp
  nav_msgs
  tf2_ros
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)
  
install (TARGETS
    merge_costmap_sub
    occupub
    DESTINATION lib/${PROJECT_NAME})

# install(
#   DESTINATION include/
# )