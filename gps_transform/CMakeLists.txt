cmake_minimum_required(VERSION 3.8)
project(gps_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)


add_executable(waypoint_publisher
  include/WaypointPublisher.hpp
  include/GPSCoordinate.hpp
  include/Point.hpp
  include/MapFrame.hpp

  src/main.cpp
  src/WaypointPublisher.cpp 
  src/GPSCoordinate.cpp
  src/Point.cpp
  src/MapFrame.cpp 
)

ament_target_dependencies(waypoint_publisher 
  rclcpp 
  std_msgs
  example_interfaces 
  sensor_msgs 
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2_ros
  rclcpp_action
  rclcpp_components
)

install(TARGETS
  waypoint_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
