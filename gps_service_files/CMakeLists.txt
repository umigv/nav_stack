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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(waypoint_publisher src/waypoint_publisher.cpp src/gps_transform_classes.cpp)
ament_target_dependencies(waypoint_publisher rclcpp std_msgs example_interfaces)

install(TARGETS
waypoint_publisher
DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


#find_package(rosidl_default_generators REQUIRED)
#set(srv_files "srv/GoalRequestService.srv")

#rosidl_generate_interfaces(${PROJECT_NAME}
#   ${srv_files}
#  "srv/GoalRequestService.srv"
#)
#ament_export_dependencies(rosidl_default_runtime)


ament_package()
