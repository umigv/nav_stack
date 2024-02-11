cmake_minimum_required(VERSION 3.8)
project(cv-integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
)

add_executable(cv-integration src/nav_stack/grid_generator.cpp)
ament_target_dependencies(cv-integration rclcpp roscpp std_msgs nav_msgs)
target_link_libraries(cv-integration ${rclcpp_LIBRARIES} ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${nav_msgs_LIBRARIES})

install(TARGETS
  cv-integration
  DESTINATION lib/${PROJECT_NAME}
)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()
