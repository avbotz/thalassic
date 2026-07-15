cmake_minimum_required(VERSION 3.8)
project(sub_control)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sub_control_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(angles REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(${PROJECT_NAME} src/sub_control.cpp src/pid_controller.cpp src/utils.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp rclcpp_action sub_control_interfaces tf2 tf2_geometry_msgs nav_msgs robot_localization angles)

add_library(feedforward_control_core src/feedforward_core.cpp)
target_include_directories(feedforward_control_core PUBLIC include)
target_link_libraries(feedforward_control_core Eigen3::Eigen)

add_library(feedforward_controller_node src/sub_control_ff.cpp src/utils.cpp)
target_include_directories(feedforward_controller_node PUBLIC include)
target_link_libraries(feedforward_controller_node feedforward_control_core Eigen3::Eigen)
ament_target_dependencies(
  feedforward_controller_node
  angles geometry_msgs nav_msgs rclcpp rclcpp_action robot_localization std_msgs
  sub_control_interfaces tf2 tf2_geometry_msgs)

add_executable(sub_control_ff src/sub_control_ff_main.cpp)
target_link_libraries(sub_control_ff feedforward_controller_node)

install(TARGETS
  ${PROJECT_NAME}
  sub_control_ff
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_feedforward_core test/test_feedforward_core.cpp)
  target_link_libraries(test_feedforward_core feedforward_control_core)

  ament_add_gtest(test_feedforward_node test/test_feedforward_node.cpp)
  target_link_libraries(test_feedforward_node feedforward_controller_node)
  ament_target_dependencies(
    test_feedforward_node
    geometry_msgs nav_msgs rclcpp robot_localization std_msgs sub_control_interfaces)
endif()

ament_package()
