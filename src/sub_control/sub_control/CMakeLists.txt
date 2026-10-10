cmake_minimum_required(VERSION 3.16)
project(sub_control)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sub_control_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# The control law, trajectory generator and allocator, free of ROS so the unit
# tests can drive them directly.
add_library(${PROJECT_NAME}_core STATIC
  src/allocator.cpp
  src/controller.cpp
  src/reference_generator.cpp
  src/trajectory.cpp
  src/utils.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC include)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(${PROJECT_NAME} src/sub_control.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME} rclcpp sub_control_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs
  std_srvs robot_localization tf2 tf2_ros)

add_executable(thruster_monitor src/thruster_monitor.cpp)
target_link_libraries(thruster_monitor ${PROJECT_NAME}_core)
ament_target_dependencies(thruster_monitor rclcpp nav_msgs std_msgs std_srvs)

install(TARGETS
  ${PROJECT_NAME}
  thruster_monitor
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/identify_model DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_allocator.cpp
    test/test_controller.cpp
    test/test_reference_generator.cpp
    test/test_trajectory.cpp
    test/test_utils.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_core)
endif()

ament_package()
