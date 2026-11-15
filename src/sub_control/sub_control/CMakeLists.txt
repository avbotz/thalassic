cmake_minimum_required(VERSION 3.16)
project(sub_control)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(generate_parameter_library REQUIRED)
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

# The thrusters' measured thrust curves, the only copy of them: the lookup table
# is generated from the file the simulator also reads.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(THRUST_TABLE_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
add_custom_command(
  OUTPUT ${THRUST_TABLE_DIR}/sub_control/thrust_table.hpp
  COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_thrust_table
    ${CMAKE_CURRENT_SOURCE_DIR}/data/t200_thrust.csv ${THRUST_TABLE_DIR}/sub_control/thrust_table.hpp
  DEPENDS scripts/generate_thrust_table data/t200_thrust.csv
  VERBATIM)

# The control law, trajectory generator and allocator, free of ROS so the unit
# tests can drive them directly.
add_library(${PROJECT_NAME}_core STATIC
  src/allocator.cpp
  src/controller.cpp
  src/reference_generator.cpp
  src/safety.cpp
  src/trajectory.cpp
  src/utils.cpp
  ${THRUST_TABLE_DIR}/sub_control/thrust_table.hpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC include PRIVATE ${THRUST_TABLE_DIR})
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Declares and validates the node's parameters: generates
# sub_control/sub_control_parameters.hpp from the YAML.
generate_parameter_library(${PROJECT_NAME}_parameters
  src/sub_control_parameters.yaml
  src/sub_control_validators.hpp)
# The validators header is copied when CMake configures, so reconfigure when it changes.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS src/sub_control_validators.hpp)

add_executable(${PROJECT_NAME} src/sub_control.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core ${PROJECT_NAME}_parameters)
ament_target_dependencies(${PROJECT_NAME} rclcpp sub_control_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs
  std_srvs robot_localization tf2 tf2_ros diagnostic_updater)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/identify_model DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY data DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_allocator.cpp
    test/test_controller.cpp
    test/test_reference_generator.cpp
    test/test_safety.cpp
    test/test_trajectory.cpp
    test/test_utils.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_core)
endif()

ament_package()
