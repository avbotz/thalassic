cmake_minimum_required(VERSION 3.16)
project(sub_control_mcu C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sub_control_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# The mec_* sources are C, ported as directly as possible from
# Nautical-Private; the node itself is C++.
add_executable(${PROJECT_NAME}
  src/sub_control_mcu.cpp
  src/pid_controller.c
  src/util.c
  src/estimation.c
  src/mixer.c
  src/att_controller.c
  src/angvel_controller.c
  src/position_controller.c
  src/velocity_controller.c
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} rclcpp sub_control_interfaces std_msgs geometry_msgs nav_msgs robot_localization tf2 tf2_geometry_msgs)
target_link_libraries(${PROJECT_NAME} m)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
