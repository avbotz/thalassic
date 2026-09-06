cmake_minimum_required(VERSION 3.16)
project(sub_sim_sensors)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(stonefish_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sub_driver_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

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

add_library(${PROJECT_NAME}_components SHARED
  src/sim_dvl_remapper.cpp
  src/sim_thruster_republisher.cpp
  src/sim_imu_remapper.cpp
  src/sim_pressure_to_depth.cpp
  src/sim_kill_switch.cpp
  src/sim_torpedo_launcher.cpp
  src/sim_dropper.cpp
)
target_include_directories(${PROJECT_NAME}_components PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_components
  stonefish_ros2
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  sub_driver_interfaces
  nav_msgs
  tf2
)

# Each registration also generates a standalone executable of the same name, so
# the components can still be run individually (ros2 run sub_sim_sensors <name>).
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "SimDVLRemapper" EXECUTABLE sim_dvl_remapper)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "SimThrusterRepublisher" EXECUTABLE sim_thruster_republisher)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "SimIMURemapper" EXECUTABLE sim_imu_remapper)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "SimPressureToDepth" EXECUTABLE sim_pressure_to_depth)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "SimKillSwitch" EXECUTABLE sim_kill_switch)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "SimTorpedoLauncher" EXECUTABLE sim_torpedo_launcher)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "SimDropper" EXECUTABLE sim_dropper)

install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
