cmake_minimum_required(VERSION 3.8)
project(sub_sim_sensors)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(stonefish_ros2 REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sub_driver_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

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

add_executable(sim_dvl_remapper src/sim_dvl_remapper.cpp)
include_directories(sim_dvl_remapper include)
ament_target_dependencies(sim_dvl_remapper rclcpp stonefish_ros2 marine_acoustic_msgs)

add_executable(sim_thruster_republisher src/sim_thruster_republisher.cpp)
include_directories(sim_thruster_republisher include)
ament_target_dependencies(sim_thruster_republisher rclcpp std_msgs)

add_executable(sim_imu_remapper src/sim_imu_remapper.cpp)
include_directories(sim_imu_remapper include)
ament_target_dependencies(sim_imu_remapper rclcpp sensor_msgs geometry_msgs)

add_executable(sim_kill_switch src/sim_kill_switch.cpp)
include_directories(sim_kill_switch include)
ament_target_dependencies(sim_kill_switch rclcpp std_msgs)

<<<<<<< HEAD
add_executable(sim_torpedo_launcher src/sim_torpedo_launcher.cpp)
include_directories(sim_torpedo_launcher include)
ament_target_dependencies(sim_torpedo_launcher rclcpp std_msgs std_srvs sub_driver_interfaces)

add_executable(sim_dropper src/sim_dropper.cpp)
include_directories(sim_dropper include)
ament_target_dependencies(sim_dropper rclcpp sensor_msgs std_srvs sub_driver_interfaces)
=======
add_executable(sim_oak_camera_remapper src/sim_oak_camera_remapper.cpp)
target_include_directories(sim_oak_camera_remapper PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(sim_oak_camera_remapper rclcpp sensor_msgs cv_bridge OpenCV)
>>>>>>> 908c967 (feat: add preliminary vision code)

install(TARGETS
  sim_dvl_remapper
  sim_thruster_republisher
  sim_imu_remapper
  sim_kill_switch
<<<<<<< HEAD
  sim_torpedo_launcher
  sim_dropper
=======
  sim_oak_camera_remapper
>>>>>>> 908c967 (feat: add preliminary vision code)
  DESTINATION lib/${PROJECT_NAME})

ament_package()
