cmake_minimum_required(VERSION 3.8)
project(sub_sim_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(stonefish_ros2 REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

add_executable(thruster_republishers src/thruster_republishers.cpp)
include_directories(thruster_republishers include)
ament_target_dependencies(thruster_republishers rclcpp std_msgs)

add_executable(sim_imu_remapper src/sim_imu_remapper.cpp)
include_directories(sim_imu_remapper include)
ament_target_dependencies(sim_imu_remapper rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  sim_dvl_remapper
  thruster_republishers
  sim_imu_remapper
  DESTINATION lib/${PROJECT_NAME})

ament_package()
