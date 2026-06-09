cmake_minimum_required(VERSION 3.8)
project(sub_serial_drivers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sub_low src/sub_low.cpp src/serial_port.cpp)
target_include_directories(sub_low PRIVATE include)
ament_target_dependencies(sub_low rclcpp rclcpp_lifecycle std_msgs)

add_executable(naviguider_imu_driver src/naviguider_imu.cpp src/serial_port.cpp)
target_include_directories(naviguider_imu_driver PRIVATE include)
ament_target_dependencies(naviguider_imu_driver rclcpp rclcpp_lifecycle sensor_msgs std_msgs)

install(TARGETS
  sub_low
  naviguider_imu_driver
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
