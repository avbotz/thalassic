cmake_minimum_required(VERSION 3.14)
project(sub_serial_drivers)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sub_control_interfaces REQUIRED)
find_package(sub_driver_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/serial_port.cpp
  src/serial_node.cpp
  src/sub_low.cpp
  src/naviguider_imu.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME} diagnostic_updater nav_msgs rclcpp rclcpp_components
  sensor_msgs std_msgs sub_control_interfaces sub_driver_interfaces)

# Each registration also generates a standalone executable of the same name.
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sub_serial_drivers::SubLow" EXECUTABLE sub_low)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sub_serial_drivers::NaviGuiderImu" EXECUTABLE naviguider_imu_driver)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Each driver against a pretend device on a pseudo-terminal. The nodes talk only
  # to each other: Fast DDS 2.14 can deadlock destroying a node while it is
  # discovering another process (such as another package's tests).
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_serial_port.cpp
    test/test_sub_low.cpp
    test/test_naviguider_imu.cpp
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    TIMEOUT 120)
  # util: openpty()
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} util)
endif()

ament_package()
