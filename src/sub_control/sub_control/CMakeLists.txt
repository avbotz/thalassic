cmake_minimum_required(VERSION 3.8)
project(sub_control)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sub_control_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(sub_control_lib STATIC src/utils.cpp src/controller.cpp)
target_include_directories(sub_control_lib PUBLIC include)
target_link_libraries(sub_control_lib Eigen3::Eigen)

add_executable(${PROJECT_NAME} src/sub_control.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} sub_control_lib)
ament_target_dependencies(${PROJECT_NAME} rclcpp rcl_interfaces nav_msgs std_msgs geometry_msgs tf2 tf2_geometry_msgs sub_control_interfaces)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/tune_sub_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tune_sub_control)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller sub_control_lib)

  ament_add_gtest(test_utils test/test_utils.cpp)
  target_link_libraries(test_utils sub_control_lib)
endif()

ament_package()
