cmake_minimum_required(VERSION 3.16)
project(sub_control_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Error.msg"
  "msg/Setpoint.msg"
  "msg/SetpointAxes.msg"
  DEPENDENCIES std_msgs
)

ament_package()
