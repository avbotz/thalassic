cmake_minimum_required(VERSION 3.16)
project(sub_mission)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sub_control_interfaces REQUIRED)
find_package(sub_driver_interfaces REQUIRED)
find_package(sub_vision_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# add nodes

# Everything but main(): the mission executable and the tests link it.
add_library(mission_core STATIC
    src/nodes/actions/add_att_setpoint.cpp
    src/nodes/actions/actuators.cpp
    src/nodes/actions/altitude_setpoint.cpp
    src/nodes/actions/angular_velocity_setpoint.cpp
    src/nodes/actions/att_setpoint.cpp
    src/nodes/actions/average_angles.cpp
    src/nodes/actions/move_relative.cpp
    src/nodes/actions/navigate_to_transform.cpp
    src/nodes/actions/octagon_surface_sweep.cpp
    src/nodes/actions/pos_setpoint.cpp
    src/nodes/actions/save_pose.cpp
    src/nodes/actions/spin.cpp
    src/nodes/actions/stop.cpp
    src/nodes/actions/timed_velocity.cpp
    src/nodes/actions/velocity_setpoint.cpp
    src/nodes/actions/wait_until_hit.cpp
    src/nodes/node.cpp
    src/nodes/vision.cpp
    src/utils.cpp
    src/vision_client.cpp
)
target_include_directories(mission_core
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mission_core PUBLIC
    "rclcpp"
    "ament_index_cpp"
    "behaviortree_cpp"
    "geometry_msgs"
    "nav_msgs"
    "std_msgs"
    "sub_control_interfaces"
    "sub_driver_interfaces"
    "sub_vision_interfaces"
    "tf2"
    "tf2_ros"
)

add_executable(mission
    src/nodes/mission.cpp
)
target_link_libraries(mission PRIVATE mission_core)

# Only watches the kill switch and runs `mission` in a child process. Links
# mission_core for resolveMissionPath(), to check the mission at launch.
add_executable(restart
    src/nodes/restart.cpp
)
target_link_libraries(restart PRIVATE mission_core)

install(TARGETS
    mission
    restart
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    # Loads every packaged mission and drives the movement nodes against a
    # scripted control/error; no other ROS node needs to be running.
    ament_add_gtest(test_mission test/test_mission.cpp)
    target_link_libraries(test_mission mission_core)
    target_compile_definitions(test_mission PRIVATE SUB_MISSION_RESOURCES="${CMAKE_CURRENT_SOURCE_DIR}/resources")
endif()

install(DIRECTORY
    resources
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
