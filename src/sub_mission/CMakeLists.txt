cmake_minimum_required(VERSION 3.5)
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
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sub_control_interfaces REQUIRED)
find_package(sub_driver_interfaces REQUIRED)
find_package(sub_vision_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

# add nodes

set(MISSION_ACTION_SOURCES
    src/nodes/actions/add_att_setpoint.cpp
    src/nodes/actions/actuators.cpp
    src/nodes/actions/angular_velocity_setpoint.cpp
    src/nodes/actions/control_setpoint.cpp
    src/nodes/actions/average_angles.cpp
    src/nodes/actions/move_relative.cpp
    src/nodes/actions/move_relative_pos.cpp
    src/nodes/actions/navigate_to_transform.cpp
    src/nodes/actions/octagon_surface_sweep.cpp
    src/nodes/actions/save_gate_home.cpp
    src/nodes/actions/sleep.cpp
    src/nodes/actions/spin.cpp
    src/nodes/actions/wait_until_hit.cpp
)

add_executable(mission
    src/nodes/mission.cpp
    src/nodes/node.cpp
    ${MISSION_ACTION_SOURCES}
    src/nodes/vision.cpp
    src/utils.cpp
    src/vision_client.cpp
)

add_executable(restart
    src/nodes/restart.cpp
    src/nodes/node.cpp
    ${MISSION_ACTION_SOURCES}
    src/nodes/vision.cpp
    src/utils.cpp
    src/vision_client.cpp
)

foreach(node IN ITEMS mission restart)
    target_include_directories(${node}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    ament_target_dependencies(${node}
        "rclcpp"
        "rclcpp_action"
        "geometry_msgs"
        "sub_control_interfaces"
        "sub_driver_interfaces"
        "sub_vision_interfaces"
        "tf2"
        "tf2_ros"
    )

    install(TARGETS
        ${node}
        DESTINATION lib/${PROJECT_NAME}
    )
endforeach()

ament_target_dependencies(mission
    "ament_index_cpp"
    "behaviortree_cpp"
)

ament_target_dependencies(restart
    "ament_index_cpp"
    "behaviortree_cpp"
)

install(DIRECTORY
    resources
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
