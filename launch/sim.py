from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("stonefish_ros2"),
                    "launch",
                    "stonefish_simulator.launch.py"
                ])
            ]),
            launch_arguments = {
                "simulation_data" : PathJoinSubstitution([FindPackageShare("sub_sim"), "data"]),
                "scenario_desc" : PathJoinSubstitution([FindPackageShare("sub_sim"), "scenarios", "woollett.scn"]),
                "simulation_rate" : "300.0",
                "window_res_x" : "1900",
                "window_res_y" : "1000",
                "rendering_quality" : "high",
            }.items()
        )
    ])