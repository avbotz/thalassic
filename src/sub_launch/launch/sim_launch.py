from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from sub_sim.randomize_locs import randomize_scenario_locations
from sub_sim.generate_robot import render_robot_scenario


def _render_scn(context, *_, **__):
    lc = lambda k: LaunchConfiguration(k).perform(context)

    DX = float(lc("DX"))
    DY = float(lc("DY"))
    DZ = float(lc("DZ"))
    DYAW = float(lc("DYAW"))
    SEED = int(lc("seed")) if lc("seed") != "" and lc("seed").isdigit() else None


    scenario_file = Path(get_package_share_directory("sub_sim")) / "scenarios" / "woollett.scn.j2"
    robot_scenario_file = Path(get_package_share_directory("sub_sim")) / "data" / "robots" / "marlin_v2" / "layout.scn.j2"

    robot_rendered_path = render_robot_scenario(robot_scenario_file)

    temp_path = randomize_scenario_locations(
        scenario_template_file=scenario_file,
        DX=DX,
        DY=DY,
        DZ=DZ,
        DYAW=DYAW,
        seed=SEED,
        ROBOT_SCENARIO_PATH=robot_rendered_path,
    )

    return [SetLaunchConfiguration("scenario_file", temp_path)]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("seed", default_value=""),
        DeclareLaunchArgument("DX", default_value="0.25"),
        DeclareLaunchArgument("DY", default_value="0.25"),
        DeclareLaunchArgument("DZ", default_value="0.10"),
        DeclareLaunchArgument("DYAW", default_value="0.10"),
    ]

    render = OpaqueFunction(function=_render_scn)

    include_stonefish = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("sub_launch"), "launch", "marlin_v2_launch.py"])]
        ),
        launch_arguments={
            "simulation_data": PathJoinSubstitution([FindPackageShare("sub_sim"), "data"]),
            "scenario_desc": LaunchConfiguration("scenario_file"),
            "simulation_rate": "300.0",
            "window_res_x": "1900",
            "window_res_y": "1000",
            "rendering_quality": "medium",
            "use_sim_time": "true",
        }.items(),
    )

    include_transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("stonefish_ros2"), "launch", "stonefish_simulator.launch.py"])]
        ),
    )

    return LaunchDescription(args + [render, include_stonefish, include_transforms])
