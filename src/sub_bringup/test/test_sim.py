"""The whole stack against the headless simulator (sim_launch.py headless:=true).

It must arm once the simulated kill switch is released, run the pid_tuning
mission (position, relative and attitude moves, each completed only on an
acknowledged control/error) to SUCCESS, and cut the thrusters when the IMU
stops. No GPU or display
is needed, so it runs in CI; it takes a minute or two.

It runs on its own ROS domain and Foxglove port, so a simulator someone already
has open on the same machine is neither disturbed nor mistaken for this one.
"""

import os
import re
import unittest

import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from composition_interfaces.srv import ListNodes, UnloadNode
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Before anything starts: every process launched below inherits these.
os.environ["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 50)
FOXGLOVE_PORT = str(20000 + os.getpid() % 10000)

MISSION_TIMEOUT = 240.0  # [s] the mission takes about 60 in simulated time


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("sub_bringup"), "launch", "sim_launch.py")
        ),
        launch_arguments={
            "headless": "true",
            "seed": "3",
            "mission": "pid_tuning",
            "groot_port": "0",
            "foxglove_port": FOXGLOVE_PORT,
        }.items(),
    )
    return LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestSimulatedRun(unittest.TestCase):
    def test_run(self, proc_output):
        # sim_kill_switch releases after 6 s; sub_control arms once its sensors are fresh.
        proc_output.assertWaitFor(re.compile(r"sub_control\]: armed$", re.MULTILINE), timeout=90)
        proc_output.assertWaitFor(
            "Mission 'pid_tuning' finished with SUCCESS", timeout=MISSION_TIMEOUT
        )

        # Losing the IMU (its bridge unloaded from the sensor container) must cut the thrusters.
        unload_component("/marlin_v3/sim_sensors_container", "/marlin_v3/sim_imu_remapper")
        proc_output.assertWaitFor("no IMU for 1.00 s: thrusters off", timeout=20)


def unload_component(container: str, node_name: str) -> None:
    """Unloads a composable node through its container's services."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_sim_unloader")
        listing = call(node, ListNodes, f"{container}/_container/list_nodes", ListNodes.Request())
        unique_id = dict(zip(listing.full_node_names, listing.unique_ids, strict=True))[node_name]
        result = call(
            node,
            UnloadNode,
            f"{container}/_container/unload_node",
            UnloadNode.Request(unique_id=unique_id),
        )
        assert result.success, result.error_message
        node.destroy_node()
    finally:
        rclpy.shutdown()


def call(node, service_type, name: str, request, timeout: float = 30.0):
    client = node.create_client(service_type, name)
    assert client.wait_for_service(timeout_sec=timeout), f"{name} not available"
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    assert future.done(), f"{name} did not answer"
    return future.result()
