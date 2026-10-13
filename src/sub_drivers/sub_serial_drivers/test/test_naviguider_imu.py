"""The NaviGuider driver against a pretend module on a pseudo-terminal."""

import os
import threading
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu
from sub_serial_drivers.naviguider_imu_driver import NaviGuiderIMUDriver


@pytest.fixture
def rig():
    rclpy.init()
    master, slave = os.openpty()
    namespace = f"/test_naviguider_{os.getpid()}"
    node = NaviGuiderIMUDriver(
        namespace=namespace,
        parameter_overrides=[
            Parameter("device", value=os.ttyname(slave)),
            Parameter("max_sample_age", value=0.2),
        ],
    )
    peer = rclpy.create_node("peer", namespace=namespace)
    received = []
    subscription = peer.create_subscription(Imu, "imu/data", received.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    node.trigger_configure()
    node.trigger_activate()
    # Matched before anything is sent, or the first messages are lost under load.
    deadline = time.monotonic() + 10.0
    while subscription.get_publisher_count() == 0 and time.monotonic() < deadline:
        time.sleep(0.02)
    time.sleep(0.1)

    def send(*lines):
        os.write(master, "".join(f"{line}\n" for line in lines).encode())

    yield send, received
    executor.shutdown()
    node.destroy_node()
    peer.destroy_node()
    rclpy.shutdown()
    spinner.join(timeout=2.0)
    os.close(master)
    os.close(slave)


def wait(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.01)
    return predicate()


GYRO = "1.0,4,0.1,0.2,0.3,3"
ORIENTATION = "1.0,15,0.0,0.0,0.0,1.0,3"
ACCEL = "1.0,1,0.0,0.0,9.8,3"


def test_publishes_only_once_gyro_and_orientation_have_reported(rig):
    send, received = rig
    send(ACCEL)
    time.sleep(0.2)
    assert received == []
    send(GYRO, ORIENTATION, ACCEL)
    assert wait(lambda: len(received) == 1)
    assert received[0].angular_velocity.z == pytest.approx(0.3)
    assert received[0].orientation.w == pytest.approx(1.0)


def test_a_sensor_that_stops_stops_the_messages(rig):
    send, received = rig
    send(GYRO, ORIENTATION, ACCEL)
    assert wait(lambda: len(received) == 1)
    # The gyro goes quiet; the orientation keeps coming.
    time.sleep(0.3)
    send(ORIENTATION, ACCEL)
    time.sleep(0.2)
    assert len(received) == 1
    # The gyro is back (and the orientation still coming).
    send(GYRO, ORIENTATION, ACCEL)
    assert wait(lambda: len(received) == 2)


def test_garbage_and_non_finite_values_are_ignored(rig):
    send, received = rig
    send("junk", "1.0,4,nan,0.2,0.3,3", ORIENTATION, ACCEL)
    time.sleep(0.2)
    assert received == []
