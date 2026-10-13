"""sub_low against a pretend board on a pseudo-terminal: the protocol, the
command watchdog, stale commands and reconnecting after the link drops."""

import os
import select
import tempfile
import threading
import time
from pathlib import Path

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from sub_serial_drivers.sub_low import (
    MAX_COMMAND,
    SubLow,
    parse_line,
    sanitize,
    thruster_lines,
)

from sub_control_interfaces.msg import ThrusterCommand


def test_thruster_lines_batch_only_from_protocol_2():
    commands = [0.1, -0.2, 0.0, 0.6, -0.6, 0.0125, 0.0, 0.333]
    assert thruster_lines(commands, 2) == ("m 0.100 -0.200 0.000 0.600 -0.600 0.013 0.000 0.333\n")
    lines = thruster_lines(commands, 1).splitlines()
    assert lines[0] == "p 0 0.100"
    assert lines[7] == "p 7 0.333"
    assert len(lines) == 8


def test_sanitize_stops_non_finite_and_clamps():
    assert sanitize(float("nan")) == 0.0
    assert sanitize(float("inf")) == 0.0
    assert sanitize(2.0) == MAX_COMMAND
    assert sanitize(-2.0) == -MAX_COMMAND
    assert sanitize(0.25) == 0.25


@pytest.mark.parametrize(
    ("line", "parsed"),
    [
        (b"x 1\n", ("kill", (True,))),
        (b"x 0\r\n", ("kill", (False,))),
        (b"d 0.532\n", ("depth", (0.532,))),
        (b"d -0.010\n", ("depth", (-0.01,))),
        (b"h 2 123456 3\n", ("heartbeat", (2, 123456, 3))),
        (b"d nan\n", None),
        (b"d 0.5[00:00:01.234,000] <err> pressure\n", None),
        (b"d\n", None),
        (b"x\n", None),
        (b"h 2 12\n", None),
        (b"i 0.1 0.2 0.3\n", None),
        (b"\n", None),
        (b"[00:00:01.000,000] <err> main: something\n", None),
    ],
)
def test_parse_line(line, parsed):
    assert parse_line(line) == parsed


class Board:
    """The board's end of a pseudo-terminal, reached through a symlink like /dev/maritime."""

    def __init__(self, link: Path):
        self.link = link
        self.master = -1
        self.buffer = b""
        self.plug()

    def plug(self):
        self.master, slave = os.openpty()
        target = os.ttyname(slave)
        os.close(slave)
        tmp = self.link.with_suffix(".new")
        tmp.unlink(missing_ok=True)
        tmp.symlink_to(target)
        tmp.replace(self.link)
        self.buffer = b""

    def unplug(self):
        os.close(self.master)
        self.master = -1

    def send(self, text: str):
        os.write(self.master, text.encode())

    def lines(self, timeout: float = 0.5) -> list[str]:
        """What the host wrote within `timeout`, as complete lines."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ready, _, _ = select.select([self.master], [], [], 0.02)
            if ready:
                try:
                    self.buffer += os.read(self.master, 4096)
                except OSError:
                    break
        *complete, self.buffer = self.buffer.split(b"\n")
        return [line.decode() for line in complete]

    def wait_for(self, predicate, timeout: float = 3.0) -> list[str]:
        seen = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            seen += self.lines(0.05)
            if predicate(seen):
                return seen
        raise AssertionError(f"not seen within {timeout} s; got {seen[-20:]}")


@pytest.fixture
def rig():
    rclpy.init()
    with tempfile.TemporaryDirectory() as directory:
        board = Board(Path(directory) / "maritime")
        namespace = f"/test_sub_low_{os.getpid()}"
        node = SubLow(
            namespace=namespace,
            parameter_overrides=[
                Parameter("device", value=str(board.link)),
                Parameter("serial_timeout", value=0.05),
                Parameter("reconnect_period", value=0.1),
                Parameter("command_timeout", value=0.5),
            ],
        )
        peer = rclpy.create_node("peer", namespace=namespace)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(peer)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        node.trigger_configure()
        node.trigger_activate()
        publisher = peer.create_publisher(ThrusterCommand, "control/thrusters", 10)
        kills = []
        peer.create_subscription(
            Bool,
            "kill_switch",
            lambda msg: kills.append(msg.data),
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )

        def command(values, age=0.0):
            msg = ThrusterCommand()
            stamp = peer.get_clock().now() - rclpy.duration.Duration(seconds=age)
            msg.header.stamp = stamp.to_msg()
            msg.command = [float(v) for v in values]
            publisher.publish(msg)

        # Connecting stops the thrusters first.
        assert "a 0" in board.wait_for(lambda seen: "a 0" in seen)
        # The subscription has to be matched before commands get through.
        deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.02)
        yield board, command, kills, node
        executor.shutdown()
        node.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()
        spinner.join(timeout=2.0)


def test_commands_are_eight_p_lines_until_the_board_says_protocol_2(rig):
    board, command, _, _ = rig
    command([0.1] * 8)
    seen = board.wait_for(lambda seen: any(line.startswith("p 7 ") for line in seen))
    assert [line for line in seen if line.startswith("p ")][:8] == [
        f"p {i} 0.100" for i in range(8)
    ]

    board.send("h 2 1000 0\n")
    time.sleep(0.2)
    command([0.2, -0.2, 0, 0, 0, 0, 0, 9.0])
    seen = board.wait_for(lambda seen: any(line.startswith("m ") for line in seen))
    assert "m 0.200 -0.200 0.000 0.000 0.000 0.000 0.000 0.600" in seen


def test_stale_commands_are_ignored_and_silence_stops_the_thrusters(rig):
    board, command, _, _ = rig
    command([0.3] * 8, age=2.0)
    assert not any(line.startswith("p ") for line in board.lines(0.3))

    command([0.3] * 8)
    board.wait_for(lambda seen: "p 0 0.300" in seen)
    # Nothing more: the watchdog stops them after command_timeout.
    seen = board.wait_for(lambda seen: "a 0" in seen, timeout=2.0)
    assert "a 0" in seen


def test_kill_switch_lines_are_published(rig):
    board, _, kills, _ = rig
    board.send("x 1\n")
    board.send("x 0\n")
    deadline = time.monotonic() + 3.0
    while kills[-2:] != [True, False] and time.monotonic() < deadline:
        time.sleep(0.02)
    assert kills[-2:] == [True, False]


def test_reconnects_after_the_link_drops(rig):
    board, command, _, node = rig
    board.send("h 2 5000 0\n")
    time.sleep(0.2)
    board.unplug()
    deadline = time.monotonic() + 3.0
    while node._serial is not None and time.monotonic() < deadline:
        time.sleep(0.02)
    assert node._serial is None

    board.plug()
    # Reopened, thrusters stopped first, and back to protocol 1 until the
    # board's heartbeat says otherwise.
    board.wait_for(lambda seen: "a 0" in seen)
    command([0.1] * 8)
    board.wait_for(lambda seen: "p 0 0.100" in seen)
