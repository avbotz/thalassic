# Low-Level Board

`sub_low` is the ROS2 lifecycle bridge to the low-level board. The board is reached over USB CDC ECM, which appears on Linux as a normal Ethernet interface. The node connects to the board over TCP, forwards thruster commands, and publishes kill-switch state.

## Network Setup

Expected USB CDC ECM addresses:

| Endpoint | Address |
|---|---|
| Host computer / Jetson | `192.168.7.1/24` |
| Low-level board | `192.168.7.2` |
| Board TCP port | `7777` |

After plugging in the board, verify that Linux created the USB network interface:

```bash
ip -brief addr
```

Look for an interface like `usb0` or `enx...` with `192.168.7.1/24`. If the interface exists but has no address, assign one manually:

```bash
sudo ip addr add 192.168.7.1/24 dev <interface>
sudo ip link set <interface> up
```

Replace `<interface>` with the actual USB ECM interface name.

Check board reachability:

```bash
ping 192.168.7.2
```

The board should also answer the low-level ping command:

```bash
printf 'p\n' | nc -w 2 192.168.7.2 7777
```

Expected response:

```text
pong
```

## Run

Start the lifecycle node:

```bash
ros2 run sub_low sub_low --ros-args \
  -p ecm_host:=192.168.7.2 \
  -p ecm_port:=7777
```

The defaults are already `192.168.7.2:7777`, so this is equivalent:

```bash
ros2 run sub_low sub_low
```

## Lifecycle Init

`sub_low` is a lifecycle node. Starting the executable creates the node, but it does not forward thruster commands until it is configured and activated.

In another terminal:

```bash
ros2 lifecycle set /sub_low configure
ros2 lifecycle set /sub_low activate
```

Lifecycle behavior:

| Transition | Behavior |
|---|---|
| `configure` | Connects to the board, creates publishers/subscribers, starts the poll timer |
| `activate` | Enables kill-switch publishing, board polling, and thruster forwarding |
| `deactivate` | Sends zero commands to all thrusters and stops forwarding/polling |
| `cleanup` | Closes the board connection and clears ROS entities |
| `shutdown` | Sends zero commands and closes the board connection |

Check state:

```bash
ros2 lifecycle get /sub_low
```

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `ecm_host` | `192.168.7.2` | Low-level board IP address on the USB CDC ECM network |
| `ecm_port` | `7777` | TCP port served by the low-level board |
| `ecm_connect_timeout_ms` | `1000` | TCP connect timeout during lifecycle `configure` |

## ROS Interfaces

Subscriptions:

| Topic | Type | Behavior |
|---|---|---|
| `control/thruster_0` through `control/thruster_7` | `std_msgs/msg/Float64` | Sends `t <index> <value>\n` to the board while active |

Publishers:

| Topic | Type | Behavior |
|---|---|---|
| `kill_switch` | `std_msgs/msg/Bool` | Publishes board lines of the form `x <0-or-1>` |

## Verbose Logging

Run with debug logs:

```bash
ros2 run sub_low sub_low --ros-args --log-level DEBUG
```

Or target this node only:

```bash
ros2 run sub_low sub_low --ros-args --log-level sub_low:=debug
```

Parameters and logging can be combined:

```bash
ros2 run sub_low sub_low --ros-args \
  -p ecm_host:=192.168.7.2 \
  -p ecm_port:=7777 \
  --log-level sub_low:=debug
```

## Troubleshooting

If `configure` fails, first verify the board outside ROS:

```bash
ip -brief addr
ping 192.168.7.2
printf 'p\n' | nc -w 2 192.168.7.2 7777
```

If there is no USB network interface, check USB enumeration and driver binding:

```bash
lsusb
lsusb -t
```

The board should appear as a CDC Ethernet device bound to a Linux USB networking driver such as `cdc_ether`. If `lsusb -t` shows `Driver=[none]`, Linux has enumerated the USB device but has not created the network interface yet.
