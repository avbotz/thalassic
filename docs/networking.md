# Networking

The Jetson, the DVL, the GigE down camera, and the router all connect to a switch inside the MEB.

| Device | Address | Set by |
|---|---|---|
| Jetson AGX Orin | `192.168.5.245/24` | `deploy/jetson/files/60-thalassic-vehicle.yaml`, installed as a netplan file |
| Water Linked DVL A50 | `192.168.5.250` | Configured on the DVL itself; the driver is pointed at it by `sub_bringup/config/marlin_v3.yaml` |
| FLIR Blackfly S (down camera) | DHCP | Should be static too — open item |
| Laptops and anything else temporary | DHCP, `192.168.5.100`–`192.168.5.240` | The router |

The vehicle network has no route to the internet, and the netplan file deliberately declares no gateway and no nameservers: a default route on this interface would break the internet connection of a laptop plugged into the switch.

`192.168.5.0/24` rather than the `192.168.0.0/24` this used to be on: that is the default subnet of most consumer routers, so a laptop that has ever been on a home network already has routes for it. See [Decision 11](decisions.md).

## Changing the subnet

Two of the three addresses live on the devices, not in this repository, so the order matters — get it wrong and the sub is unreachable until it is on a bench with a serial console:

1. **The router**: LAN address and DHCP pool.
2. **The DVL**, through its web interface. It answers at its old address until you change it, so do this from a laptop that can still reach both.
3. **The Jetson**: `sudo deploy/jetson/setup.sh --apply-network`, from the console rather than over SSH — the link drops as it is applied.

The down camera is on DHCP and picks up the change by itself.

## MTU

`end0` runs at MTU 9000. GigE Vision sends a frame as a burst of packets, and jumbo frames are what let the down camera hold its frame rate. The switch and the camera have to agree; a 1500-byte hop anywhere in the path shows up as incomplete frames rather than as an error.

The driver still requests 1500-byte GigE packets (`gev_scps_packet_size` in `sub_bringup/config/blackfly_down.yaml`) — raising it to 9000 now that the interface allows it has not been tested end to end.

## Time

The Jetson serves NTP to the DVL and the camera (`deploy/jetson/files/thalassic-ntp.conf`). It has no battery-backed clock and no upstream server, so it advertises stratum 10: the sensors agree with the Jetson, not with UTC, which is all the timestamps need. `chronyc clients` on the Jetson lists who has asked.

## ROS 2 discovery

Discovery is separate from addressing and is set by `THALASSIC_DDS_PROFILE` — see [DDS profiles](setup.md#dds-profiles). The vehicle uses subnet discovery on domain 42; a laptop opts in with `THALASSIC_DDS_PROFILE=vehicle pixi shell`.
