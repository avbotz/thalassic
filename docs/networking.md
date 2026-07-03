# Networking

The Jetson, DVL, and down camera are interconnected via Ethernet.

The Jetson, DVL, down camera, and Router connect to a switch inside of the meb.

The router should have DHCP enable and should assign IP addresses from 192.168.0.100 to 192.168.0.240.

The Jetson is configured to have the static IP of 192.168.0.245. The DVL is configured to have the static IP of 192.168.0.250.

The down camera currently uses DHCP to obtain an IP address, though it should be changed.

Any temporary connected devices for debugging and testing (laptops, etc.) should obtain an IP address via DHCP.
