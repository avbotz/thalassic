#!/usr/bin/env bash
# Per-boot setting for the GigE Vision down camera on the Jetson: the jumbo
# frame MTU is not yet declared in netplan, so set it after each boot.
# (Socket buffer sizes are persisted by deploy/jetson/setup.sh.)
#
#   sudo deploy/jetson/gige-camera-runtime.sh [interface]   (default: end0)
set -euo pipefail
IFACE="${1:-end0}"
ip link set dev "$IFACE" mtu 9000
echo "$IFACE: mtu 9000"
