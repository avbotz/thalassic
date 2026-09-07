#!/usr/bin/env bash
# Vehicle-only OS configuration for the Jetson AGX Orin. Idempotent; re-run
# after reflashing or after changing anything under udev/ or deploy/jetson/.
#
#   sudo deploy/jetson/setup.sh [vehicle-user]     (default user: avbotz)
#
# What it configures and why (see docs/deployment.md):
#   * "flirimaging" group + udev rules so the FLIR camera is usable without root
#     (spinnaker_camera_driver builds against a downloaded SDK; nothing else
#     from the Spinnaker package is installed).
#   * usbfs memory limit of 1000 MB for USB3 Vision cameras, persisted with
#     systemd-tmpfiles instead of a bootloader edit (the Jetson boots with
#     extlinux, not GRUB).
#   * Socket buffer sizes for the GigE camera, persisted under /etc/sysctl.d.
#   * All udev rules from udev/ (Pico, IMU, front camera, FLIR).
#   * THALASSIC_DDS_PROFILE=vehicle for every login shell, so the ROS graph on
#     the sub uses subnet discovery and the fixed vehicle domain.
#
# Nothing here starts the ROS stack. It is always launched by hand.
set -euo pipefail
umask 022   # files written below must stay world-readable

if [ "$(id -u)" -ne 0 ]; then
    echo "run with sudo" >&2
    exit 1
fi

VEHICLE_USER="${1:-avbotz}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if ! id "$VEHICLE_USER" >/dev/null 2>&1; then
    echo "user '$VEHICLE_USER' does not exist" >&2
    exit 1
fi

echo "==> Groups for $VEHICLE_USER"
groupadd -f flirimaging
usermod -aG flirimaging,dialout,video "$VEHICLE_USER"

echo "==> udev rules"
install -m 0644 "$REPO_ROOT"/deploy/udev/*.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger

echo "==> usbfs memory limit (USB3 Vision cameras)"
cat > /etc/tmpfiles.d/thalassic-usbfs.conf <<'EOF'
# Raise the usbfs buffer limit for USB3 Vision cameras (FLIR Spinnaker).
w /sys/module/usbcore/parameters/usbfs_memory_mb - - - - 1000
EOF
systemd-tmpfiles --create /etc/tmpfiles.d/thalassic-usbfs.conf || true

echo "==> GigE camera socket buffers"
cat > /etc/sysctl.d/60-thalassic-gige.conf <<'EOF'
# Larger receive/transmit buffers for the GigE Vision down camera.
net.core.rmem_max = 10485760
net.core.wmem_max = 10485760
EOF
sysctl --system >/dev/null

echo "==> DDS profile for login shells"
cat > /etc/profile.d/thalassic.sh <<'EOF'
# Installed by thalassic/deploy/jetson/setup.sh: the vehicle uses the
# subnet-wide DDS profile (see thalassic scripts/setup.sh).
export THALASSIC_DDS_PROFILE=vehicle
EOF

cat <<EOF

Done. Remaining manual items:
  * Log out and back in (or reboot) so the new groups and profile apply.
  * Jumbo frames on the camera interface are still set per boot by
    deploy/jetson/gige-camera-runtime.sh until the interface is declared in netplan.
  * After 'pixi install', run deploy/jetson/link_jetpack_python.sh as $VEHICLE_USER
    to expose JetPack's tensorrt/cuda modules to the environment.
EOF
