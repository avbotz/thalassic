#!/usr/bin/env bash
# Vehicle OS configuration for the Jetson AGX Orin.
#
#   sudo deploy/jetson/setup.sh
#
# Re-run after reflashing or after editing anything under deploy/udev or deploy/jetson/files.
#
# Setting applied survive reboots.

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "run with sudo" >&2
    exit 1
fi

VEHICLE_USER="avbotz"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FILES="$REPO_ROOT/deploy/jetson/files"

if ! id "$VEHICLE_USER" >/dev/null 2>&1; then
    echo "user '$VEHICLE_USER' does not exist (it is created when the Jetson is flashed)" >&2
    exit 1
fi

warn() { echo "  ! $*" >&2; }

echo "==> Groups for $VEHICLE_USER"
# flirimaging owns the FLIR device nodes, dialout the serial MCUs, video the
# USB cameras. Without these the stack only runs as root.
groupadd -f flirimaging
usermod -aG flirimaging,dialout,video "$VEHICLE_USER"

echo "==> udev rules"
install -m 0644 "$REPO_ROOT"/deploy/udev/*.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger

echo "==> Socket buffers (GigE down camera)"
install -m 0644 "$FILES/60-thalassic-gige.conf" /etc/sysctl.d/60-thalassic-gige.conf
sysctl --system >/dev/null

echo "==> DDS profile for login shells"
install -m 0644 "$FILES/thalassic.sh" /etc/profile.d/thalassic.sh

echo "==> NTP server for the DVL and the down camera"
if [ -d /etc/chrony/conf.d ]; then
    install -m 0644 "$FILES/thalassic-ntp.conf" /etc/chrony/conf.d/thalassic.conf
    systemctl enable chrony
    systemctl restart chrony || warn "could not start chrony"
else
    warn "chrony is not installed, so the vehicle serves no time."
    warn "systemd-timesyncd cannot do this - it is a client only. Install it with:"
    warn "    sudo apt install chrony"
    warn "then re-run this script. Skipping for now."
fi

echo "==> Jetson power mode"
if [ -x /usr/sbin/nvpmodel ]; then
    install -m 0644 "$FILES/thalassic-power-mode.service" \
        /etc/systemd/system/thalassic-power-mode.service
    systemctl daemon-reload
    systemctl enable --now thalassic-power-mode.service
    /usr/sbin/nvpmodel -q | sed 's/^/    /'
else
    warn "no /usr/sbin/nvpmodel: not a Jetson, skipping the power mode unit."
fi

echo "==> Fan at full speed"
# nvfancontrol would ramp the fan back down from its thermal curve, so it goes
# before ours is started.
if systemctl is-enabled nvfancontrol.service >/dev/null 2>&1; then
    systemctl disable --now nvfancontrol.service
    echo "    nvfancontrol disabled"
fi
install -m 0755 "$FILES/thalassic-fan-max" /usr/local/sbin/thalassic-fan-max
install -m 0644 "$FILES/thalassic-fan.service" /etc/systemd/system/thalassic-fan.service
systemctl daemon-reload
if systemctl enable --now thalassic-fan.service; then
    systemctl show -p ExecMainStatus --value thalassic-fan.service |
        sed 's/^/    fan unit exit status: /'
else
    warn "the fan unit did not start; 'systemctl status thalassic-fan' says why."
fi

echo "==> Static address and jumbo frames"
NETPLAN_DEST=/etc/netplan/60-thalassic-vehicle.yaml
if ! command -v netplan >/dev/null 2>&1; then
    warn "no netplan on this machine: skipping the address and MTU."
else
    # 0600 keeps netplan from warning about world-readable configuration.
    install -m 0600 "$FILES/60-thalassic-vehicle.yaml" "$NETPLAN_DEST"
    # A broken file here costs the vehicle its network on the next boot, so
    # back out rather than leave one installed.
    if ! netplan generate; then
        rm -f "$NETPLAN_DEST"
        netplan generate || true
        warn "netplan rejected 60-thalassic-vehicle.yaml (error above); removed it"
        warn "again so the vehicle keeps the network it has."
        exit 1
    fi
    netplan apply
fi

cat <<EOF

Done. Remaining manual steps:
  * Log out and back in (or reboot) so the new groups and the DDS profile apply.
  * After 'pixi install', run deploy/jetson/link_jetpack_python.sh as $VEHICLE_USER
    to expose JetPack's tensorrt/cuda modules to the environment.
EOF
