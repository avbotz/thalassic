#!/usr/bin/env bash
# Vehicle OS configuration for the Jetson AGX Orin. Run as root.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FILES="$REPO_ROOT/deploy/jetson/files"

step() { echo "==> $*"; }
die()  { echo " ! $*" >&2; exit 1; }
trap 'echo " ! failed during the step above; fix the error and rerun" >&2' ERR

[ "$(id -u)" -eq 0 ]         || die "run with sudo"
[ -e /etc/nv_tegra_release ] || die "not a Jetson (no /etc/nv_tegra_release)"

step "Packages"
if ! command -v chronyd >/dev/null; then
  apt-get update
  apt-get install -y chrony
fi

step "Groups for $USER"
usermod -aG dialout,video "$USER"   # dialout: serial MCUs, video: USB cameras

step "udev rules"
install -m 0644 "$REPO_ROOT"/deploy/udev/*.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger

step "Socket buffers (GigE down camera)"
install -m 0644 "$FILES/60-thalassic-gige.conf" /etc/sysctl.d/
sysctl --system >/dev/null

step "DDS profile for login shells"
install -m 0644 "$FILES/thalassic.sh" /etc/profile.d/

step "NTP server"
install -m 0644 "$FILES/thalassic-ntp.conf" /etc/chrony/conf.d/thalassic.conf
systemctl enable chrony
systemctl restart chrony

step "Power mode and fan"
systemctl disable --now nvfancontrol.service
install -m 0755 "$FILES/thalassic-fan-max" /usr/local/sbin/
install -m 0644 "$FILES/thalassic-power-mode.service" "$FILES/thalassic-fan.service" /etc/systemd/system/
systemctl daemon-reload
systemctl enable thalassic-power-mode.service thalassic-fan.service
systemctl restart thalassic-power-mode.service thalassic-fan.service
nvpmodel -q | sed 's/^/    /'

step "Static address and jumbo frames"
# Validate netplan config before installing it.
stage="$(mktemp -d)"
trap 'rm -rf "$stage"' EXIT
mkdir "$stage/etc"
cp -a /etc/netplan "$stage/etc/"
install -m 0600 "$FILES/60-thalassic-vehicle.yaml" "$stage/etc/netplan/"
netplan generate --root-dir "$stage" || die "netplan rejected 60-thalassic-vehicle.yaml (error above); nothing was changed"
install -m 0600 "$FILES/60-thalassic-vehicle.yaml" /etc/netplan/   # 0600 or netplan warns
netplan apply

cat <<EOF

Done. Remaining manual steps:
  * Log out and back in (or reboot) so the new groups and the DDS profile apply.
  * Once the weights are fetched: 'pixi run build-engines'.
EOF