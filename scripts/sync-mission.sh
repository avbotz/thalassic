#!/usr/bin/env bash
# Copy the mission trees to the vehicle once the laptop's sub_mission tests pass
# on them.
#
#   pixi run sync-mission                  to avbotz@192.168.5.245
#   pixi run sync-mission user@host        to another vehicle
#
# Run it with the vehicle killed. Under `restart`, each release starts a new
# mission process, which reads the XML from disk, so the next release runs the
# synced trees; a stack launched without `restart` has to be relaunched. The
# install space links to src (symlink-install, colcon_defaults.yaml), so an
# edited file needs no build; a new file needs its link, and this builds
# sub_mission on the vehicle to make it.
#
# The tests load every mission against this checkout's C++ nodes, so they only
# vouch for the vehicle while its sub_mission sources are the same: the script
# stops before copying anything if they are not.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

VEHICLE="${1:-avbotz@192.168.5.245}"
REMOTE=thalassic  # the vehicle's checkout, from its user's home (docs/deployment.md)

echo "==> Testing sub_mission"
colcon build --packages-select sub_mission
# OFF: with the tether in, a vehicle DDS profile would put the test's nodes on
# the vehicle's graph.
if ! ROS_AUTOMATIC_DISCOVERY_RANGE=OFF colcon test --packages-select sub_mission --return-code-on-test-failure; then
  colcon test-result --test-result-base build/sub_mission --verbose || true
  echo "sync-mission: tests failed, nothing copied" >&2
  exit 1
fi

# One connection, and one password prompt, for every ssh and rsync below.
control=$(mktemp -d)
trap 'ssh -o ControlPath="$control/ssh" -O exit "$VEHICLE" 2>/dev/null || true; rm -rf "$control"' EXIT
SSH=(ssh -o ControlMaster=auto -o ControlPath="$control/ssh" -o ControlPersist=60 -o ConnectTimeout=5)

# The C++ the tests ran against, as one hash; run here and on the vehicle.
sources_hash='cd src/sub_mission && find src include -type f -print0 | LC_ALL=C sort -z | xargs -0 sha256sum | sha256sum'
if [ "$(bash -c "$sources_hash")" != "$("${SSH[@]}" "$VEHICLE" "cd $REMOTE && $sources_hash")" ]; then
  echo "sync-mission: sub_mission's C++ on $VEHICLE differs from this checkout's, so these tests do not" \
    "cover what it runs. Bring the vehicle's code in line and build it there first; nothing copied" >&2
  exit 1
fi

echo "==> Copying src/sub_mission/resources to $VEHICLE"
# --delete: what the tests loaded is all there is, so no tree left over on the
# vehicle can replace a tested one by ID. Contents alone decide what is sent
# (--checksum, no -t), so a file only a checkout touched is not listed.
changes=$(rsync -rl --checksum --delete --itemize-changes -e "${SSH[*]}" \
  src/sub_mission/resources/ "$VEHICLE:$REMOTE/src/sub_mission/resources/")
if [ -z "$changes" ]; then
  echo "already up to date"
else
  printf '%s\n' "$changes"
fi

if grep -q '^<f+++++++++ ' <<<"$changes"; then
  echo "==> Building sub_mission on $VEHICLE to link the new files"
  # Not a login shell, so pixi is not on the PATH yet.
  "${SSH[@]}" "$VEHICLE" "cd $REMOTE && PATH=\$HOME/.pixi/bin:\$PATH pixi run --frozen colcon build --packages-select sub_mission"
fi

echo "sync-mission: done; under restart, the next release runs these trees"
