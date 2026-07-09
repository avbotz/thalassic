#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

ROS_WS_DIR="$(pwd)"
DIST=jazzy
JOBS="$(nproc)"

TMP_ROOT="$(mktemp -d)"
trap 'rm -rf "$TMP_ROOT"' EXIT

sudo apt-get update
sudo apt-get install -y software-properties-common curl ca-certificates gnupg lsb-release
sudo add-apt-repository -y universe

# ROS 2 apt source
ROS_APT_SOURCE_VERSION="$(curl -fsSL \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
UBUNTU_CODENAME="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"   # works on Ubuntu derivatives
curl -fsSL -o "$TMP_ROOT/ros2-apt-source.deb" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb"
sudo dpkg -i "$TMP_ROOT/ros2-apt-source.deb"

# Install dependencies
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git libeigen3-dev \
  pkg-config python3-pip python3-venv wget \
  python3-colcon-common-extensions python3-rosdep \
  ros-"${DIST}"-desktop ros-"${DIST}"-depthai-ros-v3 \
  libglm-dev libsdl2-dev libfreetype6-dev unzip

python3 -m venv .venv --system-site-packages
source .venv/bin/activate
python3 -m pip install torch kornia onnxruntime

git submodule update --init --recursive

# Install Stonefish
STONEFISH_DIR="$TMP_ROOT/stonefish"
git clone --depth 1 --branch fixes-merged https://github.com/kethan1/stonefish "$STONEFISH_DIR"
cmake -S "$STONEFISH_DIR" -B "$STONEFISH_DIR/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$STONEFISH_DIR/build" -j"$JOBS"
sudo cmake --install "$STONEFISH_DIR/build"

# Install Spinnaker SDK
ARCH="$(dpkg --print-architecture)"
SPIN_DIR="$TMP_ROOT/spinnaker"
mkdir -p "$SPIN_DIR"

python3 -m pip install gdown
gdown "https://drive.google.com/file/d/1AQnH7Sm7h4AmqqT7L2I_35XCxWarIhic/view" -O "$SPIN_DIR/spinnaker-4.4.0.99-24.04.zip"

unzip -q "$SPIN_DIR/spinnaker-4.4.0.99-24.04.zip" -d "$SPIN_DIR"
# named x64_noble but folder contains both arm and x64 packages
cd "$SPIN_DIR/x64_noble"
tar -xzf "spinnaker-4.4.0.99-noble-${ARCH}-pkg.tar.gz"
cd "spinnaker-4.4.0.99-noble-${ARCH}"
if [[ "$ARCH" == "arm64" ]]; then
  sudo sh install_spinnaker_arm.sh
else
  sudo sh install_spinnaker.sh
fi

cd "$ROS_WS_DIR"

set +u; source setup.sh; set -u  # references unbound variables

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install -y -i --from-paths src --skip-keys="pcl"

colcon build
