# Sourced by pixi on every `pixi run` / `pixi shell` (see [activation] in pixi.toml).

if [ -n "$PIXI_PROJECT_ROOT" ]; then
    _thalassic_root="$PIXI_PROJECT_ROOT"
elif [ -n "$ZSH_VERSION" ]; then
    _thalassic_root="$(cd "$(dirname "${(%):-%x}")/.." && pwd)"
elif [ -n "$BASH_VERSION" ]; then
    _thalassic_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

if [ -z "$CONDA_PREFIX" ]; then
    echo "thalassic: run 'pixi shell' (or 'pixi run <task>') instead of sourcing this file." >&2
    unset _thalassic_root
    return 0
fi

# source colcon workspace overlay if it exists
if [ -n "$ZSH_VERSION" ]; then
    _thalassic_overlay="$_thalassic_root/install/setup.zsh"
elif [ -n "$BASH_VERSION" ]; then
    _thalassic_overlay="$_thalassic_root/install/setup.bash"
else
    _thalassic_overlay="$_thalassic_root/install/setup.sh"
fi
if [ -f "$_thalassic_overlay" ]; then
    . "$_thalassic_overlay"
fi

# DDS profile
# dev (default): discovery limited to this machine, computers on the same network will not conflict
# vehicle:       the Jetson (selected system-wide by /etc/profile.d/thalassic.sh) and any laptop plugged into the sub.
#                Discovery spans the subnet on a fixed domain.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if [ "${THALASSIC_DDS_PROFILE:-dev}" = vehicle ]; then
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    export ROS_DOMAIN_ID="${THALASSIC_ROS_DOMAIN_ID:-42}"
else
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    export ROS_DOMAIN_ID="${THALASSIC_ROS_DOMAIN_ID:-$(( $(id -u) % 100 ))}"
fi

# FLIR camera setup
_thalassic_cti="$_thalassic_root/install/lib/spinnaker-gentl/Spinnaker_GenTL.cti"
if [ -f "$_thalassic_cti" ]; then
    export SPINNAKER_GENTL64_CTI="$_thalassic_cti"
fi

# conda provides SDL2 using a shim that passes calls to SDL3
# SDL2 uses x11 by default, while SDL3 uses wayland
# When using Wayland, the title bar disappears.
# Switches back to x11. No issues with this approach, can be removed when Stonefish migrates to SDL3.
export SDL_VIDEODRIVER=x11

unset _thalassic_root _thalassic_overlay _thalassic_cti
