if [ -e /etc/nv_tegra_release ] && [ "$PIXI_ENVIRONMENT_NAME" != jetson ]; then
  echo "This is a Jetson. Use 'pixi run -e jetson <task>' or 'pixi shell -e jetson'." >&2
fi

# Source the colcon workspace overlay if it has been built
if [ -n "$ZSH_VERSION" ]; then
  _thalassic_sh=zsh
elif [ -n "$BASH_VERSION" ]; then
  _thalassic_sh=bash
else
  _thalassic_sh=sh
fi
_thalassic_overlay="$PIXI_PROJECT_ROOT/install/setup.$_thalassic_sh"
if [ -f "$_thalassic_overlay" ]; then
  . "$_thalassic_overlay"
fi

unset _thalassic_sh _thalassic_overlay
