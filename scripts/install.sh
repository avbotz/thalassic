#!/usr/bin/env bash
# Setup on a fresh machine. Safe to re-run:
#  - installs pixi if it is missing
#  - fetches the submodules
#  - creates the environment from pixi.lock (on the Jetson, its own: jetson)
#  - installs the git hooks (not on the Jetson)
#  - builds the workspace
#  - downloads the model weights in weights/manifest.toml
#
# Jetson configuration is described in `deploy/jetson`.

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

if ! command -v pixi >/dev/null 2>&1; then
  if [ ! -x "$HOME/.pixi/bin/pixi" ]; then
    echo "==> Installing pixi to ~/.pixi/bin"
    curl -fsSL https://pixi.sh/install.sh | sh
  fi
  export PATH="$HOME/.pixi/bin:$PATH"
fi

env=default
if [ -e /etc/nv_tegra_release ]; then
  env=jetson
fi

echo "Fetching submodules"
git submodule update --init --recursive

echo "Creating the pixi environment '$env' from pixi.lock"
pixi install --frozen -e "$env"

# The hooks run in the default environment, which the Jetson does not have.
if [ "$env" = default ]; then
  echo "Installing the git hooks"
  pixi run --frozen pre-commit install
fi

echo "Building the workspace"
pixi run --frozen -e "$env" build

echo "Downloading the model weights"
pixi run --frozen -e "$env" fetch-weights
