#!/usr/bin/env bash
# Setup on a fresh machine. Safe to re-run:
#  - installs pixi if it is missing
#  - fetches the submodules
#  - creates the environment from pixi.lock
#  - builds the workspace
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

echo "Fetching submodules"
git submodule update --init --recursive

echo "Creating the pixi environment from pixi.lock"
pixi install --frozen

echo "Building the workspace"
pixi run build
