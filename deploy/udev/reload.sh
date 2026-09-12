#!/usr/bin/env bash

set -euo pipefail
sudo udevadm control --reload-rules
sudo udevadm trigger
