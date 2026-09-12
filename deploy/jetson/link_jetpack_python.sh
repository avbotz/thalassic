#!/usr/bin/env bash
# Expose JetPack's GPU Python modules (tensorrt, cuda-python) to the pixi environment on the Jetson. Run as the vehicle user after `pixi install`.
#
# JetPack installs these bindings into the system interpreter only, and they are not available from conda-forge or PyPI for the Orin.
# A .pth file appends the system dist-packages directory to sys.path *after* the environment's own site-packages.
#
# This only works when both interpreters share the same minor version, which is why pixi.toml pins python = 3.12.* (Ubuntu 24.04's interpreter).
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."

pixi run python - <<'PY'
import subprocess, sys, sysconfig
from pathlib import Path

env = sysconfig.get_python_version()
jetpack = subprocess.run(
    ["/usr/bin/python3", "-c", "import sysconfig; print(sysconfig.get_python_version())"],
    capture_output=True, text=True, check=True).stdout.strip()
if env != jetpack:
    sys.exit(f"Python mismatch: environment {env} vs JetPack {jetpack}.\n"
             "JetPack's compiled modules cannot be used; install NVIDIA wheels via pixi.toml instead.")

pth = Path(sysconfig.get_paths()["purelib"]) / "zz-jetpack-dist-packages.pth"
pth.write_text("/usr/lib/python3/dist-packages\n")
print(f"wrote {pth}")
PY

# A fresh interpreter is needed for the .pth file to take effect.
pixi run python - <<'PY'
import importlib
for mod in ("tensorrt", "cuda"):
    try:
        m = importlib.import_module(mod)
        print(f"{mod}: ok ({getattr(m, '__version__', 'no version attr')})")
    except Exception as exc:  # noqa: BLE001
        print(f"{mod}: NOT importable ({exc})")
PY
