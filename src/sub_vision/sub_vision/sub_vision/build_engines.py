"""Build a TensorRT engine for each ONNX model in weights/ that has none.

An engine is specific to the device, the TensorRT version and the export it was built from
(``backends.engine_path`` names it by all three), so this runs on the Jetson after
`pixi run fetch-weights` and after a JetPack upgrade. Each build takes minutes.

    build_engines                  # every <model>.onnx in weights/
    build_engines torp gate        # just these models
"""

import argparse
import os
import sys
from pathlib import Path

from sub_vision import backends


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("models", nargs="*", help="models, as weights/<model>.onnx (default: all)")
    parser.add_argument(
        "--model-dir",
        type=Path,
        default=Path(os.environ.get("PIXI_PROJECT_ROOT", ".")) / "weights",
        help="where the ONNX models are (default: weights/ at the workspace root)",
    )
    args = parser.parse_args()

    paths = [args.model_dir / f"{model}.onnx" for model in args.models]
    failed = False
    for path in paths or sorted(args.model_dir.glob("*.onnx")):
        try:
            if os.path.exists(backends.engine_path(str(path))):
                print(f"ok           {path.name}")
                continue
            print(f"building     {path.name}", flush=True)
            backends.build_engine(str(path))
        except Exception as error:  # report every model, not just the first failure
            print(f"failed       {path.name}: {error}", file=sys.stderr)
            failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
