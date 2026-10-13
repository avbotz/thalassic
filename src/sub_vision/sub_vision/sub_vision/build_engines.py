"""Build a TensorRT engine for each ONNX model in weights/ that has none, and check it.

An engine is specific to the device, the TensorRT version and the export it was built from
(``backends.engine_path`` names it by all three), so this runs on the Jetson after
`pixi run fetch-weights` and after a lockfile update that changes TensorRT. Each build takes
minutes. Each new engine is then run against ONNX Runtime on synthetic frames
(:mod:`check_engines`); a disagreement fails this command, but the engine is kept for
`check_engines` to look at.

    build_engines                  # every <model>.onnx in weights/
    build_engines torp gate        # just these models
"""

import argparse
import os
import sys
from pathlib import Path

from sub_vision import backends, check_engines


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
    frames = check_engines.synthetic_frames()
    failed = False
    for path in paths or sorted(args.model_dir.glob("*.onnx")):
        try:
            if os.path.exists(backends.engine_path(str(path))):
                print(f"ok           {path.name}")
                continue
            print(f"building     {path.name}", flush=True)
            backends.build_engine(str(path))
            problems, summary = check_engines.check_model(str(path), frames)
        except Exception as error:  # report every model, not just the first failure
            print(f"failed       {path.name}: {error}", file=sys.stderr)
            failed = True
            continue
        check_engines.report(path.name, problems, summary)
        failed = failed or bool(problems)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
