"""Run the gate task's ONNX model on a still image."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from sub_vision.model_manager import TASK_MODELS


def _default_model_path() -> Path:
    """The model sub_vision runs for the gate task: in weights/ at the workspace root."""
    root = Path(os.environ.get("PIXI_PROJECT_ROOT", "."))
    return root / "weights" / f"{TASK_MODELS.get('gate', 'gate')}.onnx"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="Path to a gate image.")
    parser.add_argument(
        "--model",
        type=Path,
        default=_default_model_path(),
        help="Path to an ONNX model. Defaults to the gate task's, in weights/.",
    )
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold.")
    parser.add_argument("--imgsz", type=int, default=640, help="Inference image size.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("runs/gate_model_test"),
        help="Directory where annotated predictions are saved.",
    )
    parser.add_argument(
        "--name",
        default="predict",
        help="Run name under output-dir.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    image = args.image.expanduser().resolve()
    model_path = args.model.expanduser().resolve()
    if not image.is_file():
        raise FileNotFoundError(f"image does not exist: {image}")
    if not model_path.is_file():
        raise FileNotFoundError(f"model does not exist: {model_path}")

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            "Missing dependency: ultralytics (in pixi.toml; run this inside the pixi environment)"
        ) from exc

    model = YOLO(str(model_path))
    results = model.predict(
        source=str(image),
        imgsz=args.imgsz,
        conf=args.conf,
        save=True,
        project=str(args.output_dir),
        name=args.name,
        exist_ok=True,
        retina_masks=True,
    )

    for result in results:
        names = result.names
        boxes = [] if result.boxes is None else result.boxes
        masks = 0 if result.masks is None else len(result.masks)
        print(f"image: {result.path}")
        print(f"detections: {len(boxes)}")
        print(f"masks: {masks}")
        for index, box in enumerate(boxes):
            cls_id = int(box.cls[0])
            score = float(box.conf[0])
            x0, y0, x1, y1 = [float(value) for value in box.xyxy[0]]
            print(
                f"{index}: class={cls_id} ({names.get(cls_id, cls_id)}) "
                f"score={score:.3f} bbox=({x0:.1f},{y0:.1f},{x1:.1f},{y1:.1f})"
            )

    print(f"annotated output: {args.output_dir / args.name}")


if __name__ == "__main__":
    main()
