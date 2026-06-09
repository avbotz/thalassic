import os

import numpy as np

MatLike = np.typing.NDArray[np.uint8]

# Network input is square; YOLOv10 default. Exposed via param on the node.
DEFAULT_INPUT_SIZE = 640


class UltralyticsBackend:
    """A loaded YOLO detector (Ultralytics API) that turns a BGR image into detections.

    ``infer`` returns an ``(N, 6)`` float32 array of
    ``[x1, y1, x2, y2, score, class_id]`` in original-image pixel coordinates.
    Ultralytics handles letterboxing, the (NMS-free YOLOv10) decode, and scaling
    boxes back to original-image pixels internally.
    """

    name = "ultralytics"

    def __init__(self, pt_path: str, device: str, input_size: int, conf_threshold: float):
        from ultralytics import YOLO

        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self._model = YOLO(pt_path)
        self._device = device

    def infer(self, image_bgr: np.ndarray) -> np.ndarray:
        results = self._model.predict(
            image_bgr,
            imgsz=self.input_size,
            conf=self.conf_threshold,
            device=self._device,
            verbose=False,
        )
        boxes = results[0].boxes
        if boxes is None or boxes.shape[0] == 0:
            return np.empty((0, 6), dtype=np.float32)
        xyxy = boxes.xyxy.cpu().numpy()
        conf = boxes.conf.cpu().numpy().reshape(-1, 1)
        cls = boxes.cls.cpu().numpy().reshape(-1, 1)
        return np.hstack([xyxy, conf, cls]).astype(np.float32)

    def close(self) -> None:
        """Release resources. No-op for the Ultralytics backend."""


# --------------------------------------------------------------------------- #
# Device detection and backend construction
# --------------------------------------------------------------------------- #
def detect_device(override: str) -> str:
    """Resolve the compute device. ``override`` is ``auto`` / ``cpu`` / ``cuda``."""
    if override != "auto":
        return override
    try:
        import torch

        return "cuda" if torch.cuda.is_available() else "cpu"
    except Exception:
        return "cpu"


def build_backend(
    task: str,
    model_dir: str,
    device_pref: str,
    input_size: int,
    conf_threshold: float,
    log,
) -> UltralyticsBackend:
    """Load the Ultralytics detector for ``task`` from ``<model_dir>/<task>.pt``.

    The ``.pt`` is the source of truth; Ultralytics uses CUDA automatically when
    the resolved device is ``cuda`` and a GPU build of torch is installed.
    """
    pt_path = os.path.join(model_dir, f"{task}.pt")
    device = detect_device(device_pref)
    backend = UltralyticsBackend(pt_path, device, input_size, conf_threshold)
    log(f"loaded '{backend.name}' backend on device '{device}' for task '{task}'")
    return backend
