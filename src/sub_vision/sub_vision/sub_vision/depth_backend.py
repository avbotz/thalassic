"""Optional Depth Anything V2 inference support.

Depth Anything V2 produces *relative* depth.  This module intentionally only
turns an RGB frame into a registered floating-point map; task processors are
responsible for establishing any metric calibration.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass

import cv2
import numpy as np

from sub_vision.backends import TensorRTRunner


@dataclass(frozen=True)
class DepthResult:
    map: np.ndarray
    age_s: float
    infer_ms: float
    produced: bool
    source_stamp_ns: int
    current: bool


class DepthAnythingBackend:
    """ONNX Runtime backend for a fixed-size Depth Anything V2 export.

    The exported graph must accept ``[1, 3, H, W]`` RGB tensors normalized
    with ImageNet mean/std and return one dense relative-depth tensor.
    """

    name = "onnxruntime"

    def __init__(self, model_path: str, device: str, input_size: int):
        import onnxruntime as ort

        providers = (["CUDAExecutionProvider", "CPUExecutionProvider"] if device == "cuda"
                     else ["CPUExecutionProvider"])
        self._session = ort.InferenceSession(model_path, providers=providers)
        self._input = self._session.get_inputs()[0]
        self._input_name = self._input.name
        shape = self._input.shape
        self.input_size = int(shape[2]) if len(shape) == 4 and isinstance(shape[2], int) else input_size

    def infer(self, image_bgr: np.ndarray) -> np.ndarray:
        original_h, original_w = image_bgr.shape[:2]
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.input_size, self.input_size), interpolation=cv2.INTER_CUBIC)
        tensor = rgb.astype(np.float32) / 255.0
        tensor = (tensor - np.array([0.485, 0.456, 0.406], dtype=np.float32)) / np.array(
            [0.229, 0.224, 0.225], dtype=np.float32
        )
        tensor = np.ascontiguousarray(tensor.transpose(2, 0, 1)[None])
        output = np.asarray(self._session.run(None, {self._input_name: tensor})[0]).squeeze()
        if output.ndim != 2 or not np.all(np.isfinite(output)):
            raise ValueError(f"unexpected depth output shape/content: {output.shape}")
        return cv2.resize(output.astype(np.float32), (original_w, original_h), interpolation=cv2.INTER_CUBIC)

    def close(self) -> None:
        self._session = None


class TensorRTDepthAnythingBackend:
    """Depth preprocessing/postprocessing around the shared TensorRT runner."""

    name = "tensorrt"

    def __init__(self, engine_path: str, input_size: int):
        self._runner = TensorRTRunner(engine_path, (1, 3, input_size, input_size))
        if len(self._runner.output_names) != 1:
            self._runner.close()
            raise ValueError(
                "Depth Anything TensorRT engine must have exactly one input and one output; "
                f"found {len(self._runner.output_names)} output(s)"
            )
        self.input_size = int(self._runner.input_shape[2])

    def infer(self, image_bgr: np.ndarray) -> np.ndarray:
        h, w = image_bgr.shape[:2]
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.input_size, self.input_size), interpolation=cv2.INTER_CUBIC)
        tensor = rgb.astype(np.float32) / 255.0
        tensor = (tensor - np.array([0.485, 0.456, 0.406], dtype=np.float32)) / np.array([0.229, 0.224, 0.225], dtype=np.float32)
        host_input = np.ascontiguousarray(tensor.transpose(2, 0, 1)[None], dtype=self._runner.input_dtype)
        output = self._runner.infer(host_input)[0].astype(np.float32, copy=False).squeeze()
        if output.ndim != 2 or not np.all(np.isfinite(output)):
            raise ValueError(f"unexpected depth output shape/content: {output.shape}")
        return cv2.resize(output, (w, h), interpolation=cv2.INTER_CUBIC)

    def close(self) -> None:
        self._runner.close()


def _build_backend(model_path: str, backend_pref: str, device: str, input_size: int):
    root, _ = os.path.splitext(model_path)
    engine_path = root + ".engine"
    selected = "tensorrt" if backend_pref == "auto" and os.path.exists(engine_path) else backend_pref
    if selected == "auto":
        selected = "onnxruntime"
    if selected == "tensorrt":
        path = engine_path if os.path.exists(engine_path) else model_path
        if not os.path.exists(path):
            raise FileNotFoundError(f"missing Depth Anything TensorRT engine: {path}")
        return TensorRTDepthAnythingBackend(path, input_size)
    if selected == "onnxruntime":
        return DepthAnythingBackend(model_path, device, input_size)
    raise ValueError("depth_backend must be auto|tensorrt|onnxruntime")


class DepthManager:
    """Rate-limited optional depth model with a short freshness contract."""

    def __init__(self, model_path: str, backend: str, device: str, input_size: int, rate_hz: float, max_age_s: float, log):
        self._backend = _build_backend(model_path, backend, device, input_size)
        self._period_s = 1.0 / max(rate_hz, 0.01)
        self._max_age_s = max_age_s
        self._last_map: np.ndarray | None = None
        self._last_time = float("-inf")
        self._last_stamp_ns = 0
        self.last_infer_ms = float("nan")
        self.stale_frames = 0
        log(f"loaded optional Depth Anything backend ({self._backend.name}, {self._backend.input_size}px)")

    def infer_if_due(self, image_bgr: np.ndarray, source_stamp_ns: int) -> DepthResult | None:
        now = time.monotonic()
        produced = False
        if now - self._last_time >= self._period_s:
            started = time.perf_counter()
            self._last_map = self._backend.infer(image_bgr)
            self._last_time = time.monotonic()
            self._last_stamp_ns = source_stamp_ns
            self.last_infer_ms = (time.perf_counter() - started) * 1000.0
            produced = True
        age = time.monotonic() - self._last_time
        if self._last_map is None or age > self._max_age_s:
            self.stale_frames += 1
            return None
        return DepthResult(
            self._last_map, age, self.last_infer_ms, produced,
            self._last_stamp_ns, produced and source_stamp_ns != 0 and source_stamp_ns == self._last_stamp_ns,
        )

    def close(self) -> None:
        self._backend.close()
