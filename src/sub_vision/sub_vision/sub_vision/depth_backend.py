"""Depth Anything V2 relative depth, run beside the detector.

Depth Anything V2 outputs relative (affine-invariant inverse) depth: larger is nearer, with an
unknown scale and offset. This module only turns a frame into such a map at the frame's
resolution; a post-processor that needs metres calibrates it against geometry it knows (torp fits
it to the board plane PnP solves).
"""

from __future__ import annotations

import time
from collections.abc import Callable

import cv2
import numpy as np

from sub_vision import backends

# ImageNet normalization the model was trained with
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class DepthModel:
    """A fixed-size Depth Anything V2 export, rate limited.

    The graph takes one ``[1, 3, H, W]`` normalized RGB tensor and returns one dense map.
    """

    def __init__(
        self,
        model_dir: str,
        name: str,
        backend_pref: str,
        device_pref: str,
        input_size: int,
        rate_hz: float,
        log: Callable[[str], None],
    ):
        if not rate_hz > 0.0:
            raise ValueError(f"depth rate {rate_hz} Hz is not positive")
        self.backend, path = backends.model_path(model_dir, name, backend_pref)
        if self.backend == "tensorrt":
            self._runner = backends.TensorRTRunner(path, (1, 3, input_size, input_size))
            if len(self._runner.output_names) != 1:
                self._runner.close()
                raise ValueError(f"'{path}' has {len(self._runner.output_names)} outputs, not 1")
            self.input_size = int(self._runner.input_shape[2])
        else:
            import onnxruntime as ort

            providers = (
                ["CUDAExecutionProvider", "CPUExecutionProvider"]
                if backends.onnx_device(device_pref) == "cuda"
                else ["CPUExecutionProvider"]
            )
            self._session = ort.InferenceSession(path, providers=providers)
            self._input_name = self._session.get_inputs()[0].name
            shape = self._session.get_inputs()[0].shape
            static = len(shape) == 4 and isinstance(shape[2], int)
            self.input_size = int(shape[2]) if static else input_size

        self._period_s = 1.0 / rate_hz
        self._last_time = float("-inf")
        self.last_infer_ms = float("nan")
        log(f"Loaded depth model '{name}' via {self.backend} ({self.input_size}px)")

    def infer_if_due(self, image_bgr: np.ndarray) -> np.ndarray | None:
        """This frame's relative depth (float32, image-sized) if a period has passed since the
        last one finished, else ``None``. A map is only ever handed out with the frame it came
        from."""
        if time.monotonic() - self._last_time < self._period_s:
            return None

        h, w = image_bgr.shape[:2]
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.input_size, self.input_size), interpolation=cv2.INTER_CUBIC)
        tensor = (rgb.astype(np.float32) / 255.0 - _MEAN) / _STD
        tensor = np.ascontiguousarray(tensor.transpose(2, 0, 1)[None])

        t0 = time.perf_counter()
        try:
            if self.backend == "tensorrt":
                output = self._runner.infer(tensor)[0]
            else:
                output = self._session.run(None, {self._input_name: tensor})[0]
        finally:
            # From the end, so a model slower than the period (or failing) still leaves frames
            # to the detector
            self._last_time = time.monotonic()
        self.last_infer_ms = (time.perf_counter() - t0) * 1000.0

        output = np.asarray(output, dtype=np.float32).squeeze()
        if output.ndim != 2:
            raise ValueError(f"unexpected depth output of shape {output.shape}")
        if not np.all(np.isfinite(output)):
            raise ValueError("depth output has non-finite values")
        return cv2.resize(output, (w, h), interpolation=cv2.INTER_CUBIC)

    def close(self) -> None:
        if self.backend == "tensorrt":
            self._runner.close()
        else:
            self._session = None
