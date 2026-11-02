from __future__ import annotations

import ast
import os
from collections.abc import Callable
from dataclasses import dataclass

import cv2
import numpy as np
import numpy.typing as npt
import onnxruntime as ort

MatLike = npt.NDArray[np.uint8]

# The values the device preference takes. "auto" runs a model on the first of GPU_PROVIDERS that
# can run it, else on the CPU; "gpu" fails to load it instead of using the CPU.
DEVICES = ("auto", "cpu", "gpu")

# ONNX Runtime's execution providers for a GPU on Linux, in its own order of preference. An
# installed build has at most a few of them; the rest are skipped. Named rather than taken from
# what the build lists: conda-forge's x86 CUDA build also lists TensorRT, first, uninstalled.
GPU_PROVIDERS = ("CUDAExecutionProvider", "MIGraphXExecutionProvider", "WebGpuExecutionProvider")
CPU_PROVIDER = "CPUExecutionProvider"

NMS_IOU_THRESHOLD = 0.45
_PAD_COLOR = (114, 114, 114)

# Threads of ONNX Runtime on the CPU (see Runner)
CPU_THREADS = 4


@dataclass(frozen=True)
class InferenceResult:
    """Boxes as ``(N, 6)`` rows of ``[x1, y1, x2, y2, score, class]`` in image pixels, and per box
    its instance mask (uint8 0/255, image-sized) when the model segments, else ``None``."""

    boxes: np.ndarray
    masks: tuple[np.ndarray | None, ...]


def letterbox(image_bgr: MatLike, input_size: int) -> tuple[np.ndarray, float, tuple[int, int]]:
    """The ``[1, 3, input_size, input_size]`` RGB float blob a detector takes: the image scaled
    to fit, centred on gray padding, divided by 255. Also the scale and the (left, top) padding,
    which map boxes back to image pixels."""
    h, w = image_bgr.shape[:2]
    gain = min(input_size / h, input_size / w)

    new_w, new_h = int(round(w * gain)), int(round(h * gain))
    if (new_w, new_h) != (w, h):
        image_bgr = cv2.resize(image_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    # Center the image
    pad_x, pad_y = (input_size - new_w) / 2, (input_size - new_h) / 2
    top, bottom = int(round(pad_y - 0.1)), int(round(pad_y + 0.1))
    left, right = int(round(pad_x - 0.1)), int(round(pad_x + 0.1))

    padded = cv2.copyMakeBorder(
        image_bgr, top, bottom, left, right, cv2.BORDER_CONSTANT, value=_PAD_COLOR
    )

    # Replaces manual numpy transpose, type casting, and channel swapping
    blob = cv2.dnn.blobFromImage(padded, scalefactor=1.0 / 255.0, swapRB=True)
    return blob, gain, (left, top)


class Detector:
    """A YOLO detect or segment model: letterboxing in, box and mask decoding out, around the
    runner that executes it."""

    def __init__(self, runner: Runner, conf_threshold: float):
        self._runner = runner
        self.name = runner.name
        self.input_size = int(runner.input_shape[2])
        self.conf_threshold = conf_threshold
        self.class_count = self._class_count_from_metadata(runner.metadata)
        # End-to-end exports (NMS in the graph) emit [x1, y1, x2, y2, score, class] rows,
        # followed by mask coefficients in segmentation models, so width alone cannot tell them
        # from a raw head.
        self.end2end = runner.metadata.get("end2end", "false").lower() == "true"

    def close(self) -> None:
        self._runner.close()

    def infer(self, image_bgr: MatLike) -> InferenceResult:
        """Runs the full inference pipeline on a single image."""
        blob, gain, pad = letterbox(image_bgr, self.input_size)
        outputs = [output.astype(np.float32, copy=False) for output in self._runner.run(blob)]
        return self._postprocess(outputs, gain, pad, image_bgr.shape[:2])

    @staticmethod
    def _class_count_from_metadata(metadata: dict[str, str]) -> int | None:
        names = metadata.get("names")
        if not names:
            return None
        try:
            parsed = ast.literal_eval(names)
        except (SyntaxError, ValueError):
            return None
        if isinstance(parsed, dict):
            return len(parsed)
        if isinstance(parsed, list):
            return len(parsed)
        return None

    def _postprocess(
        self,
        outputs: list[np.ndarray],
        gain: float,
        pad: tuple[int, int],
        orig_hw: tuple[int, int],
    ) -> InferenceResult:
        """Filters detections and scales bounding boxes back to original image size."""
        out = np.squeeze(outputs[0])  # Removes batch dim, e.g. (1, 300, 6) -> (300, 6)

        if out.ndim != 2:
            raise ValueError(f"Unexpected model output shape {outputs[0].shape}")

        # Distinguish end-to-end heads (YOLOv10, YOLO26) from older raw YOLOv8 heads
        if self.end2end or out.shape[1] == 6:
            dets = out[out[:, 4] >= self.conf_threshold]
        else:
            dets = self._decode_raw_head(out)

        if len(dets) == 0:
            return InferenceResult(np.empty((0, 6), dtype=np.float32), ())

        dets = self._rescale_boxes(dets, gain, pad, orig_hw)
        masks = self._decode_masks(dets, outputs[1], gain, pad, orig_hw) if len(outputs) > 1 else ()
        return InferenceResult(dets[:, :6], masks or (None,) * len(dets))

    def _rescale_boxes(
        self, dets: np.ndarray, gain: float, pad: tuple, orig_hw: tuple
    ) -> np.ndarray:
        """Removes letterbox padding and scales boxes back to original image pixels."""
        dets = dets.copy()
        pad_x, pad_y = pad
        orig_h, orig_w = orig_hw

        # Un-pad and un-scale
        dets[:, [0, 2]] = (dets[:, [0, 2]] - pad_x) / gain
        dets[:, [1, 3]] = (dets[:, [1, 3]] - pad_y) / gain

        # Clip boxes to image boundaries
        dets[:, [0, 2]] = np.clip(dets[:, [0, 2]], 0, orig_w)
        dets[:, [1, 3]] = np.clip(dets[:, [1, 3]], 0, orig_h)

        # Discard invalid boxes where max < min
        keep = (dets[:, 2] > dets[:, 0]) & (dets[:, 3] > dets[:, 1])
        return dets[keep].astype(np.float32)

    def _decode_raw_head(self, out: np.ndarray) -> np.ndarray:
        """Fallback: Decode a raw (4+nc, M) YOLOv8-style head and run NMS."""
        if out.shape[0] < out.shape[1]:
            out = out.T

        class_end = 4 + self.class_count if self.class_count is not None else None
        scores = out[:, 4:class_end]
        cls_ids = scores.argmax(axis=1)
        confs = scores.max(axis=1)  # Faster than 2D array masking

        keep = confs >= self.conf_threshold
        if not keep.any():
            return np.empty((0, 6), dtype=np.float32)

        xywh = out[keep, :4]
        confs, cls_ids = confs[keep], cls_ids[keep]

        # Convert [center_x, center_y, w, h] to [x1, y1, w, h] for OpenCV NMS
        x1y1 = xywh[:, :2] - (xywh[:, 2:] / 2)
        nms_boxes = np.hstack([x1y1, xywh[:, 2:]])

        idx = cv2.dnn.NMSBoxes(
            nms_boxes.tolist(), confs.tolist(), self.conf_threshold, NMS_IOU_THRESHOLD
        )

        if len(idx) == 0:
            return np.empty((0, 6), dtype=np.float32)

        idx = np.asarray(idx).flatten()
        xyxy = np.hstack([x1y1, x1y1 + xywh[:, 2:]])

        # Segmentation heads carry mask coefficients after the class scores
        coeffs = out[keep, class_end:][idx] if class_end is not None else np.empty((len(idx), 0))
        return np.hstack(
            [xyxy[idx], confs[idx, None], cls_ids[idx, None].astype(np.float32), coeffs]
        )

    def _decode_masks(
        self,
        dets: np.ndarray,
        prototype: np.ndarray,
        gain: float,
        pad: tuple[int, int],
        orig_hw: tuple[int, int],
    ) -> tuple[np.ndarray | None, ...]:
        """Decode Ultralytics segment masks into original-image pixels.

        Segmentation models emit mask coefficients after each box and a ``[1, C, H, W]``
        prototype tensor; a box's mask is its coefficients times the prototypes, cropped to the
        box. Returns ``()`` for any other model.
        """
        proto = np.squeeze(prototype)
        if dets.shape[1] <= 6 or proto.ndim != 3 or dets.shape[1] - 6 != proto.shape[0]:
            return ()
        logits = dets[:, 6:] @ proto.reshape(proto.shape[0], -1)
        logits = logits.reshape(-1, proto.shape[1], proto.shape[2])
        probs = 1.0 / (1.0 + np.exp(-np.clip(logits, -80.0, 80.0)))

        orig_h, orig_w = orig_hw
        pad_x, pad_y = pad
        # Prototype pixels per model-input pixel
        scale_x, scale_y = proto.shape[2] / self.input_size, proto.shape[1] / self.input_size
        masks: list[np.ndarray | None] = []
        for prob, box in zip(probs, dets[:, :4], strict=True):
            mask = np.zeros((orig_h, orig_w), dtype=np.uint8)
            x0, y0, x1, y1 = np.round(box).astype(int)
            x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, orig_w), min(y1, orig_h)
            if x1 > x0 and y1 > y0:
                # The prototypes sampled at the box's pixels only (resizing them to the whole
                # image took most of the post-processing): each pixel's centre, through the
                # letterbox, in prototype pixels.
                to_proto = np.float32(
                    [
                        [gain * scale_x, 0.0, ((x0 + 0.5) * gain + pad_x) * scale_x - 0.5],
                        [0.0, gain * scale_y, ((y0 + 0.5) * gain + pad_y) * scale_y - 0.5],
                    ]
                )
                region = cv2.warpAffine(
                    prob,
                    to_proto,
                    (x1 - x0, y1 - y0),
                    flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                    borderMode=cv2.BORDER_REPLICATE,
                )
                mask[y0:y1, x0:x1] = (region >= 0.5).astype(np.uint8) * 255
            masks.append(mask)
        return tuple(masks)


class Runner:
    """One network in an ONNX Runtime session on one execution provider, with a fixed input shape.
    Any operator the provider lacks runs on the CPU."""

    def __init__(self, onnx_path: str, provider: str, fallback_input_shape: tuple[int, ...]):
        # On the CPU: by default a thread per core, each spinning between operators and frames.
        # On the laptop that took 7.7 cores for 24 Hz of the gate model, and starved the
        # simulator; 4 sleeping threads keep 24 Hz (35 ms instead of 25 per frame) on 2.7.
        options = ort.SessionOptions()
        options.intra_op_num_threads = CPU_THREADS
        options.add_session_config_entry("session.intra_op.allow_spinning", "0")
        options.add_session_config_entry("session.inter_op.allow_spinning", "0")
        # Left to itself, ONNX Runtime moves a session to the CPU with only a printed warning when
        # a provider fails to start or fails a run. Which provider runs a model is open_runner's
        # decision, so a failure has to reach it.
        self._session = ort.InferenceSession(
            onnx_path, options, providers=[provider], enable_fallback=0
        )
        active = self._session.get_providers()[0]
        if active != provider:  # also how a provider that cannot start shows, without an exception
            raise RuntimeError(f"{provider} did not start, the session is on {active}")
        self.name = f"onnxruntime/{provider.removesuffix('ExecutionProvider').lower()}"
        self.metadata = dict(self._session.get_modelmeta().custom_metadata_map)

        model_input = self._session.get_inputs()[0]
        self._input_name = model_input.name
        # Dynamic dimensions are names or None
        static = all(isinstance(dim, int) for dim in model_input.shape)
        self.input_shape = tuple(model_input.shape) if static else fallback_input_shape

    def run(self, x: np.ndarray) -> list[np.ndarray]:
        """Every output for input ``x``, in graph order."""
        return self._session.run(None, {self._input_name: x})

    def close(self) -> None:
        """Release the session, and any GPU memory it holds."""
        self._session = None


def open_runner(
    onnx_path: str,
    device: str,
    fallback_input_shape: tuple[int, ...],
    log: Callable[[str], None],
) -> Runner:
    """``onnx_path`` loaded for inference on ``device`` (one of :data:`DEVICES`): on the first of
    :data:`GPU_PROVIDERS` that runs it, then on the CPU, which ``log`` is told the reason for when
    a GPU was tried. ``fallback_input_shape`` stands in for a dynamic ONNX input.
    """
    if not os.path.exists(onnx_path):
        raise FileNotFoundError(f"Missing ONNX model: {onnx_path}")
    if device == "cpu":
        return Runner(onnx_path, CPU_PROVIDER, fallback_input_shape)

    available = ort.get_available_providers()
    failures = []
    for provider in GPU_PROVIDERS:
        if provider not in available:
            continue
        try:
            runner = Runner(onnx_path, provider, fallback_input_shape)
            # A build lists every provider it was compiled with, and one with no kernels for this
            # GPU only fails on its first inference.
            runner.run(np.zeros(runner.input_shape, dtype=np.float32))
            return runner
        except Exception as error:  # the next provider may run it
            failures.append(f"{provider}: {error}")

    if device == "gpu":
        reason = "; ".join(failures) or f"ONNX Runtime only has {available}"
        raise RuntimeError(f"no GPU can run '{onnx_path}': {reason}")
    if failures:
        log(f"running '{onnx_path}' on the CPU: {'; '.join(failures)}")
    return Runner(onnx_path, CPU_PROVIDER, fallback_input_shape)
