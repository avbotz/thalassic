import os
import ast
from abc import ABC, abstractmethod
from dataclasses import dataclass

import cv2
import numpy as np
import numpy.typing as npt

MatLike = npt.NDArray[np.uint8]

DEFAULT_INPUT_SIZE = 640
NMS_IOU_THRESHOLD = 0.45
_PAD_COLOR = (114, 114, 114)


@dataclass(frozen=True)
class InferenceResult:
    """Detector boxes plus optional model-derived instance masks."""

    boxes: np.ndarray
    masks: tuple[np.ndarray | None, ...]


class DetectionBackend(ABC):
    """Shared pre/post processing. Subclasses implement `_run` for inference."""

    name = "base"

    def __init__(
        self,
        input_size: int,
        conf_threshold: float,
        class_count: int | None = None,
        end2end: bool = False,
    ):
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.class_count = class_count
        # ONNX exports with embedded NMS retain a [x1, y1, x2, y2,
        # confidence, class, ...mask_coefficients] row.  The optional mask
        # coefficients mean those exports are not necessarily six columns.
        self.end2end = end2end

    # Context managers ensure backend resources (like CUDA pointers) are safely freed
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self) -> None:
        """Release backend resources. Default is a no-op."""
        pass

    @abstractmethod
    def _run(self, blob: np.ndarray) -> np.ndarray | list[np.ndarray]:
        """Execute the network on the processed blob. Must be overridden."""
        pass

    def infer(self, image_bgr: MatLike) -> InferenceResult:
        """Runs the full inference pipeline on a single image."""
        blob, gain, pad = self._preprocess(image_bgr)
        raw_output = self._run(blob)
        return self._postprocess(raw_output, gain, pad, image_bgr.shape[:2])

    def _preprocess(
        self, image_bgr: MatLike
    ) -> tuple[np.ndarray, float, tuple[int, int]]:
        """Letterbox and normalize image to network input size."""
        h, w = image_bgr.shape[:2]
        gain = min(self.input_size / h, self.input_size / w)

        new_w, new_h = int(round(w * gain)), int(round(h * gain))
        if (new_w, new_h) != (w, h):
            image_bgr = cv2.resize(
                image_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR
            )

        # Center the image
        pad_x, pad_y = (self.input_size - new_w) / 2, (self.input_size - new_h) / 2
        top, bottom = int(round(pad_y - 0.1)), int(round(pad_y + 0.1))
        left, right = int(round(pad_x - 0.1)), int(round(pad_x + 0.1))

        padded = cv2.copyMakeBorder(
            image_bgr, top, bottom, left, right, cv2.BORDER_CONSTANT, value=_PAD_COLOR
        )

        # Replaces manual numpy transpose, type casting, and channel swapping
        blob = cv2.dnn.blobFromImage(padded, scalefactor=1.0 / 255.0, swapRB=True)
        return blob, gain, (left, top)

    def _postprocess(
        self,
        output: np.ndarray | list[np.ndarray],
        gain: float,
        pad: tuple[int, int],
        orig_hw: tuple[int, int],
    ) -> InferenceResult:
        """Filters detections and scales bounding boxes back to original image size."""
        outputs = output if isinstance(output, list) else [output]
        out = np.squeeze(outputs[0])  # Removes batch dim, e.g. (1, 300, 6) -> (300, 6)

        if out.ndim != 2:
            raise ValueError(f"Unexpected model output shape {output.shape}")

        # Distinguish embedded-NMS/end-to-end outputs from older raw YOLO
        # heads.  Segmentation exports have 6 + mask-coefficient columns, so
        # width alone cannot identify them.
        if self.end2end or out.shape[1] == 6:
            dets = out[out[:, 4] >= self.conf_threshold]
        else:
            dets = self._decode_raw_head(out)

        if len(dets) == 0:
            return InferenceResult(np.empty((0, 6), dtype=np.float32), ())

        dets = self._rescale_boxes(dets, gain, pad, orig_hw)
        masks = self._decode_masks(dets, outputs[1], gain, pad, orig_hw) if len(outputs) > 1 else ()
        return InferenceResult(dets[:, :6], masks or tuple(None for _ in dets))

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

        extra = out[keep, class_end:][idx] if class_end is not None else np.empty((len(idx), 0))
        return np.hstack(
            [xyxy[idx], confs[idx, None], cls_ids[idx, None].astype(np.float32), extra]
        )

    def _decode_masks(
        self,
        dets: np.ndarray,
        prototype: np.ndarray,
        gain: float,
        pad: tuple[int, int],
        orig_hw: tuple[int, int],
    ) -> tuple[np.ndarray | None, ...]:
        """Decode Ultralytics segment masks into original-image coordinates.

        Segmentation models expose 32 mask coefficients after the class scores
        plus a ``[1, 32, H, W]`` prototype tensor.  Other model types retain
        their normal box-only behavior.
        """
        proto = np.squeeze(prototype)
        if dets.shape[1] <= 6 or proto.ndim != 3 or dets.shape[1] - 6 != proto.shape[0]:
            return ()
        coefficients = dets[:, 6:]
        mask_logits = coefficients @ proto.reshape(proto.shape[0], -1)
        mask_logits = mask_logits.reshape(-1, proto.shape[1], proto.shape[2])
        mask_probs = 1.0 / (1.0 + np.exp(-np.clip(mask_logits, -80.0, 80.0)))

        original_height, original_width = orig_hw
        pad_x, pad_y = pad
        unpadded_width = int(round(original_width * gain))
        unpadded_height = int(round(original_height * gain))
        masks: list[np.ndarray | None] = []
        for probability, box in zip(mask_probs, dets[:, :4]):
            model_mask = cv2.resize(probability, (self.input_size, self.input_size), interpolation=cv2.INTER_LINEAR)
            mask = model_mask[pad_y:pad_y + unpadded_height, pad_x:pad_x + unpadded_width]
            if mask.shape != (unpadded_height, unpadded_width):
                masks.append(None)
                continue
            mask = cv2.resize(mask, (original_width, original_height), interpolation=cv2.INTER_LINEAR)
            binary = (mask >= 0.5).astype(np.uint8) * 255
            x0, y0, x1, y1 = np.round(box).astype(int)
            binary[:max(0, y0), :] = 0
            binary[min(original_height, y1):, :] = 0
            binary[:, :max(0, x0)] = 0
            binary[:, min(original_width, x1):] = 0
            masks.append(binary)
        return tuple(masks)


class OnnxBackend(DetectionBackend):
    """ONNX Runtime execution backend."""

    name = "onnxruntime"

    def __init__(
        self, onnx_path: str, device: str, input_size: int, conf_threshold: float
    ):
        import onnxruntime as ort

        providers = (
            ["CUDAExecutionProvider", "CPUExecutionProvider"]
            if device == "cuda"
            else ["CPUExecutionProvider"]
        )

        self._session = ort.InferenceSession(onnx_path, providers=providers)
        metadata = self._session.get_modelmeta().custom_metadata_map
        super().__init__(
            input_size,
            conf_threshold,
            class_count=self._class_count_from_metadata(self._session),
            end2end=str(metadata.get("end2end", "false")).lower() == "true",
        )
        self._input_name = self._session.get_inputs()[0].name
        # Lock to static graph size if available
        shape = self._session.get_inputs()[0].shape
        if len(shape) == 4 and isinstance(shape[2], int):
            self.input_size = int(shape[2])

    def _run(self, blob: np.ndarray) -> list[np.ndarray]:
        return self._session.run(None, {self._input_name: blob})

    @staticmethod
    def _class_count_from_metadata(session) -> int | None:
        names = session.get_modelmeta().custom_metadata_map.get("names")
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


class TensorRTRunner:
    """Run a one-input TensorRT engine through CUDA-Python.

    Detector and depth models intentionally share this low-level runner. Their
    image preprocessing and postprocessing remain model-specific.
    """

    def __init__(self, engine_path: str, fallback_input_shape: tuple[int, ...]):
        import tensorrt as trt
        from cuda.bindings import runtime as cudart

        self._cudart = cudart
        self._load_engine(engine_path, trt, fallback_input_shape)
        self._allocate_buffers()

    def _load_engine(self, engine_path: str, trt_module, fallback_input_shape: tuple[int, ...]):
        """Deserializes the TensorRT engine and extracts metadata."""
        # The runtime must outlive every engine it deserializes.  Keeping these
        # references also makes the ownership relationship explicit.
        self._logger = trt_module.Logger(trt_module.Logger.WARNING)
        self._runtime = trt_module.Runtime(self._logger)
        with open(engine_path, "rb") as f:
            self._engine = self._runtime.deserialize_cuda_engine(f.read())

        if self._engine is None:
            raise RuntimeError(f"Failed to deserialize TensorRT engine '{engine_path}'")

        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError("Failed to create TensorRT execution context")

        # Identify input and output tensor names
        names = [
            self._engine.get_tensor_name(i) for i in range(self._engine.num_io_tensors)
        ]
        input_names = [n for n in names if self._engine.get_tensor_mode(n) == trt_module.TensorIOMode.INPUT]
        self.output_names = tuple(
            n for n in names if self._engine.get_tensor_mode(n) == trt_module.TensorIOMode.OUTPUT
        )
        if len(input_names) != 1 or not self.output_names:
            raise ValueError(
                "TensorRT engines must have one input and at least one output; "
                f"found {len(input_names)} input(s) and {len(self.output_names)} output(s)"
            )
        self._input_name = input_names[0]

        # Handle dynamic vs static shapes
        in_shape = list(self._engine.get_tensor_shape(self._input_name))
        if -1 in in_shape:
            in_shape = list(fallback_input_shape)
            if not self._context.set_input_shape(self._input_name, tuple(in_shape)):
                raise ValueError(f"TensorRT engine rejected input shape {tuple(in_shape)}")

        self.input_shape = tuple(in_shape)
        self.input_dtype = np.dtype(
            trt_module.nptype(self._engine.get_tensor_dtype(self._input_name))
        )
        self._output_specs = []
        for name in self.output_names:
            shape = tuple(self._context.get_tensor_shape(name))
            if -1 in shape:
                raise ValueError(f"TensorRT engine has unresolved output dimensions for '{name}': {shape}")
            dtype = np.dtype(trt_module.nptype(self._engine.get_tensor_dtype(name)))
            self._output_specs.append((name, shape, dtype))

    def _allocate_buffers(self):
        """Allocates GPU memory pointers and streams for execution."""
        self._host_outputs = {
            name: np.empty(shape, dtype=dtype) for name, shape, dtype in self._output_specs
        }

        in_bytes = int(np.prod(self.input_shape)) * self.input_dtype.itemsize
        self._d_input = self._check_cuda(self._cudart.cudaMalloc(in_bytes))
        self._d_outputs = {
            name: self._check_cuda(self._cudart.cudaMalloc(host_output.nbytes))
            for name, host_output in self._host_outputs.items()
        }
        self._stream = self._check_cuda(self._cudart.cudaStreamCreate())

        self._context.set_tensor_address(self._input_name, self._d_input)
        for name, device_output in self._d_outputs.items():
            self._context.set_tensor_address(name, device_output)

    def _check_cuda(self, result):
        """Unwrap a cuda-python result, raising an exception on error."""
        err, *values = result
        if err != self._cudart.cudaError_t.cudaSuccess:
            raise RuntimeError(f"CUDA runtime error: {err}")
        return values[0] if values else None

    def infer(self, host_input: np.ndarray) -> list[np.ndarray]:
        """Run inference and return a copied result for every output tensor."""
        host_input = np.ascontiguousarray(host_input, dtype=self.input_dtype)
        if host_input.shape != self.input_shape:
            raise ValueError(
                f"TensorRT input shape mismatch: engine expects {self.input_shape}, got {host_input.shape}"
            )

        # 1. Copy input data to GPU
        self._check_cuda(
            self._cudart.cudaMemcpyAsync(
                self._d_input,
                host_input.ctypes.data,
                host_input.nbytes,
                self._cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
                self._stream,
            )
        )

        # 2. Run inference
        if not self._context.execute_async_v3(self._stream):
            raise RuntimeError("TensorRT inference failed")

        # 3. Copy results back to Host
        for name, host_output in self._host_outputs.items():
            self._check_cuda(
                self._cudart.cudaMemcpyAsync(
                    host_output.ctypes.data,
                    self._d_outputs[name],
                    host_output.nbytes,
                    self._cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
                    self._stream,
                )
            )

        # 4. Wait for completion
        self._check_cuda(self._cudart.cudaStreamSynchronize(self._stream))

        # Copies safely survive frame-to-frame buffer reuse. Keep TensorRT's
        # output ordering so segmentation prototypes remain available.
        return [self._host_outputs[name].copy() for name in self.output_names]

    def close(self) -> None:
        """Safely free CUDA device buffers and engines."""
        cudart = getattr(self, "_cudart", None)
        if not cudart:
            return

        if getattr(self, "_d_input", None) is not None:
            cudart.cudaFree(self._d_input)
        for device_output in getattr(self, "_d_outputs", {}).values():
            cudart.cudaFree(device_output)
        if getattr(self, "_stream", None) is not None:
            cudart.cudaStreamDestroy(self._stream)

        self._d_input = self._stream = None
        self._d_outputs = {}
        self._context = self._engine = self._runtime = self._logger = None


class TensorRTBackend(DetectionBackend):
    """TensorRT detector backend using the shared CUDA-Python engine runner."""

    name = "tensorrt"

    def __init__(self, engine_path: str, input_size: int, conf_threshold: float):
        super().__init__(input_size, conf_threshold)
        self._runner = TensorRTRunner(engine_path, (1, 3, input_size, input_size))
        if len(self._runner.input_shape) == 4:
            self.input_size = int(self._runner.input_shape[2])

    def _run(self, blob: np.ndarray) -> list[np.ndarray]:
        return [output.astype(np.float32, copy=False) for output in self._runner.infer(blob)]

    def close(self) -> None:
        self._runner.close()


def build_backend(
    task: str,
    model_dir: str,
    backend_pref: str,
    device_pref: str,
    input_size: int,
    conf_threshold: float,
    log,
) -> DetectionBackend:

    engine_path = os.path.join(model_dir, f"{task}.engine")
    onnx_path = os.path.join(model_dir, f"{task}.onnx")

    # Resolve "auto" to TensorRT if engine exists, else ONNX
    if backend_pref == "auto":
        backend_pref = "tensorrt" if os.path.exists(engine_path) else "onnxruntime"

    if backend_pref == "tensorrt":
        if not os.path.exists(engine_path):
            raise FileNotFoundError(f"Missing TensorRT engine: {engine_path}")
        backend = TensorRTBackend(engine_path, input_size, conf_threshold)
        device = "cuda"

    elif backend_pref == "onnxruntime":
        if not os.path.exists(onnx_path):
            raise FileNotFoundError(f"Missing ONNX model: {onnx_path}")

        device = (
            device_pref
            if device_pref != "auto"
            else (
                "cuda"
                if "CUDAExecutionProvider"
                in __import__("onnxruntime").get_available_providers()
                else "cpu"
            )
        )
        backend = OnnxBackend(onnx_path, device, input_size, conf_threshold)

    else:
        raise ValueError(
            f"Unknown backend '{backend_pref}' (auto|tensorrt|onnxruntime)"
        )

    log(f"Loaded '{backend.name}' backend on device '{device}' for task '{task}'")
    return backend
