import os
from abc import ABC, abstractmethod

import cv2
import numpy as np
import numpy.typing as npt

MatLike = npt.NDArray[np.uint8]

DEFAULT_INPUT_SIZE = 640
NMS_IOU_THRESHOLD = 0.45
_PAD_COLOR = (114, 114, 114)


class DetectionBackend(ABC):
    """Shared pre/post processing. Subclasses implement `_run` for inference."""

    name = "base"

    def __init__(self, input_size: int, conf_threshold: float):
        self.input_size = input_size
        self.conf_threshold = conf_threshold

    # Context managers ensure backend resources (like CUDA pointers) are safely freed
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self) -> None:
        """Release backend resources. Default is a no-op."""
        pass

    @abstractmethod
    def _run(self, blob: np.ndarray) -> np.ndarray:
        """Execute the network on the processed blob. Must be overridden."""
        pass

    def infer(self, image_bgr: MatLike) -> np.ndarray:
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
        output: np.ndarray,
        gain: float,
        pad: tuple[int, int],
        orig_hw: tuple[int, int],
    ) -> np.ndarray:
        """Filters detections and scales bounding boxes back to original image size."""
        out = np.squeeze(output)  # Removes batch dim, e.g. (1, 300, 6) -> (300, 6)

        if out.ndim != 2:
            raise ValueError(f"Unexpected model output shape {output.shape}")

        # Distinguish between YOLOv10 (end-to-end) and older YOLOv8 heads
        if out.shape[1] == 6:
            dets = out[out[:, 4] >= self.conf_threshold]
        else:
            dets = self._decode_raw_head(out)

        if len(dets) == 0:
            return np.empty((0, 6), dtype=np.float32)

        return self._rescale_boxes(dets, gain, pad, orig_hw)

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

        scores = out[:, 4:]
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

        return np.hstack(
            [xyxy[idx], confs[idx, None], cls_ids[idx, None].astype(np.float32)]
        )


class OnnxBackend(DetectionBackend):
    """ONNX Runtime execution backend."""

    name = "onnxruntime"

    def __init__(
        self, onnx_path: str, device: str, input_size: int, conf_threshold: float
    ):
        super().__init__(input_size, conf_threshold)
        import onnxruntime as ort

        providers = (
            ["CUDAExecutionProvider", "CPUExecutionProvider"]
            if device == "cuda"
            else ["CPUExecutionProvider"]
        )

        self._session = ort.InferenceSession(onnx_path, providers=providers)
        self._input_name = self._session.get_inputs()[0].name

        # Lock to static graph size if available
        shape = self._session.get_inputs()[0].shape
        if len(shape) == 4 and isinstance(shape[2], int):
            self.input_size = int(shape[2])

    def _run(self, blob: np.ndarray) -> np.ndarray:
        return self._session.run(None, {self._input_name: blob})[0]


class TensorRTBackend(DetectionBackend):
    """TensorRT execution backend via CUDA-Python."""

    name = "tensorrt"

    def __init__(self, engine_path: str, input_size: int, conf_threshold: float):
        super().__init__(input_size, conf_threshold)
        import tensorrt as trt
        from cuda import cudart

        self._cudart = cudart

        self._load_engine(engine_path, trt)
        self._allocate_buffers()

    def _load_engine(self, engine_path: str, trt_module):
        """Deserializes the TensorRT engine and extracts metadata."""
        logger = trt_module.Logger(trt_module.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self._engine = trt_module.Runtime(logger).deserialize_cuda_engine(f.read())

        if self._engine is None:
            raise RuntimeError(f"Failed to deserialize TensorRT engine '{engine_path}'")

        self._context = self._engine.create_execution_context()

        # Identify input and output tensor names
        names = [
            self._engine.get_tensor_name(i) for i in range(self._engine.num_io_tensors)
        ]
        self._input_name = next(
            n
            for n in names
            if self._engine.get_tensor_mode(n) == trt_module.TensorIOMode.INPUT
        )
        self._output_name = next(
            n
            for n in names
            if self._engine.get_tensor_mode(n) == trt_module.TensorIOMode.OUTPUT
        )

        # Handle dynamic vs static shapes
        in_shape = list(self._engine.get_tensor_shape(self._input_name))
        if -1 in in_shape:
            in_shape = [1, 3, self.input_size, self.input_size]
            self._context.set_input_shape(self._input_name, tuple(in_shape))
        elif len(in_shape) == 4:
            self.input_size = int(in_shape[2])

        self._input_shape = tuple(in_shape)
        self._output_shape = tuple(self._context.get_tensor_shape(self._output_name))

        self._input_dtype = np.dtype(
            trt_module.nptype(self._engine.get_tensor_dtype(self._input_name))
        )
        self._output_dtype = np.dtype(
            trt_module.nptype(self._engine.get_tensor_dtype(self._output_name))
        )

    def _allocate_buffers(self):
        """Allocates GPU memory pointers and streams for execution."""
        self._host_output = np.empty(self._output_shape, dtype=self._output_dtype)

        in_bytes = int(np.prod(self._input_shape)) * self._input_dtype.itemsize
        out_bytes = self._host_output.nbytes

        self._d_input = self._check_cuda(self._cudart.cudaMalloc(in_bytes))
        self._d_output = self._check_cuda(self._cudart.cudaMalloc(out_bytes))
        self._stream = self._check_cuda(self._cudart.cudaStreamCreate())

        self._context.set_tensor_address(self._input_name, self._d_input)
        self._context.set_tensor_address(self._output_name, self._d_output)

    def _check_cuda(self, result):
        """Unwrap a cuda-python result, raising an exception on error."""
        err, *values = result
        if err != self._cudart.cudaError_t.cudaSuccess:
            raise RuntimeError(f"CUDA runtime error: {err}")
        return values[0] if values else None

    def _run(self, blob: np.ndarray) -> np.ndarray:
        """Executes inference asychronously on the GPU."""
        host_input = np.ascontiguousarray(blob, dtype=self._input_dtype)

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
        self._check_cuda(
            self._cudart.cudaMemcpyAsync(
                self._host_output.ctypes.data,
                self._d_output,
                self._host_output.nbytes,
                self._cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
                self._stream,
            )
        )

        # 4. Wait for completion
        self._check_cuda(self._cudart.cudaStreamSynchronize(self._stream))

        # Must return a copy so the array safely survives frame-to-frame loop overwrites
        return self._host_output.astype(np.float32, copy=True)

    def close(self) -> None:
        """Safely free CUDA device buffers and engines."""
        cudart = getattr(self, "_cudart", None)
        if not cudart:
            return

        if getattr(self, "_d_input", None) is not None:
            cudart.cudaFree(self._d_input)
        if getattr(self, "_d_output", None) is not None:
            cudart.cudaFree(self._d_output)
        if getattr(self, "_stream", None) is not None:
            cudart.cudaStreamDestroy(self._stream)

        self._d_input = self._d_output = self._stream = None
        self._context = self._engine = None


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
