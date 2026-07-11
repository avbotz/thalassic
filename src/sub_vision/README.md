# sub_vision

Perception for the Marlin V2 AUV: YOLOv10 detection with per-task OpenCV
post-processing. Inference runs through
**TensorRT** (serialized engine) or **ONNX Runtime**; pre-processing
(letterbox) and post-processing (decode + box rescaling) live in
`backends.py`. Targets the Jetson AGX Orin (TensorRT, CUDA) in production and
any x86 box (GPU or CPU-only ONNX Runtime) for dev — the **same code** runs on
both; only the selected backend/device differs.

This directory holds two ROS 2 (Jazzy) packages:

| Package | Build type | Contents |
| --- | --- | --- |
| `sub_vision_interfaces` | `ament_cmake` | `Detection.msg`, `DetectionArray.msg`, `LoadModel.srv` |
| `sub_vision` | `ament_python` | model manager, post-processors, node |

> The detector is **YOLOv10 (NMS-free / end-to-end)**: the exported head emits
> `(1, N, 6)` boxes directly, so the pipeline runs **no** separate NMS step.
> Raw YOLOv8-style exports (`(1, 4+nc, M)`) are also decoded, with NMS applied
> in the backend.

---

## The topic contract (real camera == sim)

`sub_vision` reads one RGB stream plus its intrinsics and publishes detections.
The input topics are plain parameters, so there are **no `if sim:` branches and
no bridge nodes** — each launch file points the node at its camera source and
`cv_bridge` normalizes any 8-bit color encoding to `bgr8` on receipt.

All topics are relative to the node namespace (e.g. `/marlin_v2`):

| Topic (parameter) | Type | Notes |
| --- | --- | --- |
| `rgb_topic` (default `front_camera/image_raw`) | `sensor_msgs/Image` | any 8-bit color encoding |
| `camera_info_topic` (default `front_camera/camera_info`) | `sensor_msgs/CameraInfo` | intrinsics; also feed the per-detection bearings |
| **out:** `detections_topic` (default `vision/detections`) | `sub_vision_interfaces/DetectionArray` | detections + bearings + 3D metadata |
| **out:** `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | model state + timing |

| Launch file | Camera | Parameter override |
| --- | --- | --- |
| `sim_launch.py` | Stonefish `front_camera` | `rgb_topic: front_camera/image_color` (rgb8) |
| `pool_test_launch.py` | Logitech C922 via `usb_cam` | `rgb_topic: front_camera/image_raw` (OAK-D driver block present but commented out) |

The depth camera is **not** used: `sub_vision` subscribes to RGB only.
Bounding boxes, bearings, and `pose` are expressed in the camera **optical**
frame (`header.frame_id` from the source image).

### image_transport

rclpy has no `image_transport` bindings, so the node implements the convention
with a parameter: `image_transport: raw` (default) subscribes `<rgb_topic>`
directly; `compressed` subscribes `<rgb_topic>/compressed`
(`sensor_msgs/CompressedImage`) instead — use it when frames cross the network,
e.g. running vision on a laptop against the sub's cameras. Any other value
fails fast at startup.

---

## Loading / swapping models live

`sub_vision` keeps **one** model resident. Swap it at runtime via the service:

```bash
ros2 service call /marlin_v2/sub_vision/load_model \
    sub_vision_interfaces/srv/LoadModel "{task: 'gate'}"
```

* If the requested task is already active, it's a no-op that still returns
  `success: true`.
* Otherwise the new model is loaded **outside** the inference lock, then swapped
  in atomically and the previous one is freed — a swap can never race a frame in
  flight (`model_manager.ModelManager`).
* `default_task` (param) pre-loads a model on startup.

---

## Model loading & backend/device selection

Models live in `model_dir` as exported graphs:

* `<task>.engine` — serialized **TensorRT** engine (device-specific; built on
  the Jetson it runs on).
* `<task>.onnx` — exported **ONNX** graph for ONNX Runtime.

`sub_mission` requests role-agnostic model names. For example,
mission XML `task="gate"` calls `LoadModel` with `task: 'gate'`. Task
post-processors also tolerate `_survey` and `_search` suffixed task names from
other loaders by falling back to the base task processor when one is registered.

Selection, per the `backend` param:

1. `auto` (default) prefers `<task>.engine` (TensorRT) and falls back to
   `<task>.onnx` (ONNX Runtime). `tensorrt` / `onnxruntime` force one and fail
   if the file is missing.
2. The `device` param applies to ONNX Runtime only: `auto` picks `cuda` when
   the CUDA execution provider is available, else `cpu`. TensorRT is always
   CUDA.
3. The backend owns all pre/post processing: letterbox to the square network
   input (gray padding), BGR→RGB CHW float normalization, decode of the
   NMS-free YOLOv10 head (or raw YOLOv8 head + NMS), and rescaling boxes back
   to original-image pixels.
4. After load, a few **warmup** inferences run; mean/last timings are reported
   on `/diagnostics`.

The inference stack (`onnxruntime` / `tensorrt` + `cuda-python`) is installed
via pip / JetPack, **not** rosdep. Both imports are lazy, so the package
builds, lints, and unit-tests on a machine without them.

---

## Adding a new mission task

No core edits — add a model and a post-processor:

1. **Model:** drop `<model_dir>/<task>.onnx` (exported YOLOv10 graph) and/or a
   Jetson-built `<model_dir>/<task>.engine`. Optionally add
   `<model_dir>/<task>.names` for human-readable class labels.
2. **Post-processor:** add `sub_vision/post_processors/<task>.py`:

   ```python
   from sub_vision.post_processors.base import TaskPostProcessor
   from sub_vision.post_processors.registry import register_post_processor

   @register_post_processor("buoy")
   class BuoyPostProcessor(TaskPostProcessor):
       def process(self, detections, rgb_image, depth_image, camera_info):
           # detections already carry the 2D box; depth_image is always None
           # (no depth camera). Add task-specific work here: fill det.pose /
           # det.pose_valid via cv2.solvePnP on known geometry, set
           # det.distance_m, append det.extra KeyValues, etc.
           return detections
   ```

3. Register the module in `registry._BUILTIN_MODULES` (or import it anywhere the
   node loads). `gate.py` is a worked `cv2.solvePnP` example.

A task with no registered processor still publishes detections with 2D boxes and
the default `distance_m` proxy; only `pose` is left invalid.

---

## Message reference

`Detection.msg` builds on `vision_msgs/Detection2D` (bbox + class id/score) and
adds:

* `float32 bearing_horizontal` / `bearing_vertical` — radians from the camera
  optical axis to the bbox center, computed from the `CameraInfo` intrinsics.
  Positive horizontal = target right of center, positive vertical = target
  below center. Always filled (0 when the source is uncalibrated). This is the
  steering signal `sub_mission`'s vision leaves consume (`AlignToDetection`
  yaws/dives until the relevant bearing is ~0).
* `float32 distance_m` — distance to the object in meters. Default proxy =
  `(bbox_height / image_height) * CAMERA_HEIGHT_M` (`sub_vision/constants.py`);
  a task post-processor may overwrite it with a better estimate.
* `geometry_msgs/Pose pose` + `bool pose_valid` — 6-DOF pose when recoverable.
* `diagnostic_msgs/KeyValue[] extra` — task-specific metadata, so new tasks
  never require a message change.

`DetectionArray.msg` = `Header header` + `string task` + `Detection[] detections`.

---

## Tests

```bash
colcon test --packages-select sub_vision --merge-install
```

There is no unit-test suite yet (and no linters are configured for this
package by design); the command above is a build/packaging sanity check.
