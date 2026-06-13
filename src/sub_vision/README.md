# sub_vision

Perception for the Marlin V2 AUV: YOLOv10 detection with per-task OpenCV
post-processing and depth-based 3D metadata. Inference runs through
**TensorRT** (serialized engine) or **ONNX Runtime**; pre-processing
(letterbox) and post-processing (decode + box rescaling) live in
`backends.py`. Targets the Jetson AGX Orin (TensorRT, CUDA) in production and
any x86 box (GPU or CPU-only ONNX Runtime) for dev — the **same code** runs on
both; only the selected backend/device differs.

This directory holds two ROS 2 (Jazzy) packages:

| Package | Build type | Contents |
| --- | --- | --- |
| `sub_vision_interfaces` | `ament_cmake` | `Detection.msg`, `DetectionArray.msg`, `LoadModel.srv` |
| `sub_vision` | `ament_python` | model manager, post-processors, depth metadata, node |

> The detector is **YOLOv10 (NMS-free / end-to-end)**: the exported head emits
> `(1, N, 6)` boxes directly, so the pipeline runs **no** separate NMS step.
> Raw YOLOv8-style exports (`(1, 4+nc, M)`) are also decoded, with NMS applied
> in the backend.

---

## The topic contract (real camera == sim)

`sub_vision` subscribes to one set of topics regardless of whether it is fed by
the real OAK-D Pro or the simulator. There are **no `if sim:` branches**
downstream — the Stonefish bridge (`sub_sim_sensors/sim_oak_camera_remapper`)
makes the sim output byte-compatible with the real driver.

All topics are relative to the node namespace (e.g. `/marlin_v2`):

| Topic | Type | Encoding / units |
| --- | --- | --- |
| `oak/rgb/image_raw` | `sensor_msgs/Image` | `bgr8` |
| `oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | intrinsics |
| `oak/stereo/image_raw` | `sensor_msgs/Image` | `16UC1`, **millimeters**, `0` = no return |
| `oak/stereo/camera_info` | `sensor_msgs/CameraInfo` | intrinsics |
| **out:** `vision/detections` | `sub_vision_interfaces/DetectionArray` | detections + 3D metadata |
| **out:** `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | model state + timing |

| Source | How it produces the contract |
| --- | --- |
| **Real** | `depthai_ros_driver_v3` with RGB↔depth alignment enabled (`config/oak_d_pro.yaml`). Depth is native `16UC1` mm. |
| **Sim** | Stonefish publishes `front_camera` (rgb8) + `depth_camera` (`32FC1` m). `sim_oak_camera_remapper` converts → `bgr8` + `16UC1` mm, remaps topics/`frame_id`, passes `CameraInfo`, keeps sim timestamps. |

Bounding boxes, `distance_m`, and `pose` are all expressed in the camera
**optical** frame (`header.frame_id` from the source image).

### Launch

`pool_test_launch.py` selects the camera source with a launch arg:

```bash
ros2 launch sub_bringup pool_test_launch.py                  # real OAK-D driver
ros2 launch sub_bringup pool_test_launch.py use_sim:=true    # Stonefish bridge
```

`sim_launch.py` brings up Stonefish + the bridge + `sub_vision` for full sim.

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
           # detections already carry the 2D box + depth-derived distance_m.
           # Add task-specific work here: fill det.pose / det.pose_valid via
           # cv2.solvePnP on known geometry, append det.extra KeyValues, etc.
           return detections
   ```

3. Register the module in `registry._BUILTIN_MODULES` (or import it anywhere the
   node loads). `gate.py` is a worked `cv2.solvePnP` example.

A task with no registered processor still publishes detections with 2D boxes and
`distance_m`; only `pose` is left invalid.

---

## Message reference

`Detection.msg` builds on `vision_msgs/Detection2D` (bbox + class id/score) and
adds:

* `float32 distance_m` — median valid depth inside the bbox (NaN if none).
* `geometry_msgs/Pose pose` + `bool pose_valid` — 6-DOF pose when recoverable.
* `diagnostic_msgs/KeyValue[] extra` — task-specific metadata, so new tasks
  never require a message change.

`DetectionArray.msg` = `Header header` + `string task` + `Detection[] detections`.

---

## Tests

```bash
colcon test --packages-select sub_vision sub_sim_sensors --merge-install
```

* `sub_vision` (pytest): depth distance estimation / deprojection
  (`test_metadata.py`) and the post-processor registry
  (`test_post_processor_registry.py`).
* `sub_sim_sensors` (gtest): the meters→millimeters depth conversion
  (`test_depth_conversion.cpp`).
