# sub_vision

Perception for the Marlin V3 AUV: YOLO detection and segmentation with
per-task OpenCV post-processing. Inference runs through
**TensorRT** (serialized engine) or **ONNX Runtime**; pre-processing
(letterbox) and post-processing (decode, box rescaling, instance masks) live
in `backends.py`. Targets the Jetson AGX Orin (TensorRT, CUDA) in production and
any x86 box (GPU or CPU-only ONNX Runtime) for dev — the **same code** runs on
both; only the selected backend/device differs.

The pipeline is two ROS 2 (Jazzy) packages in this directory (beside
`sim_labeling` and `sub_color_correction`, which have their own READMEs):

| Package | Build type | Contents |
| --- | --- | --- |
| `sub_vision_interfaces` | `ament_cmake` | `Detection.msg`, `DetectionArray.msg`, `LoadModel.srv` |
| `sub_vision` | `ament_python` | model manager, post-processors, node |

> End-to-end exports (YOLOv10, YOLO26) emit `(1, N, 6)` boxes directly, so
> the pipeline runs **no** separate NMS step. Raw YOLOv8/11-style exports
> (`(1, 4+nc, M)`) are also decoded, with NMS applied in the backend.
> Segmentation exports add mask coefficients to each box and a
> `(1, 32, H, W)` prototype output; the backend turns them into one
> image-sized mask per detection for the post-processors. ONNX Runtime reads
> which head a model has from its export metadata (`end2end`, `names`);
> TensorRT engines have none, so it is read from the output shapes.

---

## The topic contract (real camera == sim)

`sub_vision` reads one RGB stream plus its intrinsics and publishes detections.
The input topics are plain parameters, so there are **no `if sim:` branches and
no bridge nodes** — each launch file points the node at its camera source and
`cv_bridge` normalizes any 8-bit color encoding to `bgr8` on receipt.

All topics are relative to the node namespace (e.g. `/marlin_v3`):

| Topic (parameter) | Type | Notes |
| --- | --- | --- |
| `rgb_topic` (default `front_camera/image_raw`) | `sensor_msgs/Image` | any 8-bit color encoding |
| `camera_info_topic` (default `front_camera/camera_info`) | `sensor_msgs/CameraInfo` | intrinsics; also feed the per-detection bearings |
| **out:** `detections_topic` (default `vision/detections`) | `sub_vision_interfaces/DetectionArray` | detections + bearings + 3D metadata |
| **out:** `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | model state + timing, one status per instance |
| **out:** `~/depth`, `~/depth/color` | `sensor_msgs/Image` | relative depth (32FC1) and a Turbo view (bgr8), only with `depth_debug` |

`sim_launch.py` and `pool_test_launch.py` each run one instance per camera
from the shared `config/sub_vision.yaml`, setting the topics (`vision_node` in
`sub_bringup/entities.py`):

| Node | Camera | `rgb_topic` (sim / vehicle) | `detections_topic` |
| --- | --- | --- | --- |
| `sub_vision` | front | `front_camera/image_color` / `front_camera/image_raw` | `vision/detections` |
| `sub_vision_down` | down | `down_camera/image_color` / `down_camera/image_raw` | `vision/detections_down` |

The depth camera is **not** used: `sub_vision` subscribes to RGB only.
Bounding boxes, bearings, and `pose` are expressed in the camera **optical**
frame (`header.frame_id` from the source image). Bearings go through the
`CameraInfo` distortion model (`geometry.py`).

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
ros2 service call /marlin_v3/sub_vision/load_model \
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

Models live in `model_dir` (default `weights`; a relative path is from the
workspace root, `$PIXI_PROJECT_ROOT`, so `<repo>/weights` in sim and on the
vehicle) as exported graphs:

* `<model>.engine` — serialized **TensorRT** engine (device-specific; built on
  the Jetson it runs on).
* `<model>.onnx` — exported **ONNX** graph for ONNX Runtime.

`pixi run fetch-weights` downloads the ONNX files listed in
`weights/manifest.toml` from the GitHub releases
([docs/vision.md](../../docs/vision.md#model-weights)).

The model is the task's name unless `model_manager.TASK_MODELS` maps it to
another file: `gate` runs `ffc_rs_26`.

`sub_mission` requests role-agnostic model names. For example,
mission XML `task="gate"` calls `LoadModel` with `task: 'gate'`. A task with a
`_survey` or `_search` suffix and no post-processor of its own uses its base
task's (`gate_survey` → `gate`). The suffix affects only that lookup: loading
`gate_survey` still needs a `gate_survey` model file or a `TASK_MODELS` entry.

Selection, per the `backend` param:

1. `auto` (default) prefers `<task>.engine` (TensorRT) and falls back to
   `<task>.onnx` (ONNX Runtime). `tensorrt` / `onnxruntime` force one and fail
   if the file is missing.
2. The `device` param applies to ONNX Runtime only: `auto` picks `cuda` when
   the CUDA execution provider is available, else `cpu`. TensorRT is always
   CUDA.
3. The backend owns all pre/post processing: letterbox to the square network
   input (gray padding), BGR→RGB CHW float normalization, decode of an
   end-to-end head (or a raw YOLOv8/11 head + NMS) and its segment masks, and
   rescaling boxes back to original-image pixels.
4. After load, `warmup_iterations` (3) **warmup** inferences run on a blank
   frame; mean/last timings are reported on `/diagnostics`. A warmup failure
   fails the load, and the previous model stays active.

`onnxruntime` comes from pixi; `tensorrt` and `cuda-python`
(`cuda.bindings.runtime`) from JetPack on the Jetson
(`deploy/jetson/link_jetpack_python.sh`). Both imports are lazy, so the
package builds and lints on a machine without them.

---

## Relative depth (Depth Anything V2)

`depth_enabled: true` runs Depth Anything V2 Small (`depth_model`, default
`depth_anything_v2_vits`, looked up in `model_dir` like a task, with the same
`backend` and `device`) beside the detector, at most `depth_rate_hz` (5 Hz)
counted from the end of the last run. It runs in the frame callback, so each
run delays that frame's detection (~0.4 s on a laptop CPU). Its output is
relative inverse depth (larger is nearer, no scale) at the image's
resolution; it is handed to the post-processor only with the frame it was
computed from, and is `None` on the frames between. Only `torp` uses it. Off
by default; `depth_debug: true` publishes it on `~/depth` and `~/depth/color`.

---

## Annotation visualizer

`annotation_visualizer` draws `vision/detections` on `front_camera/image_color`
(`image_transport: compressed` reads `<image>/compressed`) and publishes
`vision/debug_image` and its JPEG copy `vision/debug_image/compressed`. It
draws only while one of them has a subscriber.

---

## Adding a new mission task

No core edits — add a model and a post-processor:

1. **Model:** drop `<model_dir>/<task>.onnx` (an ultralytics detect or segment
   export: end-to-end YOLOv10/YOLO26, or a raw YOLOv8/11 head) and/or a
   Jetson-built `<model_dir>/<task>.engine`.
2. **Post-processor:** add `sub_vision/post_processors/<task>.py`:

   ```python
   from sub_vision.post_processors.base import TaskPostProcessor
   from sub_vision.post_processors.registry import register_post_processor

   @register_post_processor("buoy")
   class BuoyPostProcessor(TaskPostProcessor):
       def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
           # detections already carry the 2D box; model_masks has each one's
           # mask from a segmentation model, depth_image the relative depth
           # on frames it ran on (else None). Add task-specific work here:
           # fill det.pose / det.pose_valid via cv2.solvePnP on known
           # geometry, set det.distance_m, append det.extra KeyValues, etc.
           return detections
   ```

3. Register the module in `registry._BUILTIN_MODULES` (or import it anywhere the
   node loads). `torp.py` is a worked `cv2.solvePnP` example. The node keeps
   one processor per task, so it may carry state between frames.

A task with no registered processor still publishes detections with 2D boxes and
bearings; `distance_m` is NaN and `pose` is left invalid.

---

## Post-processors

Angles follow `Detection.msg`: bearings are radians in the optical convention
(positive = right of / below center). Yaws are degrees, counter-clockwise
positive (REP-103): the heading change that would square the vehicle up.
Values in `extra` are strings.

| Task | Model (classes) | Output |
| --- | --- | --- |
| `gate` | `ffc_rs_26` (0 blood, 1 buoy, 2 compass, 3 circle, 4 fire, 5 hammer_and_wrench, 6 slalom, 7 sos) | the role icons (1, 2, 5, 7) left to right, after a synthetic class `8` left-lane target when the left panel's role is known |
| `slalom`, `slalom_redpoles_osu` | `slalom` (0 red pole, 1 white pole), `slalom_redpoles_osu` (0 red pole) | only the nearest red pole, as class `2` |
| `path`, `path_marker` | `path`, `path_marker` (0 marker); down camera | every detection, with the marker's yaw; one OpenCV-found detection when the model finds none |
| `torp`, `torp_find` | `torp` (0 board, 1 hole; segments), `torp_find` (0 board) | boards by score with PnP pose, then the rest; the hole to fire at as class `torp_aim_hole` |
| `octagon_table` | table | `distance_m` from the table's 0.546 m width |
| `octagon_search_image` | 0 SOS, 1 hammer_and_wrench, 2 compass, 3 buoy | classes merged by role: `0` = SOS + buoy, `1` = compass + hammer_and_wrench; `distance_m` from width (0.4 m, 0.3 m) |

### gate

The panels are split at the widest gap between icons; with one panel in view
it counts as the left one only while near the image's left half. The class-8
target spans the left panel's icons, at their score-weighted centre and the
principal point's row: `bearing_vertical` is 0, `distance_m` the median over
its icons of `0.152 m * fx / icon width`, `pose_valid` false. Its `extra`:

| Key | Meaning |
| --- | --- |
| `gate_target` | `left_opening` |
| `gate_role` | `SURVEY` or `SEARCH`: the left panel's role, by summed icon score |
| `gate_role_label` | `Survey & Repair` or `Search & Rescue` |
| `left_icon_classes`, `right_icon_classes` | comma-separated class ids on each panel |
| `left_icon_names`, `right_icon_names` | the same as names |
| `two_panel_view` | `true` when both panels are in view |
| `estimated_distance_m` | `distance_m`, m |
| `aim_bearing_horizontal`, `aim_bearing_vertical` | `bearing_horizontal` (rad) and 0 |
| `pose_semantics` | `left_gate_lane` |

### slalom

Class-0 boxes scoring at least 0.10 whose crop is at least 5% clearly red,
deduplicated (IoU 0.7); the tallest wins. `distance_m` is
`0.90 m * fy / box height`, its range along the optical axis; `pose_valid` is
false.

| Key | Meaning |
| --- | --- |
| `target` | `red_divider_center` |
| `pole_range_m` | `distance_m`, m |
| `source_class` | the model's class for it (`0`) |
| `pose_semantics` | `pole_center` |

### path

The marker's long axis is estimated inside its box from the model's mask (if
any), Otsu blobs darker and brighter than the floor, and line segments. Two of
them agreeing within 12° are averaged; otherwise only one with confidence of
at least 0.68 is used. A box touching the image edge, or holding two similar
warm blobs, gets no yaw. The fallback detection (class `0`, score 0.25) is the
most central closed elongated dark blob.

| Key | Meaning |
| --- | --- |
| `detection_source` | `opencv_fallback` on the fallback detection only |
| `target` | `path_marker` |
| `orientation_status` | `available` or `unavailable` |
| `yaw_deg` | deg, CCW positive, in [-90, 90): the yaw that lines the vehicle up with the marker. The marker is a line, so its ends are not told apart. Assumes the down camera's image top is forward and image right starboard (its mount in `marlin_v3.yaml`) |
| `orientation_confidence` | 0-1 |
| `orientation_source` | the estimates used, `+`-joined: `model_mask`, `dark_contour`, `bright_contour`, `line_segments` |
| `pose_semantics` | `orientation` |

### torp

The board's corners come from the first that works: the relative depth map
(`depth_relative`: a plane in the box with an edge inside the padded box), the
model's mask (`yolo_segment`), the board's colour in the box
(`opencv_contour`: an outline inside the padded box), or the box
(`bbox_fallback`). `cv2.solvePnP` (IPPE) on a
0.6096 m square gives the board pose in the optical frame: `pose.position` is
the board centre, `pose.orientation` the board frame (x right, y down, z into
the board). `bearing_horizontal` is `atan2(x, z)` and `distance_m`
`hypot(x, z)` (horizontal range) of that centre. Holes stay in the output; the
confident hole (score ≥ 0.5) nearest the aim point on the solved board, or
without a pose nearest the box's upper centre, becomes class `torp_aim_hole`
with `hole_role: aim_target`.

| Key | Meaning |
| --- | --- |
| `pose_std_dev_txyz_rxyz` | six comma-separated standard deviations (m, m, m, rad, rad, rad) per pixel of corner noise, from the reprojection Jacobian, or `unavailable` |
| `image_roll_deg` | deg, the board's twist in the image (telemetry, not steering) |
| `pnp_reprojection_error_px` | RMS px; fits over 50 px are rejected |
| `bbox_corners_px` | the corners used, `x,y;x,y;x,y;x,y` (top-left, bottom-left, bottom-right, top-right) |
| `corner_source` | `depth_relative`, `yolo_segment`, `opencv_contour` or `bbox_fallback` |
| `corner_quality` | 0-1, how well the corners fit the box |
| `pose_semantics` | `board_normal_heading`: use `heading_yaw_deg`, not the pose orientation, to turn |
| `heading_status` | `available`, or `unavailable` when PnP solved `bbox_fallback` corners or a box at the image edge (neither shows which way the board faces) |
| `heading_yaw_deg` | deg, CCW positive: the yaw that squares the camera to the board face |
| `aim_point_status` | `available` or `unavailable` (box corners, a box at the image edge, depth-plane pose) |
| `aim_bearing_horizontal`, `aim_bearing_vertical` | rad, optical convention: the aim point (board x 0, y −0.18 m, the simulator board's upper-centre hole) on the solved board |
| `pose_error` | why PnP failed |
| `depth_status` | `validated` (depth calibrated on this solve), `calibration_rejected`, `fallback` (pose from depth), or `unavailable` (PnP failed, no fallback: `pose_valid` false, `distance_m` NaN) |
| `depth_pose_source` | `pnp_validated` or `cached_pnp_calibrated_plane` |
| `depth_range_m` | m, median calibrated depth over the board |
| `depth_pnp_range_residual_m` | m, `depth_range_m` minus the PnP centre's z |
| `depth_calibration_age_s` | s, age of the calibration a fallback used (at most 0.5) |
| `depth_plane_residual_m` | m, median distance of the depth points from the fallback plane |

With depth on, a PnP solve on the best board's whole outline (not box corners,
nor a box at the image edge) fits metres to the relative depth on the board
plane. When PnP fails on a depth frame within 0.5 s of one, the pose comes
from a plane fitted to the calibrated depth inside the corners: position and
heading only (`aim_point_status: unavailable`).

---

## Message reference

`Detection.msg` builds on `vision_msgs/Detection2D` (bbox + class id/score) and
adds:

* `float32 bearing_horizontal` / `bearing_vertical` — radians from the camera
  optical axis to the bbox center (or a point the post-processor knows better:
  gate, torp), computed from the `CameraInfo` intrinsics and distortion.
  Positive horizontal = target right of center, positive vertical = target
  below center. Always filled; 0 when the source is uncalibrated (`CameraInfo`
  K zero), which the node logs a throttled warning for. This is the
  steering signal `sub_mission`'s vision leaves consume (`AlignToDetection`
  yaws/dives until the relevant bearing is ~0).
* `float32 distance_m` — distance to the object in meters; NaN unless the
  task's post-processor ranges it (e.g. from a known width or `cv2.solvePnP`).
* `geometry_msgs/Pose pose` + `bool pose_valid` — 6-DOF pose when recoverable.
* `diagnostic_msgs/KeyValue[] extra` — task-specific metadata, so new tasks
  never require a message change.

`DetectionArray.msg` = `Header header` + `string task` + `Detection[] detections`.

---

## Tests

```bash
colcon test --packages-select sub_vision --merge-install
```

There is no unit-test suite yet (and no ament linters by design: `pixi run
lint` checks it with ruff, with the rest of the repo); the command above is a
build/packaging sanity check.
