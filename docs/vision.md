# Vision Terms

`sub_mission` vision nodes consume `sub_vision_interfaces/DetectionArray` messages from the front and down camera vision nodes. Mission XML uses task-agnostic model names such as `gate`, and mission-to-vision load requests send those names unchanged.

## Detection

A detection is one object reported by `sub_vision`. It includes the 2D object result, horizontal and vertical bearings from the camera centerline, optional distance, optional 6-DOF pose, and task-specific metadata such as `yaw_deg`.

Mission nodes filter detections by:

| Field | Meaning |
|---|---|
| `camera` | `front` or `down` detection stream |
| `task` | Task-agnostic model/task name |
| `class_id` | Optional object class within the model |
| `min_score` | Minimum confidence |

`AlignToDetection`, `LateralAlignToDetection`, `ForwardContinuousAlign`, `OrientToDetectionAtDist` and the yaw sweeps (`SweepCheck`, `ForwardSweepAlign`, `SweepAngle`) steer on front-camera bearings and fail on `camera="down"`. The `Down*` nodes have no `camera` port and read the down camera.

## Tasks

Each task's post-processor in `sub_vision` shapes what missions see. Every `extra` key, with its units and sign, is in [the sub_vision README](../src/sub_vision/README.md#post-processors).

| Task | Model | Mission-facing output |
|---|---|---|
| `gate` | `ffc_rs_26` | Class `8`: the left lane, below the left panel's role icons, with `gate_role` (`SURVEY`/`SEARCH`) and range from icon size |
| `slalom`, `slalom_redpoles_osu` | same name | Class `2`: the nearest red pole, ranged from its height |
| `path`, `path_marker` | same name | `yaw_deg` |
| `torp`, `torp_find` | same name | Board pose by PnP, `heading_yaw_deg`, aim-point bearings; class `torp_aim_hole` is the hole to fire at |
| `octagon_table`, `octagon_search_image` | same name | Range from known widths; search images as class `0` (SOS, buoy) or `1` (compass, hammer and wrench) |

The trees also load `torp_fire_blood` (front camera in `torp.xml`, down in `bins.xml`), which has no post-processor: 2D boxes and bearings only, `distance_m` NaN.

Models are `<task>.engine` or `<task>.onnx` in `weights/` at the workspace root, except where `sub_vision/model_manager.py` maps a task to another model (`gate` → `ffc_rs_26`).

## Align

Align means use detection bearings to center the target in the camera view.

For front-camera alignment, horizontal bearing usually becomes a yaw command. For down-camera alignment, horizontal and vertical bearings become x/y position offsets so the sub moves over the target. Some align nodes can also adjust depth from vertical bearing or measured distance.

Examples:

| Node | Align behavior |
|---|---|
| `AlignToDetection` | Front-camera yaw alignment, optionally depth |
| `ForwardContinuousAlign` | Move forward at a fixed velocity while yaw-aligning to front detections |
| `LateralAlignToDetection` | Move sideways (and in z) onto a front detection, keeping the heading |
| `DownAlignToDetection` | Center over a down-camera target |
| `DownForwardAlign` | Move forward while applying down-camera centering offsets |

## Orient

Orient means use the object's reported orientation, not just its image center. The orientation signal is metadata: `yaw_deg` or `heading_yaw_deg` (below). `sub_mission` does not read the pose as a yaw.

Orientation metadata is the yaw change that would square the vehicle up with the object, in degrees, counter-clockwise positive (REP-103): `yaw_deg` (path marker, a line, so in \[-90, 90)) and `heading_yaw_deg` (torpedo board face). A `pose_semantics` other than `orientation` says the pose's rotation is not a vehicle yaw.

`OrientToDetectionAtDist` squares the sub to the object's physical orientation while holding a desired standoff distance. This is different from alignment: alignment says "look at the object"; orientation says "face the object's plane at the intended angle."

Example: center on a torpedo board first, then orient to the board face before shooting.

## Sweep

Sweep means search by trying a pattern of headings and sampling detections at each heading, or, on the down camera, a pattern of positions. A yaw sweep succeeds when enough fresh detections are observed; the node then turns to the average of the headings toward them, each taken from the heading it was seen on.

Examples:

| Node | Sweep behavior |
|---|---|
| `SweepCheck` | Yaw sweep in place |
| `ForwardSweepAlign` | Yaw sweep with forward moves between passes |
| `DownForwardSweepAlign` | Down-camera forward scan/align while moving |

Only `LoadModel` loads a model: XML calls it before the relevant search or align block, which also keeps the active model clear in the behavior tree.
