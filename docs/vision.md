# Vision Terms

`sub_mission` vision nodes consume `sub_vision_interfaces/DetectionArray`
messages from the front and down camera vision nodes. Mission XML uses
task-agnostic model names such as `gate`, and mission-to-vision load requests
send those names unchanged.

## Detection

A detection is one object reported by `sub_vision`. It includes the 2D object
result, horizontal and vertical bearings from the camera centerline, optional
distance, optional 6-DOF pose, and task-specific metadata such as `yaw_deg`.

Mission nodes filter detections by:

| Field | Meaning |
|---|---|
| `camera` | `front` or `down` detection stream |
| `task` | Task-agnostic model/task name |
| `class_id` | Optional object class within the model |
| `min_score` | Minimum confidence |

## Align

Align means use detection bearings to center the target in the camera view.

For front-camera alignment, horizontal bearing usually becomes a yaw command.
For down-camera alignment, horizontal and vertical bearings become x/y position
offsets so the sub moves over the target. Some align nodes can also adjust depth
from vertical bearing or measured distance.

Examples:

| Node | Align behavior |
|---|---|
| `AlignToDetection` | Front-style yaw alignment, optionally depth |
| `ForwardContinuousAlign` | Move forward at fixed velocity while yaw-aligning to front detections |
| `DownAlignToDetection` | Center over a down-camera target |
| `DownForwardAlign` | Move forward while applying down-camera centering offsets |

## Orient

Orient means use the object's reported orientation, not just its image center.
The current orientation signal is metadata such as `yaw_deg`, with pose yaw as a
fallback when available.

`OrientToDetectionAtDist` squares the sub to the object's physical orientation
while holding a desired standoff distance. This is different from alignment:
alignment says "look at the object"; orientation says "face the object's plane
at the intended angle."

Example: center on a torpedo board first, then orient to the board face before
shooting.

## Sweep

Sweep means search by trying a pattern of headings and sampling detections at
each heading. A sweep succeeds when enough fresh detections are observed and the
node can steer toward their average bearing.

Examples:

| Node | Sweep behavior |
|---|---|
| `SweepCheck` | Yaw sweep in place |
| `ForwardSweepAlign` | Yaw sweep with forward moves between passes |
| `DownForwardSweepAlign` | Down-camera forward scan/align while moving |

Sweep nodes still load their model internally as a redundant safeguard, but XML
should call `LoadModel` explicitly before the relevant search or align block so
the active model is clear in the behavior tree.
