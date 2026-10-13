# sim_labeling

`sim_labeling` turns Stonefish segmentation-camera frames into YOLOv11
segmentation labels. It pairs each front RGB frame with the segmentation frame
rendered with it, maps Stonefish object pixel values to YOLO class IDs, and
writes:

- `images/*.png`
- `labels/*.txt`
- `dataset.yaml`

YOLO segmentation rows use:

```text
<class_id> <x1> <y1> <x2> <y2> ... <xN> <yN>
```

Coordinates are normalized to `[0, 1]` relative to the RGB image size.

Each saved frame is `images/seg_<wall-clock time>.png` with
`labels/seg_<wall-clock time>.txt`, under the `output_dir` parameter (default
`train_imgs`, relative to the working directory). Frames showing no annotated
prop are not saved. Each connected region of an object gets its own polygon,
which follows the region's outline to within a pixel; regions smaller than
`min_bbox_area` pixels (default 400) are dropped. `dataset.yaml` is written only
if it is missing, and lists the same `images` for `train` and `val`: split the
images before validating a model on them.

Run it with `labeling:=true` on `sim_launch.py`, which adds the segmentation
camera to the robot and starts the node.

## How Frames Are Matched

Stonefish renders one scheduled camera per frame, so on its own the
segmentation camera would never see the same scene as the front camera. The
robot scenario pairs it with the front camera instead
(`<paired_camera name="front_camera"/>` in `layout.scn.j2`): it is rendered in
the same frames as the front camera, from its viewpoint and with its projection,
at its own rate (10 Hz). Both images of such a frame are stamped with the
frame's capture time, and the node matches them exactly by stamp
(`message_filters.TimeSynchronizer`). Front frames without a segmentation frame
are dropped.

## Class IDs

The YOLO class ID order is fixed by `get_class_names()` in
`sim_labeling/classes.py`. Keep this table in sync with that function and
with any trained model or dataset generated from this package.

| Class ID | Class name |
| ---: | --- |
| 0 | `gate` |
| 1 | `path` |
| 2 | `slalom` |
| 3 | `bin` |
| 4 | `torpboard` |
| 5 | `octagon` |
| 6 | `table` |
| 7 | `bottle_red` |
| 8 | `bottle_yellow` |
| 9 | `ladle_red` |
| 10 | `ladle_yellow` |

`background` is not a YOLO class. Objects mapped to `background` are excluded
from saved annotations.

## Class IDs 11+

There are currently no defined YOLO classes past ID `10`. IDs `11` and above
are available for future mission props, but they are not reserved for any
specific object until they are added to `get_class_names()`.

When adding a new class, append it to the end of `get_class_names()` instead of
inserting it into the middle of the list. For example, the next new class should
be ID `11`, then `12`, and so on. Reusing or reordering existing IDs changes
the meaning of already-generated `.txt` labels and any model trained from
those labels.

Do not add a class ID here without also adding a matching object-name prefix in
`get_default_prop_definitions()` unless the class is intentionally unused. The
labeling node only emits annotations for objects that match a non-background
prefix and resolve to a class name in `get_class_names()`.

## Object Name Prefixes

Stonefish object names are assigned to classes by prefix matching in
`get_default_prop_definitions()`.

| Object name prefix | Assigned class |
| --- | --- |
| `woollett_` | `background` |
| `natatorium_` | `background` |
| `gate` | `gate` |
| `path` | `path` |
| `slalom` | `slalom` |
| `bin` | `bin` |
| `torpboard_` | `torpboard` |
| `octagon` | `octagon` |
| `table` | `table` |
| `bottle_red` | `bottle_red` |
| `bottle_yellow` | `bottle_yellow` |
| `ladle_red` | `ladle_red` |
| `ladle_yellow` | `ladle_yellow` |
| `marlin_v3` | `background` |

Each prop is one static named after the prop. Stonefish numbers repeated
names (`slalom`, `slalom1`, `slalom2`), so the prefixes have no trailing
underscore. Objects matching no prefix are background.

Prefix order matters. If two prefixes can match the same object name, the first
entry in `get_default_prop_definitions()` wins.

## Segmentation Pixel Values

Stonefish segmentation pixel values are not YOLO class IDs:

- Stonefish draws each named object (static, dynamic or animated entity, robot,
  sensor, actuator) with its own pixel value, shared by all of its meshes. A
  robot's links are one object, `<robot>_Dynamics`; its sensors and thrusters
  are `<robot>/<name>`.
- Pixel value `0` is background, and `65534` marks ocean particles (marine
  snow), named `ocean_particles`.
- The simulator publishes the object name of every pixel value as a latched
  `vision_msgs/LabelInfo` on `sim/segment/label_info`, republished when objects
  are added. The node maps those names to classes when it arrives.
- Pixel values are assigned in the order objects are first drawn, so they can
  change between scenarios. They should not be hard-coded in training data
  tools; use the `label_info` topic.

To inspect the labels of a running simulation:

```bash
ros2 topic echo --once --qos-durability transient_local \
  /marlin_v3/sim/segment/label_info
```

The node also logs the objects it found for each class when the labels arrive.

## Updating Classes

When adding, removing, or reordering classes:

1. Update `get_class_names()` in `sim_labeling/classes.py`.
2. Update `get_default_prop_definitions()` if new scenario object prefixes
   should be annotated.
3. Update this README's class and prefix tables.
4. Append new classes as IDs `11+` unless intentionally migrating a whole
   dataset and model.
5. Regenerate `dataset.yaml` for new datasets. Existing annotations keep their
   numeric class IDs and must be migrated if the order changed.
