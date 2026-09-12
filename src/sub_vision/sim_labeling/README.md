# sim_labeling

`sim_labeling` turns Stonefish segmentation-camera frames into YOLOv11
segmentation labels. It pairs each front RGB frame with the corresponding
16-bit segmentation frame, maps Stonefish object pixel values to YOLO class
IDs, and writes:

- `images/*.png`
- `labels/*.txt`
- `dataset.yaml`

YOLO segmentation rows use:

```text
<class_id> <x1> <y1> <x2> <y2> ... <xN> <yN>
```

Coordinates are normalized to `[0, 1]` relative to the RGB image size.

## Class IDs

The YOLO class ID order is fixed by `get_class_names()` in
`sim_labeling/parse_ids.py`. Keep this table in sync with that function and
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

`background` is not a YOLO class. Objects mapped to `background` receive
`class_id = -1` internally and are excluded from saved annotations.

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

Scenario object names are assigned to classes by prefix matching in
`get_default_prop_definitions()`.

| Object name prefix | Assigned class |
| --- | --- |
| `gate_` | `gate` |
| `path_` | `path` |
| `slalom_` | `slalom` |
| `bin_` | `bin` |
| `torpboard_` | `torpboard` |
| `octagon` | `octagon` |
| `table_` | `table` |
| `bottle_red` | `bottle_red` |
| `bottle_yellow` | `bottle_yellow` |
| `ladle_red` | `ladle_red` |
| `ladle_yellow` | `ladle_yellow` |
| `woollett_` | `background` |
| `natatorium_` | `background` |
| `marlin_v3/` | `background` |

Prefix order matters. If two prefixes can match the same object name, the first
entry in `get_default_prop_definitions()` wins.

## Segmentation Pixel IDs

Stonefish segmentation pixel IDs are not the same as YOLO class IDs:

- Stonefish assigns each rendered object a sequential `objectId` while parsing
  the scenario.
- The segmentation camera writes `objectId + 1` into the 16-bit image.
- Pixel value `0` is background.
- `sim_labeling` computes the current scenario's `segmentation_pixel_value ->
  class_id` mapping at startup.

These pixel IDs depend on the fully expanded scenario XML, include order, and
which objects allocate graphics or physics IDs. They should not be hard-coded
in training data tools. Use `build_pixel_to_class_id()` or the `parse_ids.py`
CLI instead.

## Default Woollett Object IDs 1-30

For the default `woollett.scn.j2` scenario, Stonefish object ID `0` is consumed
by the ocean before entities are parsed. The first scenario entity IDs are
listed below.

`objectId` is Stonefish's internal ID. `seg_px` is the 16-bit segmentation image
pixel value, which is `objectId + 1` for graphical IDs. Physical IDs are listed
because Stonefish allocates them, but they are not the visible segmentation
label used by `sim_labeling`.

| objectId | seg_px | Object | Role | YOLO class |
| ---: | ---: | --- | --- | --- |
| 1 | 2 | `woollett_black_trim` | graphical | background |
| 2 | - | `woollett_black_trim` | physical | background |
| 3 | 4 | `woollett_blue_trim` | graphical | background |
| 4 | - | `woollett_blue_trim` | physical | background |
| 5 | 6 | `woollett_pole_separator_1` | graphical | background |
| 6 | - | `woollett_pole_separator_1` | physical | background |
| 7 | 8 | `woollett_pole_separator_2` | graphical | background |
| 8 | - | `woollett_pole_separator_2` | physical | background |
| 9 | 10 | `woollett_sides` | graphical | background |
| 10 | - | `woollett_sides` | physical | background |
| 11 | 12 | `woollett_bottom` | graphical | background |
| 12 | - | `woollett_bottom` | physical | background |
| 13 | 14 | `gate_red_part` | graphical | `gate` |
| 14 | - | `gate_red_part` | physical | `gate` |
| 15 | 16 | `gate_black_part` | graphical | `gate` |
| 16 | - | `gate_black_part` | physical | `gate` |
| 17 | 18 | `gate_pvc` | graphical | `gate` |
| 18 | - | `gate_pvc` | physical | `gate` |
| 19 | 20 | `gate_left_image` | graphical | `gate` |
| 20 | - | `gate_left_image` | physical | `gate` |
| 21 | 22 | `gate_right_image` | graphical | `gate` |
| 22 | - | `gate_right_image` | physical | `gate` |
| 23 | 24 | `path_stand` | graphical | `path` |
| 24 | - | `path_stand` | physical | `path` |
| 25 | 26 | `path_orange_plastic` | graphical | `path` |
| 26 | - | `path_orange_plastic` | physical | `path` |
| 27 | 28 | `path_beads` | graphical | `path` |
| 28 | - | `path_beads` | physical | `path` |
| 29 | 30 | `slalom_white_poles` | graphical | `slalom` |
| 30 | - | `slalom_white_poles` | physical | `slalom` |

To inspect the IDs for a rendered scenario:

```bash
python3 src/sub_vision/sim_labeling/sim_labeling/parse_ids.py \
  /path/to/scenario.scn \
  /path/to/sub_sim/data
```

The CLI prints every Stonefish object, its graphical object ID, the rendered
segmentation pixel value, its matched prop class, and the final prop groups.

## Updating Classes

When adding, removing, or reordering classes:

1. Update `get_class_names()` in `sim_labeling/parse_ids.py`.
2. Update `get_default_prop_definitions()` if new scenario object prefixes
   should be annotated.
3. Update this README's class and prefix tables.
4. Append new classes as IDs `11+` unless intentionally migrating a whole
   dataset and model.
5. Regenerate `dataset.yaml` for new datasets. Existing annotations keep their
   numeric class IDs and must be migrated if the order changed.
