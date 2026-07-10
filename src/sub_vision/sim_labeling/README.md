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
| `marlin_v2/` | `background` |

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
4. Regenerate `dataset.yaml` for new datasets. Existing annotations keep their
   numeric class IDs and must be migrated if the order changed.
