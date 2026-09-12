# sub_color_correction

ROS 2 wrapper for DeepSeeColor RGB-D underwater color correction.

This package adapts the DeepSeeColor model from `warplab/DeepSeeColor` for streaming use with the Marlin front RGB camera and co-located Stonefish depth camera.

## Dependencies

ROS dependencies:

- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- `message_filters`
- `python3-opencv`

Python dependencies:

- `torch`
- `kornia` recommended
- `numpy`

DeepSeeColor's upstream environment uses PyTorch, torchvision, and Kornia. This streaming node only imports PyTorch directly and uses Kornia for the depth morphology step when available.

## Launch

The depth camera is not on Marlin V3 right now, so no launch file starts this
node; the block that did was removed from `sim_launch.py` (see
[Decision 6](../../../docs/decisions.md)). Run it directly:

```bash
ros2 run sub_color_correction deepseecolor_node --ros-args \
  -r __ns:=/marlin_v3 -p device:=cpu
```

The node subscribes to:

- `/marlin_v3/oak/rgb/image_raw`
- `/marlin_v3/oak/stereo/image_raw`

It publishes:

- `/marlin_v3/oak/rgb/image_color_corrected`

## Smoke Test

After the sim and DeepSeeColor node are running:

```bash
ros2 run sub_color_correction deepseecolor_smoke_test
```

The test waits for raw RGB, depth, and corrected RGB frames, then checks encodings, dimensions, positive depth pixels, and whether corrected output is being published.

## Visual Compare

Show a timestamp-matched raw/corrected pair in an OpenCV window:

```bash
ros2 run sub_color_correction deepseecolor_compare
```

Save a side-by-side PNG instead:

```bash
ros2 run sub_color_correction deepseecolor_compare -- --no-window --save /tmp/deepseecolor_compare.png
```

Crop simulator artifacts out of the top of the comparison:

```bash
ros2 run sub_color_correction deepseecolor_compare -- --no-window --crop-top 120 --save /tmp/deepseecolor_compare.png
```

## Depth Snapshot

Save a normalized depth PNG:

```bash
ros2 run sub_color_correction depth_snapshot -- --save debug_outputs/depth_snapshot.png
```
