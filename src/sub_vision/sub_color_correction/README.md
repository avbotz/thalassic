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

## Launch With Sim

```bash
ros2 launch sub_bringup sim_launch.py enable_deepseecolor:=true
```

Use CPU if CUDA is unavailable:

```bash
ros2 launch sub_bringup sim_launch.py enable_deepseecolor:=true deepseecolor_device:=cpu
```

The node subscribes to:

- `/marlin_v2/oak/rgb/image_raw`
- `/marlin_v2/oak/stereo/image_raw`

It publishes:

- `/marlin_v2/oak/rgb/image_color_corrected`

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
