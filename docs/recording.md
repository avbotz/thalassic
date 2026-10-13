# Recording

`pixi run sim` and `pixi run pool` can record a run into one [MCAP](https://mcap.dev) file with `record:=true`: every topic, zstd-compressed, with the cameras stored as JPEG. Foxglove opens the file directly (*Open local file*), and `ros2 bag` plays it back.

```
bags/2026-10-08_18-00-00_sim_prelim/
├── 2026-10-08_18-00-00_sim_prelim_0.mcap
└── metadata.yaml
```

Each run gets a folder named after its start time, `sim` or the robot's name, and the mission if there is one. `bags/` is in the workspace root and ignored by git. The recorder is `ros2 bag record`, started by `recorder()` in `sub_bringup/sub_bringup/entities.py`; the launch stops it when the run ends, which finishes the file.

| Launch argument | Default | |
|---|---|---|
| `record` | `false` | `true` records the run |
| `bag_dir` | `<workspace>/bags` | Where each run's folder goes |
| `jpeg_quality` | `85` | JPEG quality of the cameras' compressed images, which the bag records and Foxglove shows. image_transport's own default is 95 |

```bash
pixi run sim record:=true
pixi run pool record:=true bag_dir:=/mnt/ssd/bags jpeg_quality:=95
```

## What a bag holds

Every topic as it was published, including `/tf_static`, `/rosout` (every node's log) and `/diagnostics`, plus, in the sim, the robot description and the ground truth (`sim/odometry`, `sim/objects/*`). In the sim the bag is stamped with sim time, like the messages in it. These are left out:

| Left out | Why |
|---|---|
| Raw colour camera images (`front_camera/image_color` and `down_camera/image_color` in the sim, `front_camera/image_raw` on the vehicle) | Recorded as JPEG instead: the `<image>/compressed` copy that image_transport publishes beside them |
| image_transport's `theora`, `zstd` and `compressedDepth` copies | The same images again |
| `vision/debug_image` and `vision/debug_image_down`, raw and JPEG | Drawn from the cameras' images and the detections, which are both recorded. The annotators only draw while something subscribes, so recording them would also cost the CPU to draw them |
| `sub_vision/depth`, `sub_vision/depth/color` (in the sim with `vision_depth:=true`) | Depth Anything's maps, for viewing: about 6 MB a frame |
| `/clock` (sim) | The bag's timestamps are already sim time; `ros2 bag play --clock` publishes it again |
| `sim/segment` (sim, with `labeling:=true`) | `sim_labeling` saves what the segmentation camera sees itself |

The vehicle's down camera (the Blackfly) is the exception to JPEG: it sends a Bayer mosaic, which the JPEG copy stores as a grey image, so its `image_raw` is recorded raw. That is lossless (zstd in the bag), and Foxglove shows it in colour. So is the sim's depth camera (`depth_camera:=true`): its `sim/depth_camera/image_depth` is 32-bit float depth in metres, which JPEG cannot hold, so it is recorded raw and its JPEG copy is left out.

The bag's metadata (`custom_data` in `metadata.yaml`, and the MCAP's `rosbag2` metadata record) holds the launch arguments that pick the run (`robot_name`, `mission`, `role`, `gains`, and in the sim `seed`, `course`, `current`, `sensor_noise`, `DX`...`DYAW`) and the commit, from `git describe --always --dirty`. The sim always seeds its scenario, with a random seed when none is given, so a recorded run's course can be rebuilt with `pixi run sim seed:=<seed> course:=<course>`.

## Size

Images are almost all of it. Measured in the sim with the `prelim` mission: two 1440×1080 cameras, at the 9-13 Hz the laptop renders them at. The sim's cameras now match the vehicle's (front 1920×1080 at 24 Hz, down 1280×1024 at 5 Hz), so these figures need measuring again.

| | Per frame | At 13 Hz |
|---|---|---|
| Front camera, JPEG 85 | 424 KB | 5.5 MB/s |
| Down camera, JPEG 85 | 152-162 KB | 2.0 MB/s |
| Everything else | | 0.16 MB/s |

That comes to 0.3-0.45 GB per minute of sim. At image_transport's default quality of 95 the frames are 750 KB and 295 KB, nearly twice the size. Raw frames would be 4.7 MB each, 120 MB/s for both cameras. The sim's front camera is the expensive one because the pool's caustics are fine detail everywhere, which JPEG compresses poorly. The vehicle's sizes have not been measured yet.

With `depth_camera:=true` the depth camera is most of the bag. Its 1920×1080 float frames are 8.3 MB each, which zstd shrinks to 1.1-1.4 MB depending on the scene. In a two-minute sim run it rendered at 9.8 Hz, about 0.8 GB a minute, and the whole bag came to 1.2 GB a minute. Where nothing is within its 20 m, including the water surface, a pixel is 0.

The MCAP is compressed in chunks with zstd at level 1 (`sub_bringup/config/mcap_writer.yaml`), which shrinks everything but the images to a quarter, at over a gigabyte a second. zstd's fastest level, which the `zstd_fast` preset uses, left that a sixth larger and the depth camera's frames twice the size. Level 19 (the `zstd_small` preset) saved only another 10% and compressed at 6-15 MB/s on a laptop core, slower than the depth camera alone sends data. JPEG does not compress at any level.

**Why JPEG, not video:** H.264 (`foxglove_compressed_video_transport`, which Foxglove plays) came out only 1.1-1.4 times smaller than JPEG at the same PSNR on the sim's frames. The caustics change every frame, so there is little for the encoder to reuse between frames. That gain did not justify a new dependency, frames that depend on earlier keyframes, and losing direct replay: `sub_vision` reads JPEG with `image_transport: compressed`. Real pool footage is smoother, so video is worth measuring again once there are vehicle bags.

## Replay

```bash
ros2 bag info bags/<run>
ros2 bag play bags/<run> --clock 100     # sim bags: nodes need use_sim_time:=true
```

Recording made no difference to the cameras' frame rate beyond run-to-run noise: in back-to-back sim runs they ran at 11-14 Hz with and without it.

## If the recorder is killed

If the recorder is killed rather than stopped (power lost, `kill -9`), the `.mcap` file is still there, but without its index (Foxglove still opens it, more slowly) or the `metadata.yaml` that `ros2 bag` needs. Rebuild that with:

```bash
ros2 bag reindex bags/<run>
```

What the recorder had not yet written is lost: about the last second if the process is killed, plus whatever Linux had not yet flushed to disk if the power is cut.
