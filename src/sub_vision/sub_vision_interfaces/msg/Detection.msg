# A single object detection enriched with 3D metadata.
#
# Builds on vision_msgs/Detection2D for the 2D result (bounding box, class
# id/label, score) so downstream consumers can reuse the standard tooling,
# and adds the perception fields this project needs on top.

# Standard 2D detection: bbox (pixels of the source image) + class hypotheses
# (id/label/score) + source header.
vision_msgs/Detection2D detection

# Bearing from the camera optical axis to the bbox center, in radians,
# computed from the camera intrinsics and distortion: atan(x/z) and atan(y/z)
# of the ray through it; a task's post-processor may aim them at a point it
# knows better instead (gate, torp: see the sub_vision README). Positive
# horizontal = target right of center, positive vertical = target below center
# (optical-frame convention). This is the steering signal sub_mission
# consumes: yaw/translate until the relevant bearing is ~0. Always filled; 0
# when the camera is uncalibrated (CameraInfo K is zero).
float32 bearing_horizontal
float32 bearing_vertical

# Estimated distance from the camera optical frame to the object, in meters.
# NaN when unknown: only a task post-processor fills it in, from geometry it
# knows (e.g. a known width, or cv2.solvePnP).
float32 distance_m

# Full 6-DOF pose of the object in the camera optical frame, when the task's
# post-processor can recover it (e.g. via cv2.solvePnP on known geometry).
# Only meaningful when pose_valid is true; otherwise left at identity/zero.
geometry_msgs/Pose pose
bool pose_valid

# Task-specific metadata as free-form key/value pairs. Lets new tasks attach
# extra information (e.g. "yaw_deg", "heading_yaw_deg") without changing this
# message definition.
diagnostic_msgs/KeyValue[] extra
