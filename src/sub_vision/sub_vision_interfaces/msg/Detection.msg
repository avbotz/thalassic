# A single object detection enriched with 3D metadata.
#
# Builds on vision_msgs/Detection2D for the 2D result (bounding box, class
# id/label, score) so downstream consumers can reuse the standard tooling,
# and adds the perception fields this project needs on top.

# Standard 2D detection: bbox + class hypotheses (id/label/score) + source header.
vision_msgs/Detection2D detection

# Estimated distance from the camera optical frame to the object, in meters.
# Derived from the aligned depth image + CameraInfo (see sub_vision metadata).
# NaN if no valid depth was available inside the bounding box.
float32 distance_m

# Full 6-DOF pose of the object in the camera optical frame, when the task's
# post-processor can recover it (e.g. via cv2.solvePnP on known geometry).
# Only meaningful when pose_valid is true; otherwise left at identity/zero.
geometry_msgs/Pose pose
bool pose_valid

# Task-specific metadata as free-form key/value pairs. Lets new tasks attach
# extra information (e.g. "orientation_deg", "side") without changing this
# message definition.
diagnostic_msgs/KeyValue[] extra
