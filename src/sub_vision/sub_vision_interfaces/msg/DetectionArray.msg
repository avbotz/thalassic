# All detections produced from a single camera frame for one mission task.

# header.stamp matches the source image; header.frame_id is the camera optical
# frame the bounding boxes, distances, and poses are expressed in.
std_msgs/Header header

# Mission task / model that produced these detections (e.g. "gate", "buoy").
string task

Detection[] detections
