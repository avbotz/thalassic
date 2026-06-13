"""Tunable constants for sub_vision distance estimation.

These live apart from the node so they can be adjusted (or imported by tests
and post-processors) without touching the perception pipeline code.
"""

# Reference camera height, in meters, used to turn a bounding box's vertical
# extent (as a fraction of the image height) into a distance estimate:
#
#     distance_m = (bbox_height_px / image_height_px) * CAMERA_HEIGHT_M
#
# This is a coarse, depth-camera-free proxy; tune it per camera/mission.
CAMERA_HEIGHT_M = 1.0
