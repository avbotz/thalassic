"""Example post-processor for the ``gate`` task.

Estimates the gate's orientation by solving the Perspective-n-Point problem
between the detected bounding box corners and the gate's known real-world
geometry. This is illustrative: a production gate detector would localize the
actual gate posts rather than reusing the bbox corners, but it shows exactly
where per-task OpenCV work lives and how to fill in the 6-DOF ``pose``.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
from diagnostic_msgs.msg import KeyValue

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import register_post_processor

# Known gate geometry in meters (object frame, origin at gate center). Ordered
# top-left, top-right, bottom-right, bottom-left to match the bbox corners.
_GATE_WIDTH_M = 1.5
_GATE_HEIGHT_M = 1.5
_GATE_OBJECT_POINTS = np.array(
    [
        [-_GATE_WIDTH_M / 2.0, -_GATE_HEIGHT_M / 2.0, 0.0],
        [_GATE_WIDTH_M / 2.0, -_GATE_HEIGHT_M / 2.0, 0.0],
        [_GATE_WIDTH_M / 2.0, _GATE_HEIGHT_M / 2.0, 0.0],
        [-_GATE_WIDTH_M / 2.0, _GATE_HEIGHT_M / 2.0, 0.0],
    ],
    dtype=np.float64,
)


@register_post_processor("gate")
class GatePostProcessor(TaskPostProcessor):
    """Recovers gate orientation via solvePnP on the bbox corners."""

    def process(self, detections, rgb_image, depth_image, camera_info):
        k = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)
        dist = np.array(camera_info.d, dtype=np.float64)

        for det in detections.detections:
            bbox = det.detection.bbox
            half_w = bbox.size_x / 2.0
            half_h = bbox.size_y / 2.0
            cx = bbox.center.position.x
            cy = bbox.center.position.y
            image_points = np.array(
                [
                    [cx - half_w, cy - half_h],
                    [cx + half_w, cy - half_h],
                    [cx + half_w, cy + half_h],
                    [cx - half_w, cy + half_h],
                ],
                dtype=np.float64,
            )

            ok, rvec, tvec = cv2.solvePnP(
                _GATE_OBJECT_POINTS,
                image_points,
                k,
                dist,
                flags=cv2.SOLVEPNP_IPPE,
            )
            if not ok:
                det.pose_valid = False
                continue

            self._fill_pose(det, rvec, tvec)
            det.pose_valid = True
            yaw_deg = math.degrees(self._yaw_from_rvec(rvec))
            det.extra.append(KeyValue(key="yaw_deg", value=f"{yaw_deg:.1f}"))

        return detections

    @staticmethod
    def _fill_pose(det, rvec, tvec):
        det.pose.position.x = float(tvec[0])
        det.pose.position.y = float(tvec[1])
        det.pose.position.z = float(tvec[2])
        qx, qy, qz, qw = GatePostProcessor._rvec_to_quaternion(rvec)
        det.pose.orientation.x = qx
        det.pose.orientation.y = qy
        det.pose.orientation.z = qz
        det.pose.orientation.w = qw

    @staticmethod
    def _rvec_to_quaternion(rvec):
        rot, _ = cv2.Rodrigues(rvec)
        # Standard rotation-matrix -> quaternion (x, y, z, w).
        trace = rot[0, 0] + rot[1, 1] + rot[2, 2]
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (rot[2, 1] - rot[1, 2]) / s
            qy = (rot[0, 2] - rot[2, 0]) / s
            qz = (rot[1, 0] - rot[0, 1]) / s
        elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            qw = (rot[2, 1] - rot[1, 2]) / s
            qx = 0.25 * s
            qy = (rot[0, 1] + rot[1, 0]) / s
            qz = (rot[0, 2] + rot[2, 0]) / s
        elif rot[1, 1] > rot[2, 2]:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            qw = (rot[0, 2] - rot[2, 0]) / s
            qx = (rot[0, 1] + rot[1, 0]) / s
            qy = 0.25 * s
            qz = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            qw = (rot[1, 0] - rot[0, 1]) / s
            qx = (rot[0, 2] + rot[2, 0]) / s
            qy = (rot[1, 2] + rot[2, 1]) / s
            qz = 0.25 * s
        return float(qx), float(qy), float(qz), float(qw)

    @staticmethod
    def _yaw_from_rvec(rvec):
        rot, _ = cv2.Rodrigues(rvec)
        return math.atan2(rot[1, 0], rot[0, 0])
