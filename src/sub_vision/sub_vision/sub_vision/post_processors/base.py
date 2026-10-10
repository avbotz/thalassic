"""Abstract interface every per-task post-processor implements."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np
    from sensor_msgs.msg import CameraInfo
    from sub_vision_interfaces.msg import DetectionArray


class TaskPostProcessor(ABC):
    """Refines a :class:`DetectionArray` for one mission task.

    The model manager fills in the 2D detections. A post-processor then adds
    task-specific enrichment -- most importantly the 6-DOF ``pose`` and
    ``distance_m`` (e.g. via ``cv2.solvePnP`` on known object geometry) and
    any ``extra`` key/value metadata.

    The node keeps one instance per task for its lifetime, so a processor may
    carry state between frames (torp keeps its depth calibration). It must not
    block; heavy one-time setup belongs in ``__init__``.
    """

    @abstractmethod
    def process(
        self,
        detections: DetectionArray,
        rgb_image: np.ndarray,
        depth_image: np.ndarray | None,
        camera_info: CameraInfo,
        model_masks: Sequence[np.ndarray | None] | None = None,
    ) -> DetectionArray:
        """Enrich and return the detections.

        Args:
            detections: ``sub_vision_interfaces/DetectionArray`` with 2D boxes
                already populated (``distance_m`` is NaN until a processor
                fills it in).
            rgb_image: Detection image as a ``numpy`` BGR array.
            depth_image: Relative depth (Depth Anything V2, float32, larger is
                nearer, image-sized) on the frames the depth model ran on,
                else ``None``. Never from another frame.
            camera_info: ``sensor_msgs/CameraInfo`` for the frame.
            model_masks: Per detection, in order, its instance mask (uint8
                0/255, image-sized) from a segmentation model, else ``None``.

        Returns:
            The (possibly mutated) ``DetectionArray``.
        """
        raise NotImplementedError
