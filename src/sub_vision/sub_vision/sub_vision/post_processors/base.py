"""Abstract interface every per-task post-processor implements."""

from __future__ import annotations

from abc import ABC, abstractmethod


class TaskPostProcessor(ABC):
    """Refines a :class:`DetectionArray` for one mission task.

    The model manager fills in the 2D detections and depth-derived
    ``distance_m`` for each detection. A post-processor then adds task-specific
    enrichment -- most importantly the 6-DOF ``pose`` (e.g. via
    ``cv2.solvePnP`` on known object geometry) and any ``extra`` key/value
    metadata.

    Implementations must be stateless across frames (single-frame detection
    only) and must not block; heavy one-time setup belongs in ``__init__``.
    """

    @abstractmethod
    def process(self, detections, rgb_image, depth_image, camera_info):
        """Enrich and return the detections.

        Args:
            detections: ``sub_vision_interfaces/DetectionArray`` with 2D boxes
                and ``distance_m`` already populated.
            rgb_image: Detection image as a ``numpy`` BGR array.
            depth_image: Aligned depth as a ``numpy`` ``uint16`` array (mm),
                or ``None`` if depth was unavailable for this frame.
            camera_info: ``sensor_msgs/CameraInfo`` for the frame.

        Returns:
            The (possibly mutated) ``DetectionArray``.
        """
        raise NotImplementedError
