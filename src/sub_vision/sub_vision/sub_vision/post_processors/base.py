"""Abstract interface every per-task post-processor implements."""

from __future__ import annotations

from abc import ABC, abstractmethod


class TaskPostProcessor(ABC):
    """Refines a :class:`DetectionArray` for one mission task.

    The model manager fills in the 2D detections. A post-processor then adds
    task-specific enrichment -- most importantly the 6-DOF ``pose`` and
    ``distance_m`` (e.g. via ``cv2.solvePnP`` on known object geometry) and
    any ``extra`` key/value metadata.

    Implementations must be stateless across frames (single-frame detection
    only) and must not block; heavy one-time setup belongs in ``__init__``.
    """

    @abstractmethod
    def process(self, detections, rgb_image, depth_image, camera_info, model_masks=None):
        """Enrich and return the detections.

        Args:
            detections: ``sub_vision_interfaces/DetectionArray`` with 2D boxes
                already populated (``distance_m`` is NaN until a processor
                fills it in).
            rgb_image: Detection image as a ``numpy`` BGR array.
            depth_image: Always ``None`` (the depth camera is not used);
                kept in the signature for interface stability.
            camera_info: ``sensor_msgs/CameraInfo`` for the frame.
            model_masks: Optional YOLO instance masks, one per raw detection.

        Returns:
            The (possibly mutated) ``DetectionArray``.
        """
        raise NotImplementedError
