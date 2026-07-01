"""PnP post-processor for the trash table task."""

from __future__ import annotations

from sub_vision.post_processors.object_points import TRASH_TABLE_OBJECT_POINTS
from sub_vision.post_processors.planar import MultiPartPlanarObjectPostProcessor
from sub_vision.post_processors.registry import register_post_processor


@register_post_processor("trash_table")
@register_post_processor("trash")
class TrashTablePostProcessor(MultiPartPlanarObjectPostProcessor):
    def __init__(self):
        super().__init__(
            TRASH_TABLE_OBJECT_POINTS,
            min_parts=1,
            output_class_name="trash_table",
            max_reprojection_error=100.0,
        )
