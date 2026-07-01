"""PnP post-processor for torpedo target detections."""

from __future__ import annotations

from sub_vision.post_processors.object_points import TORPEDO_OBJECT_POINTS
from sub_vision.post_processors.planar import SinglePlanarObjectPostProcessor
from sub_vision.post_processors.registry import register_post_processor


@register_post_processor("torpedo")
@register_post_processor("torpedoes")
class TorpedoPostProcessor(SinglePlanarObjectPostProcessor):
    def __init__(self):
        super().__init__(
            TORPEDO_OBJECT_POINTS["torpedo_1"],
            set(TORPEDO_OBJECT_POINTS),
        )
