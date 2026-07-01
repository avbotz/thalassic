"""PnP post-processor for the bin task."""

from __future__ import annotations

from sub_vision.post_processors.object_points import BIN_OBJECT_POINTS
from sub_vision.post_processors.planar import SinglePlanarObjectPostProcessor
from sub_vision.post_processors.registry import register_post_processor


@register_post_processor("bin")
class BinPostProcessor(SinglePlanarObjectPostProcessor):
    def __init__(self):
        super().__init__(BIN_OBJECT_POINTS, {"bin"})
