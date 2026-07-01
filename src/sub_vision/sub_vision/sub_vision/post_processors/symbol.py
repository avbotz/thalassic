"""PnP post-processor for square symbol targets."""

from __future__ import annotations

from sub_vision.post_processors.object_points import SYMBOL_OBJECT_POINTS
from sub_vision.post_processors.planar import SinglePlanarObjectPostProcessor
from sub_vision.post_processors.registry import register_post_processor


@register_post_processor("symbol")
@register_post_processor("symbols")
@register_post_processor("shark_fish")
class SymbolPostProcessor(SinglePlanarObjectPostProcessor):
    def __init__(self):
        super().__init__(
            SYMBOL_OBJECT_POINTS["reef_shark"],
            set(SYMBOL_OBJECT_POINTS),
        )
