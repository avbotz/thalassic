"""Task-name -> post-processor registry and lookup."""

from __future__ import annotations

import importlib
from typing import Dict, List, Optional, Type

from sub_vision.post_processors.base import TaskPostProcessor

# Concrete post-processor classes that ship with this package. Imported lazily
# so that simply importing the registry does not drag in OpenCV / numpy.
_BUILTIN_MODULES = (
    "sub_vision.post_processors.gate",
    "sub_vision.post_processors.octagon_table",
    "sub_vision.post_processors.octagon_search_image",
    "sub_vision.post_processors.path_marker",
    "sub_vision.post_processors.slalom",
    "sub_vision.post_processors.torp",
)

_REGISTRY: Dict[str, Type[TaskPostProcessor]] = {}


def register_post_processor(task: str):
    """Class decorator registering a post-processor for ``task``."""

    def decorator(cls: Type[TaskPostProcessor]) -> Type[TaskPostProcessor]:
        if not issubclass(cls, TaskPostProcessor):
            raise TypeError(f"{cls.__name__} must subclass TaskPostProcessor")
        _REGISTRY[task] = cls
        return cls

    return decorator


def load_builtin_post_processors() -> None:
    """Import the shipped processor modules so their decorators run."""
    for module in _BUILTIN_MODULES:
        importlib.import_module(module)


def get_post_processor(task: str) -> Optional[TaskPostProcessor]:
    """Instantiate the processor registered for ``task``, or ``None``.

    Returning ``None`` lets the caller fall back to a no-op (detections keep
    their 2D metadata but get no pose or distance).
    """
    cls = _REGISTRY.get(task)
    if cls is None:
        for suffix in ("_survey", "_search"):
            if task.endswith(suffix):
                cls = _REGISTRY.get(task[: -len(suffix)])
                break
    return cls() if cls is not None else None


def registered_tasks() -> List[str]:
    """Names of all currently registered tasks."""
    return sorted(_REGISTRY)
