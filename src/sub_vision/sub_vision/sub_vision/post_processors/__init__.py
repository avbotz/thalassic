"""Pluggable per-task post-processing.

Adding a mission task means adding one model file plus one
:class:`~sub_vision.post_processors.base.TaskPostProcessor` subclass decorated
with :func:`register_post_processor` -- no edits to the core node.

Concrete processors (which may pull in OpenCV etc.) are imported lazily by the
node via :func:`load_builtin_post_processors`, so importing this package alone
stays cheap and dependency-free (useful for tests and linting).
"""

from sub_vision.post_processors.base import TaskPostProcessor
from sub_vision.post_processors.registry import (
    get_post_processor,
    load_builtin_post_processors,
    register_post_processor,
    registered_tasks,
)

__all__ = [
    "TaskPostProcessor",
    "get_post_processor",
    "load_builtin_post_processors",
    "register_post_processor",
    "registered_tasks",
]
