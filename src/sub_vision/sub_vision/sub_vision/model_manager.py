"""Thread-safe loading, swapping, and inference of a single resident model."""

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from sub_vision import backends

# Tasks whose model file is not <task>.onnx/.engine. Mission XML loads tasks by what they are for,
# so the model behind one can change here without touching the missions.
TASK_MODELS = {
    # The front-camera model of RoboSub 2026's printed images; the gate reads its role icons.
    "gate": "ffc_rs_26",
}


@dataclass
class LoadResult:
    """Outcome of a :meth:`ModelManager.load` call (mirrors LoadModel.srv)."""

    success: bool
    message: str
    active_model: str
    load_time_s: float


@dataclass
class WarmupStats:
    """Timing collected during post-load warmup, for diagnostics."""

    iterations: int
    mean_ms: float
    last_ms: float


class ModelManager:
    """Owns the one resident detector and serializes inference against swaps.

    Only one model is resident at a time. ``load`` builds the new backend
    *without* holding the inference lock (loading a model can be slow), then
    atomically swaps it in under the lock and frees the previous one, so a model
    swap can never race a frame in flight.
    """

    def __init__(
        self,
        model_dir: str,
        backend_pref: str,
        device_pref: str,
        input_size: int,
        conf_threshold: float,
        warmup_iterations: int,
        log: Callable[[str], None],
    ):
        if backend_pref not in backends.BACKENDS:
            raise ValueError(f"backend '{backend_pref}' is not one of {backends.BACKENDS}")
        if device_pref not in backends.DEVICES:
            raise ValueError(f"device '{device_pref}' is not one of {backends.DEVICES}")
        if input_size <= 0:
            raise ValueError(f"input_size {input_size} is not positive")
        if not 0.0 <= conf_threshold <= 1.0:
            raise ValueError(f"conf_threshold {conf_threshold} is not in [0, 1]")
        self._model_dir = model_dir
        self._backend_pref = backend_pref
        self._device_pref = device_pref
        self._input_size = input_size
        self._conf_threshold = conf_threshold
        self._warmup_iterations = warmup_iterations
        self._log = log

        # Guards the active backend reference and serializes inference vs. swap.
        self._lock = threading.Lock()
        self._backend: backends.DetectionBackend | None = None
        self._active_task: str | None = None
        self._last_warmup: WarmupStats | None = None

    @property
    def active_task(self) -> str | None:
        return self._active_task

    @property
    def active_model(self) -> str:
        if self._backend is None:
            return ""
        return f"{self._active_task} ({self._backend.name})"

    @property
    def last_warmup(self) -> WarmupStats | None:
        return self._last_warmup

    def load(self, task: str) -> LoadResult:
        """Make ``task`` the active model, loading it from its file.

        No-ops (success) if ``task`` is already active. On failure the previous model stays active.
        """
        if not task:
            return LoadResult(False, "empty task name", self.active_model, 0.0)

        if task == self._active_task and self._backend is not None:
            return LoadResult(True, f"'{task}' already active", self.active_model, 0.0)

        start = time.perf_counter()
        model = TASK_MODELS.get(task, task)
        try:
            # Slow path (model load) runs outside the lock.
            backend = backends.build_backend(
                task=model,
                model_dir=self._model_dir,
                backend_pref=self._backend_pref,
                device_pref=self._device_pref,
                input_size=self._input_size,
                conf_threshold=self._conf_threshold,
                log=self._log,
            )
        except Exception as exc:  # report load failures to the caller
            return LoadResult(
                False, f"failed to load '{task}' ({model}): {exc}", self.active_model, 0.0
            )

        # A model that cannot run on a blank frame would fail on every camera frame: keep the
        # previous one instead.
        try:
            warmup = self._warmup(backend)
        except Exception as exc:
            backend.close()
            return LoadResult(
                False,
                f"failed to load '{task}' ({model}): warmup inference failed: {exc}",
                self.active_model,
                0.0,
            )

        # Atomic swap: replace the active backend and free the old one.
        with self._lock:
            old = self._backend
            self._backend = backend
            self._active_task = task
            self._last_warmup = warmup
        if old is not None:
            old.close()

        load_time = time.perf_counter() - start
        msg = f"loaded '{task}'" + (f" ({model})" if model != task else "") + f" via {backend.name}"
        if warmup is not None:
            msg += f" (warmup {warmup.mean_ms:.1f} ms/iter)"
        self._log(msg)
        return LoadResult(True, msg, self.active_model, load_time)

    def infer(self, image_bgr: np.ndarray) -> tuple[str, backends.InferenceResult] | None:
        """Run detection under the lock, as ``(task, result)``: the task is read under the same
        lock, so it always names the model that ran. ``None`` if no model is loaded."""
        with self._lock:
            if self._backend is None:
                return None
            return self._active_task, self._backend.infer(image_bgr)

    def close(self) -> None:
        """Free the resident model; none is active after."""
        with self._lock:
            backend, self._backend, self._active_task = self._backend, None, None
        if backend is not None:
            backend.close()

    def _warmup(self, backend: backends.DetectionBackend) -> WarmupStats | None:
        """Run a few inferences so the first real frame isn't penalized. Raises what inference
        raises."""
        if self._warmup_iterations <= 0:
            return None
        dummy = np.zeros((self._input_size, self._input_size, 3), dtype=np.uint8)
        times = []
        for _ in range(self._warmup_iterations):
            t0 = time.perf_counter()
            backend.infer(dummy)
            times.append((time.perf_counter() - t0) * 1000.0)
        return WarmupStats(len(times), float(np.mean(times)), times[-1])
