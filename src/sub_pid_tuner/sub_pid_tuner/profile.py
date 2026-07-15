from __future__ import annotations

import hashlib
import math
import os
import shutil
import tempfile
from datetime import datetime
from pathlib import Path

import yaml


PID_BANKS = ("pos_pid", "vel_pid", "att_pid", "ang_pid")
PID_NAMES = {f"{bank}.{axis}" for bank in PID_BANKS for axis in "xyz"}
BOOL_NAMES = {"feedforward_enabled"}
SCALAR_NAMES = {
    "power_limit",
    "odom_timeout_s",
    "cmd_vel_timeout_s",
    "antiwindup_gain",
    "spin.max_yaw_rate",
    "spin.done_angle",
    "spin.done_rate",
    "spin.deceleration",
}
VECTOR_LENGTHS = {
    "reference.position_kp": 3,
    "reference.attitude_kp": 3,
    "reference.max_velocity": 6,
    "reference.max_acceleration": 6,
    "model.effective_mass": 6,
    "model.linear_drag": 6,
    "model.quadratic_drag": 6,
    "model.trim": 6,
    "model.restoring_stiffness": 6,
    "feedback.kp": 6,
    "feedback.ki": 6,
    "feedback.integral_limit": 6,
}
PERSISTENT_NAMES = PID_NAMES | BOOL_NAMES | SCALAR_NAMES | set(VECTOR_LENGTHS)


class GainProfile:
    def __init__(self, path: Path | None, controller_name: str):
        self.path = path.expanduser().resolve() if path else None
        self.controller_name = controller_name
        self.values: dict[str, object] = {}
        self.digest = ""
        if self.path:
            self.reload()

    @property
    def available(self) -> bool:
        return self.path is not None

    def reload(self) -> dict[str, object]:
        if self.path is None:
            self.values = {}
            return {}
        raw = self.path.read_bytes()
        document = yaml.safe_load(raw)
        params = self._parameters(document)
        self.digest = hashlib.sha256(raw).hexdigest()
        values: dict[str, object] = {}
        for name in BOOL_NAMES:
            value = self._nested(params, name)
            if value is not None:
                values[name] = bool(value)
        for name in SCALAR_NAMES:
            value = self._nested(params, name)
            if value is not None:
                number = float(value)
                if not math.isfinite(number):
                    raise ValueError(f"{name} must be finite")
                values[name] = number
        for name, length in VECTOR_LENGTHS.items():
            value = self._nested(params, name)
            if value is not None:
                values[name] = self._finite_array(name, value, length)
        for bank in PID_BANKS:
            axes = params.get(bank, {})
            if not isinstance(axes, dict):
                continue
            for axis in "xyz":
                if axis in axes:
                    values[f"{bank}.{axis}"] = self._finite_list(axes[axis])
        self.values = values
        return dict(values)

    def ensure_unchanged(self) -> None:
        if self.path is None:
            raise RuntimeError("no persistent profile was configured for this dashboard")
        raw = self.path.read_bytes()
        if self.digest and hashlib.sha256(raw).hexdigest() != self.digest:
            raise RuntimeError(
                "profile changed on disk; reload before saving so external edits are not overwritten"
            )

    def save(self, runtime_parameters: dict[str, dict]) -> dict:
        self.ensure_unchanged()
        raw = self.path.read_bytes()
        document = yaml.safe_load(raw)
        params = self._parameters(document)
        written: dict[str, object] = {}
        for name in sorted(PID_NAMES):
            metadata = runtime_parameters.get(name)
            if metadata is None:
                continue
            value = metadata.get("value")
            bank, axis = name.split(".")
            gains = self._finite_list(value)
            if bank not in params or not isinstance(params[bank], dict):
                params[bank] = {}
            params[bank][axis] = gains
            written[name] = gains

        for name in sorted(BOOL_NAMES):
            metadata = runtime_parameters.get(name)
            if metadata is not None:
                value = metadata.get("value")
                if not isinstance(value, bool):
                    raise ValueError(f"{name} must be true or false")
                self._set_nested(params, name, value)
                written[name] = value

        for name in sorted(SCALAR_NAMES):
            metadata = runtime_parameters.get(name)
            if metadata is not None:
                number = float(metadata.get("value"))
                if not math.isfinite(number):
                    raise ValueError(f"{name} must be finite")
                self._set_nested(params, name, number)
                written[name] = number

        for name, length in VECTOR_LENGTHS.items():
            metadata = runtime_parameters.get(name)
            if metadata is not None:
                values = self._finite_array(name, metadata.get("value"), length)
                self._set_nested(params, name, values)
                written[name] = values

        if not written:
            raise RuntimeError(f"no persistable controller parameters found for {self.controller_name}")

        backup_dir = Path.home() / ".local/state/avbotz/dashboard/backups"
        backup_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        backup = backup_dir / f"{self.path.name}.{stamp}.bak"
        shutil.copy2(self.path, backup)

        encoded = yaml.safe_dump(document, sort_keys=False).encode()
        descriptor, temporary = tempfile.mkstemp(prefix=f".{self.path.name}.", dir=self.path.parent)
        try:
            with os.fdopen(descriptor, "wb") as stream:
                stream.write(encoded)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        except Exception:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass
            raise

        self.values = written
        self.digest = hashlib.sha256(encoded).hexdigest()
        return {
            "path": str(self.path),
            "backup": str(backup),
            "values": dict(written),
        }

    @staticmethod
    def _parameters(document: object) -> dict:
        if not isinstance(document, dict) or not document:
            raise ValueError("profile is not a ROS parameter YAML mapping")
        root = next(iter(document.values()))
        if not isinstance(root, dict) or not isinstance(root.get("ros__parameters"), dict):
            raise ValueError("profile is missing ros__parameters")
        return root["ros__parameters"]

    @staticmethod
    def _finite_list(values: object) -> list[float]:
        if not isinstance(values, (list, tuple)):
            raise ValueError("PID gains must be an array")
        result = [float(value) for value in values]
        if len(result) not in (3, 4) or any(not math.isfinite(value) for value in result):
            raise ValueError("PID gains must contain three or four finite values")
        return result

    @staticmethod
    def _finite_array(name: str, values: object, length: int) -> list[float]:
        if not isinstance(values, (list, tuple)):
            raise ValueError(f"{name} must be an array")
        result = [float(value) for value in values]
        if len(result) != length or any(not math.isfinite(value) for value in result):
            raise ValueError(f"{name} must contain {length} finite values")
        return result

    @staticmethod
    def _nested(parameters: dict, name: str):
        value = parameters
        for part in name.split("."):
            if not isinstance(value, dict) or part not in value:
                return None
            value = value[part]
        return value

    @staticmethod
    def _set_nested(parameters: dict, name: str, value: object) -> None:
        target = parameters
        parts = name.split(".")
        for part in parts[:-1]:
            child = target.get(part)
            if not isinstance(child, dict):
                child = {}
                target[part] = child
            target = child
        target[parts[-1]] = value
