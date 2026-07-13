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
PERSISTENT_NAMES = PID_NAMES | {"power_limit"}


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
        if "power_limit" in params:
            values["power_limit"] = float(params["power_limit"])
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
        for name in sorted(PERSISTENT_NAMES):
            metadata = runtime_parameters.get(name)
            if metadata is None:
                continue
            value = metadata.get("value")
            if name == "power_limit":
                number = float(value)
                if not math.isfinite(number):
                    raise ValueError("power_limit must be finite")
                params[name] = number
                written[name] = number
            else:
                bank, axis = name.split(".")
                gains = self._finite_list(value)
                if bank not in params or not isinstance(params[bank], dict):
                    params[bank] = {}
                params[bank][axis] = gains
                written[name] = gains

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
