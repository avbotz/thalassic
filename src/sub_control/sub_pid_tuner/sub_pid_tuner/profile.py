"""The sub_control profile the controller was started with (sub_bringup
config/control/gains.yaml or gains_sim.yaml), and saving tuned values to it.

Saving rewrites only the values that changed, each in place on its own line,
so the comments, layout and number formatting of everything else survive and
the file's diff is just the tuning.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import shutil
import tempfile
from pathlib import Path
from typing import Any

import yaml

# Thruster faults are runtime state (set by hand or by thruster_monitor), not tuning.
NOT_SAVED = frozenset({"thruster_health", "failed_thrusters"})

# `key: value  # comment`, with the value and comment optional.
_KEY_LINE = re.compile(
    r"^(?P<indent> *)(?P<key>[^\s#:'\"-][^:#]*?|\"[^\"]*\"|'[^']*'):(?=\s|$)[ \t]*"
)


class GainProfile:
    def __init__(self, path: Path):
        # The installed file is a symlink to the source with --symlink-install;
        # write through it so the change lands in the source tree.
        self.path = path.expanduser().resolve()
        self.values: dict[str, Any] = {}
        self.digest = ""
        self.reload()

    def reload(self) -> dict[str, Any]:
        raw = self.path.read_bytes()
        self.values = flatten(parameters(yaml.safe_load(raw)))
        self.digest = hashlib.sha256(raw).hexdigest()
        return dict(self.values)

    def ensure_unchanged(self) -> bytes:
        """The file's contents, which must be what the dashboard last read or wrote."""
        raw = self.path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != self.digest:
            raise RuntimeError(
                f"{self.path.name} changed on disk; reload before saving so those edits are not overwritten"
            )
        return raw

    def save(self, runtime: dict[str, Any]) -> dict:
        """Write the runtime values of the parameters this file sets.

        `runtime` holds the controller's writable parameters; ones the file
        does not set (the launch file's, failed_thrusters) are left out.
        """
        # Bytes, not read_text(): newline translation would rewrite CRLF line endings.
        text = self.ensure_unchanged().decode()
        changed = {
            name: value
            for name, value in runtime.items()
            if name in self.values and name not in NOT_SAVED and value != self.values[name]
        }
        if changed:
            data = rewrite(text, changed).encode()
            # Replaced atomically: a crash leaves the old file or the new one.
            descriptor, temporary = tempfile.mkstemp(
                prefix=f".{self.path.name}.", dir=self.path.parent
            )
            try:
                with os.fdopen(descriptor, "wb") as stream:
                    stream.write(data)
                    stream.flush()
                    os.fsync(stream.fileno())
                shutil.copymode(self.path, temporary)
                os.replace(temporary, self.path)
            finally:
                Path(temporary).unlink(missing_ok=True)
            self.values.update(changed)
            self.digest = hashlib.sha256(data).hexdigest()
        return {"path": str(self.path), "written": changed}


def parameters(document: Any) -> dict:
    """The ros__parameters of a parameter file with one node section (sub_control's are under /**)."""
    sections = [
        section["ros__parameters"]
        for section in (document.values() if isinstance(document, dict) else [])
        if isinstance(section, dict) and isinstance(section.get("ros__parameters"), dict)
    ]
    if len(sections) != 1:
        raise ValueError("the profile must have exactly one ros__parameters section")
    return sections[0]


def flatten(tree: dict, prefix: str = "") -> dict[str, Any]:
    """Nested parameter mappings to ROS names: {gains: {x: ...}} -> {"gains.x": ...}."""
    result = {}
    for key, value in tree.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            result.update(flatten(value, f"{name}."))
        else:
            result[name] = value
    return result


def rewrite(text: str, changes: dict[str, Any]) -> str:
    """Replace the values of `changes` in a parameter file's text, keeping everything else."""
    lines = text.splitlines(keepends=True)
    located = _value_lines(lines)
    missing = sorted(set(changes) - set(located))
    if missing:
        raise RuntimeError(
            "can only rewrite values written on one line, as `name: value`; not "
            + ", ".join(missing)
        )
    for name, value in changes.items():
        lines[located[name]] = _replace_value(lines[located[name]], _format(value))
    result = "".join(lines)

    # The rest of the file must read back exactly as before.
    expected = flatten(parameters(yaml.safe_load(text))) | changes
    try:
        same = flatten(parameters(yaml.safe_load(result))) == expected
    except (yaml.YAMLError, ValueError):
        same = False
    if not same:
        raise RuntimeError("rewriting the profile in place changed more than the tuned values")
    return result


def _value_lines(lines: list[str]) -> dict[str, int]:
    """Line index of every one-line value under ros__parameters, by parameter name."""
    path: list[tuple[int, str]] = []
    located = {}
    for index, line in enumerate(lines):
        match = _KEY_LINE.match(line)
        if match is None:
            continue  # blank, a comment, or a block sequence item
        indent = len(match["indent"])
        while path and path[-1][0] >= indent:
            path.pop()
        path.append((indent, match["key"].strip("'\"")))
        value, _ = _split_comment(line[match.end() :].rstrip("\r\n"))
        keys = [key for _, key in path]
        if value and "ros__parameters" in keys[:-1]:
            located[".".join(keys[keys.index("ros__parameters") + 1 :])] = index
    return located


def _split_comment(text: str) -> tuple[str, str]:
    """`value  # comment` -> ("value", "  # comment")."""
    quote = None
    for index, char in enumerate(text):
        if quote:
            quote = None if char == quote else quote
        elif char in "'\"":
            quote = char
        elif char == "#" and (index == 0 or text[index - 1] in " \t"):
            value = text[:index].rstrip()
            return value, text[len(value) :]
    return text.rstrip(), ""


def _replace_value(line: str, value: str) -> str:
    match = _KEY_LINE.match(line)
    ending = line[len(line.rstrip("\r\n")) :]
    old, comment = _split_comment(line[match.end() :].rstrip("\r\n"))
    head = line[: match.end()] + value
    if not comment:
        return head + ending
    # Keep the comment in its column if the new value leaves room.
    gap = len(comment) - len(comment.lstrip())
    column = match.end() + len(old) + gap
    return head.ljust(max(column, len(head) + min(gap, 2))) + comment.lstrip() + ending


def _format(value: Any) -> str:
    """A parameter value as YAML that ROS reads back as the same type."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float) and math.isfinite(value):
        text = repr(value)
        # 1e-07 would read back as a string.
        return text if "." in text or "e" not in text else text.replace("e", ".0e")
    if isinstance(value, int):
        return str(value)
    if isinstance(value, str):
        return json.dumps(value)
    if isinstance(value, list | tuple):
        return "[" + ", ".join(_format(item) for item in value) + "]"
    raise ValueError(f"cannot write {value!r} to the profile")
