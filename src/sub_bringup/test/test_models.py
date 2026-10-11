"""Every model a mission tree loads is in weights/manifest.toml, so a clean
checkout (`pixi run fetch-weights`) can run every packaged mission."""

import ast
import re
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MISSION_RESOURCES = ROOT / "src" / "sub_mission" / "resources"
MODEL_MANAGER = ROOT / "src" / "sub_vision" / "sub_vision" / "sub_vision" / "model_manager.py"
MANIFEST = ROOT / "weights" / "manifest.toml"

# Models the trees load that have not been released yet. Shrink this as they are:
# the test fails both for a new missing model and for one listed here that is no
# longer missing.
KNOWN_MISSING = {"octagon_search_image", "octagon_table", "torp_fire_blood"}


def task_models() -> dict[str, str]:
    """sub_vision's TASK_MODELS, read from its source (no inference stack needed)."""
    tree = ast.parse(MODEL_MANAGER.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "TASK_MODELS" for target in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError("TASK_MODELS not found in model_manager.py")


def tree_tasks() -> set[str]:
    tasks = set()
    for xml in MISSION_RESOURCES.glob("*/*.xml"):
        tasks.update(re.findall(r'\btask="([^"{}]+)"', xml.read_text()))
    return tasks


def released_models() -> set[str]:
    manifest = tomllib.loads(MANIFEST.read_text())
    return {Path(name).stem for name in manifest if name.endswith(".onnx")}


def test_trees_load_tasks():
    assert tree_tasks(), f"no task= found under {MISSION_RESOURCES}"


def test_every_model_a_tree_loads_is_released():
    mapping = task_models()
    needed = {mapping.get(task, task) for task in tree_tasks()}
    missing = needed - released_models()
    assert missing <= KNOWN_MISSING, (
        f"trees load models that are not in weights/manifest.toml: {sorted(missing - KNOWN_MISSING)}"
    )
    assert missing == KNOWN_MISSING, (
        f"no longer missing (remove from KNOWN_MISSING): {sorted(KNOWN_MISSING - missing)}"
    )
