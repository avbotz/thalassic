"""
Parse Stonefish scenario XML files and assign segmentation IDs
exactly as Stonefish does internally.

Stonefish assigns each mesh a sequential 'objectId' via BuildObject().
The segmentation camera renders (objectId + 1) as the pixel value
(0 = background).

Key rules from Stonefish's ScenarioParser.cpp:
  1. <include> elements are expanded (children appended to root) in
     document order, recursively.
  2. After flattening, entities are parsed by TAG NAME in this fixed order:
       <static>  ->  <animated>  ->  <dynamic>  ->  <robot>
     Within each tag, elements are iterated in document order.
  3. Each static "model" (Obstacle) allocates 2 IDs:
       graObjectId = next_id      (used by segmentation camera)
       phyObjectId = next_id + 1
  4. Each dynamic "model" (Polyhedron) also allocates 2 IDs (same layout).
  5. Robot compound parts (external_part) are each Polyhedrons and each
     allocate 2 IDs. Parts with only <physical> mesh still allocate 2.
     Robot non-compound links also allocate 2 IDs each.
  6. Sensors/actuators do NOT allocate IDs unless they have a <visual>
     element (rare).

The segmentation pixel value for an object is: graObjectId + 1
"""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional
from dataclasses import dataclass, field


@dataclass
class SegObject:
    """Represents an entity that gets one or more segmentation IDs."""
    name: str
    entity_type: str          # "static", "dynamic", "robot_part"
    gra_object_id: int        # graphical object id (used by segmentation cam)
    phy_object_id: int        # physical object id
    seg_pixel_value: int      # = gra_object_id + 1
    parent_prop: str          # the "prop" or task-level group this belongs to
    class_id: int             # YOLOv11 class id for the parent prop


@dataclass
class PropGroup:
    """A logical prop composed of one or more SegObjects."""
    name: str
    class_name: str
    class_id: int
    seg_ids: list[int] = field(default_factory=list)  # pixel values


def _resolve_includes(root: ET.Element, base_dir: Path) -> ET.Element:
    """
    Recursively resolve <include file="..."> elements, replicating
    Stonefish's IncludeFiles() behaviour.

    Each <include> is replaced by the children of the included file's
    <scenario> root. Arguments (<arg name="..." value="...">) are
    substituted into $(arg name) placeholders.
    """
    while True:
        include_el = root.find("include")
        if include_el is None:
            break

        file_attr = include_el.get("file")
        if file_attr is None:
            root.remove(include_el)
            continue

        inc_path = base_dir / file_attr

        # Collect args
        args = {}
        for arg_el in include_el.findall("arg"):
            aname = arg_el.get("name")
            aval = arg_el.get("value")
            if aname is not None and aval is not None:
                args[aname] = aval

        # Load included file
        inc_tree = ET.parse(str(inc_path))
        inc_root = inc_tree.getroot()

        # Substitute $(arg ...) placeholders
        _substitute_args(inc_root, args)

        # Recursively resolve nested includes
        _resolve_includes(inc_root, inc_path.parent)

        # Remove the <include> element
        root.remove(include_el)

        # Append children of included <scenario> to root
        for child in inc_root:
            root.append(child)

    return root


def _substitute_args(element: ET.Element, args: dict[str, str]):
    """Substitute $(arg name) in all attribute values and text."""
    for key, val in element.attrib.items():
        for arg_name, arg_val in args.items():
            val = val.replace(f"$(arg {arg_name})", arg_val)
        element.attrib[key] = val

    if element.text:
        for arg_name, arg_val in args.items():
            element.text = element.text.replace(f"$(arg {arg_name})", arg_val)

    for child in element:
        _substitute_args(child, args)


def _has_visual_mesh(element: ET.Element) -> bool:
    """Check if an element has a separate <visual> mesh."""
    return element.find("visual") is not None


def _has_model_type(element: ET.Element) -> bool:
    return element.get("type") == "model"


def _count_compound_parts(element: ET.Element) -> list[str]:
    """
    For a compound (robot base_link), return the names of external_part
    elements in order.
    """
    parts = []
    for ext in element.findall("external_part"):
        name = ext.get("name", "unknown")
        parts.append(name)
    return parts


def _is_compound(element: ET.Element) -> bool:
    return element.get("type") == "compound"


def parse_scenario_ids(
    scenario_path: str | Path,
    data_dir: str | Path,
    prop_definitions: dict[str, str] | None = None,
) -> tuple[dict[int, str], list[PropGroup], list[SegObject]]:
    """
    Parse a Stonefish scenario file and compute segmentation IDs.

    Args:
        scenario_path: Path to the top-level .scn file (already rendered
                       from Jinja, with includes NOT yet resolved — we do
                       that ourselves).
        data_dir:      The Stonefish data directory (base for mesh paths
                       and include file resolution).
        prop_definitions: Optional mapping from object name patterns to
                          prop/class names. If None, a default mapping
                          is used.

    Returns:
        pixel_to_prop: dict mapping segmentation pixel value -> prop name
        prop_groups:   list of PropGroup objects
        all_objects:   list of all SegObject entries
    """
    scenario_path = Path(scenario_path)
    data_dir = Path(data_dir)

    tree = ET.parse(str(scenario_path))
    root = tree.getroot()

    # Resolve all includes
    _resolve_includes(root, data_dir)

    # Collect elements by tag in Stonefish's parsing order
    statics = list(root.iter("static"))
    animateds = list(root.iter("animated"))
    dynamics = list(root.iter("dynamic"))
    robots = list(root.iter("robot"))

    # Filter: only top-level elements (direct children of root or included)
    # Actually, after include resolution, everything that was a direct child
    # of <scenario> in any file is now a direct child of root.
    statics = [e for e in root.findall("static")]
    animateds = [e for e in root.findall("animated")]
    dynamics = [e for e in root.findall("dynamic")]
    robots = [e for e in root.findall("robot")]

    # The ocean (if present) builds one OpenGL object before any entities,
    # consuming object ID 0.  We must account for this offset.
    env = root.find("environment")
    ocean_present = env is not None and env.find("ocean") is not None
    next_id = [1 if ocean_present else 0]  # mutable counter

    def alloc_id() -> int:
        """Allocate the next object ID (mirrors BuildObject)."""
        oid = next_id[0]
        next_id[0] += 1
        return oid

    all_objects: list[SegObject] = []

    # --- Process static elements ---
    for elem in statics:
        name = elem.get("name", "unknown")
        etype = elem.get("type", "")

        if etype == "model":
            # Obstacle::BuildGraphicalObject allocates graMesh FIRST,
            # then phyMesh SECOND.  Segmentation camera uses graObjectId.
            gra_id = alloc_id()
            phy_id = alloc_id()
            all_objects.append(SegObject(
                name=name,
                entity_type="static",
                gra_object_id=gra_id,
                phy_object_id=phy_id,
                seg_pixel_value=gra_id + 1,
                parent_prop="",
                class_id=-1,
            ))
        elif etype == "plane":
            # Plane: 1 ID (StaticEntity::BuildGraphicalObject)
            phy_id = alloc_id()
            all_objects.append(SegObject(
                name=name,
                entity_type="static",
                gra_object_id=phy_id,
                phy_object_id=phy_id,
                seg_pixel_value=phy_id + 1,
                parent_prop="",
                class_id=-1,
            ))
        elif etype == "terrain":
            # Terrain: 1 ID
            phy_id = alloc_id()
            all_objects.append(SegObject(
                name=name,
                entity_type="static",
                gra_object_id=phy_id,
                phy_object_id=phy_id,
                seg_pixel_value=phy_id + 1,
                parent_prop="",
                class_id=-1,
            ))
        elif etype in ("box", "cylinder", "sphere"):
            # Obstacle primitive: 2 IDs (gra first, phy second)
            gra_id = alloc_id()
            phy_id = alloc_id()
            all_objects.append(SegObject(
                name=name,
                entity_type="static",
                gra_object_id=gra_id,
                phy_object_id=phy_id,
                seg_pixel_value=gra_id + 1,
                parent_prop="",
                class_id=-1,
            ))

    # --- Process animated elements ---
    for elem in animateds:
        name = elem.get("name", "unknown")
        etype = elem.get("type", "")

        if etype == "model":
            if _has_visual_mesh(elem):
                gra_id = alloc_id()
                phy_id = alloc_id()
            else:
                gra_id = alloc_id()
                phy_id = gra_id
            all_objects.append(SegObject(
                name=name,
                entity_type="animated",
                gra_object_id=gra_id,
                phy_object_id=phy_id,
                seg_pixel_value=gra_id + 1,
                parent_prop="",
                class_id=-1,
            ))
        elif etype == "empty":
            pass  # No mesh, no ID

    # --- Process dynamic elements ---
    for elem in dynamics:
        _parse_solid_ids(elem, "dynamic", alloc_id, all_objects)

    # --- Process robot elements ---
    for robot_elem in robots:
        robot_name = robot_elem.get("name", "robot")

        # base_link is parsed first
        base_link = robot_elem.find("base_link")
        if base_link is not None:
            _parse_solid_ids(base_link, "robot_part", alloc_id, all_objects,
                             ns=robot_name)

        # Then other links in document order
        for link_elem in robot_elem.findall("link"):
            _parse_solid_ids(link_elem, "robot_part", alloc_id, all_objects,
                             ns=robot_name)

        # Sensors with <visual> (rare but possible)
        for sensor_elem in robot_elem.findall("sensor"):
            vis = sensor_elem.find("visual")
            if vis is not None:
                s_name = sensor_elem.get("name", "sensor")
                gra_id = alloc_id()
                all_objects.append(SegObject(
                    name=f"{robot_name}/{s_name}",
                    entity_type="sensor",
                    gra_object_id=gra_id,
                    phy_object_id=gra_id,
                    seg_pixel_value=gra_id + 1,
                    parent_prop="",
                    class_id=-1,
                ))

    # --- Assign prop groups ---
    if prop_definitions is None:
        prop_definitions = get_default_prop_definitions()

    prop_groups, pixel_to_prop = _assign_props(all_objects, prop_definitions)

    return pixel_to_prop, prop_groups, all_objects


def _parse_solid_ids(
    elem: ET.Element,
    entity_type: str,
    alloc_id,
    all_objects: list[SegObject],
    ns: str = "",
):
    """
    Parse a solid element (dynamic body or robot link) and allocate IDs.
    Handles compound types (multiple external_part) and simple models.
    """
    name = elem.get("name", "unknown")
    if ns:
        full_name = f"{ns}/{name}"
    else:
        full_name = name

    etype = elem.get("type", "")

    if etype == "compound":
        # Compound: each external_part is a separate Polyhedron
        for part_elem in elem.findall("external_part"):
            _parse_solid_ids(part_elem, entity_type, alloc_id, all_objects, ns=ns if ns else name)

        # Internal parts
        for part_elem in elem.findall("internal_part"):
            _parse_solid_ids(part_elem, entity_type, alloc_id, all_objects, ns=ns if ns else name)

    elif etype == "model":
        # SolidEntity::BuildGraphicalObject allocates 1 ID only
        # (graObjectId = BuildObject(phyMesh); phyObjectId = graObjectId)
        gra_id = alloc_id()
        phy_id = gra_id
        all_objects.append(SegObject(
            name=full_name,
            entity_type=entity_type,
            gra_object_id=gra_id,
            phy_object_id=phy_id,
            seg_pixel_value=gra_id + 1,
            parent_prop="",
            class_id=-1,
        ))
    elif etype in ("box", "cylinder", "sphere", "torus", "wing"):
        # Simple shapes: SolidEntity::BuildGraphicalObject → 1 ID
        gra_id = alloc_id()
        all_objects.append(SegObject(
            name=full_name,
            entity_type=entity_type,
            gra_object_id=gra_id,
            phy_object_id=gra_id,
            seg_pixel_value=gra_id + 1,
            parent_prop="",
            class_id=-1,
        ))


def get_default_prop_definitions() -> dict[str, str]:
    """
    Returns a mapping from object name prefixes/patterns to prop class names.

    The key is a prefix that will be matched against object names.
    The value is the class name for YOLO labeling.

    Objects not matching any prefix are assigned class "background" and
    excluded from annotation.
    """
    return {
        # Pool (not annotated — background)
        "woollett_": "background",
        "natatorium_": "background",

        # Gate task
        "gate_": "gate",

        # Path markers
        "path_": "path",

        # Slalom
        "slalom_": "slalom",

        # Bin task
        "bin_": "bin",

        # Torpedo board
        "torpboard_": "torpboard",

        # Octagon task
        "octagon": "octagon",
        "table_": "table",
        "bottle_red": "bottle_red",
        "bottle_yellow": "bottle_yellow",
        "ladle_red": "ladle_red",
        "ladle_yellow": "ladle_yellow",

        # Robot parts (not annotated)
        "marlin_v2/": "background",
    }


def get_class_names() -> list[str]:
    """Return the ordered list of YOLO class names (index = class_id)."""
    return [
        "gate",       # 0
        "path",       # 1
        "slalom",     # 2
        "bin",        # 3
        "torpboard",  # 4
        "octagon",    # 5
        "table",      # 6
        "bottle_red",    # 7
        "bottle_yellow", # 8
        "ladle_red",     # 9
        "ladle_yellow",  # 10
    ]


def _assign_props(
    all_objects: list[SegObject],
    prop_definitions: dict[str, str],
) -> tuple[list[PropGroup], dict[int, str]]:
    """
    Assign each SegObject to a prop group based on name matching.

    Returns:
        prop_groups: list of PropGroup
        pixel_to_prop: mapping from pixel value to prop class name
    """
    class_names = get_class_names()
    class_name_to_id = {n: i for i, n in enumerate(class_names)}

    # Group objects by prop name
    prop_objects: dict[str, list[SegObject]] = {}

    for obj in all_objects:
        matched_class = None
        for prefix, class_name in prop_definitions.items():
            if obj.name.startswith(prefix):
                matched_class = class_name
                break

        if matched_class is None or matched_class == "background":
            obj.parent_prop = "background"
            obj.class_id = -1
            continue

        obj.parent_prop = matched_class
        obj.class_id = class_name_to_id.get(matched_class, -1)

        if matched_class not in prop_objects:
            prop_objects[matched_class] = []
        prop_objects[matched_class].append(obj)

    # Build prop groups
    prop_groups = []
    for class_name, objects in prop_objects.items():
        class_id = class_name_to_id.get(class_name, -1)
        seg_ids = [o.seg_pixel_value for o in objects]
        prop_groups.append(PropGroup(
            name=class_name,
            class_name=class_name,
            class_id=class_id,
            seg_ids=seg_ids,
        ))

    # Build pixel -> prop mapping
    pixel_to_prop: dict[int, str] = {}
    for obj in all_objects:
        if obj.parent_prop != "background":
            pixel_to_prop[obj.seg_pixel_value] = obj.parent_prop

    return prop_groups, pixel_to_prop


def build_pixel_to_class_id(
    scenario_path: str | Path,
    data_dir: str | Path,
    prop_definitions: dict[str, str] | None = None,
) -> tuple[dict[int, int], list[str]]:
    """
    Convenience function: returns mapping from segmentation pixel value
    to YOLO class_id, plus the list of class names.

    Returns:
        pixel_to_class: dict[pixel_value -> class_id]
        class_names: list of class name strings (index = class_id)
    """
    pixel_to_prop, prop_groups, all_objects = parse_scenario_ids(
        scenario_path, data_dir, prop_definitions
    )

    class_names = get_class_names()
    class_name_to_id = {n: i for i, n in enumerate(class_names)}

    pixel_to_class: dict[int, int] = {}
    for px_val, prop_name in pixel_to_prop.items():
        cid = class_name_to_id.get(prop_name, -1)
        if cid >= 0:
            pixel_to_class[px_val] = cid

    return pixel_to_class, class_names


def main():
    import sys

    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <scenario.scn> <data_dir>")
        sys.exit(1)

    scenario = sys.argv[1]
    data = sys.argv[2]

    pixel_to_prop, prop_groups, all_objs = parse_scenario_ids(scenario, data)

    print("=== All Objects ===")
    for o in all_objs:
        print(f"  ID {o.gra_object_id:3d}  seg_px={o.seg_pixel_value:3d}  "
              f"type={o.entity_type:12s}  prop={o.parent_prop:15s}  "
              f"name={o.name}")

    print("\n=== Prop Groups ===")
    for g in prop_groups:
        print(f"  class={g.class_id:2d}  name={g.class_name:15s}  "
              f"seg_ids={g.seg_ids}")

    print("\n=== Pixel -> Prop ===")
    for px, prop in sorted(pixel_to_prop.items()):
        print(f"  pixel {px:3d} -> {prop}")


if __name__ == "__main__":
    main()
