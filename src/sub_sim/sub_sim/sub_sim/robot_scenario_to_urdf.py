"""
Convert a Stonefish scenario XML robot definition into a URDF.

This converter focuses on geometry + kinematic tree:
- <looks> -> URDF <material>
- <base_link>/<link> -> URDF <link>
- compound links -> multiple <visual>/<collision> on one URDF link
- <joint> -> URDF <joint>
It ignores sensors/actuators/ROS and Stonefish hydrodynamics tags.
"""

# WARNING: This code is almost entirely GPT generated with a few manual tweaks.
# While it has been tested and reviewed, it may contain subtle bugs.
# Review the generated urdf file carefully after changing any robot's layout.scn.j2.

import os
import sys
import math
import tempfile
from pathlib import Path
from dataclasses import dataclass
import typing

import xml.etree.ElementTree as ET
from xml.dom import minidom


Vec3 = tuple[float, float, float]
Mat3 = list[list[float]]


@dataclass(frozen=True)
class Transform:
    R: Mat3
    t: Vec3

    @staticmethod
    def identity() -> "Transform":
        return Transform([[1, 0, 0], [0, 1, 0], [0, 0, 1]], (0.0, 0.0, 0.0))

    @staticmethod
    def from_xyz_rpy(xyz: Vec3, rpy: Vec3) -> "Transform":
        return Transform(_rpy_to_mat(*rpy), xyz)

    def __matmul__(self, other: "Transform") -> "Transform":
        R = _mat_mul(self.R, other.R)
        t = _vec_add(self.t, _mat_vec(self.R, other.t))
        return Transform(R, t)


def _parse_floats(
    s: str | None, n: int | None = None, default: list[float] | None = None
) -> list[float]:
    if s is None:
        return default or []

    vals = [float(x) for x in s.strip().split()]

    if n is not None and len(vals) == 1:
        vals = vals * n

    if n is not None and len(vals) != n:
        raise ValueError(f"Expected {n} floats, got {len(vals)} from {s!r}")

    return vals


def _fmt(x: float) -> str:
    return f"{x:.6g}"


def _fmt_vec(v: Vec3) -> str:
    return " ".join(_fmt(x) for x in v)


def _rpy_to_mat(r: float, p: float, y: float) -> Mat3:
    """
    URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _mat_mul(A: Mat3, B: Mat3) -> Mat3:
    return [
        [sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)
    ]


def _mat_vec(A: Mat3, v: Vec3) -> Vec3:
    return (
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    )


def _vec_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mat_to_rpy(R: Mat3) -> Vec3:
    """
    Inverse of R = Rz(yaw)*Ry(pitch)*Rx(roll)
    Returns (roll, pitch, yaw).
    """
    sy = math.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0])
    singular = sy < 1e-9

    if not singular:
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], sy)
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        roll = math.atan2(-R[1][2], R[1][1])
        pitch = math.atan2(-R[2][0], sy)
        yaw = 0.0

    return (roll, pitch, yaw)


def _read_transform(elem: ET.Element | None) -> Transform:
    if elem is None:
        return Transform.identity()
    xyz = _parse_floats(elem.get("xyz"), n=3, default=[0.0, 0.0, 0.0])
    rpy = _parse_floats(elem.get("rpy"), n=3, default=[0.0, 0.0, 0.0])
    return Transform.from_xyz_rpy((xyz[0], xyz[1], xyz[2]), (rpy[0], rpy[1], rpy[2]))


def _pretty_xml(elem: ET.Element) -> str:
    rough = ET.tostring(elem, encoding="utf-8")
    reparsed = minidom.parseString(rough)
    return reparsed.toprettyxml(indent="  ")


def _parse_looks(
    scenario_root: ET.Element,
) -> dict[str, tuple[float, float, float, float]]:
    looks: dict[str, tuple[float, float, float, float]] = {}
    looks_el = scenario_root.find("looks")

    if looks_el is None:
        return looks

    for look in looks_el.findall("look"):
        name = look.get("name")
        if not name:
            continue

        if look.get("rgb"):
            rgb = _parse_floats(look.get("rgb"), n=3)
            rgba = (rgb[0], rgb[1], rgb[2], 1.0)
        elif look.get("gray"):
            # look gray cannot be none here due to get("name") check above
            g = float(typing.cast(str, look.get("gray")))
            rgba = (g, g, g, 1.0)
        else:
            rgba = (0.5, 0.5, 0.5, 1.0)

        looks[name] = rgba

    return looks


def _add_urdf_materials(
    urdf_robot: ET.Element, looks: dict[str, tuple[float, float, float, float]]
) -> None:
    for name, rgba in looks.items():
        mat = ET.SubElement(urdf_robot, "material", {"name": name})
        ET.SubElement(mat, "color", {"rgba": " ".join(_fmt(c) for c in rgba)})


def _get_look_name(node: ET.Element) -> str | None:
    look_el = node.find("look")
    if look_el is None:
        return None
    return look_el.get("name")


def _ensure_scale_str(scale_attr: str | None) -> str | None:
    if scale_attr is None:
        return None
    vals = _parse_floats(scale_attr, n=None)
    if len(vals) == 1:
        vals = vals * 3
    if len(vals) != 3:
        return scale_attr
    return " ".join(_fmt(v) for v in vals)


def _mesh_filename_out(fname: str, mesh_prefix: Path | str) -> str:
    if not fname:
        return fname

    if isinstance(mesh_prefix, str):
        return mesh_prefix + fname

    return str(mesh_prefix / fname)


def _add_visual_or_collision(
    urdf_link: ET.Element,
    kind: str,
    geom_type: str,
    geom_kwargs: dict[str, str],
    origin_T: Transform,
    material_name: str | None = None,
    comment: str | None = None,
) -> None:
    assert kind in ("visual", "collision")

    if comment:
        urdf_link.append(ET.Comment(comment))

    top = ET.SubElement(urdf_link, kind)

    # origin (pose of geometry w.r.t link frame)
    rpy = _mat_to_rpy(origin_T.R)
    if any(abs(x) > 1e-12 for x in origin_T.t) or any(abs(a) > 1e-12 for a in rpy):
        ET.SubElement(
            top, "origin", {"xyz": _fmt_vec(origin_T.t), "rpy": _fmt_vec(rpy)}
        )

    geom = ET.SubElement(top, "geometry")

    if geom_type == "mesh":
        ET.SubElement(geom, "mesh", geom_kwargs)
    elif geom_type == "box":
        ET.SubElement(geom, "box", geom_kwargs)  # size="x y z"
    elif geom_type == "cylinder":
        ET.SubElement(geom, "cylinder", geom_kwargs)  # radius=, length=
    elif geom_type == "sphere":
        ET.SubElement(geom, "sphere", geom_kwargs)  # radius=
    else:
        raise ValueError(f"Unsupported geometry type: {geom_type}")

    if kind == "visual" and material_name:
        ET.SubElement(top, "material", {"name": material_name})


def _parse_model_geometry(
    node: ET.Element, which: str, mesh_prefix: Path | str
) -> tuple[str, dict[str, str], Transform] | None:
    """
    Parse:
      <physical><mesh filename=... scale=.../><origin .../></physical>
    or:
      <visual>...</visual>
    """
    blk = node.find(which)
    if blk is None:
        return None
    mesh = blk.find("mesh")
    if mesh is None:
        return None

    fname = mesh.get("filename", "")
    out_fname = _mesh_filename_out(fname, mesh_prefix)
    kwargs: dict[str, str] = {"filename": out_fname}

    scale_str = _ensure_scale_str(mesh.get("scale"))
    if scale_str:
        kwargs["scale"] = scale_str

    origin_T = _read_transform(blk.find("origin"))
    return ("mesh", kwargs, origin_T)


def _parse_primitive_geometry(
    node: ET.Element,
) -> tuple[str, dict[str, str], Transform] | None:
    """
    Parse:
      type="box"      <dimensions xyz="..."/>      + optional <origin .../>
      type="cylinder" <dimensions radius="" height=""/> + optional <origin .../>
      type="sphere"   <dimensions radius=""/>
    """
    typ = (node.get("type") or "").strip()
    dims = node.find("dimensions")
    if dims is None:
        return None

    origin_T = _read_transform(node.find("origin"))

    if typ == "box":
        xyz = _parse_floats(dims.get("xyz"), n=3)
        return ("box", {"size": " ".join(_fmt(x) for x in xyz)}, origin_T)

    if typ == "cylinder":
        r = float(typing.cast(str, dims.get("radius")))
        h = float(typing.cast(str, dims.get("height")))
        return ("cylinder", {"radius": _fmt(r), "length": _fmt(h)}, origin_T)

    if typ == "sphere":
        r = float(typing.cast(str, dims.get("radius")))
        return ("sphere", {"radius": _fmt(r)}, origin_T)

    # TODO: Implement Torus and Wing primitive geometries if needed.

    return None


def _get_mass_value(node: ET.Element) -> float | None:
    # TODO: Calculate mass using physical obj file like Stonefish does

    m = node.find("mass")
    if m is None:
        return None
    v = m.get("value")
    if v is None:
        return None
    return float(v)


def _add_inertial_epsilon(
    urdf_link: ET.Element,
    mass: float,
    inertia_epsilon: float,
    origin_xyz: Vec3 = (0.0, 0.0, 0.0),
) -> None:
    """
    Insert a URDF inertial with a diagonal epsilon inertia.
    This is not physically accurate, but keeps many URDF consumers from rejecting the file.
    """
    inertial = ET.SubElement(urdf_link, "inertial")
    if any(abs(x) > 1e-12 for x in origin_xyz):
        ET.SubElement(inertial, "origin", {"xyz": _fmt_vec(origin_xyz), "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": _fmt(mass)})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": _fmt(inertia_epsilon),
            "ixy": "0",
            "ixz": "0",
            "iyy": _fmt(inertia_epsilon),
            "iyz": "0",
            "izz": _fmt(inertia_epsilon),
        },
    )


def _convert_compound_into_link_geometries(
    compound_node: ET.Element,
    urdf_link: ET.Element,
    mesh_prefix: Path | str,
    include_internal_parts_in_geometry: bool,
) -> float:
    """
    Add multiple visual/collision elements to a URDF link from Stonefish compound parts.
    Returns: total mass found across parts (sum of explicit <mass value="..."/>).
    """
    total_mass = 0.0

    externals = list(compound_node.findall("external_part"))
    internals = list(compound_node.findall("internal_part"))

    parts_for_geometry = externals + (
        internals if include_internal_parts_in_geometry else []
    )

    # Always sum mass from both external and internal parts if present (mass distribution).
    for p in externals + internals:
        mv = _get_mass_value(p)
        if mv is not None:
            total_mass += mv

    for part in parts_for_geometry:
        part_name = part.get("name", "part")
        part_tf = _read_transform(part.find("compound_transform"))
        look = _get_look_name(part)

        model_phys = _parse_model_geometry(part, "physical", mesh_prefix)
        model_vis = _parse_model_geometry(part, "visual", mesh_prefix)
        prim = _parse_primitive_geometry(part)

        # Visual
        if model_vis:
            gtype, kwargs, local_T = model_vis
        elif prim:
            gtype, kwargs, local_T = prim
        elif model_phys:
            # fallback: show physical mesh if visual is missing
            gtype, kwargs, local_T = model_phys
        else:
            continue

        _add_visual_or_collision(
            urdf_link,
            "visual",
            gtype,
            kwargs,
            part_tf @ local_T,
            material_name=look,
            comment=f"{part_name} visual",
        )

        # Collision
        if model_phys:
            gtype, kwargs, local_T = model_phys
        elif prim:
            gtype, kwargs, local_T = prim

        _add_visual_or_collision(
            urdf_link,
            "collision",
            gtype,
            kwargs,
            part_tf @ local_T,
            comment=f"{part_name} collision",
        )

    return total_mass


def _convert_body_node_to_urdf_link(
    robot_name: str,
    body_node: ET.Element,
    urdf_robot: ET.Element,
    mesh_prefix: Path | str,
    include_internal_parts_in_geometry: bool,
    add_inertial: bool,
    default_mass_if_missing: float,
    inertia_epsilon: float,
) -> None:
    """
    Handles <base_link> and <link> nodes.
    """
    name = body_node.get("name")
    if not name:
        raise ValueError("Body node missing name attribute")

    urdf_link = ET.SubElement(urdf_robot, "link", {"name": f"{robot_name}/{name}"})
    typ = (body_node.get("type") or "").strip()

    if typ == "compound":
        total_mass = _convert_compound_into_link_geometries(
            body_node,
            urdf_link,
            mesh_prefix=mesh_prefix,
            include_internal_parts_in_geometry=include_internal_parts_in_geometry,
        )

        if add_inertial:
            mass_for_urdf = total_mass if total_mass > 0.0 else default_mass_if_missing
            if mass_for_urdf > 0.0:
                _add_inertial_epsilon(urdf_link, mass_for_urdf, inertia_epsilon)

        return

    look = _get_look_name(body_node)

    model_phys = _parse_model_geometry(body_node, "physical", mesh_prefix)
    model_vis = _parse_model_geometry(body_node, "visual", mesh_prefix)
    prim = _parse_primitive_geometry(body_node)

    # Visual
    if model_vis:
        gtype, kwargs, transform = model_vis
        _add_visual_or_collision(
            urdf_link, "visual", gtype, kwargs, transform, material_name=look
        )
    elif prim:
        gtype, kwargs, transform = prim
        _add_visual_or_collision(
            urdf_link, "visual", gtype, kwargs, transform, material_name=look
        )
    elif model_phys:
        gtype, kwargs, transform = model_phys
        _add_visual_or_collision(
            urdf_link, "visual", gtype, kwargs, transform, material_name=look
        )

    # Collision
    if model_phys:
        gtype, kwargs, transform = model_phys
        _add_visual_or_collision(urdf_link, "collision", gtype, kwargs, transform)
    elif prim:
        gtype, kwargs, transform = prim
        _add_visual_or_collision(urdf_link, "collision", gtype, kwargs, transform)

    if add_inertial:
        mv = _get_mass_value(body_node)
        mass_for_urdf = mv if (mv is not None and mv > 0.0) else default_mass_if_missing
        if mass_for_urdf > 0.0:
            _add_inertial_epsilon(urdf_link, mass_for_urdf, inertia_epsilon)


def _convert_joint_node_to_urdf(
    robot_name: str,
    joint_node: ET.Element,
    urdf_robot: ET.Element,
    default_effort: float,
    default_velocity: float,
    revolute_lower: float,
    revolute_upper: float,
    prismatic_lower: float,
    prismatic_upper: float,
) -> None:
    name = joint_node.get("name") or "joint"
    jtype = joint_node.get("type", "fixed").strip()

    # Map to URDF joint types
    if jtype not in (
        "fixed",
        "revolute",
        "prismatic",
        "continuous",
        "floating",
        "planar",
    ):
        # Best-effort fallback
        sys.stderr.write(
            f"[WARN] Unknown joint type {jtype!r} for joint {name!r}, emitting as 'fixed'\n"
        )
        jtype = "fixed"

    urdf_joint = ET.SubElement(urdf_robot, "joint", {"name": name, "type": jtype})

    parent = joint_node.find("parent")
    child = joint_node.find("child")
    if parent is None or child is None:
        raise ValueError(f"Joint {name!r} missing <parent> or <child>")

    ET.SubElement(
        urdf_joint, "parent", {"link": f"{robot_name}/{parent.get('name', '')}"}
    )
    ET.SubElement(
        urdf_joint, "child", {"link": f"{robot_name}/{child.get('name', '')}"}
    )

    origin_T = _read_transform(joint_node.find("origin"))
    rpy = _mat_to_rpy(origin_T.R)
    if any(abs(x) > 1e-12 for x in origin_T.t) or any(abs(a) > 1e-12 for a in rpy):
        ET.SubElement(
            urdf_joint, "origin", {"xyz": _fmt_vec(origin_T.t), "rpy": _fmt_vec(rpy)}
        )

    axis_el = joint_node.find("axis")
    if axis_el is not None and axis_el.get("xyz"):
        axis = _parse_floats(axis_el.get("xyz"), n=3)
        ET.SubElement(urdf_joint, "axis", {"xyz": " ".join(_fmt(a) for a in axis)})

    # URDF generally expects limits for revolute/prismatic (except continuous)
    if jtype == "revolute":
        ET.SubElement(
            urdf_joint,
            "limit",
            {
                "lower": _fmt(revolute_lower),
                "upper": _fmt(revolute_upper),
                "effort": _fmt(default_effort),
                "velocity": _fmt(default_velocity),
            },
        )
    elif jtype == "prismatic":
        ET.SubElement(
            urdf_joint,
            "limit",
            {
                "lower": _fmt(prismatic_lower),
                "upper": _fmt(prismatic_upper),
                "effort": _fmt(default_effort),
                "velocity": _fmt(default_velocity),
            },
        )


def load_scenario_with_includes(path: Path) -> ET.Element:
    """
    Minimal include support:
      <include file="other.scn">
        <arg name="robot_name" value="X"/>
      </include>

    We inline included file children into the main root <scenario>.
    """
    tree = ET.parse(str(path))
    root = tree.getroot()
    if root.tag != "scenario":
        raise ValueError(f"Expected root <scenario>, got <{root.tag}> in {path}")

    base_dir = path.parent

    # Process includes iteratively (supports nested includes)
    changed = True
    while changed:
        changed = False
        includes = list(root.findall("include"))
        for inc in includes:
            inc_file = inc.get("file")
            if not inc_file:
                root.remove(inc)
                changed = True
                continue

            inc_path = (base_dir / inc_file).resolve()
            if not inc_path.exists():
                raise FileNotFoundError(f"Include file not found: {inc_path}")

            inc_tree = ET.parse(str(inc_path))
            inc_root = inc_tree.getroot()
            if inc_root.tag != "scenario":
                raise ValueError(
                    f"Included file {inc_path} root is <{inc_root.tag}>, expected <scenario>"
                )

            # inline children
            for child in list(inc_root):
                root.append(child)

            root.remove(inc)
            changed = True

    return root


def _xml_scenario_to_urdf(
    scenario_root: ET.Element,
    robot_name: str,
    mesh_prefix: Path | str,
    include_internal_parts_in_geometry: bool = False,
    add_inertial: bool = True,
    default_mass_if_missing: float = 0.0,
    inertia_epsilon: float = 1e-6,
    default_effort: float = 1.0,
    default_velocity: float = 1.0,
    revolute_lower: float = -math.pi,
    revolute_upper: float = math.pi,
    prismatic_lower: float = -1.0,
    prismatic_upper: float = 1.0,
) -> ET.Element:
    looks = _parse_looks(scenario_root)

    robots = list(scenario_root.findall("robot"))
    if not robots:
        raise ValueError("No <robot> found in scenario")

    robot_node: ET.Element | None = None
    if robot_name:
        for r in robots:
            if r.get("name") == robot_name:
                robot_node = r
                break
        if robot_node is None:
            raise ValueError(
                f"No <robot name={robot_name!r}> found; available: {[r.get('name') for r in robots]}"
            )
    else:
        robot_node = robots[0]

    out_robot_name = robot_node.get("name") or "robot"
    urdf_robot = ET.Element("robot", {"name": out_robot_name})

    # Define URDF materials for Stonefish looks
    _add_urdf_materials(urdf_robot, looks)

    base = robot_node.find("base_link")
    if base is None:
        raise ValueError("Robot has no <base_link>")

    # Convert base link and other links
    _convert_body_node_to_urdf_link(
        robot_name,
        base,
        urdf_robot,
        mesh_prefix=mesh_prefix,
        include_internal_parts_in_geometry=include_internal_parts_in_geometry,
        add_inertial=add_inertial,
        default_mass_if_missing=default_mass_if_missing,
        inertia_epsilon=inertia_epsilon,
    )

    for ln in robot_node.findall("link"):
        _convert_body_node_to_urdf_link(
            robot_name,
            ln,
            urdf_robot,
            mesh_prefix=mesh_prefix,
            include_internal_parts_in_geometry=include_internal_parts_in_geometry,
            add_inertial=add_inertial,
            default_mass_if_missing=default_mass_if_missing,
            inertia_epsilon=inertia_epsilon,
        )

    # Convert joints
    for jn in robot_node.findall("joint"):
        _convert_joint_node_to_urdf(
            robot_name,
            jn,
            urdf_robot,
            default_effort=default_effort,
            default_velocity=default_velocity,
            revolute_lower=revolute_lower,
            revolute_upper=revolute_upper,
            prismatic_lower=prismatic_lower,
            prismatic_upper=prismatic_upper,
        )

    return urdf_robot


def robot_scenario_to_urdf(
    scenario_xml: Path,
    robot_name: str,
    mesh_prefix: Path | str = "",
    include_internal_parts_in_geometry: bool = False,
    add_inertial: bool = True,
    default_mass_if_missing: float = 0.0,
    inertia_epsilon: float = 1e-6,
    default_effort: float = 1.0,
    default_velocity: float = 1.0,
    revolute_lower: float = -math.pi,
    revolute_upper: float = math.pi,
    prismatic_lower: float = -1.0,
    prismatic_upper: float = 1.0,
) -> Path:
    scenario_root = load_scenario_with_includes(scenario_xml)

    urdf_robot = _xml_scenario_to_urdf(
        scenario_root=scenario_root,
        robot_name=robot_name,
        mesh_prefix=mesh_prefix,
        include_internal_parts_in_geometry=include_internal_parts_in_geometry,
        add_inertial=add_inertial,
        default_mass_if_missing=default_mass_if_missing,
        inertia_epsilon=inertia_epsilon,
        default_effort=default_effort,
        default_velocity=default_velocity,
        revolute_lower=revolute_lower,
        revolute_upper=revolute_upper,
        prismatic_lower=prismatic_lower,
        prismatic_upper=prismatic_upper,
    )

    xml_str = _pretty_xml(urdf_robot)

    fd, temp_path = tempfile.mkstemp(prefix=scenario_xml.stem, suffix=".urdf.xml")
    with os.fdopen(fd, "w") as f:
        f.write(xml_str)

    return Path(temp_path)
