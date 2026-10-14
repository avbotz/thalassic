"""
Convert a robot of a rendered Stonefish scenario into a URDF, for robot_state_publisher to
publish so RViz and Foxglove can draw it:
- <looks> -> URDF <material>
- <base_link>/<link> -> URDF <link>, with a <visual> and a <collision> for each external part
  of a compound (internal parts are not drawn)
- <joint> -> URDF <joint>
Masses, sensors, actuators and hydrodynamics are not converted. Review the URDF (in Foxglove)
after changing a robot's layout.scn.j2.
"""

import math
import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

Vec3 = tuple[float, float, float]
Mat3 = list[list[float]]

# URDF requires limits on these joints; robot_state_publisher does not use them.
LIMITS = {"revolute": (-math.pi, math.pi), "prismatic": (-1.0, 1.0)}


@dataclass(frozen=True)
class Transform:
    """A rotation matrix R (a list of rows), then a translation t."""

    R: Mat3
    t: Vec3

    @staticmethod
    def identity() -> "Transform":
        return Transform([[1, 0, 0], [0, 1, 0], [0, 0, 1]], (0.0, 0.0, 0.0))

    @staticmethod
    def from_xyz_rpy(xyz: Vec3, rpy: Vec3) -> "Transform":
        """Roll, pitch and yaw as URDF and Stonefish apply them: R = Rz(yaw) Ry(pitch) Rx(roll)."""
        cr, cp, cy = (math.cos(a) for a in rpy)
        sr, sp, sy = (math.sin(a) for a in rpy)
        R = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
        return Transform(R, xyz)

    def rpy(self) -> Vec3:
        """The roll, pitch and yaw of R. At pitch ±90°, where only their sum or difference is
        defined, yaw is 0."""
        R = self.R
        cp = math.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0])
        pitch = math.atan2(-R[2][0], cp)
        if cp < 1e-9:
            return (math.atan2(-R[1][2], R[1][1]), pitch, 0.0)
        return (math.atan2(R[2][1], R[2][2]), pitch, math.atan2(R[1][0], R[0][0]))

    def __matmul__(self, other: "Transform") -> "Transform":
        A, B, v = self.R, other.R, other.t
        R = [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        tx, ty, tz = (
            self.t[i] + (A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2]) for i in range(3)
        )
        return Transform(R, (tx, ty, tz))


# A URDF geometry: its tag (mesh, box, ...), its attributes and its origin.
Geometry = tuple[str, dict[str, str], Transform]


def _vec3(text: str | None) -> Vec3:
    """An "x y z" attribute, or one value for all three (as a mesh's scale); 0 0 0 if absent."""
    values = [float(v) for v in (text or "0 0 0").split()]
    x, y, z = values * 3 if len(values) == 1 else values
    return (x, y, z)


def _fmt(*values: float) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def _transform(origin: ET.Element | None) -> Transform:
    """An <origin> or <compound_transform>; the identity if there is none."""
    if origin is None:
        return Transform.identity()
    return Transform.from_xyz_rpy(_vec3(origin.get("xyz")), _vec3(origin.get("rpy")))


def _add_origin(element: ET.Element, transform: Transform) -> None:
    """Gives `element` the transform as its <origin>, unless it is the identity (the default)."""
    rpy = transform.rpy()
    if any(abs(v) > 1e-12 for v in (*transform.t, *rpy)):
        ET.SubElement(element, "origin", {"xyz": _fmt(*transform.t), "rpy": _fmt(*rpy)})


def _mesh(node: ET.Element, which: str, mesh_prefix: str) -> Geometry | None:
    """A model's <physical> or <visual> mesh."""
    mesh = node.find(f"{which}/mesh")
    if mesh is None:
        return None
    attributes = {"filename": mesh_prefix + mesh.attrib["filename"]}
    if mesh.get("scale"):
        attributes["scale"] = _fmt(*_vec3(mesh.get("scale")))
    return "mesh", attributes, _transform(node.find(f"{which}/origin"))


def _primitive(node: ET.Element) -> Geometry | None:
    dimensions = node.find("dimensions")
    if dimensions is None:
        return None
    shape, size = node.get("type"), dimensions.attrib
    if shape == "box":
        attributes = {"size": _fmt(*_vec3(size["xyz"]))}
    elif shape == "cylinder":
        attributes = {"radius": _fmt(float(size["radius"])), "length": _fmt(float(size["height"]))}
    elif shape == "sphere":
        attributes = {"radius": _fmt(float(size["radius"]))}
    else:  # A torus or wing, which URDF has no geometry for
        return None
    return shape, attributes, _transform(node.find("origin"))


def _add_part(link: ET.Element, part: ET.Element, mesh_prefix: str) -> None:
    """Adds a body's or a compound part's visual (its visual mesh, else its primitive, else its
    physical mesh) and collision (its physical mesh, else its primitive) to `link`."""
    physical = _mesh(part, "physical", mesh_prefix)
    primitive = _primitive(part)
    shapes = {
        "visual": _mesh(part, "visual", mesh_prefix) or primitive or physical,
        "collision": physical or primitive,
    }
    # A part's pose in its compound; a body has none.
    pose = _transform(part.find("compound_transform"))
    look = part.find("look")
    for kind, shape in shapes.items():
        if shape is None:
            continue
        if part.tag == "external_part":  # A compound's parts share its link
            link.append(ET.Comment(f"{part.get('name')} {kind}"))
        tag, attributes, origin = shape
        element = ET.SubElement(link, kind)
        _add_origin(element, pose @ origin)
        ET.SubElement(ET.SubElement(element, "geometry"), tag, attributes)
        if kind == "visual" and look is not None:
            ET.SubElement(element, "material", {"name": look.attrib["name"]})


def robot_scenario_to_urdf(scenario_xml: Path, robot_name: str, mesh_prefix: str) -> Path:
    """Write the URDF of the robot `robot_name` in the rendered scenario `scenario_xml` to a
    temporary file, which the caller deletes. Its links are named `robot_name`/<link>, as
    Stonefish names them; `mesh_prefix` is put before the mesh filenames, which are relative
    to Stonefish's data directory."""
    scenario = ET.parse(scenario_xml).getroot()
    robot = scenario.find(f"robot[@name='{robot_name}']")
    if robot is None:
        raise ValueError(f"{scenario_xml} has no <robot name={robot_name!r}>")

    urdf = ET.Element("robot", {"name": robot_name})
    for look in scenario.findall("looks/look"):
        rgb = _vec3(look.get("rgb")) if look.get("rgb") else (float(look.get("gray", 0.5)),) * 3
        material = ET.SubElement(urdf, "material", {"name": look.attrib["name"]})
        ET.SubElement(material, "color", {"rgba": _fmt(*rgb, 1.0)})

    for body in robot.findall("base_link") + robot.findall("link"):
        link = ET.SubElement(urdf, "link", {"name": f"{robot_name}/{body.attrib['name']}"})
        compound = body.get("type") == "compound"
        for part in body.findall("external_part") if compound else [body]:
            _add_part(link, part, mesh_prefix)

    for joint in robot.findall("joint"):
        # Stonefish's joint types (fixed, revolute and prismatic) are URDF's.
        kind = joint.get("type", "fixed")
        element = ET.SubElement(urdf, "joint", {"name": joint.attrib["name"], "type": kind})
        for end in ("parent", "child"):
            node = joint.find(end)
            if node is None:
                raise ValueError(f"Joint {joint.get('name')!r} has no <{end}>")
            ET.SubElement(element, end, {"link": f"{robot_name}/{node.attrib['name']}"})
        _add_origin(element, _transform(joint.find("origin")))
        axis = joint.find("axis")
        if axis is not None:
            ET.SubElement(element, "axis", {"xyz": _fmt(*_vec3(axis.get("xyz")))})
        if kind in LIMITS:
            lower, upper = LIMITS[kind]
            limit = {"lower": _fmt(lower), "upper": _fmt(upper), "effort": "1", "velocity": "1"}
            ET.SubElement(element, "limit", limit)

    ET.indent(urdf)
    fd, path = tempfile.mkstemp(prefix=scenario_xml.stem, suffix=".urdf.xml")
    with os.fdopen(fd, "w") as f:
        f.write(ET.tostring(urdf, encoding="unicode"))
    return Path(path)
