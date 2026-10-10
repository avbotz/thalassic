"""
YOLO classes of the simulated props, and the objects that belong to each.

Stonefish draws every named object (entity, sensor, actuator) in the
segmentation image with its own pixel value, and publishes the object names on
the latched `label_info` topic next to the image (vision_msgs/LabelInfo).
Objects are assigned to classes by matching their names against prefixes.
"""


def get_default_prop_definitions() -> dict[str, str]:
    """
    Returns a mapping from object name prefixes to class names.

    Objects not matching any prefix are assigned class "background" and
    excluded from annotation. The first matching prefix wins.
    """
    return {
        # Pool (not annotated — background)
        "woollett_": "background",
        "natatorium_": "background",
        # Each prop is one static named after it; Stonefish numbers repeats
        # (slalom, slalom1, slalom2), so these match without an underscore.
        # Gate task
        "gate": "gate",
        # Path markers
        "path": "path",
        # Slalom
        "slalom": "slalom",
        # Bin task
        "bin": "bin",
        # Torpedo board
        "torpboard_": "torpboard",
        # Octagon task
        "octagon": "octagon",
        "table": "table",
        "bottle_red": "bottle_red",
        "bottle_yellow": "bottle_yellow",
        "ladle_red": "ladle_red",
        "ladle_yellow": "ladle_yellow",
        # The robot (not annotated): its body is marlin_v3_Dynamics, its
        # sensors and thrusters marlin_v3/<name>.
        "marlin_v3": "background",
    }


def get_class_names() -> list[str]:
    """Return the ordered list of YOLO class names (index = class_id)."""
    return [
        "gate",  # 0
        "path",  # 1
        "slalom",  # 2
        "bin",  # 3
        "torpboard",  # 4
        "octagon",  # 5
        "table",  # 6
        "bottle_red",  # 7
        "bottle_yellow",  # 8
        "ladle_red",  # 9
        "ladle_yellow",  # 10
    ]


def class_name_for(object_name: str, prop_definitions: dict[str, str]) -> str:
    """Return the class name of a Stonefish object, "background" if it has none."""
    for prefix, class_name in prop_definitions.items():
        if object_name.startswith(prefix):
            return class_name
    return "background"


def build_pixel_to_class_id(
    object_names: dict[int, str],
    prop_definitions: dict[str, str] | None = None,
) -> dict[int, int]:
    """
    Map segmentation pixel values to YOLO class IDs.

    Args:
        object_names:     segmentation pixel value -> Stonefish object name,
                          from the label_info topic.
        prop_definitions: object name prefix -> class name; the default
                          mapping if None.

    Returns:
        pixel value -> class ID, for the pixel values of annotated objects only.
    """
    if prop_definitions is None:
        prop_definitions = get_default_prop_definitions()
    class_name_to_id = {n: i for i, n in enumerate(get_class_names())}

    pixel_to_class: dict[int, int] = {}
    for pixel_value, name in object_names.items():
        class_id = class_name_to_id.get(class_name_for(name, prop_definitions), -1)
        if class_id >= 0:
            pixel_to_class[pixel_value] = class_id
    return pixel_to_class
