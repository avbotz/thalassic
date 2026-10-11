"""The prop-to-class table agrees with the class list the datasets are written with."""

from sim_labeling.classes import (
    build_pixel_to_class_id,
    class_name_for,
    get_class_names,
    get_default_prop_definitions,
)


def test_every_class_has_a_prop_and_every_prop_a_class():
    classes = get_class_names()
    assert len(set(classes)) == len(classes)
    targets = set(get_default_prop_definitions().values()) - {"background"}
    assert targets == set(classes)


def test_repeated_props_and_the_pool_and_robot():
    props = get_default_prop_definitions()
    assert class_name_for("slalom2", props) == "slalom"
    assert class_name_for("torpboard_v2", props) == "torpboard"
    assert class_name_for("woollett_floor", props) == "background"
    assert class_name_for("marlin_v3/front_camera", props) == "background"
    assert class_name_for("something_else", props) == "background"


def test_pixel_map_keeps_annotated_objects_only():
    pixels = build_pixel_to_class_id({1: "gate", 2: "woollett_wall", 3: "ladle_yellow1", 4: "x"})
    classes = get_class_names()
    assert pixels == {1: classes.index("gate"), 3: classes.index("ladle_yellow")}
