import os
from glob import glob

from setuptools import find_packages, setup

package_name = "sub_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AVBotz",
    maintainer_email="avbotzco@gmail.com",
    description="YOLOv10 perception pipeline (ONNX Runtime / TensorRT) for the Marlin V2 AUV.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sub_vision = sub_vision.vision_node:main",
            "annotation_visualizer = sub_vision.annotating_video:main",
            "test_gate_model = sub_vision.run_gate_model:main",
        ],
    },
)
