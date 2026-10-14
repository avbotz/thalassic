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
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="AVBotz",
    maintainer_email="software@avbotz.com",
    description="YOLO detection pipeline (ONNX Runtime / TensorRT) for the Marlin V3 AUV.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sub_vision = sub_vision.vision_node:main",
            "build_engines = sub_vision.build_engines:main",
        ],
    },
)
