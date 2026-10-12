import os
from glob import glob

from setuptools import find_packages, setup

package_name = "sub_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*_launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
        # Every config file, keeping its directory (config/devices/..., config/vehicles/...).
        *(
            (f"share/{package_name}/{os.path.dirname(path)}", [path])
            for path in glob("config/**/*.*", recursive=True)
        ),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="AVBotz",
    maintainer_email="software@avbotz.com",
    description="Launch and configuration files",
    license="MIT",
    entry_points={
        "console_scripts": [
            "static_transforms = sub_bringup.static_transforms:main",
            "preflight = sub_bringup.preflight:main",
        ],
    },
)
