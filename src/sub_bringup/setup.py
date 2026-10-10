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
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.xml")),
        (f"share/{package_name}/config/control", glob("config/control/*.yaml")),
        (f"share/{package_name}/config/devices", glob("config/devices/*.yaml")),
        (f"share/{package_name}/config/vehicles", glob("config/vehicles/*.yaml")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AVBotz",
    maintainer_email="software@avbotz.com",
    description="Launch and configuration files",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "static_transforms = sub_bringup.static_transforms:main",
        ],
    },
)
