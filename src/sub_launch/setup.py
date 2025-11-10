import os
from glob import glob
from setuptools import find_packages, setup

package_name = "sub_launch"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob(os.path.join("launch", "*launch.[pxy][yma]*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AVBotz",
    maintainer_email="avbotzco@gmail.com",
    description="Launch files",
    license="Proprietary",
    entry_points={
        "console_scripts": [],
    },
)
