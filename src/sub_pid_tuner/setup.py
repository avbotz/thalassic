import os
from glob import glob
from setuptools import find_packages, setup

package_name = "sub_pid_tuner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/web",
            glob(os.path.join("web", "*")),
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="AVBotz",
    maintainer_email="avbotzco@gmail.com",
    description="Browser-based live ROS debugging dashboard",
    license="Proprietary",
    entry_points={"console_scripts": ["dashboard = sub_pid_tuner.server:main"]},
)
