from glob import glob

from setuptools import find_packages, setup

package_name = "sub_pid_tuner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/web", glob("web/*")),
    ],
    install_requires=["setuptools", "aiohttp", "pyyaml"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="AVBotz",
    maintainer_email="software@avbotz.com",
    description="Browser dashboard for tuning sub_control live",
    license="Proprietary",
    entry_points={"console_scripts": ["dashboard = sub_pid_tuner.server:main"]},
)
