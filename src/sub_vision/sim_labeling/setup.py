from setuptools import find_packages, setup

package_name = "sim_labeling"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AVBotz",
    maintainer_email="avbotzco@gmail.com",
    description="Automatic labeling based on a segmentation camera",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "labeling = sim_labeling.node:main",
            "parse_ids = sim_labeling.parse_ids:main",
        ],
    },
)
