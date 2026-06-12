from setuptools import find_packages, setup

package_name = "sub_color_correction"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AVBotz",
    maintainer_email="avbotzco@gmail.com",
    description="RGB-D underwater color correction for Marlin camera streams.",
    license="Proprietary AND AGPL-3.0-only",
    entry_points={
        "console_scripts": [
            "deepseecolor_node = sub_color_correction.deepseecolor_node:main",
            "deepseecolor_smoke_test = sub_color_correction.deepseecolor_smoke_test:main",
            "deepseecolor_compare = sub_color_correction.deepseecolor_compare:main",
            "depth_snapshot = sub_color_correction.depth_snapshot:main",
        ],
    },
)
