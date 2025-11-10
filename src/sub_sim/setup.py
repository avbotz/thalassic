import os
from setuptools import find_packages, setup

package_name = "sub_sim"


# Source - https://stackoverflow.com/a/65432634
def generate_data_files(data_dir):
    data_files = []
    for path, dirs, files in os.walk(data_dir):
        install_dir = f"share/{package_name}/" + path
        list_entry = (install_dir, [os.path.join(path, f) for f in files if not f.startswith(".")])
        data_files.append(list_entry)

    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        *generate_data_files("scenarios"),
        *generate_data_files("data"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AVBotz",
    maintainer_email="avbotzco@gmail.com",
    description="Files and scripts relating to simulator",
    license="Proprietary",
    entry_points={
        "console_scripts": [],
    },
)
