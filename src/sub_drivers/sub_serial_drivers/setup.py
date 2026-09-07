from setuptools import find_packages, setup

package_name = "sub_serial_drivers"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="AVBotz",
    maintainer_email="avbotzco@gmail.com",
    description="Serial bridge drivers (Nautical / Maritime, NaviGuider IMU) for the Marlin V2 AUV.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "sub_low = sub_serial_drivers.sub_low:main",
            "naviguider_imu_driver = sub_serial_drivers.naviguider_imu_driver:main",
        ],
    },
)
