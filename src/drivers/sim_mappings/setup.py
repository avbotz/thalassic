from setuptools import find_packages, setup

package_name = 'sim_mappings'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AVBotz',
    maintainer_email='avbotzco@gmail.com',
    description='Simulation to real message mappings for drivers',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'sim_dvl_bridge = sim_mappings.sim_dvl_bridge:main',
        ],
    },
)
