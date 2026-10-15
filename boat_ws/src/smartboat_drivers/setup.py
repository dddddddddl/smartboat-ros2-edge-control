import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'smartboat_drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu2204',
    maintainer_email='ubuntu2204@todo.todo',
    description='Device and sensor driver nodes for the smart boat.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sr73f_radar_node = smartboat_drivers.sr73f_radar_node:main',
            'water_quality_node = smartboat_drivers.water_quality_node:main',
        ],
    },
)
