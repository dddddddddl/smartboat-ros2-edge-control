from glob import glob
from setuptools import setup

package_name = 'smartboat_px4_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rx z',
    maintainer_email='rxz542056@gmail.com',
    description='Smartboat PX4/MAVLink bridge nodes.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'cmd_vel_to_mavlink_node = smartboat_px4_bridge.cmd_vel_to_mavlink_node:main',
        ],
    },
)
