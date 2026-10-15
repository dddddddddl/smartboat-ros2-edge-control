from glob import glob
from setuptools import setup

package_name = 'smartboat_control'

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
    description='Smartboat PID control nodes.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'yaw_from_imu_node = smartboat_control.yaw_from_imu_node:main',
            'yaw_from_mavlink_node = smartboat_control.yaw_from_mavlink_node:main',
        ],
    },
)
