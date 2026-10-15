import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'smartboat_navigation'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu2204',
    maintainer_email='ubuntu2204@todo.todo',
    description='Local planning and safety supervision for the smartboat.',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'avoidance_supervisor_node = '
            'smartboat_navigation.avoidance_supervisor_node:main',
            'avoidance_executor_node = '
            'smartboat_navigation.avoidance_executor_node:main',
        ],
    },
)
