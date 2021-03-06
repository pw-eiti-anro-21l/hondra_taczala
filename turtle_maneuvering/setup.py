from setuptools import setup
import os
from glob import glob

package_name = 'turtle_maneuvering'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piotr Hondra, Michał Taczała',
    maintainer_email='piotrhondra@gmail.com',
    description='Package to maneuver turtlesim robot given any key.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move = turtle_maneuvering.maneuver:main',
            'param_talker = turtle_maneuvering.parameter:main',
        ],
    },
)
