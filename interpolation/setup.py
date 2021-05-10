from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'interpolation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('interpolation/*.rviz')),
        (os.path.join('share', package_name), glob('interpolation/interpolation/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='piotr',
    maintainer_email='piotrhondra@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'KDL = interpolation.KDL:main',
            'nonKDL = interpolation.nonkdl:main',
            'service = interpolation.service:main',
            'client = interpolation.client:main',
            'ointService = interpolation.oint_Service:main',
            'ointClient = interpolation.oint_Client:main',
            
        ],
    },
)
