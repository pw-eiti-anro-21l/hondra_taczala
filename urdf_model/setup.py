from setuptools import setup
import os
from glob import glob
from setuptools import find_packages
package_name = 'urdf_model'

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
	(os.path.join('share', package_name), glob('resource/*')),
	(os.path.join('share', package_name), glob('urdf_model/*.py')),
    ],
    install_requires=['setuptools', 'pandas'],
    zip_safe=True,
    maintainer='piotr',
    maintainer_email='piotrhondra@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = urdf_model.state_publisher:main'
        ],
    },
)
