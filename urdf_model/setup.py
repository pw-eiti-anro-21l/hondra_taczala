from setuptools import setup

package_name = 'urdf_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'urdf_maker = urdf_model.urdf_maker:main'
        ],
    },
)