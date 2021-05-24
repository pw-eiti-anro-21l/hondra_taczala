from setuptools import setup, find_packages
import os
from glob import glob

package_name = "inverse_kin"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.py")),
        (os.path.join("share", package_name), glob("urdf/*")),
        (os.path.join("share", package_name), glob("inverse_kin/*.rviz")),
        (os.path.join("share", package_name), glob("inverse_kin/inverse_kin/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="piotr",
    maintainer_email="piotrhondra@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "KDL = inverse_kin.KDL:main",
            "nonKDL = inverse_kin.nonkdl:main",
            "service = inverse_kin.service:main",
            "client = inverse_kin.client:main",
            "ointService = inverse_kin.oint_Service:main",
            "ointClient = inverse_kin.oint_Client:main",
            "ikin = inverse_kin.IKIN:main",
        ],
    },
)
