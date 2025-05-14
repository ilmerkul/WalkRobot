import glob
import os

from setuptools import find_packages, setup

package_name = "description"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(include=["description*"], exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*")),
        (os.path.join("share", package_name, "rviz"), glob.glob("rviz/*")),
        (os.path.join("share", package_name, "urdf"), glob.glob("urdf/*")),
        (os.path.join("share", package_name, "config"), glob.glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="merkul",
    maintainer_email="merkuloviv@my.msu.ru",
    description="Description of robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_description_publisher = description.robot_description_publisher:main",
            "tf_wrapper = description.tf_wrapper:main",
        ],
    },
)
