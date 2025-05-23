import glob
import os

from setuptools import find_packages, setup

package_name = "control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(
        include=["control*", "observation_prepare*"], exclude=["test"]
    ),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob.glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="merkul",
    maintainer_email="merkuloviv@my.msu.ru",
    description="Control robot package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "angles_to_effort_node = control.angles_to_effort_node:main",
            "control_angle_joint_node = control.control_angle_joint_node:main",
            "foot_contact_detector = observation_prepare.foot_contact_detector:main",
            "observation_aggregator = observation_prepare.observation_aggregator:main",
        ],
    },
)
