import glob
import os

from setuptools import find_packages, setup

package_name = "dl_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(include=["dl_control*"], exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="merkul",
    maintainer_email="merkuloviv@my.msu.ru",
    description="Neural Network for robot control",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stand_up_node = dl_control.stand_up.stand_up_node:main",
        ],
    },
)
