import glob
import os

from setuptools import find_packages, setup

package_name = "control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*")),
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
            "observation_publisher = control.observation_publisher:main",
            "planner_subscriber = control.planner_subscriber:main",
        ],
    },
)
