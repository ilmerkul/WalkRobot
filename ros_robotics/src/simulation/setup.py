import glob
import os

from setuptools import find_packages, setup


def get_package_name():
    import xml.etree.ElementTree as ET

    tree = ET.parse("package.xml")
    ros_pkg_name = tree.find("name").text
    return ros_pkg_name


package_name = get_package_name()

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob.glob("config/*")),
        (os.path.join("share", package_name, "worlds"), glob.glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="merkul",
    maintainer_email="merkuloviv@my.msu.ru",
    description="Package for simulation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spawn_entity_wrapper = simulation.spawn_entity_wrapper:main",
            "spawn_entity = simulation.spawn_entity:main",
        ],
    },
)
