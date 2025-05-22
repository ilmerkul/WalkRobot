import os
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory

pkg_path = get_package_share_directory("control")
config_controller_path = os.path.join(pkg_path, "config", "robot.yaml")


def get_joints(file_path: str = config_controller_path) -> List[str]:
    with open(file_path, "r") as file:
        config = yaml.safe_load(file)

    return config["effort_controller"]["ros__parameters"]["joints"]
