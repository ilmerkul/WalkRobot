import math
import os
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory

pkg_path = get_package_share_directory("description")
robot_property_path = os.path.join(pkg_path, "xacro", "robot_property.xacro")

pkg_control_path = get_package_share_directory("control")
config_controller_path = os.path.join(pkg_control_path, "config", "control.yaml")


def evaluate_expression(expr, constants):
    for name, value in constants.items():
        expr = expr.replace(name, str(value))

    expr = expr.replace("PI", str(math.pi))

    try:
        return eval(
            expr,
            {"__builtins__": None},
            {"sqrt": math.sqrt, "sin": math.sin, "cos": math.cos},
        )
    except:
        return expr


def parse_xacro_constants(xml_content: str) -> Dict[str, float]:
    constants = {}

    root = ET.fromstring(xml_content)

    for prop in root.findall(
        ".//xacro:property", {"xacro": "http://www.ros.org/wiki/xacro"}
    ):
        name = prop.get("name")
        value = prop.get("value")
        constants[name] = value

    for name, value in constants.items():
        if value.startswith("${") and value.endswith("}"):
            expr = value[2:-1].strip()
            try:
                constants[name] = evaluate_expression(expr, constants)
            except:
                constants[name] = value

    return {k: float(v) for k, v in constants.items()}


def parse_file_xacro_constants(xml_file: str = robot_property_path) -> Dict[str, float]:
    with open(xml_file, mode="r") as f:
        xml_content = f.readlines()

    xml_content = "\n".join(xml_content)
    return parse_xacro_constants(xml_content)


def parse_joint_class_angle_borders() -> Dict[str, Tuple[float, float]]:
    constants = parse_file_xacro_constants()
    joint_classes = get_joint_classes()

    angle_border = dict()

    angles = list(filter(lambda x: "angle" in x, constants.keys()))
    for joint_class in joint_classes:
        class_angle_names = list(filter(lambda x: joint_class in x, angles))

        class_angle_lower = list(filter(lambda x: "lower" in x, class_angle_names))
        if len(class_angle_lower) == 0:
            continue
        class_angle_lower = class_angle_lower[0]

        class_angle_upper = list(filter(lambda x: "upper" in x, class_angle_names))
        if len(class_angle_upper) == 0:
            continue
        class_angle_upper = class_angle_upper[0]

        angle_border[joint_class] = (
            constants[class_angle_lower],
            constants[class_angle_upper],
        )

    return angle_border


def get_joint_classes() -> List[str]:
    return ["servo", "leg1", "leg2"]


def get_joints(file_path: str = config_controller_path) -> List[str]:
    with open(file_path, "r") as file:
        config = yaml.safe_load(file)

    return config[list(config.keys())[0]]["effort_controller"]["ros__parameters"][
        "joints"
    ]


DIM_XYZ = 3
DIM_XYZW = 4
DIM_FOOTS = 4
JOINT_ORDER = get_joints()
DIM_JOINT = len(JOINT_ORDER)
XACRO_CONSTANTS = parse_file_xacro_constants()
