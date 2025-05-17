import math
import os
import xml.etree.ElementTree as ET
from typing import Dict, Tuple

from ament_index_python.packages import get_package_share_directory

pkg_path = get_package_share_directory("description")
robot_property_path = os.path.join(pkg_path, "urdf", "robot_property.urdf.xacro")


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


def parse_phi_border() -> Tuple[Tuple[float, float], Tuple[float, float]]:
    constants = parse_file_xacro_constants()
    phi1_border = (
        constants["leg1_angle_lower_x"],
        constants["leg1_angle_upper_x"],
    )
    phi2_border = (
        constants["leg2_angle_lower_x"],
        constants["leg2_angle_upper_x"],
    )

    return phi1_border, phi2_border
