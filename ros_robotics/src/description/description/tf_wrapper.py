#!/usr/bin/env python3
import os
from subprocess import Popen
from typing import List, Optional

import rclpy
import yaml
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.parameter import Parameter


class TFWrapper(Node):
    def __init__(self):
        super().__init__("tf_wrapper")

        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "x",
                    0.0,
                    ParameterDescriptor(
                        description="Translation in X axis (meters)",
                        type=ParameterType.PARAMETER_DOUBLE,
                    ),
                ),
                (
                    "y",
                    0.0,
                    ParameterDescriptor(
                        description="Translation in Y axis (meters)",
                        type=ParameterType.PARAMETER_DOUBLE,
                    ),
                ),
                (
                    "z",
                    0.0,
                    ParameterDescriptor(
                        description="Translation in Z axis (meters)",
                        type=ParameterType.PARAMETER_DOUBLE,
                    ),
                ),
                (
                    "roll",
                    0.0,
                    ParameterDescriptor(
                        description="Rotation around X axis (radians)",
                        type=ParameterType.PARAMETER_DOUBLE,
                    ),
                ),
                (
                    "pitch",
                    0.0,
                    ParameterDescriptor(
                        description="Rotation around Y axis (radians)",
                        type=ParameterType.PARAMETER_DOUBLE,
                    ),
                ),
                (
                    "yaw",
                    0.0,
                    ParameterDescriptor(
                        description="Rotation around Z axis (radians)",
                        type=ParameterType.PARAMETER_DOUBLE,
                    ),
                ),
                (
                    "frame-id",
                    "map",
                    ParameterDescriptor(
                        description="Parent TF frame",
                        type=ParameterType.PARAMETER_STRING,
                    ),
                ),
                (
                    "child-frame-id",
                    "base_link",
                    ParameterDescriptor(
                        description="Child TF frame",
                        type=ParameterType.PARAMETER_STRING,
                    ),
                ),
                (
                    "config_file",
                    "",
                    ParameterDescriptor(
                        description="Path to YAML config file",
                        type=ParameterType.PARAMETER_STRING,
                    ),
                ),
                (
                    "namespace",
                    "",
                    ParameterDescriptor(
                        description="Namespace",
                        type=ParameterType.PARAMETER_STRING,
                    ),
                ),
            ],
        )

        # Подписываемся на изменения параметров
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Процесс static_transform_publisher
        self.tf_process: Optional[Popen] = None

        # Инициализация трансформации
        self.update_transform()

    def load_config(self, file_path: str) -> bool:
        """Загрузка параметров из YAML-файла"""
        if not os.path.exists(file_path):
            self.get_logger().error(f"Config file not found: {file_path}")
            return False

        try:
            with open(file_path, "r") as f:
                config = yaml.safe_load(f)

            # Устанавливаем параметры из конфига
            params_to_set = []
            for param_name, param_value in config.items():
                if not self.has_parameter(param_name):
                    continue

                param_type = self.get_parameter(param_name).type_
                try:
                    if param_type == ParameterType.PARAMETER_DOUBLE:
                        validated_value = float(param_value)
                    elif param_type == ParameterType.PARAMETER_STRING:
                        validated_value = str(param_value)
                    else:
                        continue

                    params_to_set.append(
                        Parameter(param_name, param_type, validated_value)
                    )
                except (ValueError, TypeError) as e:
                    self.get_logger().warning(
                        f"Invalid type for parameter {param_name}: {type(param_value)}"
                    )
                    continue

            if params_to_set:
                self.set_parameters(params_to_set)
                return True

        except Exception as e:
            self.get_logger().error(f"Error loading config: {str(e)}")

        return False

    def update_transform(self):
        """Обновление трансформации (перезапуск publisher)"""
        # Убиваем предыдущий процесс
        if self.tf_process:
            self.tf_process.terminate()
            try:
                self.tf_process.wait(timeout=1.0)
            except:
                self.tf_process.kill()

        # flags_name = [
        #    'x', 'y', 'z',
        #    'roll', 'pitch', 'yaw',
        #    'frame-id', 'child-frame-id'
        # ]

        params = self.get_parameters_by_prefix("")
        del params["config_file"]
        del params["use_sim_time"]
        del params["namespace"]
        params = params.items()
        flags_cmd = ["" for _ in range(2 * len(params))]

        frame_id = ""
        child_frame_id = ""
        for i, flag in enumerate(params):
            flag_name, flag_param = flag
            flag_value = str(flag_param.value)

            flags_cmd[2 * i] = f"--{flag_name}"
            flags_cmd[2 * i + 1] = flag_value

            if flag_name == "frame-id":
                frame_id = flag_value
            elif flag_name == "child-frame-id":
                child_frame_id = flag_value

        namespace = self.get_parameter("namespace").value
        if not namespace.startswith("/"):
            namespace = "/" + namespace
        cmd = [
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            *flags_cmd,
            "--ros-args",
            "-r",
            f"__ns:={namespace}",
            "--remap",
            f"/tf_static:={namespace}/tf_static",
            "--remap",
            f"/tf:={namespace}/tf",
        ]

        self.tf_process = Popen(cmd)
        self.get_logger().info(
            f"TF published: {frame_id} -> {child_frame_id} with {cmd}"
        )

    def parameters_callback(self, params: List[rclpy.Parameter]):
        """Обработчик изменения параметров"""
        # Проверяем, изменились ли параметры трансформации
        # transform_params = {'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'frame-id', 'child-frame-id'}
        transform_params = self.get_parameters_by_prefix("")
        del transform_params["config_file"]
        del transform_params["use_sim_time"]
        transform_params = transform_params.keys()
        if any(p.name in transform_params for p in params):
            self.update_transform()

        # Обработка изменения config_file
        config_param = next((p for p in params if p.name == "config_file"), None)
        if config_param and config_param.value:
            self.load_config(config_param.value)

        return rclpy.node.SetParametersResult(successful=True)

    def destroy_node(self):
        """Очистка при завершении"""
        if self.tf_process:
            self.tf_process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TFWrapper()

    try:
        # Проверяем наличие конфига при старте
        config_file = node.get_parameter("config_file").value
        if config_file:
            node.load_config(config_file)

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
