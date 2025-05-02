#!/usr/bin/env python3
import subprocess

import rclpy
import yaml
from rclpy.node import Node


class SpawnEntityWrapper(Node):
    def __init__(self, args=None):
        super().__init__("spawn_entity_wrapper")

        # Загружаем параметры
        self.declare_parameter("spawn_config")
        config_file = self.get_parameter("spawn_config").value

        try:
            with open(config_file, "r") as f:
                self.config = yaml.safe_load(f)["spawn_params"]
            self.spawn_robot()
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {str(e)}")
            raise

        self.spawn_robot()

    def spawn_robot(self):
        cmd = [
            "ros2",
            "run",
            "gazebo_ros",
            "spawn_entity.py",
            "-entity",
            self.config["entity"],
            "-x",
            str(self.config["x"]),
            "-y",
            str(self.config["y"]),
            "-z",
            str(self.config["z"]),
            "-R",
            str(self.config["R"]),
            "-P",
            str(self.config["P"]),
            "-Y",
            str(self.config["Y"]),
            "-topic",
            self.config["topic"],
        ]

        self.get_logger().info(f"Executing: {' '.join(cmd)}")
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        output, _ = process.communicate()
        self.get_logger().info(output.decode())


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SpawnEntityWrapper(args=args)
        rclpy.spin(node)
    except Exception as e:
        rclpy.logging.get_logger("spawn_entity_wrapper").fatal(f"Error: {str(e)}")
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
