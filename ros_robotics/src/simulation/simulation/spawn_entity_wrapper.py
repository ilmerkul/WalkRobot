#!/usr/bin/env python3
import subprocess
import time

import rclpy
import yaml
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn


class SpawnEntityWrapper(LifecycleNode):
    def __init__(self):
        super().__init__("spawn_entity_wrapper")
        self.config = None
        self.spawned_entities = []
        self.declare_parameter("spawn_config")

        self.get_logger().info("Node started")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring...")

        try:
            config_file = self.get_parameter("spawn_config").value

            if not config_file:
                raise ValueError("spawn_config parameter is required")

            with open(config_file, "r") as f:
                self.config = yaml.safe_load(f)["spawn"]

            self.get_logger().info("Configuration successful")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"Configuration failed: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")

        if not self.config:
            self.get_logger().error("No configuration available!")
            return TransitionCallbackReturn.FAILURE

        try:
            self.spawn_robot()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Activation failed: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        self.config = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        self.config = None
        return TransitionCallbackReturn.SUCCESS

    def spawn_robot(self, entity_count: int = 1):
        entity = self.config["entity"]
        initial_x = self.config["x"]

        for i in range(entity_count):
            cmd = [
                "ros2",
                "run",
                "gazebo_ros",
                "spawn_entity.py",
                "-entity",
                f"{entity}_{i}",
                "-x",
                str(initial_x + 2 * i),
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

            self.get_logger().info(f"Spawning {entity}_{i}...")

            try:
                process = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE
                )
                stdout, stderr = process.communicate(timeout=10)

                if process.returncode == 0:
                    self.spawned_entities.append(f"{entity}_{i}")
                    self.get_logger().info(f"Successfully spawned {entity}_{i}")
                    self.get_logger().debug(stdout.decode())
                else:
                    self.get_logger().error(
                        f"Failed to spawn {entity}_{i}: {stderr.decode()}"
                    )

                time.sleep(0.5)

            except subprocess.TimeoutExpired:
                process.kill()
                self.get_logger().error(f"Timeout while spawning {entity}_{i}")
            except Exception as e:
                self.get_logger().error(f"Error spawning {entity}_{i}: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = SpawnEntityWrapper()
    executor.add_node(lc_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        lc_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
