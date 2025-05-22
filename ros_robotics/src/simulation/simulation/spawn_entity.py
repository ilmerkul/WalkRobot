#!/usr/bin/env python3
import time

import rclpy
import yaml
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Point, Pose, Quaternion
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn


class SpawnEntityNode(LifecycleNode):
    def __init__(self):
        super().__init__("spawn_entity_node")
        self.config = None
        self.spawned_entities = []
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "spawn_config",
                    "",
                    ParameterDescriptor(
                        description="spawn config",
                        type=ParameterType.PARAMETER_STRING,
                    ),
                ),
                (
                    "robot_description_content",
                    "",
                    ParameterDescriptor(
                        description="urdf robot",
                        type=ParameterType.PARAMETER_STRING,
                    ),
                ),
                (
                    "entity_count",
                    1,
                    ParameterDescriptor(
                        description="Count entity",
                        type=ParameterType.PARAMETER_INTEGER,
                    ),
                ),
            ],
        )

        self.spawn_service_name = "/spawn_entity"
        self.spawn_service = self.create_client(SpawnEntity, self.spawn_service_name)

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

    def spawn_robot(self):
        robot_description_content = self.get_parameter(
            "robot_description_content"
        ).value
        entity_count = int(self.get_parameter("entity_count").value)

        entity = self.config["entity"]
        initial_x = float(self.config["x"])

        for i in range(entity_count):
            req = SpawnEntity.Request()
            req.name = f"{entity}_{i}"
            req.xml = robot_description_content
            # req.robot_namespace = f"{entity}_{i}"
            req.reference_frame = "world"
            req.initial_pose = Pose(
                position=Point(
                    x=initial_x, y=float(self.config["y"]), z=float(self.config["z"])
                ),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )

            self.get_logger().info(f"Spawning {entity}_{i}...")

            self.call_service(self.spawn_service, req, self.spawn_service_name)

            time.sleep(0.5)

    def call_service(
        self, client, req, service_name: str, timeout_sec: float = 5.0
    ) -> bool:
        try:
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if future.done():
                future.result()
                self.get_logger().info(f"Successfully: {service_name}")
                return True
            else:
                self.get_logger().warning(
                    f"The {service_name} service did not respond, but the command could have been executed"
                )
                return True

        except Exception as e:
            self.get_logger().error(f"Error calling the service: {str(e)}")
            return False


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    node = SpawnEntityNode()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
