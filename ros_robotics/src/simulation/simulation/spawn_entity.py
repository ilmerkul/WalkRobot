#!/usr/bin/env python3
import multiprocessing
import os
import subprocess
import time
from subprocess import PIPE, Popen

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control.utils import modify_namespace_and_save_yaml_files
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
                    "xacro_file",
                    "",
                    ParameterDescriptor(
                        description="xacro file robot",
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
                (
                    "base_namespace",
                    "tropy_spot",
                    ParameterDescriptor(
                        description="robot namespace",
                        type=ParameterType.PARAMETER_STRING,
                    ),
                ),
                (
                    "tfconfig",
                    "",
                    ParameterDescriptor(
                        description="tfconfig",
                        type=ParameterType.PARAMETER_STRING,
                    ),
                ),
                (
                    "gui",
                    False,
                    ParameterDescriptor(
                        description="gui",
                        type=ParameterType.PARAMETER_BOOL,
                    ),
                ),
                (
                    "control_file",
                    "control.yaml",
                    ParameterDescriptor(
                        description="control file",
                        type=ParameterType.PARAMETER_STRING,
                    ),
                ),
                (
                    "xacro_file",
                    "",
                    ParameterDescriptor(
                        description="xacro file",
                        type=ParameterType.PARAMETER_STRING,
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
            modify_namespace_and_save_yaml_files(
                str(self.get_parameter("control_file").value),
                int(self.get_parameter("entity_count").value),
                str(self.get_parameter("base_namespace").value),
            )

            config_file = self.get_parameter("spawn_config").value

            if not config_file:
                raise ValueError("spawn_config parameter is required")

            with open(config_file, "r") as f:
                self.config = yaml.safe_load(f)["spawn"]

            self.get_logger().info("Configuration successful")

        except Exception as e:
            self.get_logger().error(f"Configuration failed: {str(e)}")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")

        if not self.config:
            self.get_logger().error("No configuration available!")
            return TransitionCallbackReturn.FAILURE

        try:
            self.spawn_robot()
        except Exception as e:
            self.get_logger().error(f"Activation failed: {str(e)}")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

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

    def process_xacro(self, namespace: str, entity_idx: int) -> str:
        xacro_file = str(self.get_parameter("xacro_file").value)
        xacro_file = os.path.join(
            get_package_share_directory("description"), "xacro", "urdf", xacro_file
        )

        cmd = [
            "xacro",
            xacro_file,
            f"namespace:=/{namespace}",
            "exclude_ros2_control:=false",
            f"entity_idx:={entity_idx}",
        ]

        process = Popen(cmd, stdout=PIPE, stderr=PIPE)
        stdout, stderr = process.communicate()

        if process.returncode != 0:
            raise RuntimeError(f"Xacro processing failed: {stderr.decode()}")

        return stdout.decode()

    def spawn_robot(self, spawn_pause: float = 6.0) -> None:
        entity_count = int(self.get_parameter("entity_count").value)
        base_namespace = str(self.get_parameter("base_namespace").value)
        control_file = str(self.get_parameter("control_file").value)
        gui = str(self.get_parameter("gui").value)
        use_sim_time = str(self.get_parameter("use_sim_time").value)
        tfconfig = str(self.get_parameter("tfconfig").value)
        xacro_file = str(self.get_parameter("xacro_file").value)
        initial_x = float(self.config["x"])

        def run_command(cmd, namespace):
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
            )

            # Вывод в реальном времени
            while True:
                output = process.stdout.readline()
                if output == "" and process.poll() is not None:
                    break
                if output:
                    self.get_logger().info(f"[{namespace}] {output.strip()}")

            if process.returncode != 0:
                self.get_logger().error(
                    f"Command failed with return code {process.returncode}"
                )

        def launch_state(gui, use_sim_time, tfconfig, xacro_file, namespace):
            cmd = [
                "ros2",
                "launch",
                "description",
                "state.launch.py",
                f"gui:={gui}",
                f"use_sim_time:={use_sim_time}",
                f"tfconfig:={tfconfig}",
                f"xacro_file:={xacro_file}",
                f"robot_namespace:={namespace}",
            ]
            return run_command(cmd, f"{namespace}-state")

        def launch_control(control_file, use_sim_time, namespace):
            cmd = [
                "ros2",
                "launch",
                "control",
                "control.launch.py",
                f"control_file:={control_file}",
                f"use_sim_time:={use_sim_time}",
                f"robot_namespace:={namespace}",
            ]
            return run_command(cmd, f"{namespace}-control")

        ps = []
        for i in range(entity_count):
            namespace = f"{base_namespace}_{i}"

            try:
                robot_description = self.process_xacro(namespace, i)
            except Exception as e:
                self.get_logger().error(
                    f"Failed to process xacro for {namespace}: {str(e)}"
                )
                continue

            req = SpawnEntity.Request()
            req.name = namespace
            req.xml = robot_description
            req.robot_namespace = ""
            req.reference_frame = "map"
            req.initial_pose = Pose(
                position=Point(
                    x=initial_x + i * float(self.config["x_distance"]),
                    y=float(self.config["y"]),
                    z=float(self.config["z"]),
                ),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )

            self.get_logger().info(f"Spawning {namespace}...")

            self.call_service(self.spawn_service, req, self.spawn_service_name)

            p1 = multiprocessing.Process(
                target=launch_state,
                kwargs={
                    "gui": gui,
                    "use_sim_time": use_sim_time,
                    "tfconfig": tfconfig,
                    "xacro_file": xacro_file,
                    "namespace": namespace,
                },
            )

            p2 = multiprocessing.Process(
                target=launch_control,
                kwargs={
                    "control_file": control_file,
                    "use_sim_time": use_sim_time,
                    "namespace": namespace,
                },
            )

            time.sleep(spawn_pause)
            p1.start()
            time.sleep(spawn_pause)
            p2.start()
            time.sleep(spawn_pause)

            ps.append(p1)
            ps.append(p2)

        for p in ps:
            p.join()

            if p.exitcode != 0:
                print("Один из процессов завершился с ошибкой")

        return None

    def call_service(
        self, client, req, service_name: str, timeout_sec: float = 5.0
    ) -> bool:
        try:
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if future.done():
                future.result()
                self.get_logger().info(f"Successfully: {service_name}")
            else:
                self.get_logger().warning(
                    f"The {service_name} service did not respond, but the command could have been executed"
                )

        except Exception as e:
            self.get_logger().error(f"Error calling the service: {str(e)}")
            return False

        return True


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
