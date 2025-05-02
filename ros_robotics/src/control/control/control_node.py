#!/usr/bin/env python3
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointStateToPositionCommand(Node):
    def __init__(self):
        super().__init__("joint_state_to_position_command")

        # Подписываемся на топик joint_states (от joint_state_publisher_gui)
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.listener_callback, 10
        )

        # Публикуем команды для position_controller
        self.publisher = self.create_publisher(
            Float64MultiArray, "/position_controller/commands", 10
        )

        # Словарь для хранения порядка суставов из конфигурации контроллера
        self.joint_order = [
            "front_right_leg1_corpus_joint",
            "front_right_leg1_leg2_joint",
            "front_left_leg1_corpus_joint",
            "front_left_leg1_leg2_joint",
            "back_right_leg1_corpus_joint",
            "back_right_leg1_leg2_joint",
            "back_left_leg1_corpus_joint",
            "back_left_leg1_leg2_joint",
        ]

        self.get_logger().info(
            "Node started. Converting JointState to Position commands..."
        )

    def listener_callback(self, msg):
        # Создаем сообщение-команду
        command_msg = Float64MultiArray()

        # Заполняем данные в правильном порядке
        for joint_name in self.joint_order:
            try:
                index = msg.name.index(joint_name)
                command_msg.data.append(random.random() * 0.1 + msg.position[index])
            except ValueError:
                self.get_logger().warn(
                    f"Joint {joint_name} not found in JointState message!"
                )
                command_msg.data.append(0.0)  # Значение по умолчанию

        # Публикуем команду
        self.publisher.publish(command_msg)
        self.get_logger().debug(f"Published command: {command_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = JointStateToPositionCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
