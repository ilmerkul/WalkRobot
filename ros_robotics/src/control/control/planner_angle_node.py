#!/usr/bin/env python3
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from utils import get_joints


class PlannerAngleNode(Node):
    def __init__(self):
        super().__init__('planner_angle_node')

        # Подписываемся на топик joint_states
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.last_state_message_callback, 10
        )

        # Публикуем команды для effort_controller
        self.publisher = self.create_publisher(JointState, 'angles_error', 10)

        self.timer = self.create_timer(1 / 30, self.timer_callback)
        self.last_state_message = None

        # Порядок суставов (должен совпадать с конфигом контроллера)
        self.joint_order = get_joints()

        self.get_logger().info('Node started. Converting Plan to angles...')

    def last_state_message_callback(self, msg):
        self.last_state_message = msg

    def timer_callback(self):
        if self.last_state_message is not None:
            self.process_message(self.last_state_message)
        else:
            self.get_logger().warn('last_state_message is None')

    def process_message(self, msg):
        command_msg = JointState()
        command_msg.name = self.joint_order

        position_error = [0.0 for _ in range(len(self.joint_order))]
        velocity = [0.0 for _ in range(len(self.joint_order))]

        for i, joint_name in enumerate(self.joint_order):
            try:
                index = msg.name.index(joint_name)
                current_pos = msg.position[index]
                current_vel = msg.velocity[index] if index < len(msg.velocity) else 0.0

                target_pos = current_pos + random.uniform(-0.05, 0.05)
                error = target_pos - current_pos

                position_error[i] = error
                velocity[i] = current_vel

            except ValueError:
                self.get_logger().warn(
                    f'Joint {joint_name} not found in JointState message!'
                )
                break

        command_msg.position = position_error
        command_msg.velocity = velocity

        self.publisher.publish(command_msg)
        self.get_logger().debug(
            f'Published error angles command: {command_msg.position}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlannerAngleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
