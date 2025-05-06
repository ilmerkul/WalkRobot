#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from utils import get_joints


class AnglesToEffort(Node):
    def __init__(self):
        super().__init__('angles_to_effort')

        self.subscription = self.create_subscription(
            JointState, '/planner/angles_error', self.listener_callback, 10
        )

        self.publisher = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10
        )

        self.timer = self.create_timer(1 / 100, self.timer_callback)
        self.last_effort_msg = None

        self.joint_order = get_joints()

        self.pid_gains = {
            'kp': 50.0,
            'kd': 5.0,
        }

        self.get_logger().info(
            'Node started. Converting JointState to Effort commands...'
        )

    def timer_callback(self):
        if self.last_effort_msg is not None:
            self.publisher.publish(self.last_effort_msg)
        else:
            self.get_logger().warn('last_effort_msg is None')

    def listener_callback(self, msg):
        command_msg = Float64MultiArray()

        for joint_name in self.joint_order:
            try:
                index = msg.name.index(joint_name)
                error = msg.position[index]
                current_vel = msg.velocity[index]

                effort = (
                    self.pid_gains['kp'] * error - self.pid_gains['kd'] * current_vel
                )

                command_msg.data.append(effort)

            except ValueError:
                self.get_logger().warn(
                    f'Joint {joint_name} not found in JointState message!'
                )
                command_msg.data.append(0.0)

        self.last_effort_msg = command_msg

        self.publisher.publish(command_msg)
        self.get_logger().debug(f'Published effort command: {command_msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = AnglesToEffort()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
