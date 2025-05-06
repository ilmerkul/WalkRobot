#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import WrenchStamped
from interface.msg import FootContacts
from rclpy.node import Node


class FootContactDetector(Node):
    def __init__(self):
        super().__init__('foot_contact_detector')

        self.prefixes = ['front_right', 'back_right', 'front_left', 'back_left']
        self.force_threshold = 5.0

        self.contacts = {prefix: False for prefix in self.prefixes}

        self._foot_subscriptions = [
            self.create_subscription(
                WrenchStamped,
                f'/sensors/force/{prefix}_foot',
                self.create_foot_callback(prefix),
                10,
            )
            for prefix in self.prefixes
        ]

        self.contacts_pub = self.create_publisher(
            FootContacts, 'force/foot_contact_state', 10
        )

        self.timer = self.create_timer(0.02, self.publish_contacts)

    def create_foot_callback(self, prefix):
        def callback(msg):
            force_z = msg.wrench.force.z
            self.contacts[prefix] = abs(force_z) > self.force_threshold

        return callback

    def publish_contacts(self):
        msg = FootContacts()
        msg.contacts = [self.contacts[prefix] for prefix in self.prefixes]
        msg.forces = [0.5 for prefix in self.prefixes]
        self.contacts_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FootContactDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
