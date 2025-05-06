import rclpy
from interface.msg import AgrObs, FootContacts
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


class ObservationAggregator(Node):
    def __init__(self):
        super().__init__('obseravation_aggregator')
        self.imu_sub = self.create_subscription(
            Imu, '/sensors/imu/filtered', self.imu_cb, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10
        )
        self.foot_sub = self.create_subscription(
            FootContacts, '/sensors/force/foot_contact_state', self.foot_cb, 10
        )
        self.pub = self.create_publisher(AgrObs, 'aggregated_observation', 10)

        self.last_imu = None
        self.last_joint = None
        self.last_foot = None

    def imu_cb(self, msg):
        self.last_imu = msg
        self.publish_aggregated()

    def joint_cb(self, msg):
        self.last_joint = msg
        self.publish_aggregated()

    def foot_cb(self, msg):
        self.last_foot = msg
        self.publish_aggregated()

    def publish_aggregated(self):
        if None in (self.last_imu, self.last_joint, self.last_foot):
            return

        msg = AgrObs()
        msg.imu = self.last_imu
        msg.joint_states = self.last_joint
        msg.foots = self.last_foot
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObservationAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
