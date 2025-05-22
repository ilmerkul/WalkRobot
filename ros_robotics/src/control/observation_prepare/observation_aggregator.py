import rclpy
from interface.msg import AgrObs, FootContacts
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState


class ObservationAggregator(Node):
    def __init__(self):
        super().__init__("obseravation_aggregator")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.imu_sub = self.create_subscription(
            Imu, "sensors/imu/filtered", self.imu_cb, qos_profile=qos
        )
        self.joint_sub = self.create_subscription(
            JointState, "observation/joint_states", self.joint_cb, qos_profile=qos
        )
        self.foot_sub = self.create_subscription(
            FootContacts,
            "sensors/force/foot_contact_state",
            self.foot_cb,
            qos_profile=qos,
        )
        self.pub = self.create_publisher(
            AgrObs, "observation/aggregated_observation", qos_profile=qos
        )

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


if __name__ == "__main__":
    main()
