import numpy as np
import rclpy
from control.utils import get_joints
from interface.msg import AgrObs
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.agr_obs = self.create_subscription(
            AgrObs,
            "observation/aggregated_observation",
            self.observation_callback,
            qos_profile=qos,
        )
        self.stand_up_pub = self.create_publisher(
            AgrObs, "control/stand_up", qos_profile=qos
        )
        self.move_pub = self.create_publisher(AgrObs, "control/move", qos_profile=qos)
        self.joint_order = get_joints()
        self.MAX_ALLOWED_ANGLE = 0.6

    def observation_callback(self, msg: AgrObs):
        orientation = msg.imu.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # roll, pitch, _ = self.quaternion_to_euler(x, y, z, w)
        # stand_up_condition = abs(roll) > self.MAX_ALLOWED_ANGLE or abs(pitch) > self.MAX_ALLOWED_ANGLE
        stand_up_condition = True

        if stand_up_condition:
            self.stand_up_pub.publish(msg)
        else:
            self.move_pub.publish(msg)

    def quaternion_to_euler(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * (np.pi / 2)
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
