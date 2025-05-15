from typing import Dict, List, Tuple

import rclpy
from control.utils import get_joints
from dl_control.interface import State
from dl_control.interface.utils import agr_obs_to_state
from dl_control.stand_up.model import StandUpAgent
from interface.msg import AgrObs
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StandUpNNNode(Node):
    def __init__(self):
        super().__init__("stand_up_nn_node")
        self.agr_obs = self.create_subscription(
            AgrObs, "/planner/stand_up", self.observation_callback, 10
        )
        self.pub = self.create_publisher(JointState, "angles_error", 10)
        self.joint_order = get_joints()
        self.agent = StandUpAgent()

    def get_pos_err_vel(
        self, msg: AgrObs, target_positions: Dict[str, float]
    ) -> Tuple[List[float], List[float]]:
        position_error = [0.0 for _ in range(len(self.joint_order))]
        velocity = [0.0 for _ in range(len(self.joint_order))]

        for i, joint_name in enumerate(self.joint_order):
            try:
                index = msg.joint_states.name.index(joint_name)
                current_pos = msg.joint_states.position[index]
                current_vel = (
                    msg.joint_states.velocity[index]
                    if index < len(msg.joint_states.velocity)
                    else 0.0
                )

                target_pos = target_positions[joint_name]
                error = target_pos - current_pos

                position_error[i] = error
                velocity[i] = current_vel

            except ValueError:
                self.get_logger().warn(
                    f"Joint {joint_name} not found in JointState message!"
                )
                break

        return position_error, velocity

    def observation_callback(self, msg: AgrObs):
        state: State = agr_obs_to_state(msg)

        action = self.agent.get_action(state).squeeze().tolist()

        corpus_joint = [name for name in self.joint_order if "corpus" in name]
        uncorpus_joint = [name for name in self.joint_order if "corpus" not in name]
        target_positions = {
            name: pos for pos, name in zip(action, [*corpus_joint, *uncorpus_joint])
        }

        position_error, velocity = self.get_pos_err_vel(msg, target_positions)

        control_msg = JointState()
        control_msg.name = self.joint_order

        control_msg.position = position_error
        control_msg.velocity = velocity

        self.pub.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StandUpNNNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
