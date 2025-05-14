import rclpy
import torch
from control.utils import get_joints
from interface.msg import AgrObs
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .stand_up_nn import StandUpNN, Trainer


class StandUpNNNode(Node):
    def __init__(self):
        super().__init__("stand_up_nn_node")
        self.agr_obs = self.create_subscription(
            AgrObs, "/planner/stand_up", self.observation_callback, 10
        )
        self.pub = self.create_publisher(JointState, "angles_error", 10)
        self.obs_dim = 30
        self.joint_order = get_joints()
        self.trainer = Trainer()
        self.nn = StandUpNN(
            self.trainer.inv_kin.phi1_border,
            self.trainer.inv_kin.phi2_border,
            self.obs_dim,
            len(self.joint_order),
        )

    def observation_callback(self, msg: AgrObs):
        control_msg = JointState()

        control_msg.name = self.joint_order

        orientation = msg.imu.orientation
        orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        angular_velocity = msg.imu.angular_velocity
        angular_velocity = [angular_velocity.x, angular_velocity.y, angular_velocity.z]
        linear_acceleration = msg.imu.linear_acceleration
        linear_acceleration = [
            linear_acceleration.x,
            linear_acceleration.y,
            linear_acceleration.z,
        ]
        foots_forces = msg.foots.forces
        position = msg.joint_states.position
        velocity = msg.joint_states.velocity

        phi1 = [
            pos for name, pos in zip(self.joint_order, position) if "corpus" in name
        ]
        phi2 = [
            pos for name, pos in zip(self.joint_order, position) if "corpus" not in name
        ]
        self.trainer.inv_kin.update_phi(phi1, phi2)

        x = [
            *orientation,
            *angular_velocity,
            *linear_acceleration,
            *foots_forces,
            *position,
            *velocity,
        ]
        if len(x) == self.obs_dim:
            x = self.nn(torch.tensor(x).unsqueeze(0))
            x = x.squeeze().tolist()
        else:
            x = [0.0 for _ in range(len(self.joint_order))]

        corpus_joint = [name for name in self.joint_order if "corpus" in name]
        uncorpus_joint = [name for name in self.joint_order if "corpus" not in name]
        target_positions = {
            name: pos for pos, name in zip(x, [*corpus_joint, *uncorpus_joint])
        }

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
