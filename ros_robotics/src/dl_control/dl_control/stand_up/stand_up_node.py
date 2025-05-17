from typing import Dict, List, Tuple

import rclpy
from control.utils import get_joints
from dl_control.interface.entity import State
from dl_control.interface.utils import agr_obs_to_state
from dl_control.stand_up.interface.entity import Action
from dl_control.stand_up.SAC import Agent
from gazebo_msgs.srv import DeleteEntity
from interface.msg import AgrObs
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty


class StandUpNNNode(Node):
    def __init__(self):
        super().__init__("stand_up_nn_node")
        self.agr_obs = self.create_subscription(
            AgrObs, "/planner/stand_up", self.observation_callback, 10
        )
        self.pub = self.create_publisher(JointState, "angles_error", 10)

        self.pause_service_name = "/pause_physics"
        self.unpause_service_name = "/unpause_physics"
        self.delete_service_name = "/delete_entity"

        self.spawn_state_service_name = "/spawn/spawn_entity_wrapper/get_state"
        self.spawn_change_state_service_name = (
            "/spawn/spawn_entity_wrapper/change_state"
        )

        self.pause_client = self.create_client(Empty, self.pause_service_name)
        if not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f"The {self.pause_service_name} service is unavailable!"
            )
        self.unpause_client = self.create_client(Empty, self.unpause_service_name)
        if not self.unpause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f"The {self.unpause_service_name} service is unavailable!"
            )
        self.delete_client = self.create_client(DeleteEntity, self.delete_service_name)
        if not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f"The {self.delete_service_name} service is unavailable!"
            )
        self.spawn_state_client = self.create_client(
            GetState, self.spawn_state_service_name
        )
        if not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f"The {self.spawn_state_service_name} service is unavailable!"
            )
        self.spawn_change_state_client = self.create_client(
            ChangeState, self.spawn_change_state_service_name
        )
        if not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f"The {self.spawn_change_state_service_name} service is unavailable!"
            )

        self.joint_order = get_joints()
        self.agent = Agent(train=True)

    def get_pos_err_vel(
        self, msg: List[AgrObs], target_position: Dict[str, List[float]]
    ) -> Tuple[List[List[float]], List[List[float]]]:
        batch_dim = len(target_position[self.joint_order[0]])
        position_error = [
            [0.0 for _ in range(len(self.joint_order))] for _ in range(batch_dim)
        ]
        velocity = [
            [0.0 for _ in range(len(self.joint_order))] for _ in range(batch_dim)
        ]

        for j in range(batch_dim):
            for i, joint_name in enumerate(self.joint_order):
                try:
                    index = msg[j].joint_states.name.index(joint_name)
                    current_pos = msg[j].joint_states.position[index]
                    current_vel = (
                        msg[j].joint_states.velocity[index]
                        if index < len(msg[j].joint_states.velocity)
                        else 0.0
                    )

                    target_pos = target_position[joint_name][j]
                    error = target_pos - current_pos

                    position_error[j][i] = error
                    velocity[j][i] = current_vel

                except ValueError:
                    self.get_logger().warn(
                        f"Joint {joint_name} not found in JointState message!"
                    )
                    break

        return position_error, velocity

    def observation_callback(self, msg: AgrObs):
        state: State = agr_obs_to_state(msg)
        action: Action = self.agent.select_action(state)

        mode = self.agent.store_transition(state=state, action=action)
        self.get_logger().info(f"Mode simulation: {mode}")
        if mode == "train":
            req = Empty.Request()
            self.get_logger().info("Pause simulation")
            self.call_service(
                client=self.pause_client, req=req, service_name=self.pause_service_name
            )
            self.get_logger().info("Train")
            self.agent.train()
            self.get_logger().info("Unpause simulation")
            self.call_service(
                client=self.unpause_client,
                req=req,
                service_name=self.unpause_service_name,
            )
        elif mode == "reset":
            entity_name = "tropy_spot_0"
            req = DeleteEntity.Request()
            req.name = entity_name
            self.get_logger().info(f"Delete entity {entity_name}")
            self.call_service(
                client=self.delete_client,
                req=req,
                service_name=self.delete_service_name,
            )

            req = GetState.Request()
            future = self.spawn_state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            self.get_logger().info(f"Successfully: {future.result()}")

            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVATE
            self.get_logger().info(f"Activate spawn entity {entity_name}")
            self.call_service(
                client=self.spawn_change_state_client,
                req=req,
                service_name=self.spawn_change_state_service_name,
            )

            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_DEACTIVATE
            self.get_logger().info(f"Deactiavte spawn entity {entity_name}")
            self.call_service(
                client=self.spawn_change_state_client,
                req=req,
                service_name=self.spawn_change_state_service_name,
            )

        target_position = {
            joint: pos.tolist() for joint, pos in action.position.items()
        }

        position_error, velocity = self.get_pos_err_vel([msg], target_position)

        control_msg = JointState()
        control_msg.name = self.joint_order

        control_msg.position = position_error[0]
        control_msg.velocity = velocity[0]

        self.pub.publish(control_msg)

    def call_service(
        self, client, req, service_name: str, timeout_sec: float = 5.0
    ) -> bool:
        try:
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if future.done():
                future.result()
                self.get_logger().info(f"Successfully: {service_name}")
                return True
            else:
                self.get_logger().warning(
                    f"The {service_name} service did not respond, but the command could have been executed"
                )
                return True

        except Exception as e:
            self.get_logger().error(f"Error calling the service: {str(e)}")
            return False


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
