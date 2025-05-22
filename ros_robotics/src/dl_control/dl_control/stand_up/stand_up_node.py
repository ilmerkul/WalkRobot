import time
from typing import Callable, Dict, List, Tuple

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
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty


class StandUpNNNode(Node):
    def __init__(self):
        super().__init__("stand_up_nn_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "train",
                    True,
                    ParameterDescriptor(
                        description="train",
                        type=ParameterType.PARAMETER_BOOL,
                    ),
                ),
            ],
        )

        self.agr_obs = self.create_subscription(
            AgrObs,
            "stand_up",
            self.observation_callback,
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        self.pub = self.create_publisher(
            JointState,
            "angles_error",
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

        self.pause_service_name = "/pause_physics"
        self.unpause_service_name = "/unpause_physics"
        self.delete_service_name = "/delete_entity"
        self.reset_world_service_name = "/reset_world"

        self.spawn_state_service_name = "/spawn/spawn_entity/get_state"
        self.spawn_change_state_service_name = "/spawn/spawn_entity/change_state"

        self.pause_client = self.create_client(Empty, self.pause_service_name)
        self.unpause_client = self.create_client(Empty, self.unpause_service_name)
        self.delete_client = self.create_client(DeleteEntity, self.delete_service_name)
        self.spawn_state_client = self.create_client(
            GetState, self.spawn_state_service_name
        )
        self.spawn_change_state_client = self.create_client(
            ChangeState, self.spawn_change_state_service_name
        )
        self.reset_world_client = self.create_client(
            Empty, self.reset_world_service_name
        )

        for client, name in zip(
            [
                self.pause_client,
                self.unpause_client,
                self.delete_client,
                self.spawn_state_client,
                self.spawn_change_state_client,
                self.reset_world_client,
            ],
            [
                self.pause_service_name,
                self.unpause_service_name,
                self.delete_service_name,
                self.spawn_state_service_name,
                self.spawn_change_state_service_name,
                self.reset_world_service_name,
            ],
        ):
            self._check_service(client, name)

        self.joint_order = get_joints()
        self.agent = Agent(train=bool(self.get_parameter("train").value))

    def _check_service(self, client, name, timeout=2.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"The {name} service is unavailable!")

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

        if state.get_current_obs_dim() == State.dim:
            mode = self.agent.store_transition(state=state, action=action)
        else:
            mode = self.agent.continue_state_dim_count_update(1)

        # self.get_logger().info(f"Mode simulation: {mode}")
        if mode == "train":
            self.get_logger().info("Train")
            self.pause_simulation(self.agent.train)
            pass
        elif mode == "reset":
            self.get_logger().info("Spawn")
            # self.pause_simulation(self.spawn_entity)
            self.pause_simulation(self.reset_world)
            pass

        target_position = {
            joint: pos.tolist() for joint, pos in action.position.items()
        }

        position_error, velocity = self.get_pos_err_vel([msg], target_position)

        control_msg = JointState()
        control_msg.name = self.joint_order

        control_msg.position = position_error[0]
        control_msg.velocity = velocity[0]

        self.pub.publish(control_msg)

    def reset_world(self) -> None:
        req = Empty.Request()
        self.get_logger().info("Reset simulation")
        self.call_service(
            client=self.reset_world_client,
            req=req,
            service_name=self.reset_world_service_name,
        )

    def spawn_entity(self, entity_name: str = "tropy_spot_0") -> None:
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
        time.sleep(5)

        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        self.get_logger().info(f"Deactiavte spawn entity {entity_name}")
        self.call_service(
            client=self.spawn_change_state_client,
            req=req,
            service_name=self.spawn_change_state_service_name,
        )

        return None

    def pause_simulation(self, func: Callable) -> None:
        req = Empty.Request()
        self.get_logger().info("Pause simulation")
        self.call_service(
            client=self.pause_client, req=req, service_name=self.pause_service_name
        )

        func()

        self.get_logger().info("Unpause simulation")
        self.call_service(
            client=self.unpause_client,
            req=req,
            service_name=self.unpause_service_name,
        )

        return None

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
