from typing import Dict, List, Tuple

import rclpy
from dl_control.interface.entity.torch import ActionMove, State
from dl_control.interface.utils.torch import (
    agr_obs_to_state,
    concat_action,
    concat_state,
)
from dl_control.stand_up.SAC.torch import Agent
from interface.msg import AgrObs, ControlSimMessage
from interface.srv import ControlSim
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class StandUpNode(Node):
    def __init__(self):
        super().__init__("stand_up_node")

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
                (
                    "min_states_size",
                    64,
                    ParameterDescriptor(
                        description="min_states_size",
                        type=ParameterType.PARAMETER_INTEGER,
                    ),
                ),
            ],
        )

        self.agr_obs = self.create_subscription(
            AgrObs,
            "/control/stand_up",
            self.observation_callback,
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

        self.control_sim_service_name = "/control_sim/command"

        self.control_sim_client = self.create_client(
            ControlSim, self.control_sim_service_name
        )

        self._check_service(self.control_sim_client, self.control_sim_service_name)

        self.joint_order = ActionMove.joint_order
        self.agent = Agent(train=bool(self.get_parameter("train").value))

        self.pubs = dict()
        self.states: Dict[str, List[State]] = dict()
        self.actions: Dict[str, List[ActionMove]] = dict()
        self.min_states_size = int(self.get_parameter("min_states_size").value)

    def _create_state(self, namespace: str) -> None:
        self.states[namespace] = []
        return None

    def _create_action(self, namespace: str) -> None:
        self.actions[namespace] = []
        return None

    def _create_pub(self, namespace: str) -> None:
        pub = self.create_publisher(
            JointState,
            f"{namespace}/control/angles_error",
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        self.pubs[namespace] = pub
        return None

    def _check_service(self, client, name, timeout: float = 2.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"The {name} service is unavailable!")
            return False

        return True

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

                    target_pos = target_position[joint_name][j][0]
                    error = target_pos - current_pos

                    position_error[j][i] = error
                    velocity[j][i] = current_vel

                except ValueError:
                    self.get_logger().warn(
                        f"Joint {joint_name} not found in JointState message!"
                    )
                    break

        return position_error, velocity

    def observation_callback(self, msg: AgrObs) -> None:
        state: State = agr_obs_to_state(msg)
        action: ActionMove = self.agent.select_action(state)

        if msg.name not in self.actions.keys():
            self._create_action(msg.name)

        if msg.name not in self.states.keys():
            self._create_state(msg.name)

        self.states[msg.name].append(state)
        self.actions[msg.name].append(action)
        if len(self.states[msg.name]) >= self.min_states_size:
            states = concat_state(self.states[msg.name])
            self.states[msg.name] = [self.states[msg.name][-1]]

            actions = concat_action(self.actions[msg.name])
            self.actions[msg.name] = [self.actions[msg.name][-1]]

            mode = self.agent.store_transition(state=states, action=actions)

            if True and mode == "train":
                metrics = self.agent.train()
                self.get_logger().info(f"{metrics}")
            elif False and mode == "reset":
                if self.call_control_service(
                    control_message=ControlSimMessage.PAUSE_MSG, entity_name=msg.name
                ):
                    self.get_logger().info("Reset")
                    self.call_control_service(
                        control_message=ControlSimMessage.RESET_MSG,
                        entity_name=msg.name,
                    )
                    self.call_control_service(
                        control_message=ControlSimMessage.UNPAUSE_MSG,
                        entity_name=msg.name,
                    )

        target_position = {
            joint: pos.tolist() for joint, pos in action.position.items()
        }

        position_error, velocity = self.get_pos_err_vel([msg], target_position)

        control_msg = JointState()
        control_msg.name = self.joint_order

        control_msg.position = position_error[0]
        control_msg.velocity = velocity[0]

        if msg.name not in self.pubs.keys():
            self._create_pub(msg.name)

        self.pubs[msg.name].publish(control_msg)

        return None

    def call_control_service(
        self, control_message: str, entity_name: str, timeout_sec: float = 30.0
    ) -> bool:
        try:
            request = ControlSim.Request()
            request.entity_name = entity_name
            request.control_message.type = control_message

            future = self.control_sim_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if future.done():
                service_response = future.result()
                if service_response is not None:
                    return bool(service_response.success)
                else:
                    raise RuntimeError("Service call returned None response")
            else:
                raise RuntimeError("Service call timed out")

        except Exception as e:
            self.get_logger().error(f"Call service error: {e}", throttle_duration_sec=5)
            return False


def main(args=None):
    rclpy.init(args=args)
    node = StandUpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
