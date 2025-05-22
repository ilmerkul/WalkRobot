from typing import Dict, Iterable

import torch
from dl_control.interface.entity import State
from dl_control.stand_up.interface.entity import Action
from interface.msg import AgrObs


def agr_obs_to_state(msg: AgrObs) -> State:
    orientation = msg.imu.orientation
    angular_velocity = msg.imu.angular_velocity
    linear_acceleration = msg.imu.linear_acceleration
    foots_forces = msg.foots.forces
    position = msg.joint_states.position
    velocity = msg.joint_states.velocity

    if len(velocity) != len(position):
        velocity = [0.0 for _ in range(len(position))]

    return State(
        orientation={
            "x": torch.tensor([[orientation.x]]),
            "y": torch.tensor([[orientation.y]]),
            "z": torch.tensor([[orientation.z]]),
            "w": torch.tensor([[orientation.w]]),
        },
        angular_velocity={
            "x": torch.tensor([[angular_velocity.x]]),
            "y": torch.tensor([[angular_velocity.y]]),
            "z": torch.tensor([[angular_velocity.z]]),
        },
        linear_acceleration={
            "x": torch.tensor([[linear_acceleration.x]]),
            "y": torch.tensor([[linear_acceleration.y]]),
            "z": torch.tensor([[linear_acceleration.z]]),
        },
        foots_forces=torch.tensor([foots_forces]),
        position=torch.tensor([position]),
        velocity=torch.tensor([velocity]),
    )


def get_zero_state() -> State:
    return State(
        orientation={
            "x": torch.tensor([0.0]),
            "y": torch.tensor([0.0]),
            "z": torch.tensor([0.0]),
            "w": torch.tensor([0.0]),
        },
        angular_velocity={
            "x": torch.tensor([0.0]),
            "y": torch.tensor([0.0]),
            "z": torch.tensor([0.0]),
        },
        linear_acceleration={
            "x": torch.tensor([0.0]),
            "y": torch.tensor([0.0]),
            "z": torch.tensor([0.0]),
        },
        foots_forces=torch.tensor([[0.0] * (Action.dim // 2)]),
        position=torch.tensor([[0.0] * Action.dim]),
        velocity=torch.tensor([[0.0] * Action.dim]),
    )


def get_sum_batch_dim(states: Iterable[State]) -> int:
    return sum(len(state) for state in states)


def concat_state(states: Iterable[State]) -> State:
    batch_dim = get_sum_batch_dim(states)
    orientation = {
        "x": torch.tensor([0.0] * batch_dim),
        "y": torch.tensor([0.0] * batch_dim),
        "z": torch.tensor([0.0] * batch_dim),
        "w": torch.tensor([0.0] * batch_dim),
    }
    angular_velocity = {
        "x": torch.tensor([0.0] * batch_dim),
        "y": torch.tensor([0.0] * batch_dim),
        "z": torch.tensor([0.0] * batch_dim),
    }
    linear_acceleration = {
        "x": torch.tensor([0.0] * batch_dim),
        "y": torch.tensor([0.0] * batch_dim),
        "z": torch.tensor([0.0] * batch_dim),
    }
    foots_forces = torch.tensor([[0.0] * (Action.dim // 2)] * batch_dim)
    position = torch.tensor([[0.0] * Action.dim] * batch_dim)
    velocity = torch.tensor([[0.0] * Action.dim] * batch_dim)

    curr_index = 0
    dcts = [orientation, angular_velocity, linear_acceleration]
    for state in states:
        state_dcts = [
            state.orientation,
            state.angular_velocity,
            state.linear_acceleration,
        ]
        for i, dct in enumerate(dcts):
            for k, t in dct.items():
                t[curr_index : (curr_index + len(state))] = state_dcts[i][k]
        curr_index += len(state)

    foots_forces = torch.concat([state.foots_forces for state in states], dim=0)
    position = torch.concat([state.position for state in states], dim=0)
    velocity = torch.concat([state.velocity for state in states], dim=0)

    return State(
        orientation=orientation,
        angular_velocity=angular_velocity,
        linear_acceleration=linear_acceleration,
        foots_forces=foots_forces,
        position=position,
        velocity=velocity,
    )


def quaternion_to_euler(
    orientation: Dict[str, torch.Tensor],
) -> Dict[str, torch.Tensor]:
    sinr_cosp = 2 * (
        orientation["w"] * orientation["x"] + orientation["y"] * orientation["z"]
    )
    cosr_cosp = 1 - 2 * (
        orientation["x"] * orientation["x"] + orientation["y"] * orientation["y"]
    )
    roll = torch.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (
        orientation["w"] * orientation["y"] - orientation["z"] * orientation["x"]
    )
    if abs(sinp) >= 1:
        pitch = torch.sign(sinp) * (torch.pi / 2)
    else:
        pitch = torch.arcsin(sinp)

    siny_cosp = 2 * (
        orientation["w"] * orientation["z"] + orientation["x"] * orientation["y"]
    )
    cosy_cosp = 1 - 2 * (
        orientation["y"] * orientation["y"] + orientation["z"] * orientation["z"]
    )
    yaw = torch.arctan2(siny_cosp, cosy_cosp)

    return {"roll": roll, "pitch": pitch, "yaw": yaw}
