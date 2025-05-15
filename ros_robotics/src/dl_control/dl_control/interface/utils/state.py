from typing import Dict

import torch
from dl_control.interface import State
from interface.msg import AgrObs


def agr_obs_to_state(msg: AgrObs) -> State:
    orientation = msg.imu.orientation
    angular_velocity = msg.imu.angular_velocity
    linear_acceleration = msg.imu.linear_acceleration
    foots_forces = msg.foots.forces
    position = msg.joint_states.position
    velocity = msg.joint_states.velocity

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
