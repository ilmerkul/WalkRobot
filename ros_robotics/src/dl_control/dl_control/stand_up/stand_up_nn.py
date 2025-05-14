from typing import Dict, Tuple

import torch
import torch.nn as nn
from control import InverseKinematics

torch.pi = torch.acos(torch.zeros(1)).item() * 2


class Trainer:
    def __init__(self):
        self.inv_kin = InverseKinematics()

        self.target_height = self.inv_kin.get_x_z()[1][0]

    def reward(self, orientation: Dict[str, torch.Tensor]) -> torch.Tensor:
        reward = 0

        rpy = self.quaternion_to_euler(orientation)
        reward += torch.cos(rpy["roll"]) + torch.cos(rpy["pitch"])

        _, curr_height = self.inv_kin.get_x_z()
        min_height = min(curr_height)
        reward = -1 * (self.target_height - min_height) ** 2

        return reward

    def quaternion_to_euler(
        self, orientation: Dict[str, torch.Tensor]
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


class StandUpNN(nn.Module):
    def __init__(
        self,
        phi1_border: Tuple[float, float],
        phi2_border: Tuple[float, float],
        in_dim: int = 29,
        out_dim: int = 8,
    ):
        super(StandUpNN, self).__init__()

        self.out_dim = out_dim

        self.lin = nn.Linear(in_dim, out_dim)
        self.act = nn.Sigmoid()

        self.phi1_border = phi1_border
        self.phi2_border = phi2_border

    def forward(self, x):
        s = self.act(self.lin(x))
        k = self.out_dim // 2

        s[:, :k] = self.phi1_border[0] + s[:, :k] * (
            self.phi1_border[1] - self.phi1_border[0]
        )
        s[:, k:] = self.phi2_border[0] + s[:, k:] * (
            self.phi2_border[1] - self.phi2_border[0]
        )
        return s
