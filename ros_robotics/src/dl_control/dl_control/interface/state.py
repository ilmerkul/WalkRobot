from typing import Dict, Tuple

import torch


class State:
    obs_dim: int = 30

    def __init__(
        self,
        orientation: Dict[str, torch.Tensor],
        angular_velocity: Dict[str, torch.Tensor],
        linear_acceleration: Dict[str, torch.Tensor],
        foots_forces: torch.Tensor,
        position: torch.Tensor,
        velocity: torch.Tensor,
    ):
        self.orientation = orientation
        self.angular_velocity = angular_velocity
        self.linear_acceleration = linear_acceleration
        self.foots_forces = foots_forces
        self.position = position
        self.velocity = velocity
