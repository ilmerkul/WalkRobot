from abc import ABC, abstractmethod

import torch
from control.utils import get_joints
from dl_control.interface.entity import State
from dl_control.interface.model import ObsNN
from dl_control.stand_up.interface.entity import Action
from torch.nn import Module


class StandUpCritic(ABC, Module, ObsNN):
    def __init__(self):
        super(StandUpCritic, self).__init__()

        self.joint_order = get_joints()
        self.zero_res = torch.tensor([[0.0]])

    @abstractmethod
    def nn_forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        pass

    def prepare_action(self, action: Action) -> torch.Tensor:
        action = torch.concat(
            [action.position[joint] for joint in self.joint_order],
            dim=-1,
        )
        return action

    def forward(self, state: State, action: Action) -> torch.Tensor:
        state = self.prepare_state(state)
        action = self.prepare_action(action)
        if state.get_current_obs_dim() != State.dim:
            x = self.zero_res
        else:
            x = self.nn_forward(state=state, action=action)

        return x
