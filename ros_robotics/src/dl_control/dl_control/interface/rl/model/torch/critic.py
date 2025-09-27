from abc import abstractmethod

import torch
from description.utils import JOINT_ORDER
from dl_control.interface.entity.torch import ActionMove, State
from dl_control.interface.model.torch import ObsNN
from torch.nn import Module


class CriticMove(Module, ObsNN):
    def __init__(self):
        super(CriticMove, self).__init__()

        self.joint_order = JOINT_ORDER

    @abstractmethod
    def nn_forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        pass

    def prepare_action(self, action: ActionMove) -> torch.Tensor:
        action = torch.concat(
            [action.position[joint] for joint in self.joint_order],
            dim=-1,
        )
        return action

    def forward(self, state: State, action: ActionMove) -> torch.Tensor:
        state = self.prepare_state(state)
        action = self.prepare_action(action)

        x = self.nn_forward(state=state, action=action)

        return x
