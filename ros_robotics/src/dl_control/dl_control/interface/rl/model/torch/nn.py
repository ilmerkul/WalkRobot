from abc import abstractmethod
from typing import Tuple

import torch
from dl_control.interface.entity.torch import ActionMove, State
from dl_control.interface.model.torch import ObsNN
from torch.nn import Module


class NNMove(Module, ObsNN):
    def __init__(self):
        super(NNMove, self).__init__()

        self.zero_action = torch.tensor([[0.0] * ActionMove.dim])
        self.zero_log_std = torch.tensor([[float("-1e2")] * ActionMove.dim])

    @abstractmethod
    def nn_forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        pass

    def forward(self, state: State) -> Tuple[torch.Tensor, torch.Tensor]:
        x = self.prepare_state(state)
        if state.get_current_obs_dim() != State.dim:
            return self.zero_action, self.zero_log_std

        return self.nn_forward(x)

    def bordered_action(
        self, action: torch.Tensor, log_prob: torch.Tensor
    ) -> ActionMove:
        action: ActionMove = ActionMove.action_from_iter(
            action_iter=action, log_prob=log_prob
        )
        action.bordered()

        return action

    @abstractmethod
    def sample_action(
        self, position_proportion: torch.Tensor, log_std: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        pass

    def sample(self, state: State) -> ActionMove:
        target_position_proportion, log_std = self.forward(state)

        action, log_prob = self.sample_action(
            position_proportion=target_position_proportion, log_std=log_std
        )

        action = self.bordered_action(action, log_prob)

        return action
