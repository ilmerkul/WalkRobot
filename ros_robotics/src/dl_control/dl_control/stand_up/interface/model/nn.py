from abc import ABC, abstractmethod
from typing import Tuple

import torch
from control.utils import get_joints
from description.utils import parse_phi_border
from dl_control.interface.entity import State
from dl_control.interface.model import ObsNN
from dl_control.stand_up.interface.entity import Action
from torch.nn import Module


class StandUpNN(ABC, Module, ObsNN):
    def __init__(self):
        super(StandUpNN, self).__init__()

        self.phi1_border, self.phi2_border = parse_phi_border()

        self.joint_order = get_joints()
        self.corpus_joint = [joint for joint in self.joint_order if "corpus" in joint]
        self.uncorpus_joint = [
            joint for joint in self.joint_order if "corpus" not in joint
        ]

        self.zero_action = torch.tensor([[0.0] * Action.dim])
        self.zero_log_std = torch.tensor([[float("-1e2")] * Action.dim])

    @abstractmethod
    def nn_forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        pass

    def forward(self, state: State) -> Tuple[torch.Tensor, torch.Tensor]:
        x = self.prepare_state(state)
        if state.get_current_obs_dim() != State.dim:
            return self.zero_action, self.zero_log_std

        return self.nn_forward(x)

    def prepare_action(self, action: torch.Tensor, log_prob: torch.Tensor) -> Action:
        k = Action.dim // 2

        position = {"corpus": None, "uncorpus": None}
        position["corpus"] = self.phi1_border[0] + action[:, :k] * (
            self.phi1_border[1] - self.phi1_border[0]
        )
        position["uncorpus"] = self.phi2_border[0] + action[:, k:] * (
            self.phi2_border[1] - self.phi2_border[0]
        )
        position = self.set_joint_position(position)

        action = Action(position=position, log_prob=log_prob)

        return action

    @abstractmethod
    def sample_action(
        self, position_proportion: torch.Tensor, log_std: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        pass

    def sample(self, state: State) -> Action:
        target_position_proportion, log_std = self.forward(state)

        action, log_prob = self.sample_action(
            position_proportion=target_position_proportion, log_std=log_std
        )

        action = self.prepare_action(action, log_prob)

        return action
