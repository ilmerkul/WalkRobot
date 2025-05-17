import os
import pathlib
from typing import Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F
from dl_control.interface.entity import State
from dl_control.interface.utils import load_config
from dl_control.stand_up.interface.entity import Action
from dl_control.stand_up.interface.model import StandUpNN

CONFIG_PATH = pathlib.Path(os.path.abspath(__file__)).parent / "config"
MODEL_CONFIG_PATH = CONFIG_PATH / "model.yaml"

ACTOR_PARAMETERS = load_config(MODEL_CONFIG_PATH.as_posix())["actor"]


class Actor(StandUpNN):
    def __init__(self, hidden_dim: int = ACTOR_PARAMETERS["hidden_dim"]):
        super(Actor, self).__init__()
        self.fc1 = nn.Linear(State.dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.mu = nn.Linear(hidden_dim, Action.dim)
        self.log_std = nn.Linear(hidden_dim, Action.dim)

    def nn_forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        mu = self.mu(x)
        log_std = self.log_std(x)
        log_std = torch.clamp(log_std, min=-20, max=2)
        return mu, log_std

    def sample_action(
        self, position_proportion: torch.Tensor, log_std: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        std = log_std.exp()
        normal = torch.distributions.Normal(position_proportion, std + 1e-7)
        z = normal.rsample()
        action = torch.tanh(z)
        log_prob = normal.log_prob(z) - torch.log(1 - action.pow(2) + 1e-6)
        log_prob = log_prob.sum(dim=-1, keepdim=True)

        return action, log_prob
