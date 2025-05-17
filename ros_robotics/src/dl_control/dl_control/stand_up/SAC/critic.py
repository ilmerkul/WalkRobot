import os
import pathlib

import torch
import torch.nn as nn
import torch.nn.functional as F
from dl_control.interface.entity import State
from dl_control.interface.utils import load_config
from dl_control.stand_up.interface.entity import Action
from dl_control.stand_up.interface.model import StandUpCritic

CONFIG_PATH = pathlib.Path(os.path.abspath(__file__)).parent / "config"
MODEL_CONFIG_PATH = CONFIG_PATH / "model.yaml"

CRITIC_PARAMETERS = load_config(MODEL_CONFIG_PATH.as_posix())["critic"]


class Critic(StandUpCritic):
    def __init__(self, hidden_dim: int = CRITIC_PARAMETERS["hidden_dim"]):
        super(Critic, self).__init__()
        self.fc1 = nn.Linear(State.dim + Action.dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, 1)

    def nn_forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        x = torch.cat([state, action], dim=1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))

        return self.fc3(x)
