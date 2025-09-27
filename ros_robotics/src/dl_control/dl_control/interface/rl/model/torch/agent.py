from typing import Dict, Union

import torch
from control import InverseKinematics
from dl_control.interface.entity.torch import ActionMove, State
from dl_control.interface.rl.train.torch import Trainer

from .nn import NNMove


class AgentMove:
    def __init__(
        self,
        inv_kin: InverseKinematics,
        nn: NNMove,
        trainer: Trainer = None,
        train: bool = False,
    ):
        self.inv_kin = inv_kin
        self.nn: NNMove = nn
        self.trainer = None
        if train:
            self.trainer = trainer
        self.train_flag = train

    def select_action(self, state: State) -> ActionMove:
        with torch.no_grad():
            action = self.nn.sample(state)
        return action

    def train(self) -> Dict[str, Dict[str, Union[float, torch.Tensor]]]:
        if self.train_flag:
            metrics = self.trainer.train()
        return metrics

    def store_transition(self, state: State, action: ActionMove) -> str:
        if self.train_flag:
            height = torch.tensor([self.inv_kin.get_x_z()[1]])
            return self.trainer.store_transition(
                state=state,
                action=action,
                height=height,
            )
        return "continue"

    def continue_state_dim_count_update(self, count: bool) -> str:
        return self.trainer.continue_state_dim_count_update(count)
