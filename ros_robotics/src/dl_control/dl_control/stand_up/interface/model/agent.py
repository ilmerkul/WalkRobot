import torch
from control import InverseKinematics
from dl_control.interface.entity import State
from dl_control.stand_up.interface.entity import Action
from dl_control.stand_up.interface.train import Trainer

from .nn import StandUpNN


class StandUpAgent:
    def __init__(
        self,
        inv_kin: InverseKinematics,
        nn: StandUpNN,
        trainer: Trainer = None,
        train: bool = False,
    ):
        self.inv_kin = inv_kin
        self.nn: StandUpNN = nn
        self.trainer = None
        if train:
            self.trainer = trainer
        self.train_flag = train

    def select_action(self, state: State) -> Action:
        action = self.nn.sample(state)
        return action

    def train(self) -> None:
        if self.train_flag:
            self.trainer.train()
        return None

    def store_transition(self, state: State, action: Action) -> str:
        if self.train:
            height = torch.tensor([self.inv_kin.get_x_z()[1]])
            return self.trainer.store_transition(
                state=state,
                action=action,
                height=height,
            )
        return "continue"
