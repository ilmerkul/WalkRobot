import torch
from control import InverseKinematics
from dl_control.stand_up.interface.model import StandUpAgent

from .actor import Actor
from .sac import SAC

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


class Agent(StandUpAgent):
    def __init__(self, train: bool = False, **kwargs):
        inv_kin = InverseKinematics()
        actor = Actor().to(device)
        trainer = SAC(actor=actor, device=device, **kwargs)

        super(Agent, self).__init__(
            inv_kin=inv_kin, nn=actor, trainer=trainer, train=train
        )
