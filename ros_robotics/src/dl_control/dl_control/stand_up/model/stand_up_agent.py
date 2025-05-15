import torch
from control import InverseKinematics
from control.utils import get_joints
from dl_control.interface import State
from dl_control.stand_up.train import Trainer

from .stand_up_nn import StandUpNN


class StandUpAgent:
    def __init__(self):
        self.joint_order = get_joints()

        self.inv_kin = InverseKinematics()
        self.nn = StandUpNN(
            self.inv_kin.phi1_border,
            self.inv_kin.phi2_border,
            State.obs_dim,
            len(self.joint_order),
        )
        self.trainer = Trainer(height=self.inv_kin.get_x_z()[1][0])

    def get_action(self, state: State) -> torch.Tensor:
        x = torch.concat(
            [
                state.orientation["x"],
                state.orientation["y"],
                state.orientation["z"],
                state.orientation["w"],
                state.angular_velocity["x"],
                state.angular_velocity["y"],
                state.angular_velocity["z"],
                state.linear_acceleration["x"],
                state.linear_acceleration["y"],
                state.linear_acceleration["z"],
                state.foots_forces,
                state.position,
                state.velocity,
            ],
            dim=-1,
        )

        if x.shape[-1] == State.obs_dim:
            action = self.nn(x)
        else:
            action = torch.zeros((x.shape[0], len(self.joint_order)))

        return action
