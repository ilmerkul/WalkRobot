import torch
from dl_control.interface.entity.torch import State

from ..nn import ObsNNInterface


class ObsNN(ObsNNInterface):
    def prepare_state(self, state: State) -> torch.Tensor:
        state = torch.concat(
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
                state.effort,
            ],
            dim=-1,
        )

        return state
