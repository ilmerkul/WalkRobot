from typing import Dict

import torch
from dl_control.interface.entity import State


class ObsNN:
    def prepare_state(self, state: State):
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
            ],
            dim=-1,
        )

        return state

    def set_joint_position(
        self, position: Dict[str, torch.Tensor]
    ) -> Dict[str, torch.Tensor]:
        corpus_joint_position = {
            name: position["corpus"][..., i] for i, name in enumerate(self.corpus_joint)
        }
        uncorpus_joint_position = {
            name: position["uncorpus"][..., i]
            for i, name in enumerate(self.uncorpus_joint)
        }
        target_position = dict(
            [*corpus_joint_position.items(), *uncorpus_joint_position.items()]
        )
        return target_position
