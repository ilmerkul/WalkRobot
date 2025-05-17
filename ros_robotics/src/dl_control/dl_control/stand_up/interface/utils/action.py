from typing import Iterable

import torch
from control.utils import get_joints
from dl_control.stand_up.interface.entity import Action

JOINTS = get_joints()


def get_sum_batch_dim(actions: Iterable[Action]) -> int:
    return sum(len(action) for action in actions)


def concat_action(actions: Iterable[Action]) -> Action:
    batch_dim = get_sum_batch_dim(actions)
    position = {joint: torch.tensor([0.0] * batch_dim) for joint in JOINTS}
    log_prob = torch.tensor([0.0] * batch_dim)

    curr_index = 0
    dcts = [position]
    for action in actions:
        action_dcts = [
            action.position,
            action.log_prob,
        ]
        for i, dct in enumerate(dcts):
            for k, t in dct.items():
                t[curr_index : (curr_index + len(action))] = action_dcts[i][k]
        curr_index += len(action)

    log_prob = torch.concat([action.log_prob for action in actions], dim=0)

    return Action(
        position=position,
        log_prob=log_prob,
    )
