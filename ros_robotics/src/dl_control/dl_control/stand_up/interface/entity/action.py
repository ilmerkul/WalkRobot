from typing import Dict

import torch
from control.utils import get_joints

ACTION_DIM = len(get_joints())


class Action:
    dim: int = ACTION_DIM

    def __init__(
        self,
        position: Dict[str, torch.Tensor],
        log_prob: torch.Tensor,
    ):

        self.position = position
        self.log_prob = log_prob

    @property
    def batch_dim(self):
        return self.position.shape[0]

    def _assert_batch_dim(self):
        for dct in [self.position]:
            for k, t in dct.items():
                assert t.shape[0] == self.batch_dim

        assert self.log_prob.shape[0] == self.batch_dim

    def __len__(self) -> int:
        return self.batch_dim

    def __getitem__(self, index: int):
        assert 0 <= index < self.batch_dim

        position = {k: v[index] for k, v in self.position.items()}
        log_prob = log_prob[index]
        return Action(position=position, log_prob=log_prob)
