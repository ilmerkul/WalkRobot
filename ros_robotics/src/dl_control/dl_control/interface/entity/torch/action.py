from typing import Dict, List, Tuple

import torch
from description.utils import (
    JOINT_ORDER,
    get_joint_classes,
    parse_joint_class_angle_borders,
)

ACTION_DIM = len(JOINT_ORDER)


class ActionMove:
    dim: int = ACTION_DIM
    joint_order: List[str] = JOINT_ORDER
    joint_classes: List[str] = get_joint_classes()
    joint_class_angle_borders: Dict[str, Tuple[float, float]] = (
        parse_joint_class_angle_borders()
    )

    def __init__(
        self,
        position: Dict[str, torch.Tensor],
        log_prob: torch.Tensor,
    ):

        self.position = position
        self.log_prob = log_prob

        self._assert_batch_dim()

    @property
    def batch_dim(self):
        return self.position[self.joint_order[0]].shape[0]

    def _assert_batch_dim(self):
        for dct in [self.position]:
            for _, t in dct.items():
                assert t.shape[0] == self.batch_dim

        assert self.log_prob.shape[0] == self.batch_dim

    def __len__(self) -> int:
        return self.batch_dim

    def __getitem__(self, index: int):
        assert 0 <= index < self.batch_dim

        position = {k: v[index, None] for k, v in self.position.items()}
        log_prob = self.log_prob[index]
        return ActionMove(position=position, log_prob=log_prob)

    def _get_joint_class_positions(self) -> Dict[str, torch.Tensor]:
        joint_classes_position = dict()
        for joint_class in self.joint_classes:
            joint_class_order = list(
                filter(lambda x: joint_class in x, self.joint_order)
            )
            joint_classes_position[joint_class] = torch.concat(
                [self.position[joint] for i, joint in enumerate(joint_class_order)],
                dim=1,
            )

        return joint_classes_position

    def bordered(self):
        joint_class_positions = self._get_joint_class_positions()
        for joint_class in self.joint_classes:
            joint_class_positions[joint_class] = self.joint_class_angle_borders[
                joint_class
            ][0] + joint_class_positions[joint_class] * (
                self.joint_class_angle_borders[joint_class][1]
                - self.joint_class_angle_borders[joint_class][0]
            )

            joint_class_order = list(
                filter(lambda x: joint_class in x, self.joint_order)
            )
            for i, joint in enumerate(joint_class_order):
                self.position[joint] = joint_class_positions[joint_class][:, i, None]

    def to(self, device: torch.device):
        self.position = {k: v.to(device) for k, v in self.position.items()}
        self.log_prob = self.log_prob.to(device)
        return self

    @staticmethod
    def action_from_iter(action_iter: torch.Tensor, log_prob: torch.Tensor):
        return ActionMove(
            position={
                joint: action_iter[:, i, None]
                for i, joint in enumerate(ActionMove.joint_order)
            },
            log_prob=log_prob,
        )
