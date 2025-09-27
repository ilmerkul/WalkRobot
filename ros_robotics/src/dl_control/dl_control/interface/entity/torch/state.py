from typing import Dict

import torch
from description.utils import DIM_FOOTS, DIM_JOINT, DIM_XYZ, DIM_XYZW


class State:
    dim_xyz: int = DIM_XYZ
    dim_xyzw: int = DIM_XYZW
    dim_foots: int = DIM_FOOTS
    dim_joint: int = DIM_JOINT
    dim: int = DIM_XYZ * 2 + DIM_XYZW + DIM_FOOTS + DIM_JOINT * 3

    def __init__(
        self,
        orientation: Dict[str, torch.Tensor],
        angular_velocity: Dict[str, torch.Tensor],
        linear_acceleration: Dict[str, torch.Tensor],
        foots_forces: torch.Tensor,
        position: torch.Tensor,
        velocity: torch.Tensor,
        effort: torch.Tensor,
    ):
        self.orientation = orientation
        self.angular_velocity = angular_velocity
        self.linear_acceleration = linear_acceleration
        self.foots_forces = foots_forces
        self.position = position
        self.velocity = velocity
        self.effort = effort

        self._assert_batch_dim()

    @property
    def batch_dim(self):
        return self.foots_forces.shape[0]

    def _assert_batch_dim(self):
        for dct in [self.orientation, self.angular_velocity, self.linear_acceleration]:
            for t in dct.values():
                assert t.shape[0] == self.batch_dim

        assert self.foots_forces.shape[0] == self.batch_dim
        assert self.position.shape[0] == self.batch_dim
        assert self.velocity.shape[0] == self.batch_dim
        assert self.effort.shape[0] == self.batch_dim

    def get_current_obs_dim(self):
        obs_dim = 0
        for dct in [self.orientation, self.angular_velocity, self.linear_acceleration]:
            for t in dct.values():
                obs_dim += t.shape[1]

        obs_dim += self.foots_forces.shape[1]
        obs_dim += self.position.shape[1]
        obs_dim += self.velocity.shape[1]
        obs_dim += self.effort.shape[1]
        return obs_dim

    def __len__(self) -> int:
        return self.batch_dim

    def __getitem__(self, index: int):
        assert 0 <= index < self.batch_dim

        orientation = {k: v[index, None] for k, v in self.orientation.items()}
        angular_velocity = {k: v[index, None] for k, v in self.angular_velocity.items()}
        linear_acceleration = {
            k: v[index, None] for k, v in self.linear_acceleration.items()
        }
        foots_forces = self.foots_forces[index, None, ...]
        position = self.position[index, None, ...]
        velocity = self.velocity[index, None, ...]
        effort = self.effort[index, None, ...]
        return State(
            orientation=orientation,
            angular_velocity=angular_velocity,
            linear_acceleration=linear_acceleration,
            foots_forces=foots_forces,
            position=position,
            velocity=velocity,
            effort=effort,
        )

    def __repr__(self):
        return (
            "State{"
            + f"orientation: {self.orientation}, angular_velocity: {self.angular_velocity}, linear_acceleration: {self.linear_acceleration}, foots_forces: {self.foots_forces}, position: {self.position}, velocity: {self.velocity}, effort: {self.effort}"
            + "}"
        )

    def to(self, device: torch.device):
        orientation = {k: t.to(device) for k, t in self.orientation.items()}
        angular_velocity = {k: t.to(device) for k, t in self.angular_velocity.items()}
        linear_acceleration = {
            k: t.to(device) for k, t in self.linear_acceleration.items()
        }

        return State(
            orientation=orientation,
            angular_velocity=angular_velocity,
            linear_acceleration=linear_acceleration,
            foots_forces=self.foots_forces.to(device),
            position=self.position.to(device),
            velocity=self.velocity.to(device),
            effort=self.effort.to(device),
        )
