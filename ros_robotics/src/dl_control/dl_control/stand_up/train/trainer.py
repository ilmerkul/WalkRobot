import torch
from dl_control.interface import State
from dl_control.interface.utils import quaternion_to_euler

torch.pi = torch.acos(torch.zeros(1)).item() * 2


class Trainer:
    def __init__(self, height: float):
        self.target_height = height
        self.target_angular_velocity = torch.tensor([0.0, 0.0, 0.0])
        self.target_velocity = torch.tensor([0.0, 0.0, 0.0])
        self.target_linear_acceleration = torch.tensor([0.0, 0.0, 9.8])

        self.force_threshold = 5.0

        self.w_orientation = 0.3
        self.w_angular = 0.3
        self.w_linear = 0.2
        self.w_force_balance = 0.1
        self.w_joint_velocity = 0.1
        self.w_height = 0.3

    def reward(
        self,
        state: State,
        height: torch.Tensor,
    ) -> torch.Tensor:
        total_reward = 0

        rpy = quaternion_to_euler(state.orientation)
        orientation_reward = -0.5 * (
            torch.sum(rpy["roll"] ** 2, dim=-1) + torch.sum(rpy["pitch"] ** 2, dim=-1)
        )

        max_height = torch.max(height, dim=-1)
        height_reward = -1 * (self.target_height - max_height) ** 2

        angular_velocity_error = 0
        for i, ax in enumerate(["x", "y", "z"]):
            angular_velocity_error += torch.sum(
                (state.angular_velocity[ax] - self.target_angular_velocity[i]) ** 2,
                dim=-1,
            )
        angular_reward = torch.exp(-0.1 * angular_velocity_error)

        linear_acceleration_error = 0
        for i, ax in enumerate(["x", "y", "z"]):
            linear_acceleration_error += torch.sum(
                (state.linear_acceleration[ax] - self.target_linear_acceleration[i])
                ** 2,
                dim=-1,
            )
        linear_reward = torch.exp(-0.1 * linear_acceleration_error)

        mean_force = torch.mean(state.foots_forces, dim=-1)
        force_diff = torch.sum((state.foots_forces - mean_force) ** 2, dim=-1)
        force_balance_reward = torch.exp(-0.01 * force_diff)
        force_balance_reward = torch.where(
            torch.sum(state.foots_forces > self.force_threshold, dim=-1) >= 2,
            force_balance_reward,
            0.0,
        )

        joint_velocity_reward = torch.exp(
            -0.1 * torch.sum((state.velocity - self.target_velocity) ** 2, dim=-1)
        )

        total_reward = (
            self.w_orientation * orientation_reward
            + self.w_angular * angular_reward
            + self.w_linear * linear_reward
            + self.w_force_balance * force_balance_reward
            + self.w_joint_velocity * joint_velocity_reward
            + self.w_height * height_reward
        )

        return total_reward
