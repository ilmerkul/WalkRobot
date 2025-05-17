import torch
from dl_control.interface.entity import State
from dl_control.interface.utils import concat_state, quaternion_to_euler
from dl_control.stand_up.interface.entity import Action
from dl_control.stand_up.interface.utils import concat_action

torch.pi = torch.acos(torch.zeros(1)).item() * 2
import os
import pathlib
import random
from abc import ABC, abstractmethod
from collections import deque
from typing import Tuple

from dl_control.interface.utils import load_config

CONFIG_PATH = pathlib.Path(os.path.abspath(__file__)).parent / "config"
MODEL_CONFIG_PATH = CONFIG_PATH / "train.yaml"

TRAINER_PARAMETERS = load_config(MODEL_CONFIG_PATH.as_posix())["trainer"]


class Trainer(ABC):
    def __init__(
        self,
        batch_dim: int = TRAINER_PARAMETERS["batch_dim"],
        memory_len: int = TRAINER_PARAMETERS["memory_len"],
        period_roll: int = TRAINER_PARAMETERS["period_roll"],
        force_threshold: float = TRAINER_PARAMETERS["force_threshold"],
        w_orientation: float = TRAINER_PARAMETERS["w_orientation"],
        w_angular: float = TRAINER_PARAMETERS["w_angular"],
        w_linear: float = TRAINER_PARAMETERS["w_linear"],
        w_force_balance: float = TRAINER_PARAMETERS["w_force_balance"],
        w_joint_velocity: float = TRAINER_PARAMETERS["w_joint_velocity"],
        w_height: float = TRAINER_PARAMETERS["w_height"],
    ):
        self.target_height = 0.0
        self.target_angular_velocity = torch.tensor([0.0, 0.0, 0.0])
        self.target_velocity = torch.tensor([0.0] * Action.dim)
        self.target_linear_acceleration = torch.tensor([0.0, 0.0, 9.8])

        self.memory = deque(maxlen=memory_len)
        self.batch_dim = batch_dim
        self.batch_count = 0

        self.period_roll = period_roll

        self.force_threshold = force_threshold

        self.w_orientation = w_orientation
        self.w_angular = w_angular
        self.w_linear = w_linear
        self.w_force_balance = w_force_balance
        self.w_joint_velocity = w_joint_velocity
        self.w_height = w_height

    def set_height(self, height: float) -> None:
        self.target_height = height

    def reward(
        self,
        state: State,
        height: torch.Tensor,
    ) -> torch.Tensor:
        total_reward = 0

        rpy = quaternion_to_euler(state.orientation)
        orientation_reward = -0.5 * (rpy["roll"] ** 2 + rpy["pitch"] ** 2)

        max_height = torch.max(height, dim=-1).values
        height_reward = -1 * (self.target_height - max_height) ** 2

        xyz = ["x", "y", "z"]

        angular_velocity_error = 0
        for i, ax in enumerate(xyz):
            angular_velocity_error += torch.sum(
                (state.angular_velocity[ax] - self.target_angular_velocity[i]) ** 2,
                dim=-1,
            )
        angular_reward = torch.exp(-0.1 * angular_velocity_error)

        linear_acceleration_error = 0
        for i, ax in enumerate(xyz):
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

    def done(self, state: State) -> torch.Tensor:
        rpy = quaternion_to_euler(state.orientation)

        return rpy["roll"] >= torch.pi / 2 or rpy["pitch"] >= torch.pi / 2

    def store_transition(
        self,
        state: State,
        action: Action,
        height: torch.Tensor,
    ) -> str:
        if state.get_current_obs_dim() != State.dim:
            return "continue_state_dim"
        reward = self.reward(state=state, height=height)
        done = self.done(state)

        batch_dim = reward.shape[0]
        for i in range(batch_dim - 1):
            self.memory.append((state[i], action[i], reward[i], state[i + 1], done[i]))

        self.batch_count += batch_dim
        if self.batch_count > self.batch_dim:
            self.batch_count = 0
            return "train"

        if torch.any(done == 1):
            return "reset"

        return "continue"

    def memory_sample(self) -> Tuple[State, Action, torch.Tensor, State, torch.Tensor]:
        batch = random.sample(self.memory, self.batch_dim)
        state, action, reward, next_state, done = zip(*batch)
        state = concat_state(state)
        next_state = concat_state(next_state)
        action = concat_action(action)
        reward = torch.concat(reward, dim=0)
        done = torch.concat(done, dim=0)
        return state, action, reward, next_state, done

    @abstractmethod
    def train_step(self) -> None:
        pass

    def train(self) -> None:
        if len(self.memory) < self.batch_dim:
            return None

        self.train_step()
