import torch
from dl_control.interface.entity.torch import ActionMove, State
from dl_control.interface.utils.torch import concat_action, concat_state

torch.pi = torch.acos(torch.zeros(1)).item() * 2
import os
import pathlib
import random
from abc import ABC, abstractmethod
from collections import deque
from typing import Dict, Tuple, Union

from dl_control.interface.utils import load_config
from torch.utils.tensorboard import SummaryWriter

TRAIN_PATH = pathlib.Path(os.path.abspath(__file__)).parent
EXP_PATH = TRAIN_PATH / "exp"
CONFIG_PATH = TRAIN_PATH / "config"
MODEL_CONFIG_PATH = CONFIG_PATH / "train.yaml"

TRAINER_PARAMETERS = load_config(MODEL_CONFIG_PATH.as_posix())["trainer"]


class Trainer(ABC):
    def __init__(
        self,
        batch_dim: int = TRAINER_PARAMETERS["batch_dim"],
        memory_len: int = TRAINER_PARAMETERS["memory_len"],
        period_roll: int = TRAINER_PARAMETERS["period_roll"],
        tensorboard: bool = TRAINER_PARAMETERS["tensorboard"],
        runs_path: Union[str, pathlib.Path] = EXP_PATH / "run",
    ):
        self.memory = deque(maxlen=memory_len)
        self.batch_dim = batch_dim
        self.batch_count = 0

        self.period_roll = period_roll

        self.continue_state_dim_count = 0
        self.continue_state_dim_count_max = 10

        if tensorboard:
            self.tb_writer = SummaryWriter(runs_path)

        self.epoch = 0
        self.iter = 0

    def set_height(self, height: float) -> None:
        self.target_height = height

    @abstractmethod
    def reward(
        self,
        state: State,
        height: torch.Tensor,
    ) -> Dict[str, torch.Tensor]:
        pass

    def _total_reward(self, reward: Dict[str, torch.Tensor]) -> torch.Tensor:
        return sum([v for v in reward.values()])

    @abstractmethod
    def done(self, state: State) -> torch.Tensor:
        pass

    def store_transition(
        self,
        state: State,
        action: ActionMove,
        height: torch.Tensor,
    ) -> str:
        self.continue_state_dim_count = 0

        reward = self.reward(state=state, height=height)
        metrics = {
            "scalar": {
                f"memory/reward/{reward_name}": torch.mean(reward_value).item()
                for reward_name, reward_value in reward.items()
            }
        }
        reward = self._total_reward(reward)
        done = self.done(state)

        batch_dim = reward.shape[0]
        for i in range(batch_dim - 1):
            self.memory.append(
                (state[i], action[i], state[i + 1], action[i + 1], reward[i], done[i])
            )

        self.add_tensorboard_metrics(metrics, self.iter)
        self.iter += 1

        self.batch_count += batch_dim
        if self.batch_count > self.batch_dim:
            self.batch_count = 0
            return "train"

        # if torch.any(done == 1):
        #    return "reset"

        return "continue"

    def continue_state_dim_count_update(self, count: int) -> str:
        self.continue_state_dim_count += count
        if self.continue_state_dim_count > self.continue_state_dim_count_max:
            self.continue_state_dim_count = 0
            return "reset"
        else:
            return "continue"

    def memory_sample(
        self,
    ) -> Tuple[State, ActionMove, torch.Tensor, State, torch.Tensor]:
        batch = random.sample(self.memory, self.batch_dim)
        state, action, next_state, next_action, reward, done = zip(*batch)
        state = concat_state(state)
        next_state = concat_state(next_state)
        action = concat_action(action)
        next_action = concat_action(next_action)
        reward = torch.tensor(reward)
        done = torch.tensor(done)
        return state, action, next_state, next_action, reward, done

    @abstractmethod
    def train_step(self) -> Dict[str, Dict[str, Union[float, torch.Tensor]]]:
        pass

    def train(self) -> Dict[str, Dict[str, Union[float, torch.Tensor]]]:
        if len(self.memory) < self.batch_dim:
            return len(self.memory)

        metrics = self.train_step()

        self.add_tensorboard_metrics(metrics, self.epoch)

        self.epoch += 1

        return metrics

    def add_tensorboard_metrics(
        self, metrics: Dict[str, Dict[str, Union[float, torch.Tensor]]], step: int
    ) -> None:
        for metric_cls in metrics.keys():
            for metric_name in metrics[metric_cls]:
                if metric_cls == "scalar" and isinstance(
                    metrics[metric_cls][metric_name], float
                ):
                    self.tb_writer.add_scalar(
                        metric_name, metrics[metric_cls][metric_name], step
                    )
                elif metric_cls == "histogram" and isinstance(
                    metrics[metric_cls][metric_name], torch.Tensor
                ):
                    self.tb_writer.add_histogram(
                        metric_name, metrics[metric_cls][metric_name], step
                    )
                else:
                    raise ValueError

        return None
