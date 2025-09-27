import os
import pathlib
from typing import Dict, Union

import torch
import torch.nn.functional as F
import torch.optim as optim
from dl_control.interface.entity.torch import ActionMove, State
from dl_control.interface.rl.train.torch import Trainer
from dl_control.interface.utils import load_config, random_string
from dl_control.interface.utils.torch import quaternion_to_euler

from .actor import Actor
from .critic import Critic

SAC_PATH = pathlib.Path(os.path.abspath(__file__)).parent
CONFIG_PATH = SAC_PATH / "config"
MODEL_CONFIG_PATH = CONFIG_PATH / "train.yaml"
EXP_PATH = SAC_PATH / "exp"

SAC_PARAMETERS = load_config(MODEL_CONFIG_PATH.as_posix())["sac"]
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


class SAC(Trainer):
    def __init__(
        self,
        actor: Actor,
        gamma: float = SAC_PARAMETERS["gamma"],
        tau: float = SAC_PARAMETERS["tau"],
        actor_lr: float = SAC_PARAMETERS["actor_lr"],
        critic_lr: float = SAC_PARAMETERS["critic_lr"],
        alpha_lr: float = SAC_PARAMETERS["alpha_lr"],
        device: torch.device = device,
        w_orientation: float = SAC_PARAMETERS["w_orientation"],
        w_angular: float = SAC_PARAMETERS["w_angular"],
        w_height: float = SAC_PARAMETERS["w_height"],
        w_effort: float = SAC_PARAMETERS["w_effort"],
        runs_path: Union[str, pathlib.Path] = EXP_PATH / f"run_{random_string()}",
        **kwargs,
    ):
        super(SAC, self).__init__(runs_path=runs_path, **kwargs)

        self.actor = actor.to(device)
        self.critic1 = Critic().to(device)
        self.critic2 = Critic().to(device)
        self.critic1_target = Critic().to(device)
        self.critic2_target = Critic().to(device)

        self.critic1_target.load_state_dict(self.critic1.state_dict())
        self.critic2_target.load_state_dict(self.critic2.state_dict())

        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=float(actor_lr))
        self.critic1_optimizer = optim.Adam(
            self.critic1.parameters(), lr=float(critic_lr)
        )
        self.critic2_optimizer = optim.Adam(
            self.critic2.parameters(), lr=float(critic_lr)
        )

        self.target_entropy = -torch.prod(
            torch.Tensor([ActionMove.dim]).to(device)
        ).item()
        self.log_alpha = torch.zeros(1, requires_grad=True, device=device)
        self.alpha_optimizer = optim.Adam([self.log_alpha], lr=float(alpha_lr))

        self.gamma = float(gamma)
        self.tau = float(tau)

        self.device = device

        self.target_height = 0.0

        self.w_orientation = w_orientation
        self.w_angular = w_angular
        self.w_height = w_height
        self.w_effort = w_effort

    def done(self, state: State) -> torch.Tensor:
        rpy = quaternion_to_euler(state.orientation)

        # return rpy["roll"] < torch.pi / 2 or rpy["pitch"] < torch.pi / 2
        return torch.zeros(rpy["roll"].shape[0])

    def reward(
        self,
        state: State,
        height: torch.Tensor,
    ) -> Dict[str, torch.Tensor]:
        rpy = quaternion_to_euler(state.orientation)
        orientation_reward = -0.5 * (rpy["roll"] ** 2 + rpy["pitch"] ** 2).squeeze(1)

        xyz = ["x", "y", "z"]

        effort_reward = -0.5 * torch.sum(state.effort**2, dim=-1)

        foots_forces_reward = 0.5 * torch.sum((state.foots_forces / 1e3) ** 2, dim=-1)

        total_reward = (
            self.w_orientation * orientation_reward
            + self.w_effort * effort_reward
            + foots_forces_reward
        )

        return {
            "orientation": self.w_orientation * orientation_reward,
            "effort": self.w_effort * effort_reward,
            "foots_forces": foots_forces_reward,
        }

    def train_step(self) -> Dict[str, Dict[str, Union[float, torch.Tensor]]]:
        state, action, next_state, next_action, reward, done = self.memory_sample()
        state = state.to(self.device)
        action = action.to(self.device)
        next_state = next_state.to(self.device)
        next_action = next_action.to(self.device)
        reward = reward.to(self.device)
        done = done.unsqueeze(1).to(self.device)

        with torch.no_grad():
            target_Q1 = self.critic1_target(next_state, next_action)
            target_Q2 = self.critic2_target(next_state, next_action)
            target_Q = (
                torch.min(target_Q1, target_Q2)
                - self.log_alpha.exp() * next_action.log_prob
            )
            target_Q = reward + (1 - done) * self.gamma * target_Q

        current_Q1 = self.critic1(state, action)
        current_Q2 = self.critic2(state, action)
        critic1_loss = F.mse_loss(current_Q1, target_Q)
        critic2_loss = F.mse_loss(current_Q2, target_Q)

        self.critic1_optimizer.zero_grad()
        self.critic2_optimizer.zero_grad()
        critic1_loss.backward(retain_graph=True)
        critic2_loss.backward()
        self.critic1_optimizer.step()
        self.critic2_optimizer.step()

        new_action = self.actor.sample(state)
        Q1 = self.critic1(state, new_action)
        Q2 = self.critic2(state, new_action)
        Q = torch.min(Q1, Q2)
        actor_loss = (self.log_alpha.exp() * new_action.log_prob - Q).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        alpha_loss = -(
            self.log_alpha * (new_action.log_prob + self.target_entropy).detach()
        ).mean()
        self.alpha_optimizer.zero_grad()
        alpha_loss.backward()
        self.alpha_optimizer.step()

        for param, target_param in zip(
            self.critic1.parameters(), self.critic1_target.parameters()
        ):
            target_param.data.copy_(
                self.tau * param.data + (1 - self.tau) * target_param.data
            )
        for param, target_param in zip(
            self.critic2.parameters(), self.critic2_target.parameters()
        ):
            target_param.data.copy_(
                self.tau * param.data + (1 - self.tau) * target_param.data
            )

        return {
            "scalar": {
                "stand_up/sac/train/critic1_loss": critic1_loss.item(),
                "stand_up/sac/train/critic2_loss": critic2_loss.item(),
                "stand_up/sac/train/actor_loss": actor_loss.item(),
                "stand_up/sac/train/alpha_loss": alpha_loss.item(),
                "stand_up/sac/train/reward": torch.mean(reward).item(),
            }
        }
