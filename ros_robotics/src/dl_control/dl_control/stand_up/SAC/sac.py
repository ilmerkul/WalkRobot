import os
import pathlib

import torch
import torch.nn.functional as F
import torch.optim as optim
from dl_control.interface.utils import load_config
from dl_control.stand_up.interface.entity import Action
from dl_control.stand_up.interface.train import Trainer

from .actor import Actor
from .critic import Critic

CONFIG_PATH = pathlib.Path(os.path.abspath(__file__)).parent / "config"
MODEL_CONFIG_PATH = CONFIG_PATH / "train.yaml"

SAC_PARAMETERS = load_config(MODEL_CONFIG_PATH.as_posix())["sac"]


class SAC(Trainer):
    def __init__(
        self,
        actor: Actor,
        gamma: float = SAC_PARAMETERS["gamma"],
        tau: float = SAC_PARAMETERS["tau"],
        actor_lr: float = SAC_PARAMETERS["actor_lr"],
        critic_lr: float = SAC_PARAMETERS["critic_lr"],
        alpha_lr: float = SAC_PARAMETERS["alpha_lr"],
        device: torch.device = torch.device("cpu"),
        **kwargs,
    ):
        super(SAC, self).__init__(**kwargs)

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

        self.target_entropy = -torch.prod(torch.Tensor([Action.dim]).to(device)).item()
        self.log_alpha = torch.zeros(1, requires_grad=True, device=device)
        self.alpha_optimizer = optim.Adam([self.log_alpha], lr=float(alpha_lr))

        self.gamma = float(gamma)
        self.tau = float(tau)

        self.device = device

    def train_step(self):
        state, action, reward, next_state, done = self.memory_sample()
        state = state.to(self.device)
        action = action.to(self.device)
        reward = reward.unsqueeze(1).to(self.device)
        next_state = next_state.to(self.device)
        done = done.unsqueeze(1).to(self.device)

        with torch.no_grad():
            next_action, next_log_prob = self.actor.sample(next_state)
            target_Q1 = self.critic1_target(next_state, next_action)
            target_Q2 = self.critic2_target(next_state, next_action)
            target_Q = (
                torch.min(target_Q1, target_Q2) - self.log_alpha.exp() * next_log_prob
            )
            target_Q = reward + (1 - done) * self.gamma * target_Q

        current_Q1 = self.critic1(state, action)
        current_Q2 = self.critic2(state, action)
        critic1_loss = F.mse_loss(current_Q1, target_Q)
        critic2_loss = F.mse_loss(current_Q2, target_Q)

        self.critic1_optimizer.zero_grad()
        critic1_loss.backward()
        self.critic1_optimizer.step()

        self.critic2_optimizer.zero_grad()
        critic2_loss.backward()
        self.critic2_optimizer.step()

        new_action, log_prob = self.actor.sample(state)
        Q1 = self.critic1(state, new_action)
        Q2 = self.critic2(state, new_action)
        Q = torch.min(Q1, Q2)
        actor_loss = (self.log_alpha.exp() * log_prob - Q).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        alpha_loss = -(
            self.log_alpha * (log_prob + self.target_entropy).detach()
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
