import torch.nn as nn
import torch
import gym
import numpy as np
from torch.distributions import Normal
import matplotlib.pyplot as plt
import random
import os

ENV_NAME = "BipedalWalker-v3"
environments_count = 20
SEED = 42
ROLLOUT_DIM = 30
GAMMA = 0.9


def seed_everything(seed: int = 42):
    random.seed(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False


def create_environment(env_name: str, seed: int, human: bool = False):
    if seed == SEED and human:
        env = gym.make(env_name, render_mode="human", hardcore=False)
    else:
        env = gym.make(env_name, hardcore=False)
    env.action_space.seed(seed)
    return env


seed_everything(SEED)
envs = [create_environment(ENV_NAME, SEED + seed_c) for seed_c in
        range(environments_count)]


class MemoryLinear(nn.Module):
    def __init__(self, emb_dim: int, batch_dim: int):
        super(MemoryLinear, self).__init__()

        self.act = nn.Sigmoid()
        self.memory_matrix = self.act(
            torch.normal(0, 1, (batch_dim, emb_dim, emb_dim)))
        self.alpha = 0.25

    def forward(self, x: torch.Tensor, beta: torch.Tensor):
        x = torch.einsum('bd,bdd->bd', x, self.memory_matrix)

        self.memory_matrix = self.act(self.memory_matrix - (
                self.alpha * beta.unsqueeze(1) * x.detach()).unsqueeze(-1))

        return x


class MemoryModule(nn.Module):
    def __init__(self, memory_count: int, emb_dim: int, batch_dim: int):
        super(MemoryModule, self).__init__()

        self.memory = [MemoryLinear(emb_dim, batch_dim) for _ in
                       range(memory_count)]
        self.softmax = nn.Softmax()

        self.centroid = torch.concatenate([self._get_centroid(memory).unsqueeze(0) for memory in self.memory], dim=0)

    def _get_centroid(self, memory_linear: MemoryLinear):
        return torch.mean(memory_linear.memory_matrix, dim=-1)

    def forward(self, x: torch.Tensor):
        beta = self.softmax(torch.einsum('cbd,bd->bc', self.centroid, x.detach()))

        x = torch.concatenate([memory(x, beta[:, i]).unsqueeze(0) for i, memory in enumerate(self.memory)], dim=0)

        x = torch.einsum('bc,cbd->bd', beta, x)

        self.centroid = torch.concatenate([self._get_centroid(memory).unsqueeze(0) for memory in self.memory], dim=0)

        return x, beta


class WorldModel(nn.Module):
    def __init__(self, observation_dim: int, world_dim: int):
        super(WorldModel, self).__init__()

        self.act = nn.Tanh()
        self.hidden_dim0 = observation_dim - 4

        self.lin0 = nn.Linear(observation_dim, self.hidden_dim0, bias=True)
        self.lin1 = nn.Linear(self.hidden_dim0, world_dim, bias=True)

    def forward(self, observation: torch.Tensor):
        # observation [batch_dim, observation_dim]

        x = self.lin0(observation)
        x = self.act(x)
        x = self.lin1(x)
        x = self.act(x)

        # x [batch_dim, world_dim]
        return x


class ActorWalker(nn.Module):
    def __init__(self, action_dim: int, observation_dim: int, batch_dim: int):
        super(ActorWalker, self).__init__()

        self.act = nn.Tanh()
        self.hidden_dim0 = observation_dim // 2
        self.hidden_dim1 = observation_dim // 4

        self.lin0 = nn.Linear(observation_dim, observation_dim, bias=True)
        self.lin1 = nn.Linear(observation_dim, observation_dim, bias=True)
        self.lin2 = nn.Linear(observation_dim, self.hidden_dim1, bias=True)
        self.lin3 = nn.Linear(self.hidden_dim1, action_dim, bias=True)

        self.memory = MemoryModule(3, self.hidden_dim1, batch_dim)

        self.log_std = nn.Parameter(torch.zeros(action_dim))
        self.w = nn.Parameter(torch.zeros(action_dim))
        self.prev_output = torch.zeros((1, action_dim))
        self.timestamp = 0

    def forward(self, observation: torch.Tensor, explore: bool = True):
        # observation [batch_dim, observation_dim]

        batch_dim = observation.shape[0]

        if self.prev_output.shape[0] != batch_dim:
            self.prev_output = torch.zeros((batch_dim, action_dim))

        x = self.lin0(observation)
        x = self.act(x)
        x = self.lin1(x)
        x = self.act(x)
        world_emb_predict = x
        x = self.lin2(x)
        x = self.act(x)
        x, beta = self.memory(x)
        x = self.lin3(x)
        x = self.act(x)

        # x_feedback = self.prev_output * torch.sin(self.w * self.timestamp)
        # norm = torch.linalg.norm(x.detach(), dim=-1) * torch.linalg.norm(x_feedback.detach(), dim=-1)
        # d = torch.einsum('bt,bt->b', x, x_feedback) / norm
        # d = d.unsqueeze(-1)
        # print(torch.linalg.norm(x, dim=-1), d.shape)
        # x = (1 - d) * x + d * x_feedback
        # self.timestamp += 1
        # self.prev_output = x.detach()

        std = torch.exp(self.log_std)
        dist = Normal(x, std)
        action = dist.sample() if explore else x
        log_prob = dist.log_prob(action).sum(-1)

        # action [batch_dim, action_dim]
        return action, log_prob, world_emb_predict, beta


class CriticWalker(nn.Module):
    def __init__(self, observation_dim: int, ):
        super(CriticWalker, self).__init__()

        self.act = nn.ReLU()
        self.hidden_dim0 = observation_dim
        self.hidden_dim1 = observation_dim // 2
        self.hidden_dim2 = observation_dim // 4

        self.lin0 = nn.Linear(2 * observation_dim, self.hidden_dim0, bias=True)
        self.lin1 = nn.Linear(self.hidden_dim0, self.hidden_dim1, bias=True)
        self.lin2 = nn.Linear(self.hidden_dim1, self.hidden_dim2, bias=True)
        self.lin3 = nn.Linear(self.hidden_dim2, 1, bias=True)

    def forward(self, observation: torch.Tensor):
        # observation [batch_dim, observation_dim]

        x = self.lin0(observation)
        x = self.act(x)
        x = self.lin1(x)
        x = self.act(x)
        x = self.lin2(x)
        x = self.act(x)
        x = self.lin3(x)

        # x [batch_dim]
        return x.squeeze(-1)


class WalkerModel(nn.Module):
    def __init__(self, action_dim: int, observation_dim: int, batch_dim: int):
        super().__init__()

        self.world_model = WorldModel(observation_dim, observation_dim - 4)
        self.actor = ActorWalker(action_dim, observation_dim - 4, batch_dim)
        self.critic = CriticWalker(observation_dim - 4)

    def forward(self, observation: torch.Tensor, explore: bool = True):
        # observation [batch_dim, observation_dim]

        world_emb = self.world_model(observation)
        actor_output = self.actor(world_emb, explore=explore)
        critic_output = self.critic(
            torch.concatenate([world_emb, actor_output[2].detach()], dim=-1))

        # (actor_output, critic_output)
        # ([batch_dim, action_dim], [batch_dim])
        return actor_output, critic_output, world_emb


action_dim = envs[0].action_space.shape[0]
observation_dim = envs[0].observation_space.shape[0]

model = WalkerModel(action_dim=action_dim,
                    observation_dim=observation_dim,
                    batch_dim=environments_count)
optimizer_actor = torch.optim.Adam(model.parameters(), lr=3e-4)

history = {
    "reward": [],
    "world_model_weight": [],
    "loss_actor": [],
    "loss_critic": [],
    "loss_world_model": []
}

gamma = torch.fill(torch.empty((environments_count, ROLLOUT_DIM)), GAMMA)
gamma[:, 0] = 1
gamma = torch.cumprod(gamma, dim=-1)

envs_flag = [True for _ in range(environments_count)]

plt.ion()
fig, ((ax_reward, ax_world_model_weight, ax_loss_world_model),
      (ax_loss_actor, ax_loss_critic, _)) = plt.subplots(2, 3)
ax_reward.set_xlabel("Step")
ax_reward.set_ylabel("Reward")
ax_reward.set_title("Reward")

ax_world_model_weight.set_xlabel("Step")
ax_world_model_weight.set_ylabel("world_model_weight")
ax_world_model_weight.set_title("world_model_weight")

ax_loss_actor.set_xlabel("Step")
ax_loss_actor.set_ylabel("loss_actor")
ax_loss_actor.set_title("loss_actor")

ax_loss_critic.set_xlabel("Step")
ax_loss_critic.set_ylabel("loss_critic")
ax_loss_critic.set_title("loss_critic")

ax_loss_world_model.set_xlabel("Step")
ax_loss_world_model.set_ylabel("loss_world_model")
ax_loss_world_model.set_title("loss_world_model")

line_reward, = ax_reward.plot(history["reward"])
line_world_model_weight, = ax_world_model_weight.plot(
    history["world_model_weight"])
line_loss_actor, = ax_loss_actor.plot(history["loss_actor"])
line_loss_critic, = ax_loss_critic.plot(history["loss_critic"])
line_loss_world_model, = ax_loss_world_model.plot(history["loss_world_model"])
data_x = []

model.train()
for k in range(100000):
    V_batch = torch.zeros((environments_count, ROLLOUT_DIM))
    reward_batch = torch.zeros((environments_count, ROLLOUT_DIM))
    log_prob_batch = torch.zeros((environments_count, ROLLOUT_DIM))
    world_emb_batch = torch.zeros(
        (environments_count, ROLLOUT_DIM, observation_dim - 4))
    world_emb_predict_batch = torch.zeros(
        (environments_count, ROLLOUT_DIM, observation_dim - 4))
    beta_batch = torch.zeros((environments_count, ROLLOUT_DIM, 3))

    observation_batch_reset = []
    for i, env in enumerate(envs):
        if envs_flag[i]:
            observation, info = env.reset()
            observation_batch_reset.append(observation)
        else:
            observation_batch_reset.append(observation_batch[i, :].numpy())

    observation_batch = torch.from_numpy(np.array(observation_batch_reset))

    envs_flag = [False for _ in range(environments_count)]
    for i in range(ROLLOUT_DIM):
        (action, log_prob, world_emb_predict, beta), V, world_emb = model(
            observation_batch)
        action = action.detach().numpy()
        V_batch[:, i] += V
        log_prob_batch[:, i] += log_prob
        world_emb_batch[:, i, :] += world_emb
        world_emb_predict_batch[:, i, :] += world_emb_predict

        beta_batch[:, i, :] += beta.detach()

        observation_batch = []
        for j, env in enumerate(envs):
            observation, reward, terminated, truncated, info = env.step(
                action[j])
            reward_batch[j, i] += reward

            if terminated or truncated:
                envs_flag[j] = True

            observation_batch.append(observation)
        observation_batch = torch.from_numpy(np.array(observation_batch))

    # V_batch [batch_dim, ROLLOUT_DIM]
    # reward_batch [batch_dim, ROLLOUT_DIM]
    # estimated_Q [batch_dim, ROLLOUT_DIM]
    estimated_Q = torch.zeros((environments_count, ROLLOUT_DIM))

    intrinsic_reward = torch.zeros((environments_count, ROLLOUT_DIM))
    for i in range(ROLLOUT_DIM - 1):
        # norm = torch.linalg.norm(
        #    world_emb_predict_batch[:, i, :].detach()) * torch.linalg.norm(
        #    world_emb_batch[:, i + 1, :].detach())
        # intrinsic_reward[:, i + 1] += -1 * torch.einsum('ba,ba->b',
        #                                                world_emb_predict_batch[:, i,
        #                                                :],
        #                                                world_emb_batch[:,
        #                                                i + 1, :]) / norm
        intrinsic_reward[:, i + 1] += torch.mean((world_emb_predict_batch[:, i,
                                                  :] - world_emb_batch[:,
                                                       i + 1,
                                                       :].detach()) ** 2,
                                                 dim=-1)

    reward_mean = torch.mean(reward_batch).item()
    if k != 0:
        reward_batch += 10 * torch.where(intrinsic_reward.detach() > 0.02,
                                          0.02, intrinsic_reward.detach())
    reward_batch -= torch.mean((beta_batch - torch.fill(torch.empty(environments_count, ROLLOUT_DIM, 3), 1/3)) ** 2, dim=-1)

    for i in range(ROLLOUT_DIM - 1):
        estim = torch.einsum('bt,bt->b',
                             gamma[:, :(ROLLOUT_DIM - i - 1)],
                             reward_batch[:, i:(ROLLOUT_DIM - 1)])
        estim += gamma[:, (ROLLOUT_DIM - i - 1)] * V_batch[:, i].detach()
        estimated_Q[:, i] += estim

    loss_critic = torch.mean((estimated_Q - V_batch) ** 2) / 10

    loss_actor = -1 * torch.mean(log_prob_batch * (
            estimated_Q - V_batch.detach()))

    loss = loss_actor + loss_critic

    if k != 0:
        loss += torch.mean(intrinsic_reward)

    optimizer_actor.zero_grad()
    loss.backward()
    torch.nn.utils.clip_grad_norm_(model.parameters(), 10)
    optimizer_actor.step()

    world_model_weight = torch.mean(model.world_model.lin1.weight).item()
    loss_actor = loss_actor.detach().item()
    loss_critic = loss_critic.detach().item()
    loss_world_model = torch.mean(intrinsic_reward).detach().item()
    if k != 0:
        reward_mean = 0.8 * history["reward"][-1] + (1 - 0.8) * reward_mean
        world_model_weight = 0.8 * history["world_model_weight"][-1] + (
                1 - 0.8) * world_model_weight
        loss_actor = 0.8 * history["loss_actor"][-1] + (1 - 0.8) * loss_actor
        loss_critic = 0.8 * history["loss_critic"][-1] + (
                1 - 0.8) * loss_critic
        loss_world_model = 0.8 * history["loss_world_model"][-1] + (
                1 - 0.8) * loss_world_model

    history["reward"].append(reward_mean)
    history["world_model_weight"].append(world_model_weight)
    history["loss_actor"].append(loss_actor)
    history["loss_critic"].append(loss_critic)
    history["loss_world_model"].append(loss_world_model)
    data_x.append(k)

    for (line, ax, name) in [(line_reward, ax_reward, "reward"),
                             (line_world_model_weight, ax_world_model_weight,
                              "world_model_weight"),
                             (line_loss_actor, ax_loss_actor, "loss_actor"),
                             (line_loss_critic, ax_loss_critic, "loss_critic"),
                             (line_loss_world_model, ax_loss_world_model,
                              "loss_world_model")]:
        line.set_ydata(history[name])
        line.set_xdata(data_x)
        ax.relim()
        ax.autoscale_view()

    fig.canvas.flush_events()

plt.ioff()
plt.show()

for env in envs:
    env.close()
