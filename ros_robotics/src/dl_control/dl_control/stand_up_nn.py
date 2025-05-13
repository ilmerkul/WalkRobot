import torch
import torch.nn as nn


class StandUpNN(nn.Module):
    def __init__(self, in_dim: int = 29, out_dim: int = 8):
        super(StandUpNN, self).__init__()

        self.lin = nn.Linear(in_dim, out_dim)
        self.act = nn.Sigmoid()

    def forward(self, x):
        return self.act(self.lin(x))
