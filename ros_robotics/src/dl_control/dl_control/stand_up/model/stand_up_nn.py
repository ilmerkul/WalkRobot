from typing import Tuple

import torch.nn as nn


class StandUpNN(nn.Module):
    def __init__(
        self,
        phi1_border: Tuple[float, float],
        phi2_border: Tuple[float, float],
        in_dim: int = 29,
        out_dim: int = 8,
    ):
        super(StandUpNN, self).__init__()

        self.out_dim = out_dim

        self.lin = nn.Linear(in_dim, out_dim)
        self.act = nn.Sigmoid()

        self.phi1_border = phi1_border
        self.phi2_border = phi2_border

    def forward(self, x):
        s = self.act(self.lin(x))
        k = self.out_dim // 2

        s[:, :k] = self.phi1_border[0] + s[:, :k] * (
            self.phi1_border[1] - self.phi1_border[0]
        )
        s[:, k:] = self.phi2_border[0] + s[:, k:] * (
            self.phi2_border[1] - self.phi2_border[0]
        )
        return s
