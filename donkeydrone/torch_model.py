"""
PyTorch CNN matching the DonkeyCar KerasLinear architecture.

5 Conv2d layers (valid padding, ReLU, Dropout 0.2) → Flatten → Dense(100) → Dense(50) → Linear(2)
Input:  (B, 3, H, W) float32 [0, 1]
Output: (B, 2) — [steering, throttle]
"""

import torch
import torch.nn as nn


class LinearModel(nn.Module):
    def __init__(self, input_shape=(3, 120, 160)):
        """
        Args:
            input_shape: (C, H, W) — channels-first, matching PyTorch convention.
        """
        super().__init__()
        self.convs = nn.Sequential(
            nn.Conv2d(input_shape[0], 24, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Conv2d(24, 32, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Conv2d(32, 64, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Conv2d(64, 64, kernel_size=3, stride=2),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.Dropout(0.2),
        )
        # Compute flatten size dynamically
        with torch.no_grad():
            dummy = torch.zeros(1, *input_shape)
            flat_size = self.convs(dummy).view(1, -1).shape[1]

        self.fc = nn.Sequential(
            nn.Linear(flat_size, 100),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(100, 50),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(50, 2),
        )

    def forward(self, x):
        x = self.convs(x)
        x = x.view(x.size(0), -1)
        return self.fc(x)
