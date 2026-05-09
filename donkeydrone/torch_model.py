"""
Modern PyTorch CNN for Autonomous Drone Flight.

Features:
- Residual connections for better gradient flow.
- Batch Normalization for training stability.
- Squeeze-and-Excitation (SE) for channel-wise attention.
- GELU activations for smoother non-linearity.
- Global Average Pooling to reduce parameter count.

Input:  (B, 3, H, W) float32 [0, 1]
Output: (B, 3) — [steering, throttle, altitude]
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


class SEBlock(nn.Module):
    """Squeeze-and-Excitation block for channel-wise attention."""
    def __init__(self, channels, reduction=16):
        super().__init__()
        self.avg_pool = nn.AdaptiveAvgPool2d(1)
        self.fc = nn.Sequential(
            nn.Linear(channels, channels // reduction, bias=False),
            nn.GELU(),
            nn.Linear(channels // reduction, channels, bias=False),
            nn.Sigmoid()
        )

    def forward(self, x):
        b, c, _, _ = x.size()
        y = self.avg_pool(x).view(b, c)
        y = self.fc(y).view(b, c, 1, 1)
        return x * y.expand_as(x)


class ResidualBlock(nn.Module):
    """Modern residual block with BatchNorm, GELU, and SE attention."""
    def __init__(self, in_channels, out_channels, stride=1):
        super().__init__()
        self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size=3, stride=stride, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(out_channels)
        self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size=3, stride=1, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(out_channels)
        self.se = SEBlock(out_channels)

        self.shortcut = nn.Sequential()
        if stride != 1 or in_channels != out_channels:
            self.shortcut = nn.Sequential(
                nn.Conv2d(in_channels, out_channels, kernel_size=1, stride=stride, bias=False),
                nn.BatchNorm2d(out_channels)
            )

    def forward(self, x):
        out = F.gelu(self.bn1(self.conv1(x)))
        out = self.bn2(self.conv2(out))
        out = self.se(out)
        out += self.shortcut(x)
        return F.gelu(out)


class LinearModel(nn.Module):
    def __init__(self, input_shape=(3, 120, 160), imu_shape=(3, 6)):
        """
        Args:
            input_shape: (C, H, W) image dimensions.
            imu_shape: (Sequence Length, Channels) e.g., (3, 6) for 3 steps of [accel, gyro].
        """
        super().__init__()
        
        # Initial feature extraction
        self.init_conv = nn.Sequential(
            nn.Conv2d(input_shape[0], 32, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(32),
            nn.GELU()
        )

        # Backbone: Stack of Residual blocks with stride-based downsampling
        self.layer1 = ResidualBlock(32, 64, stride=2)   # (H/4, W/4)
        self.layer2 = ResidualBlock(64, 128, stride=2)  # (H/8, W/8)
        self.layer3 = ResidualBlock(128, 256, stride=2) # (H/16, W/16)
        self.layer4 = ResidualBlock(256, 256, stride=1)

        # Global pooling makes the model resolution-independent and reduces params
        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))

        # IMU processing branch
        self.imu_fc = nn.Sequential(
            nn.Linear(imu_shape[0] * imu_shape[1], 32),
            nn.GELU(),
            nn.Linear(32, 32),
            nn.GELU()
        )

        # Prediction head: Vision (256) + IMU (32) = 288 input features
        self.fc = nn.Sequential(
            nn.Linear(256 + 32, 128),
            nn.GELU(),
            nn.Dropout(0.1),
            nn.Linear(128, 64),
            nn.GELU(),
            nn.Linear(64, 3)
        )

    def forward(self, img, imu):
        # Image branch
        x = self.init_conv(img)
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)
        x = self.avgpool(x)
        x = torch.flatten(x, 1)

        # IMU branch
        imu_x = torch.flatten(imu, 1)
        imu_x = self.imu_fc(imu_x)

        # Multi-modal fusion
        combined = torch.cat((x, imu_x), dim=1)
        return self.fc(combined)
