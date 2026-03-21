"""
TorchPilot — drop-in replacement for KerasLinear in the DonkeyCar vehicle loop.

Implements the same run(img_arr) → (steering, throttle, altitude) interface.
"""

import numpy as np
import torch

from torch_model import LinearModel


class TorchPilot:
    def __init__(self, input_shape=(3, 120, 160)):
        """
        Args:
            input_shape: (C, H, W) channels-first, matching LinearModel convention.
        """
        self.input_shape = input_shape
        if torch.backends.mps.is_available():
            self.device = torch.device('mps')
        elif torch.cuda.is_available():
            self.device = torch.device('cuda')
        else:
            self.device = torch.device('cpu')

        self.model = LinearModel(input_shape=input_shape).to(self.device)
        self.model.eval()

    def load(self, model_path):
        state_dict = torch.load(model_path, map_location=self.device, weights_only=True)
        self.model.load_state_dict(state_dict)
        self.model.eval()
        print(f"TorchPilot: loaded {model_path} on {self.device}")

    def run(self, img_arr):
        if img_arr is None:
            return 0.0, 0.0, 0.0

        # img_arr: (H, W, C) uint8 numpy array from camera
        arr = np.asarray(img_arr, dtype=np.float32) / 255.0
        # HWC → CHW, add batch dim
        tensor = torch.from_numpy(arr.transpose(2, 0, 1)).unsqueeze(0).to(self.device)

        with torch.no_grad():
            output = self.model(tensor)

        steering = float(output[0, 0])
        throttle = float(output[0, 1])
        altitude = float(output[0, 2])
        return steering, throttle, altitude
