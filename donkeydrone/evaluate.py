#!/usr/bin/env python3
"""Evaluate DonkeyDrone PyTorch models against recorded tubs."""

import argparse
import os

import numpy as np
import torch
from torch.utils.data import DataLoader
from torchvision import transforms

from dataset import TubDataset
from torch_model import LinearModel, ResidualBlock


AXES = ("Steering", "Throttle", "Altitude")


class LegacyImuFcLinearModel(torch.nn.Module):
    """LinearModel variant used by checkpoints saved before the IMU GRU change."""

    def __init__(self, input_shape=(3, 240, 320), imu_shape=(3, 6)):
        super().__init__()
        self.init_conv = torch.nn.Sequential(
            torch.nn.Conv2d(
                input_shape[0], 32, kernel_size=3, stride=2, padding=1, bias=False
            ),
            torch.nn.BatchNorm2d(32),
            torch.nn.GELU(),
        )
        self.layer1 = ResidualBlock(32, 64, stride=2)
        self.layer2 = ResidualBlock(64, 128, stride=2)
        self.layer3 = ResidualBlock(128, 256, stride=2)
        self.layer4 = ResidualBlock(256, 256, stride=1)
        self.avgpool = torch.nn.AdaptiveAvgPool2d((1, 1))
        self.imu_fc = torch.nn.Sequential(
            torch.nn.Linear(imu_shape[0] * imu_shape[1], 32),
            torch.nn.GELU(),
            torch.nn.Linear(32, 32),
            torch.nn.GELU(),
        )
        self.fc = torch.nn.Sequential(
            torch.nn.Linear(256 + 32, 128),
            torch.nn.GELU(),
            torch.nn.Dropout(0.1),
            torch.nn.Linear(128, 64),
            torch.nn.GELU(),
            torch.nn.Linear(64, 3),
        )

    def forward(self, img, imu):
        x = self.init_conv(img)
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)
        x = self.avgpool(x)
        x = torch.flatten(x, 1)
        imu_feat = self.imu_fc(torch.flatten(imu, 1))
        return self.fc(torch.cat((x, imu_feat), dim=1))


def get_device():
    if torch.backends.mps.is_available():
        return torch.device("mps")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def safe_corrcoef(preds, targets):
    correlations = []
    for i in range(targets.shape[1]):
        if np.std(preds[:, i]) == 0 or np.std(targets[:, i]) == 0:
            correlations.append(float("nan"))
        else:
            correlations.append(float(np.corrcoef(preds[:, i], targets[:, i])[0, 1]))
    return np.asarray(correlations, dtype=np.float32)


def load_model(model_path, device, input_shape, imu_shape):
    state_dict = torch.load(model_path, map_location=device, weights_only=True)
    if any(k.startswith("imu_fc.") for k in state_dict):
        model = LegacyImuFcLinearModel(input_shape=input_shape, imu_shape=imu_shape)
    else:
        model = LinearModel(input_shape=input_shape, imu_shape=imu_shape)
    model = model.to(device)
    model.load_state_dict(state_dict)
    model.eval()
    return model


def evaluate_model(
    model_path,
    tub_paths,
    device,
    *,
    image_h=240,
    image_w=320,
    seq_len=3,
    batch_size=32,
):
    img_transform = transforms.Compose(
        [
            transforms.Resize((image_h, image_w)),
            transforms.ToTensor(),
        ]
    )
    dataset = TubDataset(tub_paths, seq_len=seq_len, transform=img_transform)
    if len(dataset) == 0:
        raise RuntimeError("No samples found in evaluation tubs")
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=False)

    model = load_model(
        model_path,
        device,
        input_shape=(3, image_h, image_w),
        imu_shape=(seq_len, 6),
    )

    all_preds = []
    all_targets = []
    with torch.no_grad():
        for images, imus, targets in loader:
            outputs = model(images.to(device), imus.to(device))
            all_preds.append(outputs.cpu().numpy())
            all_targets.append(targets.numpy())

    preds = np.concatenate(all_preds)
    targets = np.concatenate(all_targets)
    mae = np.mean(np.abs(preds - targets), axis=0)
    rmse = np.sqrt(np.mean(np.square(preds - targets), axis=0))
    correlations = safe_corrcoef(preds, targets)
    jitter = np.mean(np.abs(np.diff(preds, axis=0)), axis=0) if len(preds) > 1 else np.zeros(3)
    return {
        "samples": len(dataset),
        "missing_imu_records": dataset.missing_imu_records,
        "mae": mae,
        "rmse": rmse,
        "corr": correlations,
        "jitter": jitter,
    }


def print_single(label, metrics):
    print(f"\n{label}")
    print(f"Samples: {metrics['samples']}  Missing IMU: {metrics['missing_imu_records']}")
    print(f"{'Axis':<12} | {'MAE':>8} | {'RMSE':>8} | {'Corr':>8} | {'Jitter':>8}")
    print("-" * 58)
    for i, axis in enumerate(AXES):
        corr = metrics["corr"][i]
        corr_text = "nan" if np.isnan(corr) else f"{corr:.4f}"
        print(
            f"{axis:<12} | {metrics['mae'][i]:8.4f} | {metrics['rmse'][i]:8.4f} | "
            f"{corr_text:>8} | {metrics['jitter'][i]:8.4f}"
        )


def print_comparison(old_metrics, new_metrics):
    print("\nComparison")
    print(f"{'Axis':<12} | {'Old MAE':>8} | {'New MAE':>8} | {'MAE Change':>10}")
    print("-" * 50)
    for i, axis in enumerate(AXES):
        old_mae = old_metrics["mae"][i]
        new_mae = new_metrics["mae"][i]
        change = ((old_mae - new_mae) / old_mae) * 100 if old_mae else float("nan")
        print(f"{axis:<12} | {old_mae:8.4f} | {new_mae:8.4f} | {change:+9.2f}%")


def main():
    parser = argparse.ArgumentParser(description="Evaluate DonkeyDrone .pth models")
    parser.add_argument("--tubs", required=True, help="Comma-separated tub directories")
    parser.add_argument("--model", help="Model path to evaluate")
    parser.add_argument("--old-model", help="Baseline model path")
    parser.add_argument("--new-model", help="Candidate model path")
    parser.add_argument("--image-h", type=int, default=240)
    parser.add_argument("--image-w", type=int, default=320)
    parser.add_argument("--seq-len", type=int, default=3)
    parser.add_argument("--batch-size", type=int, default=32)
    args = parser.parse_args()

    if not args.model and not (args.old_model and args.new_model):
        parser.error("provide --model or both --old-model and --new-model")

    tub_paths = [t.strip() for t in args.tubs.split(",") if t.strip()]
    device = get_device()
    print(f"Device: {device}")
    print(f"Tubs: {', '.join(tub_paths)}")

    kwargs = {
        "image_h": args.image_h,
        "image_w": args.image_w,
        "seq_len": args.seq_len,
        "batch_size": args.batch_size,
    }
    if args.model:
        label = os.path.basename(args.model)
        print_single(label, evaluate_model(args.model, tub_paths, device, **kwargs))
        return

    old_metrics = evaluate_model(args.old_model, tub_paths, device, **kwargs)
    new_metrics = evaluate_model(args.new_model, tub_paths, device, **kwargs)
    print_single(f"Old: {args.old_model}", old_metrics)
    print_single(f"New: {args.new_model}", new_metrics)
    print_comparison(old_metrics, new_metrics)


if __name__ == "__main__":
    main()
