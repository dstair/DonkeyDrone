#!/usr/bin/env python3
"""
Standalone PyTorch training script for DonkeyDrone CNN.

Reads tub data directly (catalog JSON + JPEG images), trains a PyTorch
LinearModel, and saves the best checkpoint as a .pth file.

Usage:
    python donkeydrone/torch_train.py --tubs=data/tub_16_26-03-01 --model=models/pilot.pth
    python donkeydrone/torch_train.py --tubs=data/tub_16_26-03-01,data/tub_17_26-03-01 --model=models/pilot.pth

Options:
    --tubs      Comma-separated list of tub directories
    --model     Output .pth model path
    --myconfig  Config file override [default: drone_config_65mm.py]
"""

import argparse
import json
import os
import time

import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split
from torchvision import transforms

from dataset import TubDataset
from torch_model import LinearModel

def get_device():
    if torch.backends.mps.is_available():
        return torch.device('mps')
    if torch.cuda.is_available():
        return torch.device('cuda')
    return torch.device('cpu')


def train(cfg, tub_paths, model_path, max_epochs_override=None):
    device = get_device()
    print(f"Device: {device}")

    image_h = getattr(cfg, 'IMAGE_H', 120)
    image_w = getattr(cfg, 'IMAGE_W', 160)
    image_d = getattr(cfg, 'IMAGE_DEPTH', 3)
    batch_size = getattr(cfg, 'BATCH_SIZE', 128)
    max_epochs = max_epochs_override or getattr(cfg, 'MAX_EPOCHS', 100)
    lr = getattr(cfg, 'LEARNING_RATE', 0.001)
    patience = getattr(cfg, 'EARLY_STOP_PATIENCE', 5)
    train_split = getattr(cfg, 'TRAIN_TEST_SPLIT', 0.8)
    seq_len = getattr(cfg, 'SEQUENCE_LENGTH', 3)

    # Define image transformations to match config resolution
    img_transform = transforms.Compose([
        transforms.Resize((image_h, image_w)),
        transforms.ToTensor(), # handles HWC -> CHW and [0, 1] scaling
    ])

    # Load dataset using the multi-modal TubDataset
    dataset = TubDataset(tub_paths, seq_len=seq_len, transform=img_transform)
    print(f"Total samples: {len(dataset)}")
    if len(dataset) == 0:
        print("ERROR: No samples found. Check --tubs path.")
        return
    if dataset.missing_imu_records:
        print(
            "WARNING: "
            f"{dataset.missing_imu_records}/{len(dataset)} samples are missing "
            "imu/acl_* or imu/gyr_* fields; zeros will be used for those records."
        )

    # Train/val split
    n_train = int(len(dataset) * train_split)
    n_val = len(dataset) - n_train
    if n_train == 0 or n_val == 0:
        print("ERROR: Need at least two samples for a train/val split.")
        return
    train_ds, val_ds = random_split(dataset, [n_train, n_val])
    print(f"Train: {n_train}, Val: {n_val}")

    pin = device.type == 'cuda'
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True,
                              num_workers=0, pin_memory=pin)
    val_loader = DataLoader(val_ds, batch_size=batch_size, shuffle=False,
                            num_workers=0, pin_memory=pin)

    # Model
    model = LinearModel(
        input_shape=(image_d, image_h, image_w), 
        imu_shape=(seq_len, 6)
    ).to(device)
    if getattr(cfg, 'PRINT_MODEL_SUMMARY', True):
        total_params = sum(p.numel() for p in model.parameters())
        print(f"Model parameters: {total_params:,}")
        print(model)

    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    criterion = nn.MSELoss()

    # Training loop
    best_val_loss = float('inf')
    epochs_no_improve = 0
    os.makedirs(os.path.dirname(model_path) or '.', exist_ok=True)

    for epoch in range(max_epochs):
        t0 = time.time()

        # Train
        model.train()
        train_loss = 0.0
        for images, imus, labels in train_loader:
            images, imus, labels = images.to(device), imus.to(device), labels.to(device)
            optimizer.zero_grad()
            outputs = model(images, imus)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * images.size(0)
        train_loss /= n_train

        # Validate
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for images, imus, labels in val_loader:
                images, imus, labels = images.to(device), imus.to(device), labels.to(device)
                outputs = model(images, imus)
                loss = criterion(outputs, labels)
                val_loss += loss.item() * images.size(0)
        val_loss /= n_val

        elapsed = time.time() - t0
        print(f"Epoch {epoch + 1}/{max_epochs} — "
              f"train_loss: {train_loss:.6f}, val_loss: {val_loss:.6f} "
              f"({elapsed:.1f}s)")

        # Early stopping + checkpoint
        if val_loss < best_val_loss - getattr(cfg, 'MIN_DELTA', 0.0005):
            best_val_loss = val_loss
            epochs_no_improve = 0
            torch.save(model.state_dict(), model_path)
            print(f"  ↳ Saved best model (val_loss: {val_loss:.6f})")
        else:
            epochs_no_improve += 1
            if getattr(cfg, 'USE_EARLY_STOP', True) and epochs_no_improve >= patience:
                print(f"Early stopping after {epoch + 1} epochs (no improvement for {patience})")
                break

    print(f"\nTraining complete. Best val_loss: {best_val_loss:.6f}")
    print(f"Model saved to: {model_path}")


def main():
    parser = argparse.ArgumentParser(description='Train PyTorch CNN for DonkeyDrone')
    parser.add_argument('--tubs', required=True, help='Comma-separated tub directories')
    parser.add_argument('--model', required=True, help='Output .pth model path')
    parser.add_argument('--myconfig', default='drone_config_65mm.py', help='Config file')
    parser.add_argument('--max-epochs', type=int, default=None, help='Override MAX_EPOCHS')
    args = parser.parse_args()

    import donkeycar as dk
    cfg = dk.load_config(
        config_path=os.path.join(os.path.dirname(__file__), 'config.py'),
        myconfig=args.myconfig,
    )

    tub_paths = [t.strip() for t in args.tubs.split(',')]
    train(cfg, tub_paths, args.model, max_epochs_override=args.max_epochs)


if __name__ == '__main__':
    main()
