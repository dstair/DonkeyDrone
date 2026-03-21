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
    --myconfig  Config file override [default: drone_config.py]
"""

import argparse
import json
import os
import time

import numpy as np
from PIL import Image
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split

from torch_model import LinearModel


class TubDataset(Dataset):
    """Reads DonkeyCar tub v2 format: catalog JSON lines + JPEG images."""

    def __init__(self, tub_paths, image_h, image_w):
        self.records = []
        self.image_h = image_h
        self.image_w = image_w

        if isinstance(tub_paths, str):
            tub_paths = [tub_paths]

        for tub_path in tub_paths:
            catalog_files = sorted(
                f for f in os.listdir(tub_path)
                if f.endswith('.catalog') and not f.endswith('_manifest')
            )
            images_dir = os.path.join(tub_path, 'images')
            for cat_file in catalog_files:
                with open(os.path.join(tub_path, cat_file)) as f:
                    for line in f:
                        line = line.strip()
                        if not line:
                            continue
                        rec = json.loads(line)
                        img_path = os.path.join(images_dir, rec['cam/image_array'])
                        if os.path.exists(img_path):
                            self.records.append({
                                'image_path': img_path,
                                'angle': rec['user/angle'],
                                'throttle': rec['user/throttle'],
                                'altitude': rec.get('user/altitude', 0.0),
                            })

    def __len__(self):
        return len(self.records)

    def __getitem__(self, idx):
        rec = self.records[idx]
        img = Image.open(rec['image_path']).convert('RGB')
        img = img.resize((self.image_w, self.image_h), Image.BILINEAR)
        # HWC uint8 → CHW float32 [0, 1]
        arr = np.array(img, dtype=np.float32) / 255.0
        tensor = torch.from_numpy(arr.transpose(2, 0, 1))
        label = torch.tensor([rec['angle'], rec['throttle'], rec['altitude']], dtype=torch.float32)
        return tensor, label


def get_device():
    if torch.backends.mps.is_available():
        return torch.device('mps')
    if torch.cuda.is_available():
        return torch.device('cuda')
    return torch.device('cpu')


def train(cfg, tub_paths, model_path):
    device = get_device()
    print(f"Device: {device}")

    image_h = getattr(cfg, 'IMAGE_H', 120)
    image_w = getattr(cfg, 'IMAGE_W', 160)
    image_d = getattr(cfg, 'IMAGE_DEPTH', 3)
    batch_size = getattr(cfg, 'BATCH_SIZE', 128)
    max_epochs = getattr(cfg, 'MAX_EPOCHS', 100)
    lr = getattr(cfg, 'LEARNING_RATE', 0.001)
    patience = getattr(cfg, 'EARLY_STOP_PATIENCE', 5)
    train_split = getattr(cfg, 'TRAIN_TEST_SPLIT', 0.8)

    # Load dataset
    dataset = TubDataset(tub_paths, image_h, image_w)
    print(f"Total samples: {len(dataset)}")
    if len(dataset) == 0:
        print("ERROR: No samples found. Check --tubs path.")
        return

    # Train/val split
    n_train = int(len(dataset) * train_split)
    n_val = len(dataset) - n_train
    train_ds, val_ds = random_split(dataset, [n_train, n_val])
    print(f"Train: {n_train}, Val: {n_val}")

    pin = device.type == 'cuda'
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True,
                              num_workers=0, pin_memory=pin)
    val_loader = DataLoader(val_ds, batch_size=batch_size, shuffle=False,
                            num_workers=0, pin_memory=pin)

    # Model
    model = LinearModel(input_shape=(image_d, image_h, image_w)).to(device)
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
        for images, labels in train_loader:
            images, labels = images.to(device), labels.to(device)
            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * images.size(0)
        train_loss /= n_train

        # Validate
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for images, labels in val_loader:
                images, labels = images.to(device), labels.to(device)
                outputs = model(images)
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
    parser.add_argument('--myconfig', default='drone_config.py', help='Config file')
    args = parser.parse_args()

    import donkeycar as dk
    cfg = dk.load_config(
        config_path=os.path.join(os.path.dirname(__file__), 'config.py'),
        myconfig=args.myconfig,
    )

    tub_paths = [t.strip() for t in args.tubs.split(',')]
    train(cfg, tub_paths, args.model)


if __name__ == '__main__':
    main()
