#!/usr/bin/env python3
"""Convert the ATNewRoboticArm_Upper measure CSV into FloBaRoID train/val npz files.

The CSV is written by ParamterIdentify: 25 columns at 500 Hz, simulation time in
seconds, and directly recorded MuJoCo joint accelerations. No filtering and no
numerical differentiation is applied here, because FloBaRoID's Data.preprocess()
would overwrite the recorded accelerations with central differences of filtered
positions.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


def parse_args() -> argparse.Namespace:
    """Parse the command line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True, help="measure CSV written by the controller")
    parser.add_argument("--out-dir", type=Path, required=True, help="directory for train.npz and val.npz")
    parser.add_argument("--dof", type=int, default=6, help="number of joints in the CSV")
    parser.add_argument(
        "--train-ratio", type=float, default=0.7, help="contiguous time split fraction used for training"
    )
    return parser.parse_args()


def expected_columns(dof: int) -> list[str]:
    """Return the CSV column names in the order written by ParamterIdentify."""
    return (
        ["time"]
        + [f"pos_{i}" for i in range(dof)]
        + [f"vel_{i}" for i in range(dof)]
        + [f"acc_{i}" for i in range(dof)]
        + [f"torque_{i}" for i in range(dof)]
    )


def load_samples(csv_path: Path, dof: int) -> tuple[dict[str, np.ndarray], float]:
    """Load the recorded samples verbatim and derive the sampling frequency from the time column."""
    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    if data.ndim == 0:
        raise ValueError(f"{csv_path} contains no data rows")

    columns = list(data.dtype.names or ())
    missing = [name for name in expected_columns(dof) if name not in columns]
    if missing:
        raise ValueError(f"{csv_path} is missing columns {missing}; regenerate it with the current measure state")

    times = np.asarray(data["time"], dtype=float)
    positions = np.column_stack([data[f"pos_{i}"] for i in range(dof)]).astype(float)
    velocities = np.column_stack([data[f"vel_{i}"] for i in range(dof)]).astype(float)
    accelerations = np.column_stack([data[f"acc_{i}"] for i in range(dof)]).astype(float)
    torques = np.column_stack([data[f"torque_{i}"] for i in range(dof)]).astype(float)

    for name, array in (
        ("time", times),
        ("positions", positions),
        ("velocities", velocities),
        ("accelerations", accelerations),
        ("torques", torques),
    ):
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{csv_path} has non-finite values in {name}")

    dt = np.diff(times)
    if dt.size == 0 or np.any(dt <= 0.0):
        raise ValueError(f"{csv_path} has a non-increasing time column")
    frequency = float(1.0 / np.median(dt))
    if not 400.0 <= frequency <= 600.0:
        raise ValueError(f"expected a 500 Hz measurement, derived {frequency:.2f} Hz from {csv_path}")

    samples: dict[str, np.ndarray] = {
        "positions": positions,
        "velocities": velocities,
        "accelerations": accelerations,
        "torques": torques,
        "times": times,
        "frequency": np.asarray(frequency),
        "positions_raw": positions.copy(),
        "velocities_raw": velocities.copy(),
        "torques_raw": torques.copy(),
    }
    return samples, frequency


def write_split(
    samples: dict[str, np.ndarray], frequency: float, out_dir: Path, train_ratio: float, csv_path: Path
) -> tuple[Path, Path, int, int]:
    """Write a contiguous time split into train.npz and val.npz."""
    n_samples = int(samples["times"].shape[0])
    n_train = int(round(n_samples * train_ratio))
    if n_train < 1 or n_train >= n_samples:
        raise ValueError(f"train ratio {train_ratio} yields no usable split for {n_samples} samples")
    if not 0.0 < train_ratio < 1.0:
        raise ValueError(f"train ratio must be in (0, 1), got {train_ratio}")

    out_dir.mkdir(parents=True, exist_ok=True)
    splits = {"train": (0, n_train), "val": (n_train, n_samples)}
    for name, (start, end) in splits.items():
        subset: dict[str, Any] = {}
        for key, value in samples.items():
            if value.ndim == 0:
                subset[key] = value
            elif value.ndim == 1:
                subset[key] = value[start:end]
            else:
                subset[key] = value[start:end, :]
        np.savez(out_dir / f"{name}.npz", **subset)

    summary = {
        "csv": str(csv_path),
        "frequency_hz": frequency,
        "samples": n_samples,
        "train_samples": n_train,
        "val_samples": n_samples - n_train,
        "train_ratio": train_ratio,
        "preprocess": "none (recorded velocities/accelerations kept verbatim)",
    }
    (out_dir / "split.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return out_dir / "train.npz", out_dir / "val.npz", n_train, n_samples - n_train


def main() -> None:
    """Convert one measure CSV into a train/val npz pair."""
    args = parse_args()
    samples, frequency = load_samples(args.csv, args.dof)
    train_path, val_path, n_train, n_val = write_split(samples, frequency, args.out_dir, args.train_ratio, args.csv)
    print(f"csv={args.csv} samples={n_train + n_val} frequency={frequency:.2f} Hz")
    print(f"train={train_path} ({n_train} samples)")
    print(f"val={val_path} ({n_val} samples)")


if __name__ == "__main__":
    main()
