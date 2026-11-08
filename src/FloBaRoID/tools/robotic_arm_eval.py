#!/usr/bin/env python3
"""Evaluate a FloBaRoID-identified URDF against train/val measurement npz files.

Joint torques are predicted with the iDynTree inverse-dynamics inertial-parameter
regressor (Y @ xStd). No inverseDynamics call is used, so the result does not depend
on the uninitialized-memory path of Model.simulateDynamicsIDynTree.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from idyntree import bindings as iDynTree

REQUIRED_KEYS: tuple[str, ...] = ("positions", "velocities", "accelerations", "torques")


@dataclass
class IdyntreeModel:
    """Keep the SWIG owner objects alive for the whole evaluation.

    iDynTree's KinDynComputations and the numpy views handed out by SWIG remain
    valid only while the ModelLoader/Model/parameter-vector owners exist. Letting
    them go out of scope produces silently garbage regressors (values around 1e96),
    the same class of bug as the dangling-buffer fixes in identification/model.py.
    """

    kin_dyn: Any
    x_std: np.ndarray
    num_dofs: int
    loader: Any
    model: Any
    x_std_vec: Any


def parse_args() -> argparse.Namespace:
    """Parse the command line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True, help="identified URDF to evaluate")
    parser.add_argument("--prior-urdf", type=Path, default=None, help="optional a priori URDF for a baseline")
    parser.add_argument("--train", type=Path, required=True, help="training measurement npz")
    parser.add_argument("--val", type=Path, required=True, help="held-out measurement npz")
    parser.add_argument("--out", type=Path, default=None, help="optional path for the JSON summary")
    return parser.parse_args()


def load_model(urdf: Path) -> IdyntreeModel:
    """Load a URDF and return KinDynComputations together with its SWIG owners."""
    loader = iDynTree.ModelLoader()
    if not loader.loadModelFromFile(str(urdf)):
        raise RuntimeError(f"could not load URDF {urdf}")
    model = loader.model()

    kin_dyn = iDynTree.KinDynComputations()
    if not kin_dyn.loadRobotModel(model):
        raise RuntimeError(f"could not build KinDynComputations for {urdf}")

    num_dofs = int(kin_dyn.getNrOfDegreesOfFreedom())
    x_std_vec = iDynTree.VectorDynSize(int(model.getNrOfLinks()) * 10)
    model.getInertialParameters(x_std_vec)
    x_std = x_std_vec.toNumPy().copy()
    return IdyntreeModel(kin_dyn, x_std, num_dofs, loader, model, x_std_vec)


def load_samples(path: Path) -> dict[str, np.ndarray]:
    """Load the arrays needed for regressor-based torque prediction."""
    with np.load(path, allow_pickle=True) as data:
        missing = [key for key in REQUIRED_KEYS if key not in data.files]
        if missing:
            raise KeyError(f"{path} is missing {missing}")
        samples = {key: np.asarray(data[key], dtype=float) for key in REQUIRED_KEYS}
    n_samples = samples["positions"].shape[0]
    for key, array in samples.items():
        if array.shape[0] != n_samples:
            raise ValueError(f"{path}: {key} has {array.shape[0]} rows, expected {n_samples}")
    return samples


def predict_torques(idyn: IdyntreeModel, samples: dict[str, np.ndarray]) -> np.ndarray:
    """Predict joint torques from Y @ xStd for every measurement sample."""
    for key, array in samples.items():
        if array.ndim != 2 or array.shape[1] != idyn.num_dofs:
            raise ValueError(f"{key} must have shape (N, {idyn.num_dofs}), got {array.shape}")
    gravity = iDynTree.Vector3()
    gravity.setVal(0, 0.0)
    gravity.setVal(1, 0.0)
    gravity.setVal(2, -9.81)

    n_samples = samples["positions"].shape[0]
    predicted = np.empty((n_samples, idyn.num_dofs), dtype=float)
    q = iDynTree.JointPosDoubleArray(idyn.num_dofs)
    dq = iDynTree.JointDOFsDoubleArray(idyn.num_dofs)
    ddq = iDynTree.JointDOFsDoubleArray(idyn.num_dofs)
    base_acceleration = iDynTree.Vector6()

    for i in range(n_samples):
        for j in range(idyn.num_dofs):
            q.setVal(j, float(samples["positions"][i, j]))
            dq.setVal(j, float(samples["velocities"][i, j]))
            ddq.setVal(j, float(samples["accelerations"][i, j]))
        idyn.kin_dyn.setRobotState(q, dq, gravity)

        regressor = iDynTree.MatrixDynSize()
        if not idyn.kin_dyn.inverseDynamicsInertialParametersRegressor(base_acceleration, ddq, regressor):
            raise RuntimeError(f"regressor computation failed at sample {i}")
        y_joint = regressor.toNumPy().copy()[6:, :]
        if y_joint.shape[1] != idyn.x_std.shape[0]:
            raise ValueError(f"regressor has {y_joint.shape[1]} columns, xStd has {idyn.x_std.shape[0]} entries")
        predicted[i, :] = y_joint @ idyn.x_std

    return predicted


def residual_metrics(measured: np.ndarray, predicted: np.ndarray) -> dict[str, float]:
    """Return relative and absolute torque residual metrics in percent and N.m."""
    residual = measured - predicted
    norm_measured = float(np.linalg.norm(measured))
    if norm_measured == 0.0:
        raise ValueError("measured torques are all zero")
    return {
        "relative_percent": 100.0 * float(np.linalg.norm(residual)) / norm_measured,
        "rms_nm": float(np.sqrt(np.mean(residual**2))),
        "mean_abs_nm": float(np.mean(np.abs(residual))),
        "max_abs_nm": float(np.max(np.abs(residual))),
    }


def evaluate(urdf: Path, datasets: dict[str, Path]) -> dict[str, dict[str, float]]:
    """Evaluate one URDF on the train and held-out datasets.

    A fresh iDynTree model is loaded for each dataset: reusing KinDynComputations
    across two predict_torques() calls leaves SWIG-side state that corrupts the
    second dataset's regressors.
    """
    result: dict[str, dict[str, float]] = {}
    for name, path in datasets.items():
        idyn = load_model(urdf)
        samples = load_samples(path)
        predicted = predict_torques(idyn, samples)
        result[name] = residual_metrics(samples["torques"], predicted)
    return result


def main() -> None:
    """Print and optionally persist the residual comparison for identified and prior models."""
    args = parse_args()
    datasets = {"train": args.train, "val": args.val}
    identified = evaluate(args.urdf, datasets)
    summary: dict[str, object] = {"identified_urdf": str(args.urdf), "identified": identified}

    print(f"model={args.urdf.name}")
    for name in datasets:
        metrics = identified[name]
        print(
            f"  {name}: relative={metrics['relative_percent']:.4f}% "
            f"rms={metrics['rms_nm']:.6f} Nm max={metrics['max_abs_nm']:.6f} Nm"
        )

    train_rel = identified["train"]["relative_percent"]
    val_rel = identified["val"]["relative_percent"]
    residual_pass = val_rel <= 5.0 and val_rel <= 2.0 * train_rel
    summary["residual_criterion"] = {
        "val_relative_percent_limit": 5.0,
        "val_to_train_ratio_limit": 2.0,
        "pass": bool(residual_pass),
    }
    print(f"residual criterion (val <= 5% and <= 2x train): {'PASS' if residual_pass else 'FAIL'}")

    if args.prior_urdf is not None:
        prior = evaluate(args.prior_urdf, datasets)
        summary["prior_urdf"] = str(args.prior_urdf)
        summary["prior"] = prior
        print(f"prior={args.prior_urdf.name}")
        for name in datasets:
            print(f"  {name}: relative={prior[name]['relative_percent']:.4f}%")

    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"saved {args.out}")


if __name__ == "__main__":
    main()
