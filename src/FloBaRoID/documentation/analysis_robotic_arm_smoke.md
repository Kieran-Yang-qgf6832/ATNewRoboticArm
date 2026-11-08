# Smoke test: FloBaRoID on the ATNewRoboticArm_Upper URDF

Purpose: verify that the FloBaRoID pipeline (trajectory -> simulator -> identifier) runs end to end on
`ATNewRoboticArm_Upper/src/arm_model/model/robotic_arm.urdf` and that the identification logic behaves
sensibly on that model. This is a pipeline check, not a parameter-quality experiment.

## Model

- 6 revolute joints (`joint1`..`joint6`), fixed base (`world` -> `base_link`), 7 links with inertia,
  total mass 6.7966 kg (iDynTree reports 7 links / 6 DOF; the `world` link is dropped by the parser).
- All joints declare `effort="50"`, `velocity="10"` - these look like placeholders for a 6.8 kg arm and
  make any torque-utilization floor unrealistic (see below).
- Mesh paths are `package://arm_model/model/*.STL`. Without a ROS index, FloBaRoID resolves them by
  finding the `meshBaseDir` path segment and joining it to the URDF directory, so copying the URDF plus a
  `model/` subdirectory next to it and setting `meshBaseDir: 'model'` makes the STLs resolve without
  touching the URDF at all.
- The URDF contains xacro/`ros2_control` blocks; iDynTree parses them as plain URDF and ignores them, so
  no stripping is needed.

Derived copies (all under `model/robotic_arm/`):

| file | role |
| --- | --- |
| `robotic_arm.urdf` | ground truth: same as upstream plus `<dynamics friction="0.05" damping="0.1"/>` per joint |
| `robotic_arm_prior.urdf` | a priori model given to the identifier: every link's mass and inertia tensor scaled by a per-link factor in [0.8, 1.25], COM unchanged, no `<dynamics>` |
| `robotic_arm_raw.urdf` | untouched copy of the upstream file |

Scaling mass and the inertia tensor by the same factor scales the pseudo-inertia matrix by that factor,
so the perturbed prior stays physically consistent.

## Configuration (`configs/robotic_arm_smoke.yaml`)

Based on `configs/kuka_lwr4.yaml` with: fixed base, bounded tanh trajectories, 2 Fourier harmonics per
joint, 100 Hz, `w_f = 1.0` (period 6.28 s, 628 samples), local optimization only (2 IPOPT iterations),
SDP with `cadRegularizationMode: observability`, friction identified simultaneously and refitted
afterwards, `limitMassVal: 6.8`, meshes from `meshBaseDir: 'model'`.

Deviations from the stock KUKA config that were required to make the run work, and why:

- `trajectoryBounded: 1`: with unbounded Fourier trajectories the amplitude repair could not find a
  feasible point, because `joint5` has a very small range (+/-0.61 rad).
- `minTorqueUtilization: 0.0`: with `effort=50` on all joints, even the 2% floor demands 1 Nm on the
  wrist joints, which the model cannot produce; optimization reported no feasible solution.
- `simulateNumStops: 0`: the sudden stop/restart segments re-integrate positions and pushed the
  trajectory outside the joint limits.
- `staticPostures` removed: with `simulateStaticSamplesPerPosture: 0` the appender builds an empty hold
  segment and `trajectory.py` fails with `IndexError` at the `hold_times[-1]` line.
- `simulateFriction: 0`: with `identifyFrictionSimultaneously: 1` the inverse-dynamics path used for
  simulation already adds the URDF friction, so enabling `simulateFriction` applies it twice.
- `simulateStribeckVelocity: 0`: the identifier models Coulomb + viscous only; Stribeck would be an
  unmodelable residual.
- friction ground truth `0.05 Nm / 0.1 Nm s/rad` instead of a first guess of `0.5 / 0.8`. At 0.5 Nm the
  wrist joints (inertial torque ~0.01 Nm) are dominated by friction and their inertias become
  unidentifiable; the result was inertias off by two orders of magnitude.

## Run

```bash
uv run trajectory.py --config configs/robotic_arm_smoke.yaml --model model/robotic_arm/robotic_arm_prior.urdf
uv run simulator.py  --config configs/robotic_arm_smoke.yaml --model model/robotic_arm/robotic_arm.urdf \
                     --trajectory model/robotic_arm/robotic_arm_prior.urdf.trajectory.npz \
                     --filename model/robotic_arm/measurements.npz
# measurement file split 70/30 in time into meas_train.npz / meas_val.npz
uv run identifier.py --config configs/robotic_arm_smoke.yaml \
                     --model model/robotic_arm/robotic_arm_prior.urdf \
                     --model_real model/robotic_arm/robotic_arm.urdf \
                     --measurements model/robotic_arm/meas_train.npz \
                     --validation model/robotic_arm/meas_val.npz \
                     --output model/robotic_arm/robotic_arm_identified.urdf
```

All three steps complete; the trajectory is reported as feasible and all 56 base parameters observable.

## Results (identifier output)

| metric | a priori | identified | truth |
| --- | --- | --- | --- |
| relative mean residual (train) | 7.54% | 1.37% | - |
| absolute mean residual (train) | 1.30 Nm | 0.244 Nm | - |
| NRMS (train) | 0.47% | 0.11% | - |
| base-parameter distance to truth | 0.10 (14.9%) | 0.02 (6.9%) | 0 |
| total mass | 7.103 kg | 7.243 kg | 6.797 kg |
| Coulomb friction error | - | RMS 0.03 Nm | Fc = 0.05 Nm |
| viscous friction error | - | RMS 0.05 | Fv = 0.1 Nm s/rad |

The identified parameters are reported as physically consistent, the base parameters move clearly
towards the truth model and the friction branches recover values of the right size.

The identified total mass drifts up by 0.45 kg (6.5%) although `limitMassVal: 6.8` is given. The
`closest-to-CAD` step pulls the weakly determined directions towards the (scaled, hence too heavy) a
priori model, and the mass constraint range of +/-0.8 kg leaves room for that drift.

## Known problem: the validation metric is not usable here

`identifier.py` prints a validation error of 41.7% (5.99 Nm) for `meas_val.npz`. That number does not
reflect the model quality: running the identifier with `--validation` pointing at the *training* file
reports 246%, although the same data is fitted to 1.37%.

Reproducing the validation path outside the CLI shows why. `Identification.estimateValidationTorques()`
writes the identified parameters to a temporary URDF, loads a *second* `KinDynComputations` and then
calls `Model.simulateDynamicsIDynTree(samples, idx, val_kinDyn, params)`. In that function the
non-cached branch allocates `LinkWrenches` and `FreeFloatingGeneralizedTorques` per sample and returns

```python
torques = gen_torques.jointTorques().toNumPy()
```

which is a NumPy view into the temporary object's buffer. Once `gen_torques` goes out of scope the buffer
is released, and the stacked result can pick up reused memory. In this environment the same call
sequence intermittently returns values around 1e180 Nm, and at best values that differ by several Nm
from the identical computation done with the cached objects. The training path is unaffected because it
goes through `self._sim_gen_torques`, which lives as long as the model.

Consequences:

- Only the training-side numbers above are trustworthy.
- Both the simulator and any custom script that passes its own `kinDyn` can silently produce garbage.
- `ext_wrenches` is also never zeroed in that branch, unlike the cached one.

A minimal fix is to copy the returned array and zero the wrenches:

```python
torques = gen_torques.jointTorques().toNumPy().copy()
```

Note that the installed `idyntree` here was built from the pinned git tag while `swig` was provided by
extracting the distribution `.deb` into a user prefix (no root). A build through the distribution
packages (`apt install swig coinor-libipopt-dev` then `uv sync`) or an official wheel should be used
before treating any of the numbers above as environment-independent.

## Reproducing the environment

`swig`, `coinor-libipopt-dev`, `coinor-libipopt1v5`, `libmumps-seq-5.6t64` and `libscotch-7.0` were
unpacked into `~/.local/opt/flobaroid-deps` (`apt-get download` + `dpkg -x`), a corrected `ipopt.pc`
points there, and the runs need:

```bash
export PATH=$HOME/.local/opt/flobaroid-deps/usr/bin:$PATH
export LD_LIBRARY_PATH=$HOME/.local/opt/flobaroid-deps/usr/lib:$HOME/.local/opt/flobaroid-deps/usr/lib/x86_64-linux-gnu
```

The shell also exports ROS Jazzy paths in `PYTHONPATH`. That makes `resource_retriever` importable and it
then fails on `package://arm_model/...`, so the runs use `env -u PYTHONPATH`.
