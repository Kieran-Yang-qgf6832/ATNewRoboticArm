# Analysis: identifying ATNewRoboticArm_Upper from a MuJoCo measure CSV

Purpose: check whether FloBaRoID can identify the current project's arm from the
CSV written by the controller-side `measure` state, end to end, without touching
`src/parameter_identify`. This is a validation run, not a parameter-quality
experiment on real hardware. Date: 2026-11-08.

## Verdict

**FAIL.** The data chain is correct, but the identification made the model worse
than the deliberately perturbed a priori model. No parameters were changed and no
retry was attempted after the criteria failed.

| criterion | threshold | measured | result |
| --- | --- | --- | --- |
| physical consistency of identified params | must pass | `Identified parameters are physical consistent` (identifier.log:154); URDF written | PASS |
| base-parameter distance to truth vs. a priori | shrink by at least 50% | 0.25 identified vs. 0.07 a priori (24.2% vs. 12.7% relative); got ~3.6x worse | FAIL |
| held-out relative torque residual | <= 5% and <= 2x train | val 23.7163%, train 23.1741% (ratio 1.02) | FAIL on the 5% limit |

The identifier's own training residual was 33.34% (a priori 45.01%, absolute
7.94 N.m vs. 10.11 N.m, NRMS 3.03% vs. 3.58%), and it printed
`neither base nor std params improved`.

## Inputs and files

All new files are inside this repository; `src/parameter_identify` was not
modified. Run artifacts live in `extra/robotic_arm_sim/` (ignored by `.gitignore`)
and the HTML report in `output/output_robotic_arm_sim_csv.html` (also ignored).

| file | role |
| --- | --- |
| `tools/robotic_arm_csv2npz.py` | CSV -> train/val npz converter, 70/30 contiguous time split, no preprocess |
| `tools/robotic_arm_eval.py` | held-out torque residual via iDynTree regressor `Y @ xStd` |
| `configs/robotic_arm_sim_csv.yaml` | derived from `configs/robotic_arm_smoke.yaml` |
| `model/robotic_arm/robotic_arm_prior.urdf` | 6-DOF a priori model (mass/inertia perturbed) |
| `/home/kangy/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper/src/arm_model/model/robotic_arm.urdf` | truth / `--model_real` (byte-identical to `model/robotic_arm/robotic_arm_raw.urdf`) |

Config differences from `robotic_arm_smoke.yaml`: `excitationFrequency` 100 -> 500,
`identifyFrictionSimultaneously` 1 -> 0, `postIdentifyFriction` 1 -> 0,
`identifyClosestToCAD` 1 -> 0, `limitOverallMass` 1 -> 0, own `outputFilename`.
`constrainToConsistent: 1`, `useAPriori: 0`, `restrictCOMtoHull: 1`,
`estimateWith: 'std'` are unchanged. The MuJoCo model has no `<dynamics>`,
`frictionloss` or `damping`, so friction identification is disabled.

## Commands

From the project root (measurement), then from `src/FloBaRoID` (offline steps):

```bash
# 1) MuJoCo excitation trajectory (headless, fixed seed, default 10 s / 1 repeat)
source /opt/ros/jazzy/setup.bash && source install/setup.bash
CSV=/home/kangy/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper/src/FloBaRoID/extra/robotic_arm_sim/measure.csv
ros2 launch launch_pack arm_measure_sim.launch.py csv_file_path:=$CSV \
    trajectory_seed:=1 trajectory_period:=10.0 trajectory_repeat_cnt:=1 \
    headless:=true step_timeout:=240.0
```

```bash
# 2) convert (from src/FloBaRoID)
env -u PYTHONPATH uv run --no-sync tools/robotic_arm_csv2npz.py \
    --csv extra/robotic_arm_sim/measure.csv --out-dir extra/robotic_arm_sim

# 3) sanity check: truth URDF must reproduce the recorded torques
env -u PYTHONPATH uv run --no-sync tools/robotic_arm_eval.py \
    --urdf /home/kangy/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper/src/arm_model/model/robotic_arm.urdf \
    --prior-urdf model/robotic_arm/robotic_arm_prior.urdf \
    --train extra/robotic_arm_sim/train.npz --val extra/robotic_arm_sim/val.npz \
    --out extra/robotic_arm_sim/truth_eval.json

# 4) identify
env -u PYTHONPATH uv run --no-sync identifier.py \
    --config configs/robotic_arm_sim_csv.yaml \
    --model model/robotic_arm/robotic_arm_prior.urdf \
    --model_real /home/kangy/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper/src/arm_model/model/robotic_arm.urdf \
    --measurements extra/robotic_arm_sim/train.npz \
    --output extra/robotic_arm_sim/identified.urdf > extra/robotic_arm_sim/identifier.log 2>&1

# 5) evaluate the identified URDF
env -u PYTHONPATH uv run --no-sync tools/robotic_arm_eval.py \
    --urdf extra/robotic_arm_sim/identified.urdf \
    --prior-urdf model/robotic_arm/robotic_arm_prior.urdf \
    --train extra/robotic_arm_sim/train.npz --val extra/robotic_arm_sim/val.npz \
    --out extra/robotic_arm_sim/eval.json
```

`env -u PYTHONPATH` is needed when ROS Jazzy is sourced: `resource_retriever`
would otherwise fail on the `package://arm_model/...` mesh paths.

## Measurement data

`measure.csv`: 4999 rows, 25 columns (`time,pos_0..5,vel_0..5,acc_0..5,
torque_0..5`), 500 Hz, 10 s of simulation time, all six joints excited
(`pos_5` span 2.37 rad). The launch exited 0; warnings were RT-scheduling and
legacy anti-windup notices, plus one shutdown-time `pal_statistics` publisher
error. Split: 3499 train / 1500 val samples. `frequency` is stored as a 0-d
array, `times` in seconds; recorded velocities and accelerations are kept
verbatim (no `Data.preprocess()`, which would replace `acc_*` with central
differences).

## Sanity check of the data chain

| model | train relative residual | val relative residual |
| --- | --- | --- |
| truth URDF | 0.0002% (RMS 1.9e-5 N.m) | 0.0002% (RMS 1.5e-5 N.m) |
| perturbed prior | 7.2306% | 7.1319% |
| identified URDF | 23.1741% | 23.7163% |

The truth model reproduces the recorded torques to machine precision on both
splits, so CSV column order, units, gravity and the qacc/position one-step
alignment are correct. The regression failure is inside FloBaRoID's estimation,
not in the measurement chain.

## Observations for a follow-up (no fix attempted here)

- `identifier.log:11` reports `LinAlgWarning: ill-conditioned matrix ... rcond =
  2.86e-139` at `identification/model.py:918` (`sla.inv(R1)` while building the
  structural-regressor linear dependencies). The run also printed
  `(re-)generating structural regressor (5000 random positions)`.
- `identifier.log:21` reports `SDP found std solution with 317997.71 squared
  residual error`; the fitted mass stays near the perturbed prior
  (7.1136 kg identified vs. 7.1032 kg a priori vs. 6.7966 kg truth).
- The parameter table shows many `hull` constraints on first moments of mass.
  Whether the SDP objective/constraints or the ill-conditioned base-parameter
  projection dominates the bad fit has not been established.
- Two implementation bugs in the new evaluation script were found and fixed
  while running this check: iDynTree SWIG owner objects must outlive the
  `KinDynComputations`/numpy views, and each dataset needs a fresh model load
  (reusing one `KinDynComputations` across two datasets silently corrupts the
  second one's regressors). This is the same class of dangling-buffer bug as the
  uncommitted patches in `identification/model.py`.

## Artifacts

`extra/robotic_arm_sim/`: `measure.csv`, `measure.log`, `train.npz`, `val.npz`,
`split.json`, `truth_eval.json`, `identifier.log`, `identified.urdf`, `eval.json`.
Report: `output/output_robotic_arm_sim_csv.html`.
