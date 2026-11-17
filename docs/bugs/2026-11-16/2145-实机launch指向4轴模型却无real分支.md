---
id: O24
title: arm_real.launch.py 指向 4 轴模型但 URDF 没有 real 分支，实机链路静默跑成 MuJoCo 仿真【高】
severity: 高
status: 已修
date: 2026-11-16 2145
commit: —
---

# O24 arm_real.launch.py 指向 4 轴模型但 URDF 没有 real 分支，实机链路静默跑成 MuJoCo 仿真【高】

> 严重度：高 ｜ 状态：已修 ｜ 日期：2026-11-16 2145

## 现象

按 [实机首次运行与标定](../../execute/04-实机首次运行与标定.md) 启动
`ros2 launch launch_pack arm_real.launch.py`：

- 进程能起来，`joint_state_broadcaster` 与 `arm_controller` 都 `active`，RViz 里模型也跟着动；
- 但插拔下位机、`lsusb -d 0483:5741` 有/无设备，行为完全一样；
- 日志里硬件插件是 `mujoco_ros2_control/MujocoSystemInterface`，不是 `arm_controller/ArmRealInterfaces`；
- `loopback:=true/false` 对日志没有任何影响。

也就是“实机 launch 看起来一切正常，实际全程没有打开 USB”。

## 根因

三层叠加，任何一层单独存在都会让链路失效：

1. **URDF 没有 real 分支**。4 轴迁移（`6f34479`）把 launch 的模型路径改成
   `arm_model/model_4dof/robotic_arm.urdf`，但这个文件只保留了 MuJoCo 硬件块，
   没有 `hardware` / `loopback` 两个 xacro 参数。launch 传进去的
   `hardware:=real loopback:=false` 被 xacro **静默忽略**，展开结果仍是
   `<plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>` + `mujoco_model`。
2. **硬件接口写死 6 关节**。`ArmRealInterfaces::on_init` 要求
   `joint_names_.size() == kProtocolMotorCount(6)`，4 轴 URDF 只有 4 个关节，
   所以即使手工补上 real 块也会直接拒绝启动。
3. **实机覆盖配置是 6 轴/5 自由度时代的**。`ros2_controller_real.yaml` 里
   `task_dim: 5`（控制器只声明不读取，属于死参数）、`default_kp/kd` 各 6 项、
   注释围绕 joint6，与 4 关节控制器不匹配。

## 修复

- `model_4dof/robotic_arm.urdf`：补 `hardware` / `loopback` / `mujoco_model` 三个
  xacro 参数，`<ros2_control>` 拆成 mujoco / real 两个分支；real 块声明 4 个关节的
  `position/velocity/effort/kp/kd/ki` 命令接口与 `position/velocity/effort/acceleration`
  状态接口，并写 `motor_count=4`、`joint_calibration_file=.../joint_calibration_4dof.yaml`。
- `ArmRealInterfaces` 泛化：关节数允许 1..6（超过 6 槽才拒绝）；新增可选硬件参数
  `motor_count`（缺省 = 关节数，须 ≤ 关节数）。接收缓冲固定 6 槽，`write()` 对没有
  URDF 关节的槽位回填**实测原始角**、速度/力矩给 0，保持原地不动；首帧状态到达前
  不发送任何指令，避免把默认 0 当成回零目标。
- 6 轴旧硬件同步显式写 `motor_count=5`，保留 joint6“无电机、状态回写指令”的旧语义。
- 新增 4 轴标定表 `config/joint_calibration_4dof.yaml`（全占位 `dir=1 / zero=0`），
  与 6 轴的 `joint_calibration.yaml` 分离；`load_calibration` 按 URDF 关节名读取。
- `ros2_controller_real.yaml` 按 4 关节重写（`task_space_dof: 4`、4 项 kp/kd、删 `task_dim: 5`），
  并注明当前只验收关节控制。
- 离线测试 `arm_real_joint_count_test` 更新为 4/6/7 关节与越界 `motor_count` 用例。

## 验证

1. `xacro model_4dof/robotic_arm.urdf hardware:=real loopback:=false` → 输出
   `arm_controller/ArmRealInterfaces` + `motor_count=4` + 4 个关节；
   `hardware:=mujoco`（默认）仍输出 MuJoCo 块。
2. `./build/arm_controller/arm_real_joint_count_test` → 5 个用例全部 PASS
   （4 关节导出 16 状态/24 命令，7 关节拒绝，越界 `motor_count` 拒绝）。
3. `ros2 launch launch_pack arm_real.launch.py loopback:=true record_bag:=false` →
   日志 `Configured ArmRealInterfaces for 4 joints, 4 motors`、
   `arm_controller: joints=4 ... task_space_dof=4`，两个控制器 `active`，
   `/joint_states` 含 `joint1..joint4`。

## 附

真机 USB 链路（VID 0x0483 / PID 0x5741、标定、首帧状态门控）还没有接真机实测；
本条目只证明“launch 不再静默跑仿真、4 关节配置能通过硬件接口的初始化与门控”。
