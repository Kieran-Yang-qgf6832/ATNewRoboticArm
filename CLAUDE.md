# CLAUDE.md

本文件是 AI 助手在本仓库工作的**规则与导航**。详细操作见 `docs/execute/`，
设计原因见 `docs/adr/`，问题记录见 `docs/bugs/`。

## 项目概述

基于 ROS 2（Jazzy）+ `ros2_control` 的 6 自由度机械臂控制系统，支持 MuJoCo 仿真与
实机（USB CDC → MCU）。核心包：`arm_controller`（控制器 / FSM / 求解器）、`tasks`
（配置驱动任务层）、`arm_model`（URDF/MJCF）、`launch_pack`（launch/config）、
`parameter_identify`（离线辨识）、`view`（视觉）。

**当前状态**：仿真链路端到端验证；实机 loopback 通过、**尚未上电验证**；
real-compat 5DOF 仿真验收通过；R5 固件看门狗未实现（拔线≠停机）。

## 文档导航

| 需要什么 | 去哪里 |
| --- | --- |
| 环境、构建、仿真/实机运行、标定、排错 | `docs/execute/README.md` |
| 技术选型与设计原因 | `docs/adr/README.md` |
| 缺陷的发现/分析/修复/验证 | `docs/bugs/README.md` |
| 版本、功能、兼容性变化 | `docs/changelog/README.md` |
| 教学材料 | `docs/teach/` |

**维护规则**：新问题写 `docs/bugs/YYYY-MM-DD/HHMM-中文短语.md` 并更新索引；
新决策写 `docs/adr/YYYY-MM-DD/HHMM-中文短语.md` 并更新 ADR 索引；有意义的变更写
`docs/changelog/YYYY-MM-DD/HHMM-中文短语.md` 并更新变更索引；运行步骤变更更新 `docs/execute/`。
README 只保留目标、特性、快速开始和文档地图。

## 常用命令

```bash
colcon build --symlink-install
colcon build --packages-select arm_controller tasks --symlink-install

# 仿真 + 任务
ros2 launch launch_pack arm_task_sim.launch.py task_sequence:=pos_move_demo

# 视觉一条龙
ros2 launch launch_pack arm_task_vision_sim.launch.py

# 实机（先读 docs/execute/04-实机首次运行与标定.md）
ros2 launch launch_pack arm_real.launch.py loopback:=true

# 参数辨识
ros2 launch launch_pack arm_measure_sim.launch.py
```

## 架构

```
ArmController (500 Hz, controller_interface)
  ├── 仿真: ArmController → SimPidController → mujoco_ros2_control/MujocoSystem
  └── 实机: ArmController → ArmRealInterfaces → USB CDC → MCU

ArmSolve → IKSolver → ModelBase (Pinocchio) + TaskMapping
tasks/task_runner（普通线程）→ exp_state / 轨迹话题 → ArmController
```

- 控制器侧 FSM：`idel` / `reset` / `cart_traj` / `joint_traj` / `servo` /
  `admittance` / `teach_pendant` / `measure`。
- 任务层：`tasks.yaml` + 封闭步骤类型（`step_types.cpp` 的 `step_spec()`）。
- 插件：`ArmController`、`SimPidController`、`ArmRealInterfaces`。

## 硬约束（违反会直接出问题）

1. **实时路径**：`ArmController::update()` 与 FSM 的
   `enter/run/check_switch/exit` 禁止 ROS 参数读写、禁止动态内存。
2. **参数声明**：新增控制器参数必须在 `ArmController::on_init` 里
   `auto_declare`，否则 YAML 里的键被静默忽略（C6）。
3. **零位形不要直接做笛卡尔运动**：零位是工作空间边界，目标会被判不可达（C1）。
4. **保持命令必须锁存**：每周期重读实测值会让位置误差恒 0，手臂在重力下坠落（F1/C4）。
5. **保持路径共用重力前馈**：`gravity_compensation` 是唯一主动输出力矩的保持路径；
   实机先关掉，确认方向后从 `scale=0.1` 起加。
6. **不要把辨识输出当作下一轮辨识的先验**：零空间误差会逐轮放大（C7）。
7. **视觉三坑**：`mujoco_plugins` 只能写在 YAML 的 `/**` 段；仿真 Python 节点不要开
   `use_sim_time`；相机订阅必须 `image_qos: reliable`（C8–C10）。
8. **实机 5 自由度**：joint6 无电机、机械锁死；`task_dim=5`；joint6 实际锁死角度必须实测。
9. **位置约束轨迹开关**：`allow_constrained_traj` 与 `real_mode` 已解耦；
   实机默认 `false`，real-compat 验收通过后再显式打开。
10. **断链≠停机**：固件无指令看门狗（R5），物理急停是唯一硬停机手段。

完整 C1–C10 见 [设计约束](docs/adr/2026-11-13/1928-设计约束C1-C10.md)。

## 控制器与求解器

- `ModelFromURDF`（Pinocchio）提供 FK、几何雅可比、RNEA、关节限位；
- `Default6DofTaskSpaceMapping` 负责任务空间语义；改任务空间时同步检查
  `position_map()` 与 `jacobian_map()`；
- `IKSolver` 用信赖域 + LM，返回 `Converged` / `Unreachable` / `Failed`；
  不可达目标在下发前拒绝，控制器回 `idel`；
- 多路点用 C³ 七次 Bezier，1..8 个用户路点，全局时间尺度。

## 修改接口时同步检查

- controller YAML（`src/launch_pack/config/ros2_controller.yaml`）；
- URDF 的 `<ros2_control>` 接口标签；
- 插件 XML（`arm_ctrl_plugin.xml` / `sim_pid_plugin.xml` / `arm_real_plugin.xml`）。

## 已知命名债务

- `Paramter` 是 `Parameter` 的历史拼写（`lib/paramter_identify/`、`ParamterIdentify`）；
- `idel` 是 `idle` 的历史拼写，且是外部参数契约（`exp_state=idel`），改名会破坏命令。
  不要在其他改动里顺手重命名。
