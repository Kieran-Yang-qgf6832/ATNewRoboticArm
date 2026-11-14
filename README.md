# RoboticArm

基于 ROS 2（Jazzy）与 `ros2_control` 的 6 自由度机械臂控制系统，支持 **MuJoCo 仿真**与
**实机（USB CDC → MCU）** 两种运行模式。内置 Pinocchio 运动学/动力学计算、控制器侧状态机、
配置驱动的任务层，以及基于 FIGAROH + CMA-ES 的动力学参数辨识工作流。

> **当前状态**
> - 仿真链路端到端验证通过（含 MuJoCo 深度相机与红色目标检测）；
> - 实机链路 loopback 冒烟通过，**尚未真机上电验证**；上电前必须先做 real-compat 5 自由度验收；
> - F1–F12、O1–O16 已闭环（O1 已证伪），R1–R4、R6 已处理；**R5 固件无指令看门狗仍未实现**（拔线≠停机）。

## 特性

- **双运行模式**：同一上层控制器 `ArmController`
  - 仿真：`ArmController → SimPidController → mujoco_ros2_control`
  - 实机：`ArmController → ArmRealInterfaces → USB CDC → MCU`
- **控制器侧 FSM**：`idel` / `reset` / `cart_traj` / `joint_traj` / `servo` /
  `admittance` / `teach_pendant` / `measure`，通过 `exp_state` 切换。
- **配置驱动任务层**：任务写在 `src/tasks/config/tasks.yaml`，新增任务不需要改 C++；
  步骤类型是封闭集合，集中在 `src/tasks/executor/step_types.cpp`。
- **模型无关的计算分层**：`ArmSolve → IKSolver → ModelBase + TaskMapping`。
- **轨迹**：五次 Hermite/Bezier 段插值；多路点用 C³ 七次 Bezier，支持 1..8 个用户路点。
- **参数辨识**：激励轨迹 + 真实 `qacc` 测量 + FIGAROH 辨识 + 物理投影，参数误差约 0.39%。
- **视觉链路**：HSV → 四边形 → 深度分面 → PnP，发布 `TargetFace` 与目标面 TF。
- **实时安全**：500 Hz 热路径不做 ROS 参数读写、不做动态内存分配；不可达目标在下发前拒绝。
- **实机兼容**：5 自由度（joint6 锁死）、关节标定、real-compat 仿真与断链检查清单。

## 架构

```
ArmController (500 Hz, controller_interface)
  ├── 仿真: ArmController → SimPidController → mujoco_ros2_control/MujocoSystem
  └── 实机: ArmController → ArmRealInterfaces → USB CDC (libusb) → MCU

ArmSolve → IKSolver → ModelBase (Pinocchio) + TaskMapping
tasks/task_runner（普通 ROS 线程）→ exp_state / 轨迹话题 → ArmController
```

控制配置：`src/launch_pack/config/ros2_controller.yaml`（仿真 500 Hz）；
实机差异用 `ros2_controller_real.yaml` 叠加覆盖（100 Hz、`task_dim=5`、`real_mode=true`、
`allow_constrained_traj=false`、重力前馈关闭）。

## 快速开始

### 构建

```bash
cd <项目根目录>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 仿真

```bash
# 仿真 + 任务（推荐入口，默认 Qt joint_console）
ros2 launch launch_pack arm_task_sim.launch.py

# 任意序列
ros2 launch launch_pack arm_task_sim.launch.py task_sequence:=pos_move_demo

# 带视觉的抬高场景 + box_move
ros2 launch launch_pack arm_task_vision_sim.launch.py
```

### 实机

> **先读 [实机首次运行与标定](docs/execute/04-实机首次运行与标定.md)**：USB 权限、loopback、
> 关节标定、Phase 3.5 real-compat 验收、Phase 6.5 断链验证。未标定前只允许 `idel`。

```bash
# 无硬件冒烟
ros2 launch launch_pack arm_real.launch.py loopback:=true

# 接真机
ros2 launch launch_pack arm_real.launch.py
```

> 仿真与实机启动控制器时都自动录 rosbag（`/joint_states` + `/arm_controller/joint_command`），
> 默认落在项目根的 `.ros/ros2bags/`；`record_bag:=false` 关闭。话题清单与回放见
> [02 仿真运行](docs/execute/02-仿真运行.md#关节命令状态录制rosbag)。

### 参数辨识

```bash
ros2 launch launch_pack arm_measure_sim.launch.py
ros2 run parameter_identify identify_arm \
  --csv measured_for_identification.csv \
  --urdf src/arm_model/model/robotic_arm.urdf \
  --output /tmp/robotic_arm_identified.urdf \
  --config src/parameter_identify/config/identify.yaml \
  --report /tmp/robotic_arm_identify_report.yaml
```

## 文档地图

| 入口 | 内容 |
| --- | --- |
| [docs/README.md](docs/README.md) | 职责矩阵、单一事实源与维护规则 |
| [docs/execute/](docs/execute/README.md) | 环境、构建、仿真/实机运行、标定、任务层、参数辨识、排错 |
| [docs/adr/](docs/adr/README.md) | 技术选型、设计原因、备选方案与决策 |
| [docs/bugs/](docs/bugs/README.md) | 缺陷的完整生命周期（发现、分析、修复、验证） |
| [docs/changelog/](docs/changelog/README.md) | 有意义的版本/功能/兼容性变化 |
| [CLAUDE.md](CLAUDE.md) | AI 助手的工作规则与导航 |

> `docs/teach/` 是本地教学材料，被 `.git/info/exclude` 排除，不随仓库分发。

## 软件包

| 包 | 类型 | 说明 |
| --- | --- | --- |
| `arm_controller` | ament_cmake | 控制核心，导出 `ArmController` / `SimPidController` / `ArmRealInterfaces` 三个插件 |
| `tasks` | ament_cmake | 配置驱动任务层与 `task_runner` |
| `arm_model` | ament_cmake | URDF、STL、MJCF 场景（含 real-compat） |
| `launch_pack` | ament_cmake | launch、控制器 YAML、RViz 配置 |
| `mujoco_ros2_control` | ament_cmake | MuJoCo 的 ros2_control 插件（第三方） |
| `parameter_identify` | ament_python | 离线参数辨识 CLI（`identify_arm`） |
| `view` | ament_python | 红色目标检测、仿真相机 TF、数值验收 |
| `robot_msgs` | ament_cmake | `CartTrajCmd` / `JointTrajCmd` / `AdmittanceCmd` / `ServoCtrlCmd` / `WaypointTrajCmd` 等 |

## 第三方库

- [`libcmaes`](https://github.com/CMA-ES/libcmaes)：CMA-ES 优化库。
- [`figaroh-plus`](https://github.com/thanhndv212/figaroh-plus)：FIGAROH 动力学参数辨识。
- [`mujoco_ros2_control`](https://github.com/dfki-ric/mujoco_ros2_control)：MuJoCo 的 ros2_control 插件。

## 开发约定

实时约束、参数声明、接口同步与文档维护约定见
[开发约定](docs/execute/08-开发约定.md) 与 [设计约束 C1–C10](docs/adr/2026-11-13/1928-设计约束C1-C10.md)。
