# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于 ROS 2 (Jazzy) + `ros2_control` 的 6 自由度机械臂控制系统，支持 MuJoCo 仿真与实机（USB CDC → MCU）两种运行模式。内置 Pinocchio 运动学/动力学计算、双层状态机控制、以及动力学参数辨识（FIGAROH + CMA-ES）完整工作流。

## 常用命令

### 构建

```bash
colcon build --symlink-install
source install/setup.bash

# 仅构建核心包（快速验证）
colcon build --packages-select arm_controller --symlink-install
colcon build --packages-select arm_controller parameter_identify --symlink-install
```

### 运行

```bash
# MuJoCo 仿真（控制器 + RViz）
ros2 launch launch_pack arm_controller_test_sim.launch.py

# 任务层：复位后执行笛卡尔轨迹
ros2 run tasks task_runner --ros-args -p task_sequence:="[reset, cart_traj]" -p use_sim_time:=true

# 任务层：复位后运动到指定关节角
ros2 run tasks task_runner --ros-args -p task_sequence:="[reset, joint_traj]" \
  -p joint_traj_target_positions:="[0.0, 0.3, -0.3, 0.0, 0.3, 0.0]"

# 切换控制器工作状态
ros2 param set /arm_controller exp_state reset

# 参数辨识：先录制测量 CSV，再离线辨识
ros2 param set /arm_controller exp_state measure
ros2 run parameter_identify identify_arm \
  --csv /tmp/measured_for_identification.csv \
  --urdf src/arm_model/model/robotic_arm.urdf \
  --output /tmp/robotic_arm_identified.urdf \
  --config src/parameter_identify/config/identify.yaml \
  --report /tmp/robotic_arm_identify_report.yaml
```

### 代码风格

- **clang-format**：LLVM 基础风格，140 列宽，4 空格缩进（见 `.clang-format`）
- **clang-tidy**：见 `.clang-tidy`（CLion 导出的大量检查规则）
- **ament_lint**：`ament_lint_auto` 已启用，但 copyright 和 cpplint 检查被跳过
- C++17 标准（CMake 设置），编译选项 `-Wall -Wextra -Wpedantic`

## 架构

### 控制链路

```
ArmController (controller_interface, 500 Hz)
  ├── 仿真: ArmController → SimPidController → mujoco_ros2_control/MujocoSystem
  └── 实机: ArmController → ArmRealInterfaces → USB CDC (libusb) → MCU
```

控制配置在 `src/launch_pack/config/ros2_controller.yaml`，默认 500 Hz。

### 双层状态机

项目有两层独立的状态机，接口形态相同（`enter/run/check_switch/exit`），但语义和约束完全不同：

| | 控制器侧 FSM | 任务层 FSM |
|---|---|---|
| 位置 | `src/arm_controller/arm/` | `src/tasks/` |
| 基类 | `FSM`（`lib/fsm/fsm.hpp`） | `TaskFSM`（`tasks/fsm/task_fsm.hpp`） |
| 运行线程 | `ArmController::update()` 实时路径 | 普通 ROS 节点线程 |
| 约束 | **禁止** ROS 参数读写、动态内存分配 | 允许阻塞服务调用与内存分配 |
| 职责 | 单状态控制逻辑（idel/reset/cart_traj/...） | 任务编排，通过 ROS 接口驱动控制器切换状态 |

任务层通过 `TaskContext` 与控制器交互：
- `exp_state` 参数服务 → 切换控制器工作状态
- `/arm_cart_traj`、`/arm_joint_traj` 话题 → 下发轨迹指令
- `/joint_states` 订阅 + TF 查询 → 获取关节角与末端位姿

任务链由 `task_runner` 节点按 `task_sequence` 参数串联，自动以 `idel` 结尾。

### 计算库分层

```
ArmSolve → IKSolver → ModelBase + TaskMapping
```

- `ModelFromURDF`（Pinocchio 实现 `ModelBase`）：FK、几何雅可比、RNEA、关节限位
- `Default6DofTaskSpaceMapping`（实现 `TaskMapping`）：任务空间语义与模型数学解耦
- `Trajectory`：五次 Hermite/Bezier 段插值，保证位置/速度/加速度连续

**约定**：修改任务空间时同步检查 `position_map()` 与 `jacobian_map()`。

### arm_controller 导出的插件

| 插件 | 类型 | 说明 |
| --- | --- | --- |
| `arm_controller/ArmController` | `ControllerInterface` | 上层控制器，每周期驱动 FSM |
| `arm_controller/SimPidController` | `ChainableControllerInterface` | 链式 PID，向仿真输出 effort |
| `arm_controller/ArmRealInterfaces` | `SystemInterface` | 实机硬件接口，USB CDC 通信 |

插件 XML：`arm_ctrl_plugin.xml`、`sim_pid_plugin.xml`、`arm_real_plugin.xml`。

### 控制器侧 FSM 状态（8 种）

`idel`（锁定）、`reset`（复位）、`cart_traj`（笛卡尔轨迹）、`joint_traj`（关节轨迹）、`servo`（速度积分伺服）、`admittance`（导纳控制）、`teach_pendant`（示教拖动）、`measure`（参数辨识测量）。

注册在 `arm_fsm_factory.hpp`，通过 `exp_state` 参数切换。

## 重要约束与注意事项

### ros2_control 实时约束

`ArmController::update()` 是 500 Hz 实时控制路径，FSM 的 `enter()/run()/check_switch()/exit()` 均可能被其间接调用：

- **禁止**调用 `get_parameter()` 等 ROS 参数读写
- **禁止**动态内存操作：不能构造临时 `std::vector`，不能 `resize/reserve/assign/push_back`
- FSM 内需要的内存只能在构造函数或控制器初始化阶段申请
- `enter()` 只能重置标量状态、拷贝数据到预分配 buffer、切换标志位
- 运行时可更新的数组参数（`default_kp/default_kd/reset_joint_pos`）长度不得超过初始化时的 `joints` 数量

### 已知问题

- `third_party/figaroh-plus/` 是空目录，使用前需手动 clone：`git clone https://github.com/thanhndv212/figaroh-plus.git third_party/figaroh-plus`
- `src/launch_pack/launch/arm_real.launch.py` 和 `arm_static_display.launch.py` 引用了不存在的包（`arm`、`arm_calc`、`arm_task`、`robot_driver`、`vision`），是从旧版本遗留的，当前不可用
- `src/arm_controller/lib/executer/` 是 DAG 组件执行器实验代码，未接入主库
- 项目无测试

### 修改控制器接口时

同步检查三处：controller YAML（`ros2_controller.yaml`）、URDF `ros2_control` 接口标签、插件 XML。

## 第三方库

- `third_party/libcmaes/`：CMA-ES 优化库（已就位，CMake 自动构建）
- `third_party/figaroh-plus/`：FIGAROH 动力学参数辨识（需手动 clone）
- `src/mujoco_ros2_control/`：MuJoCo 的 ros2_control 插件（独立 git 仓库）
