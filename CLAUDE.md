# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于 ROS 2 (Jazzy) + `ros2_control` 的 6 自由度机械臂控制系统，支持 MuJoCo 仿真与实机（USB CDC → MCU）两种运行模式。内置 Pinocchio 运动学/动力学计算、控制器侧状态机、配置驱动的任务层，以及动力学参数辨识（FIGAROH + CMA-ES）工作流——**辨识部分当前不可用（O8）**。

> **现状**：仿真链路已端到端跑通并验证；**实机从未运行过**（仅仿真）。
> 缺陷台账、设计约束与排查记录见 [`docs/bugs.md`](docs/bugs.md)——**动手前先看它**，
> 里面记了 6 条必须遵守的设计约束（零位形在工作空间边界、保持命令必须锁存、参数必须
> `auto_declare` 等）。F 系列与 O1–O7 已闭环，**O8（参数辨识数据一致性）未修**。

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

# 任务层：跑一条在 tasks.yaml 里定义的序列（退出码反映成败）
ros2 run tasks task_runner --ros-args -p task_sequence:="pick_demo" -p exit_when_idle:=true -p use_sim_time:=true

# 任务层：换一份任务配置文件
ros2 run tasks task_runner --ros-args -p tasks_config:=/path/to/my_tasks.yaml -p task_sequence:="pick_demo"

# 切换控制器工作状态
ros2 param set /arm_controller exp_state reset

# 参数辨识：一条命令起仿真 + 跑 measure_only 序列，结束后 launch 自动退出
# CSV 默认落在项目根目录：measured_for_identification.csv（不再是 /tmp）
ros2 launch launch_pack arm_measure_sim.launch.py

# 离线辨识
ros2 run parameter_identify identify_arm \
  --csv measured_for_identification.csv \
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

### 两层控制：控制器侧 FSM + 任务层配置驱动执行器

两层**不再**是形态相同的状态机。控制器侧是真正的 FSM（实时路径），任务层已重构为
**配置驱动的步骤执行器**（普通线程）：

| | 控制器侧 FSM | 任务层执行器 |
|---|---|---|
| 位置 | `src/arm_controller/arm/` | `src/tasks/` |
| 基类 | `FSM`（`lib/fsm/fsm.hpp`），接口 `enter/run/check_switch/exit` | 无 FSM 基类；`task_executor` 按步骤表顺序执行 |
| 运行线程 | `ArmController::update()` 实时路径 | 普通 ROS 节点线程 |
| 约束 | **禁止** ROS 参数读写、动态内存分配 | 允许阻塞服务调用与内存分配 |
| 职责 | 单状态控制逻辑（idel/reset/cart_traj/...） | 读 YAML，按 9 类步骤驱动控制器 |
| 扩展方式 | 新增 C++ 状态类 | **只改 YAML，不写 C++、不重新编译** |

任务定义在 `src/tasks/config/tasks.yaml`，结构为 `tool` / `points` / `sequences`。
步骤类型是**封闭集合**（`wait` / `idel` / `reset` / `joint_move` / `cart_move` /
`admittance_move` / `tool` / `teach` / `measure`），类型属性表在
`src/tasks/executor/step_types.cpp` 的 `step_spec()`——加新**类型**要改 C++，
加新**任务**只改 YAML。

任务层通过 `TaskContext` 与控制器交互：
- `exp_state` 参数服务 → 切换控制器工作状态
- `/arm_cart_traj`、`/arm_joint_traj`、`/arm_admittance` 话题 → 下发轨迹指令
- `/joint_states` 订阅 + TF 查询 → 获取关节角与末端位姿

`task_runner` 的 `task_sequence` 参数是**序列名字符串**（如 `"pick_demo"`），不是步骤列表。
进程默认常驻，改写该参数即可再跑一条；加 `exit_when_idle:=true` 让它在序列结束后退出
（退出码反映成败）。

> `tasks.yaml` 里有几条**预期失败**的反例序列：`reset_then_cart`（零位形在工作空间边界）、
> `unreachable_near` / `unreachable_far`（目标不可达）。它们退出码为 1 是正确行为。
> 注意这几条的失败方式都是「**下发前被拒绝、完全不运动**」，不是「逆解跑到一半失败」。

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

- **O8（唯一未修缺陷，高）**：参数辨识链路已跑通，但辨识不出物理可行的模型。**根因已定位**：
  辨识用的加速度是对 500 Hz 记录速度二次求导得到的，相对误差 **31.5%**（换任何平滑参数都
  只能到 20~34%）。模型与力矩都验证无误——反解加速度满足 `‖M·a + bias − τ‖ = 1.4e-16`，
  用模型正演的完美力矩能精确复原真值（残差 5.5e-14），`cond(W_base) = 138` 激励也充分。
  后果：基参数偏离先验 18.5%，`sdp` 不可行；投影路线虽能写出文件却把连杆退化成质点。
  **修法**：让 measure 把加速度记录进 CSV（`acc_*` 列 + `acceleration_source: recorded`，
  格式与 loader 都已支持）。详见 `docs/bugs.md` O8。
- `src/launch_pack/launch/` 下有**三个** launch 引用了不存在的包、当前不可用：
  `arm_real.launch.py` / `arm_static_display.launch.py`（引用 `arm`、`arm_calc`、`arm_task`、
  `robot_driver`、`vision`）、`arm_mujoco_sim.launch.py`（引用 `arm_calc`）。都是从旧版本遗留的，
  **实机路径尚未打通**。可用入口只有 `arm_controller_test_sim.launch.py` 与
  `arm_measure_sim.launch.py`。
- `src/arm_controller/lib/executer/` 是 DAG 组件执行器实验代码，未接入主库
- **没有自动化测试框架**。只有 `src/arm_controller/tools/` 下 4 个不安装的离线工具
  （`fk_dump` / `ik_solve` / `ik_benchmark` / `arm_real_joint_count_test`），
  构建后在 `build/arm_controller/` 里，用于排查与离线校验。
- **控制器状态的序列覆盖不全**：`tasks.yaml` 现有 7 条序列只用到
  `reset` / `idel` / `joint_move` / `cart_move` / `tool` / `wait` 六类步骤。
  `measure` 另有专用配置 `tasks/config/measure.yaml` + `launch_pack/arm_measure_sim.launch.py`，
  **已端到端验证**（2026-10-10：5000 样本、无 WARN/ERROR，CSV 落在项目根目录）。
  仍未验证的是 `admittance`（对应 `admittance_move` 步骤）与 `teach_pendant`（`teach`）；
  `servo` 连步骤类型都没有，只能手动设 `exp_state` 进入。

### 两个容易踩的运维细节

- **`robot_description` 话题被改名了。** `controller_manager` 是从**话题**（不是参数）
  取 URDF 的，同 ROS domain 内任何其他 `robot_state_publisher` 都会让它静默加载错模型。
  因此发布端与订阅端一起重映射到了 **`/arm_robot_description`**（RViz 配置已同步）。
  用 `ros2 topic echo /robot_description` 看不到东西是正常的。
- **`gravity_compensation` 参数**（`ros2_controller.yaml`，默认 `true`）是**所有保持
  路径**的重力前馈开关：`reset` / `idel` 与 `write_hold_command()`（`cart_traj` /
  `joint_traj` / `admittance` 的保持阶段）共用 `HoldGravityFeedforward`（`arm_fsm.hpp`）。
  这是实机上唯一一处**主动输出力矩**的保持路径，怀疑模型质量或力矩符号时先关掉它
  （关掉后复位会有约 0.124 rad 静差并触发 WARN）。

### 修改控制器接口时

同步检查三处：controller YAML（`ros2_controller.yaml`）、URDF `ros2_control` 接口标签、插件 XML。

### 新增控制器参数时

**必须在 `ArmController::on_init` 里 `auto_declare`**，否则 YAML 里写了也读不到——
控制器只读取已声明的参数，未声明的键会被静默忽略并取默认值，**写错参数名不会有任何提示**。
关键参数建议在构造阶段打一行 INFO 把实际生效值打出来（`gravity_compensation` 就是这么做的）。
详见 `docs/bugs.md` C6。

## 第三方库

- `third_party/libcmaes/`：CMA-ES 优化库（已就位，CMake 自动构建）
- `third_party/figaroh-plus/`：FIGAROH 动力学参数辨识（已就位，有完整内容）
- `src/mujoco_ros2_control/`：MuJoCo 的 ros2_control 插件（独立 git 仓库）

### `third_party/` 的两个坑（都会让 `git status` 显示"已修改"，但成因不同）

两个目录都是以 **gitlink（嵌入 git 仓库）**方式记录的，项目里**没有 `.gitmodules`**，
所以它们不是标准 submodule。两处"已修改"**都不是我改的**，也都不该在无关提交里带上。

**1. `figaroh-plus` —— 悬空 gitlink**

| | 提交 |
| --- | --- |
| 父仓库索引记录 | `8899853f80720d27e9656d658a6e2851d3f856dd` |
| 工作区实际 HEAD | `96b586f` |

关键是：**索引里记的那个提交在本地根本不存在**（`git cat-file -e 8899853` 失败；
该仓库不是浅克隆，也不是任何本地 ref 的后代）。所以「把工作区切回索引记录的状态」
这条常规做法**在这里做不到**，除非能从远端取回那个提交（也可能已被 force-push 掉了）。

现实的选择只有一个方向：**把 gitlink 更新到 `96b586f` 并提交**（即承认当前工作区状态），
或者维持现状不提交。

**2. `libcmaes` —— 构建依赖的未提交补丁**

gitlink 本身是**一致**的（索引与工作区都是 `6a53cd5`），但仓库**内部**有一处未提交改动：

```diff
-if (Boost_FOUND AND Python3_NumPy_FOUND)
+if (LIBCMAES_BUILD_PYTHON AND Boost_FOUND AND Python3_NumPy_FOUND)
```

这处补丁**构建依赖它**：`arm_controller/CMakeLists.txt` 在 `add_subdirectory(libcmaes)` 之前
设了 `LIBCMAES_BUILD_PYTHON OFF`，没有这个补丁那个开关就不生效，Python 绑定仍会被构建。
**它随时可能因为重新 clone / 重置而丢失，丢失后构建行为会变。** 要么把它固化下来
（提交进 libcmaes 或改用别的方式传参），要么明确记录它是有意为之。
