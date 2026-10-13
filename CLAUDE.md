# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于 ROS 2 (Jazzy) + `ros2_control` 的 6 自由度机械臂控制系统，支持 MuJoCo 仿真与实机（USB CDC → MCU）两种运行模式。内置 Pinocchio 运动学/动力学计算、控制器侧状态机、配置驱动的任务层，以及动力学参数辨识（FIGAROH + CMA-ES）工作流。

> **现状**：仿真链路已端到端跑通并验证；**实机从未运行过**（仅仿真）。
> 缺陷台账、设计约束与排查记录见 [`docs/bugs.md`](docs/bugs.md)——**动手前先看它**，
> 里面记了 7 条必须遵守的设计约束（零位形在工作空间边界、保持命令必须锁存、参数必须
> `auto_declare`、**不要把辨识输出当下一轮先验** 等）。F 系列与 O1–O11 已全部闭环。

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

- **O8 已修**（`5e59ef5`）：辨识用的加速度原本是对 500 Hz 记录速度二次求导得到的
  （相对误差 31.5%），把辨识结果推出了物理可行集。现在 measure **直接记录 MuJoCo 的真实
  `qacc`**（`mujoco_ros2_control` 新导出 `acceleration` 状态接口 → `ArmController` 读进
  `state_acceleration_` → `ParamterIdentify` 写 `acc_*` 列），时间列也从墙钟换成**仿真时间**。
  实测：模型残差 31.5% → **0.37%**，参数误差 11.56% → **0.39%**，伪惯量最小特征值
  −0.151 → −1.1e-10（物理校验通过）。
  注意 SDP 仍会判不可行——真值模型本身贴在可行集边界上（link5 伪惯量最小特征值 6.4e-6），
  所以默认 `physical_projection: true`。详见 `docs/bugs.md` O8。
  收尾还修了一处**采样时刻对齐**（`e08d942`）：MuJoCo 的 `qacc` 对应**步进前**的状态，
  而 `qpos/qvel` 是步进后的，直接配对让模型残差停在 0.25%；把位置/速度延后一拍配对后
  降到 **0.0003%**。
- **C7（做参数辨识前必读）**：**不要把辨识输出当成下一轮辨识的先验。** 可辨识的只有 36 个
  基参数，剩下 24 维落在零空间、完全由先验填充，所以把输出喂回去等于把误差原样搬运且只增
  不减。30 轮实测参数误差 **0.29% → 272.7%**、总质量 6.80 → 18.39 kg，而 **rmse 全程都很好**
  （1e-4 ~ 2.2），光看 rmse 发现不了。同一份数据换成"真值 URDF 当先验"只差 **0.0002%**。
  详见 `docs/bugs.md` C7。
- **O9 已修**（`00001b8`）：`arm_measure_sim.launch.py` 的覆盖参数（`csv_file_path` /
  `trajectory_period` / …）一度**全部静默失效**——launch 把它们写成 node key 为 `/**` 的
  params 文件，而 controller_manager 只转发 `arm_controller.*` 前缀的项。现在改用
  `OpaqueFunction` 生成 node key 为 `arm_controller` 的补充文件。**验证参数是否生效要查
  目标节点（`ros2 param get /arm_controller <name>`）或结果（CSV 行数），不能只看"文件里写了"。**
- **O10 已修**（`06af06d`）：激励轨迹生成原本不可复现（`CMAParameters` 的 seed 传 0，
  libcmaes 的语义是"用当前时间"），且 `k_max_fevals = 600` 对 36 维变量严重欠收敛，
  于是返回的"最优"轨迹**每次都违反软限位**（实测 21/21 次，违例 0.119 ~ 6.675 rad），
  指令远超行程把关节甩出硬限位（joint6 行程 15.4 rad、速度峰值 1064 rad/s），数据报废。
  现在：种子做成参数 `measure_trajectory_seed`（0 = 随机，非 0 = 固定可复现）、
  评估次数提到 1500、sigma 降到 0.25，并加 `find_feasible_scale()` 用二分把系数整体
  缩放到**保证可行**。详见 `docs/bugs.md` O10。
- **O11 已修**（`340e002`）：激励轨迹**没避开地面**——link6 的工具网格会钻进地面 4.6 mm
  （1.0% 的样本），MuJoCo 的地面接触冲量让模型残差出现 **81.9 N·m** 的尖峰（每 10 s 一次），
  参数误差从 0.001% 恶化到 **9.56%**。约束原本只卡末端 **frame 原点**，而 link6 的网格比
  原点低 **0.0569 m**。现在：
  - `measure_end_effector_tool_offset`（新，默认 0.07 m）——地面净空卡在**工具最低点**上；
  - `measure_traj_torque_limit_rate`（新，默认 0.6）——RNEA 算指令轨迹需要的关节力矩，
    超过 `rate × effortLimit(50 N·m)` 就惩罚（原轨迹有 10.3% 的样本 >30 N·m）；
  - `measure_end_effector_z_lower_limit` 0.1 → 0.20。
  实测 seed=1：残差 1.53% → **0.30%**（最大 81.9 → **0.26 N·m**），参数误差
  **9.5565% → 0.0011%**，激励得分只降 2%。**这两个参数的实际生效值会由控制器日志的
  `激励约束：` 那一行打出来**，改完记得去看（C6 的教训）。详见 `docs/bugs.md` O11。
  > 验证辨识结果**不要看报告里的 `rmse`**——它量的是最小二乘基参数，而写进 URDF 的是
  > 投影之后的参数，实测可以差几十倍。用 `src/parameter_identify/tools/verify_torque.py`
  > 拿最终 URDF 正演一遍（训练集 + 另一条轨迹 + 与真值模型对比 + 残差与 |dq| 的相关性）。
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
  **已端到端验证**（2026-10-10：5000 样本、无 WARN/ERROR，CSV 落在项目根目录；
  2026-10-12 追加验证 120 s 长测量：59948 行、141 s）。
  该 launch 支持 `csv_file_path` / `controller_urdf_path` / `trajectory_period` /
  `trajectory_repeat_cnt` / `move_to_start_duration` / `trajectory_seed` / `step_timeout`
  覆盖（见 O9、O10）。
  仍未验证的是 `admittance`（对应 `admittance_move` 步骤）与 `teach_pendant`（`teach`）；
  `servo` 连步骤类型都没有，只能手动设 `exp_state` 进入。

### 无界面运行（headless）

MuJoCo 自带的 UI 窗口可以关掉，纯命令行跑：`headless` 是 **mujoco_ros2_control 的硬件参数**，
写在 URDF 的 `<ros2_control><hardware>` 块里（`<param name="headless">`），不是控制器参数。
URDF 用 xacro 声明了同名 arg（默认 `false`），两个 launch 都把它透传出去：

- `arm_measure_sim.launch.py`：`headless` **默认 true**（测量不需要界面）；
- `arm_controller_test_sim.launch.py`：默认 false，想要无界面加 `headless:=true`。

headless 时走 `HeadlessAdapter`，不开窗口、不起渲染线程，物理线程照常 500 Hz 推进；
日志里是 `Running in HEADLESS mode.`（不再有 `Starting the MuJoCo rendering thread...`）。
实测同一 seed 下 headless 与带界面生成**完全相同**的激励轨迹，数据链路不受影响。

> `sim_speed_factor`（同一处的硬件参数，2.0 = 2 倍速）可以跑得比实时快。O8 修好之后
> CSV 的 `time` 列已经是**仿真时间**、加速度也是直接记录的，所以它不再会污染辨识数据——
> 但**尚未实测**（控制周期会从 2 ms 墙钟压到 1 ms，本来就偶发的丢拍可能变多）。

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
