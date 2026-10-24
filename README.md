# RoboticArm

基于 ROS 2 与 `ros2_control` 的 6 自由度机械臂控制系统，支持 MuJoCo 仿真与实机（USB CDC → MCU）两种运行模式，并内置运动学/动力学计算、状态机控制、配置驱动的任务层，以及动力学参数辨识（FIGAROH + CMA-ES）工作流。

> **当前状态**：仿真链路已端到端跑通并验证；实机链路已打通到控制器层，
> **loopback 冒烟通过，尚未在真机上电验证**。
> 缺陷台账、设计约束与排查记录见 [`docs/bugs.md`](docs/bugs.md)——F 系列与 O1–O10 已全部闭环。
> **参数辨识链路可用**：measure 记录 MuJoCo 的真实关节加速度（`acc_*` 列）与仿真时间，
> 辨识出的模型物理可行，参数误差 **0.39%**——见 [O8](docs/bugs.md) 与
> [`src/parameter_identify/README.md`](src/parameter_identify/README.md)。

## 特性

- **双运行模式**：同一上层控制器 `ArmController` 可无缝切换
  - 仿真：`ArmController` → `SimPidController` → `mujoco_ros2_control/MujocoSystem`
  - 实机：`ArmController` → `ArmRealInterfaces` → USB CDC → MCU
- **基于状态机（FSM）的控制**：8 种可切换工作状态（见下方「控制器状态机」）。
- **配置驱动的任务层**：任务由 YAML 里的步骤组合而成，新增目标点与任务类型**不需要写 C++、不需要重新编译**。
- **模型无关的计算库**：`ArmSolve → IKSolver → ModelBase + TaskMapping` 分层，任务空间语义与机器人模型数学解耦。
- **基于 Pinocchio 的刚体动力学**：正运动学、几何雅可比、RNEA 逆动力学、关节限位。
- **轨迹插值**：五次 Hermite/Bezier 段插值，保证位置、速度、加速度连续。
- **动力学参数辨识**：激励轨迹生成（傅里叶级数，可复现且保证关节限位可行）、CSV 测量
  记录（含真实关节加速度）、离线辨识与 URDF 回写。实测参数误差 0.39%、物理校验通过。
- **实时安全约束**：控制热路径严格无 ROS 参数读写、无动态内存分配。
- **失效安全**：逆解失败只保持并回退 `idel`，绝不使控制器链失活；不可达的笛卡尔目标在下发前就被拒绝。

## 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                        ArmController                         │
│        (controller_interface::ControllerInterface)           │
│  读取 URDF 关节名/default_kp/default_kd，每周期驱动 FSM        │
│  输出 position/velocity/effort/kp/kd/ki 命令                  │
│                                                              │
│     FSMArmControlFactory ── 状态机容器 + 共享运行态            │
│        ├── ModelFromURDF (Pinocchio, ModelBase 实现)          │
│        ├── Default6DofTaskSpaceMapping (TaskMapping 实现)     │
│        └── ArmSolve (IKSolver 封装)                           │
└──────────────────────────┬───────────────────────────────────┘
                           │ reference interfaces
              ┌────────────┴────────────┐
              │                         │
      ┌───────▼────────┐       ┌────────▼─────────┐
      │ SimPidController│       │ ArmRealInterfaces │
      │ (链式 PID)      │       │ (SystemInterface)  │
      └───────┬────────┘       │  CDCTrans (libusb) │
              │ effort          └────────┬─────────┘
      ┌───────▼────────┐                 │ packed struct
      │ mujoco_ros2_   │                 ▼
      │ control        │               MCU
      │ /MujocoSystem  │
      └────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                    tasks / task_runner                       │
│  普通 ROS 节点线程（允许阻塞与内存分配）                       │
│  读 config/tasks.yaml → 按步骤驱动上面的控制器                  │
│    exp_state 参数服务 / arm_cart_traj / arm_joint_traj 话题    │
│    /joint_states + TF → 关节角与末端位姿                       │
└─────────────────────────────────────────────────────────────┘
```

控制配置位于 `src/launch_pack/config/ros2_controller.yaml`，默认控制频率 **500 Hz**。

## 目录结构

```
ATNewRoboticArm/
├── src/
│   ├── arm_controller/         # 控制核心包（C++ 共享库 + 插件导出）
│   │   ├── controller/         #   ROS 2 controller 与 hardware interface 插件
│   │   ├── arm/                #   状态机（arm_fsm）与 URDF 模型封装（ModelFromURDF）
│   │   ├── tools/              #   离线排查工具（不安装，见下）
│   │   └── lib/
│   │       ├── calculate/      #   运动学/动力学/轨迹工具（ArmSolve/IKSolver/Trajectory）
│   │       ├── cdc_trans/      #   USB CDC 通信封装（libusb）
│   │       ├── fsm/            #   轻量 FSM 基类与工厂
│   │       ├── paramter_identify/ # 激励轨迹生成与测量（C++ 侧）
│   │       └── executer/       #   DAG 组件执行器实验代码（未接入主库）
│   ├── tasks/                  # 任务层：配置驱动的执行器 + task_runner 入口
│   │   ├── config/             #   任务定义：tasks.yaml（通用）+ measure.yaml（参数辨识）
│   │   ├── executor/           #   步骤类型表与执行器
│   │   ├── context/            #   与控制器交互的 ROS 接口封装
│   │   └── src/task_runner.cpp #   节点入口
│   ├── arm_model/              # URDF/STL/MJCF 模型描述包
│   ├── launch_pack/            # launch 文件、控制器 YAML、RViz 配置
│   ├── mujoco_ros2_control/    # MuJoCo 的 ros2_control 插件（上游 dfki-ric）
│   ├── parameter_identify/     # 离线动力学参数辨识（Python，ament_python）
│   └── robot_msgs/             # 自定义消息
├── third_party/
│   ├── libcmaes/               # CMA-ES 优化库
│   └── figaroh-plus/           # FIGAROH 动力学参数辨识库
├── docs/
│   ├── bugs.md                 # 缺陷台账与设计约束（重要）
│   └── superpowers/            # 任务层重构的历史设计与实施记录（已完成）
├── CLAUDE.md                   # 面向 AI 助手的项目指引
├── AGENT.md                    # arm_controller 包的开发笔记
└── README.md
```

## 软件包

| 包 | 类型 | 说明 |
| --- | --- | --- |
| `arm_controller` | ament_cmake | 控制核心，通过 pluginlib 导出 3 个插件 |
| `tasks` | ament_cmake | 配置驱动的任务层与 `task_runner` 入口 |
| `arm_model` | ament_cmake | 机械臂 URDF 描述、STL 网格、MJCF 场景 |
| `launch_pack` | ament_cmake | launch 文件与控制器配置 |
| `mujoco_ros2_control` | ament_cmake | MuJoCo 仿真硬件插件（第三方） |
| `parameter_identify` | ament_python | 离线动力学参数辨识 CLI（`identify_arm`） |
| `robot_msgs` | ament_cmake | 自定义消息：`CartTrajCmd` / `JointTrajCmd` / `AdmittanceCmd` / `ServoCtrlCmd` / `Vector` / `MotorCmd` / `MotorState` |

### `arm_controller` 导出的插件

| 插件 | 类型 | 说明 |
| --- | --- | --- |
| `arm_controller/ArmController` | `controller_interface::ControllerInterface` | 上层控制器，读取 URDF 关节名与 `default_kp/default_kd`，每周期把状态送入 FSM，输出 `position/velocity/effort/kp/kd/ki` 命令 |
| `arm_controller/SimPidController` | `controller_interface::ChainableControllerInterface` | 链式 PID，接收每关节 6 个 reference interface，向仿真硬件输出 `effort` |
| `arm_controller/ArmRealInterfaces` | `hardware_interface::SystemInterface` | 实机硬件接口，使用 libusb CDC 与 MCU 收发 packed struct 数据包。关节名取自 URDF，**关节数必须等于 MCU 通信帧的电机数（6），否则拒绝启动**；补恒 0 的 `acceleration` 状态接口；`loopback` 参数用于无硬件冒烟；状态包超时 100 ms 返回 ERROR |

## 控制器状态机

状态机由 `FSMArmControlFactory` 注册，初始状态为 `idel`，通过 `exp_state` 参数切换：

| 状态名 | 功能 |
| --- | --- |
| `idel` | 机械臂锁定在当前位置（带重力前馈） |
| `reset` | 机械臂复位（带重力前馈） |
| `cart_traj` | 执行笛卡尔空间轨迹；**下发前先做可达性预检** |
| `joint_traj` | 执行关节空间轨迹 |
| `servo` | 伺服动作，接收速度指令并积分得到期望位置 |
| `admittance` | 导纳控制 |
| `teach_pendant` | 示教器，支持外力拖动、可配置阻尼与重力补偿 |
| `measure` | 系统参数辨识（激励轨迹执行 + 数据记录） |

> `idel` / `reset` 与 `cart_traj` / `joint_traj` / `admittance` 的**保持阶段**都带重力前馈
> （共用 `HoldGravityFeedforward`，见 O6 / O7）；开关是控制器参数 `gravity_compensation`。

运行时切换示例：

```bash
ros2 param set /arm_controller exp_state reset
```

## 任务层（配置驱动）

任务定义在 `src/tasks/config/tasks.yaml`（参数辨识另有 `src/tasks/config/measure.yaml`）。
**新增目标点或新任务只需编辑 YAML**；但新增**步骤类型**要改 C++——下面的词汇表是封闭集合。

### 步骤类型（封闭集合，共 9 类）

| 类型 | 说明 |
| --- | --- |
| `wait` | 不切换控制器状态，纯等待 `duration` 秒 |
| `idel` | 切回 `idel` 并锁定当前位置 |
| `reset` | 复位；等控制器状态回到 `idel` |
| `joint_move` | 关节空间轨迹；用 `to` 引用目标点，或直接给 `target` |
| `cart_move` | 笛卡尔轨迹；`offset` 为相对当前末端位姿的位移（3 维） |
| `admittance_move` | 导纳控制下的期望轨迹；同样用 `offset` |
| `tool` | 末端执行器开关量；`action` 取 `tool` 段定义的语义名 |
| `teach` | 示教；需 `duration` |
| `measure` | 参数辨识；需 `duration` |

常用字段：`duration`（秒）、`tolerance`（到位容差）。
`reset` / `teach` / `measure` 只能从 `idel` 进出，执行器会自动先等待控制器空闲。

### 当前序列

| 序列 | 性质 | 说明 |
| --- | --- | --- |
| `reset_only` | 正常 | 仅复位后回到空闲 |
| `reset_then_joint` | 正常 | 复位后运动到 `ready` 点 |
| `ready_then_cart` | 正常 | 先关节运动离开奇异位形，再笛卡尔平移 |
| `pick_demo` | 正常 | 完整取放示例：吸住 → 等待真空 → 抬升 |
| `reset_then_cart` | **反例，预期失败** | 零位形机械臂完全伸直、处于工作空间边界，目标被判为不可达并在**下发前拒绝**（不产生任何运动） |
| `unreachable_near` | **反例，预期失败** | 从 `ready` 沿 +Z 抬升 5 cm，目标不可达 |
| `unreachable_far` | **反例，预期失败** | 从 `ready` 沿 +X 平移 2 m，远远超出工作空间 |
| `measure_only` | 正常 | **在 `measure.yaml` 里**（不在 `tasks.yaml`）：`idel → measure → idel`，参数辨识测量 |

```bash
# 跑一条序列（退出码反映成败）
ros2 run tasks task_runner --ros-args -p task_sequence:="pick_demo" -p exit_when_idle:=true -p use_sim_time:=true

# 用别的配置文件
ros2 run tasks task_runner --ros-args -p tasks_config:=/path/to/my_tasks.yaml -p task_sequence:="pick_demo"
```

> 三条实测约束（写在 `tasks.yaml` 注释里，数据见 `docs/bugs.md`）：
> 1. **不要从零位形直接做笛卡尔运动**——零位形机械臂完全伸直、处于工作空间边界，
>    目标会被判为不可达并在下发前拒绝；
> 2. **`ready` 位姿处 +Z 不可达**——该位姿已在最大高度（z ≈ 1.233 m）；
> 3. 上面两条的余量都只有 **1~1.3 mm**（判据 1e-3），属于**判据边界**案例。
>    换个目标或换个位姿，可达性会变——**下发前先想清楚目标在不在工作空间里**。

## 依赖

- **ROS 2 Jazzy / Ubuntu 24.04 / Python 3.12**
- `ros2_control`：`controller_interface`、`hardware_interface`、`controller_manager`、`realtime_tools`
- `rclcpp`、`rclcpp_lifecycle`、`pluginlib`
- `Pinocchio`（运动学/动力学）
- `Eigen3`
- `libusb-1.0`（USB CDC 通信）
- `kdl_parser` / `orocos_kdl`
- `yaml-cpp`（任务层配置解析）
- `MuJoCo`（仿真）
- Python：`numpy`、`scipy`、`pinocchio`、`pyyaml`；物理一致性重建还需要
  `picos` + `cvxopt`（缺了会退回 `nullspace`，结果可能非物理，见
  `src/parameter_identify/README.md`）

## 构建

```bash
cd <项目根目录>
colcon build --symlink-install
source install/setup.bash
```

仅构建核心包与任务层（快速验证）：

```bash
colcon build --packages-select arm_controller tasks --symlink-install
```

> 若 `colcon build` 报 `failed to create symbolic link ... Is a directory`，
> 说明 `build/<包名>/ament_cmake_python/...` 下有陈旧目录残留，
> 删掉那个同名目录再构建即可（该目录本应是符号链接）。

## 运行

### 1. 仿真（MuJoCo）——推荐入口

```bash
source install/setup.bash
ros2 launch launch_pack arm_controller_test_sim.launch.py
```

> 控制器激活后会自动执行一次 `reset`（`exp_state: reset`），把机械臂拉到零位并回到 `idel`。

### 2. 实机（控制器层）

```bash
# 无硬件冒烟：不打开 USB 设备，把指令回读成状态
ros2 launch launch_pack arm_real.launch.py loopback:=true

# 接真机
ros2 launch launch_pack arm_real.launch.py
```

> 硬件插件、控制器与 `joint_state_broadcaster` 由同一个 launch 拉起；任务层不在这个文件里，
> 确认 `arm_controller` 激活后再单独启动 `task_runner`。
>
> 仿真与实机共用同一份 `robotic_arm.urdf`：launch 用 `hardware:=real` 切到
> `ArmRealInterfaces`，仿真默认 `hardware:=mujoco`。实机参数在
> `config/ros2_controller_real.yaml` 里**叠加覆盖**仿真默认值（200 Hz、`idel`、
> 仿真增益 20%、重力前馈 0.1 倍）。
>
> 实机模式由 `ArmRealInterfaces` 通过 USB CDC（VID `0x0483` / PID `0x5740`）与 MCU 通信，
> 数据包格式见 `controller/inc/data_pack.h`（`MCUTarget1Pack` / `MCUTarget2Pack` / `MCUStatePack`）。
> 超过 100 ms 收不到状态包会返回 ERROR，由 `controller_manager` 停用控制器。
>
> **实机不支持 measure**：MCU 不上报加速度，`acc_*` 列恒为 0，辨识结果不可用。

### 关于 `/arm_robot_description`

`controller_manager` 是从**话题**（不是参数）取 URDF 的，而同 ROS domain 内任何其他
`robot_state_publisher` 都会让它静默加载错模型。因此本工作区把发布端与订阅端一起
重映射到了 **`/arm_robot_description`**（RViz 配置已同步）。

如果你有外部工具依赖标准的 `/robot_description`，需要改用 `/arm_robot_description`。

## 离线排查工具

`src/arm_controller/tools/` 下的工具**不安装**，构建后在 `build/arm_controller/` 里：

| 工具 | 用途 |
| --- | --- |
| `fk_dump <urdf> <q1> <q2> <q3> <q4> <q5> <q6>` | 打印指定关节角下的平移、四元数、旋转矩阵与任务空间 6 维向量 |
| `ik_solve <urdf> <t0..t5> <s0..s5>` | 用给定目标与初值跑一次逆解，报告 `Converged` / `Unreachable` / `Failed`、残余与耗时 |
| `ik_benchmark` | 批量采样统计收敛率与迭代次数，用于扫描求解器参数 |
| `arm_real_joint_count_test` | 不连硬件校验实机接口的关节数处理（5/6/7 关节） |

## 参数辨识工作流

> ✅ **O8 已修**：测量默认 headless 运行，CSV 直接记录 MuJoCo 的**真实关节加速度**
> （`acc_*` 列）与**仿真时间**，不再对速度二次求导。实测模型残差 31.5% → **0.37%**，
> 辨识参数与真值差 **0.39%**，`physical_validation: passed: true`。
> 详见 [`docs/bugs.md` O8](docs/bugs.md) 与 `src/parameter_identify/README.md`。

参数辨识由三部分协同完成，详细说明见 `src/parameter_identify/README.md`：

1. **激励轨迹生成 + 测量**：控制器 `measure` 状态（`ParamterMeasureState`）生成傅里叶激励轨迹并记录测量数据 CSV；
2. **离线辨识**：`parameter_identify` 包读取 CSV，基于 FIGAROH 辨识动力学参数；
3. **结果回写**：生成更新后的 URDF 与 YAML 报告。

快速流程：

```bash
# 1) 一条命令起仿真并跑测量：CSV 默认落在项目根目录 measured_for_identification.csv
#    （任务序列定义在 tasks/config/measure.yaml，控制器侧由 measure 状态执行）
ros2 launch launch_pack arm_measure_sim.launch.py

#    常用参数：csv_file_path / trajectory_period / trajectory_repeat_cnt /
#    move_to_start_duration / controller_urdf_path（控制器内部模型，只影响激励轨迹
#    生成，被控对象仍是真值 URDF）/ step_timeout（测量变长时要放大）/
#    trajectory_seed（0 = 随机不可复现，非 0 = 固定；控制器日志会打出实际种子）
#    测量默认 headless（关掉 MuJoCo UI 窗口，纯命令行跑）；想要界面加 headless:=false

# 2) 离线辨识
ros2 run parameter_identify identify_arm \
  --csv measured_for_identification.csv \
  --urdf src/arm_model/model/robotic_arm.urdf \
  --output /tmp/robotic_arm_identified.urdf \
  --config src/parameter_identify/config/identify.yaml \
  --report /tmp/robotic_arm_identify_report.yaml
```

## 开发约定

- 优先保持 `TaskMapping` 承担任务空间语义，`ModelBase` 承担机器人模型数学。
- 修改任务空间时，同步检查 `position_map()` 与 `jacobian_map()`。
- 修改控制器接口时，同步检查 controller YAML、URDF `ros2_control` 接口与插件 XML。
- **新增控制器参数时三处同步**：`ArmController::on_init` 里的 `auto_declare`、controller YAML、读取处。
  控制器只读取**已声明**的参数，YAML 里未声明的键会被静默忽略并取默认值，写错名字不会有任何提示。
- 修改后优先构建验证：`colcon build --packages-select arm_controller tasks`；
  改动涉及参数辨识时再加上 `parameter_identify`。

### ros2_control 实时约束

- `ArmController::update()` 是高频实时控制路径，禁止读写 ROS 参数、禁止动态内存操作。
- FSM 的 `enter()/run()/check_switch()/exit()` 均可能被 `update()` 间接调用，同属实时控制路径；
  状态机内需要的内存只能在构造函数或控制器非实时初始化阶段申请。
- FSM 的 `run()` 必须只读已缓存数据并写控制命令，不能调用 `get_parameter()`、不能构造临时
  `std::vector`、不能 `resize/reserve/assign/push_back`。
- FSM 的 `enter()` 只能重置标量状态、拷贝传感器/命令值到预分配 buffer、切换标志位。
- 运行时可更新的数组参数（如 `default_kp/default_kd/reset_joint_pos`）长度不得超过初始化时的
  `joints` 数量，避免状态切入时触发重新分配。

## 第三方库

以下第三方依赖以内嵌仓库方式放置在项目内：

- [`libcmaes`](https://github.com/CMA-ES/libcmaes)：CMA-ES 优化算法。
- [`figaroh-plus`](https://github.com/thanhndv212/figaroh-plus)：FIGAROH 动力学参数辨识。
- [`mujoco_ros2_control`](https://github.com/dfki-ric/mujoco_ros2_control)：MuJoCo 的 ros2_control 插件。
