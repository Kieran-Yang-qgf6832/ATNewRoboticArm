# RoboticArm

基于 ROS 2 与 `ros2_control` 的 6 自由度机械臂控制系统，支持 MuJoCo 仿真与实机（USB CDC → MCU）两种运行模式，并内置运动学/动力学计算、状态机控制、配置驱动的任务层，以及动力学参数辨识（FIGAROH + CMA-ES）工作流。

> **当前状态**：仿真链路已端到端跑通并验证（含 MuJoCo 深度相机 + 红色目标检测）；
> 实机链路已打通到控制器层，**loopback 冒烟通过，尚未在真机上电验证**。
> 缺陷台账、设计约束与排查记录见 [`docs/bugs.md`](docs/bugs.md)——F 系列与 O1–O14 已全部闭环。
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
- **视觉链路（MuJoCo 验证）**：`scene_vision.xml` 提供一台固定深度相机（640×480 / fovy 60°）
  与边长 0.35 m 的红色目标箱，`view` 包执行 HSV 分割 → 四边形拟合 → 深度分面 → PnP，
  发布 `TargetFace` 与目标面 TF；`ros2 run view check_sim_vision` 把结果经 TF 转到
  `base_link` 与真值比对（实测面心 1.37 mm、法线 0.11°、箱心 1.17 mm）。
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
ATNewRoboticArm_Upper/
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
│   ├── view/                   # 红色目标视觉检测、仿真相机 TF 与数值验收（Python）
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
| `view` | ament_python | 红色目标视觉检测（`red_quad_detector`）、仿真相机 TF（`view_sim.launch.py`）与数值验收（`check_sim_vision`） |
| `robot_msgs` | ament_cmake | 自定义消息：`CartTrajCmd` / `JointTrajCmd` / `AdmittanceCmd` / `ServoCtrlCmd` / `Vector` / `MotorCmd` / `MotorState` / `TargetFace` |

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

> `cart_traj` / `joint_traj` 在**运动过程中**也响应 `exp_state=idel`：轨迹立即中止并就地保持。
> 这是 `tasks` 的 `joint_console` 里 `stop` 的实现基础；切到**其他**状态仍要求轨迹走完（STOP）。

运行时切换示例：

```bash
ros2 param set /arm_controller exp_state reset
```

## 任务层（配置驱动）

任务定义在 `src/tasks/config/tasks.yaml`（参数辨识另有 `src/tasks/config/measure.yaml`）。
**新增目标点或新任务只需编辑 YAML**；但新增**步骤类型**要改 C++——下面的词汇表是封闭集合。

### 步骤类型（封闭集合，共 12 类）

| 类型 | 说明 |
| --- | --- |
| `wait` | 不切换控制器状态，纯等待 `duration` 秒 |
| `idel` | 切回 `idel` 并锁定当前位置 |
| `reset` | 复位；等控制器状态回到 `idel` |
| `joint_move` | 关节空间轨迹；用 `to` 引用目标点，或直接给 `target` |
| `joint_console` | 终端交互式关节运动；运行时**逐项输入**（一次一个数）：6 个关节角（**度**）→ 执行时间（秒，回车取默认）；回车取消本组、`exit` 结束、运动中 `stop` 立即停止。步骤写 `ui: true` 时改为 Qt5 窗口 |
| `cart_move` | 笛卡尔轨迹；`offset` 为相对当前末端位姿的位移（3 维），严格保持姿态 |
| `pos_move` | 把末端移到基座系绝对位置（`to` / `position`）；可选 `orientation` 约束 link6 z 轴方向 |
| `box_move` | 读取视觉目标面 TF（默认 `target_face`），工具 z 对齐面 z、工具尖端到面心；5D 预检不可达时自动回退到只到位置 |
| `admittance_move` | 导纳控制下的期望轨迹；同样用 `offset` |
| `tool` | 末端执行器开关量；`action` 取 `tool` 段定义的语义名 |
| `teach` | 示教；需 `duration` |
| `measure` | 参数辨识；需 `duration` |

常用字段：`duration`（秒）、`tolerance`（到位容差）。
`reset` / `joint_console` / `teach` / `measure` 只能从 `idel` 进出，执行器会自动先等待控制器空闲。

`pos_move` 与 `cart_move` 的区别：`cart_move` 用相对 `offset` 且**严格保持起点姿态**；
`pos_move` 用 `positions` 段的基座系 `[x, y, z]` 绝对目标（或直接给 `position`），
默认**姿态不参与 IK 收敛判据**。`CartTrajCmd` 的 `mode` 字段按轨迹下发
`MODE_POSITION_ONLY`（默认 `MODE_FULL_POSE=0`，旧发布者行为不变，`Vector` 仍是 6 维）：
控制器在 `TaskMapping` 里显式投影掉 3 个旋转误差/雅可比行，IK 只解位置，
并用零空间「就近」偏好抑制姿态乱动；预检覆盖终点加有限等分中间点。
任务层结束时量测姿态漂移，超过 `max_drift_deg`（默认见 `position_move` 段）
**只告警、不判失败**。`real_mode: true` 时该模式在下发前被拒绝（实机 5 自由度、
尚未验证）。控制器参数 `pos_move_tolerance` / `pos_move_nullspace_weight` /
`pos_move_precheck_samples` 在构造阶段读取并打 INFO（C6），实测整定见 `docs/bugs.md` §9。

`pos_move` 还可选 `orientation: [x, y, z, w]`（基座系四元数）：只取它的 z 轴，
按轨迹下发 `MODE_POSITION_AND_TOOL_AXIS`，即位置 3 维 + link6 z 方向 2 维、
绕 z 滚转自由；z 轴角误差超过 `pos_move_axis_tolerance_deg`（默认 1°）判失败，
不再告警全姿态漂移。目标姿态以起点实测姿态为基准、按「起点 z → 目标 z 的最小旋转」
重建；两者反平行（180°）时直接用给定四元数消歧。`box_move` 复用这条路径。

`box_move` 读取 `base_link -> target_face` 的 TF，同时检查 `/target_face` 的 `valid`、
`child_frame_id` 与消息/TF 时间戳新鲜度；拿到一帧后**快照**。5D 目标位置 =
面心 − `tool_offset` × 面 z（工具尖端到面心），工具 z 与面 z 同向。若控制器预检在
**未运动**时拒绝，自动回退到 position-only（同一 link6 位置、姿态不约束）并打 WARN；
运动中途失败不回退。全局默认在 `tasks.yaml` 的 `box_move` 段
（`frame` / `tool_offset` / `target_timeout` / `max_age`），步骤可覆盖 `frame` /
`tool_offset` / `tolerance`。

`joint_console` 的输入先过 `tasks.yaml` 的 `joint_limits` 段（每关节限位，度）与
`max_step_deg`（单步最大变化量，相对当前实测角），超限只提示重输、不下发任何指令。

### 当前序列

| 序列 | 性质 | 说明 |
| --- | --- | --- |
| `reset_only` | 正常 | 仅复位后回到空闲 |
| `reset_then_joint` | 正常 | 复位后运动到 `ready` 点 |
| `ready_then_cart` | 正常 | 先关节运动离开奇异位形，再笛卡尔平移 |
| `pick_demo` | 正常 | 完整取放示例：吸住 → 等待真空 → 抬升 |
| `joint_console_only` | 正常 | 交互式：终端逐行输入关节角（度）+ 可选执行时间，自由试位姿 |
| `pos_move_demo` | 正常 | 只约束位置的绝对目标：从 `ready` 向下 5 cm；实测位置残差 0.0009 m，姿态漂移被量测并告警 |
| `pos_move_direct` | 正常 | 同上，但直接给 `position` 坐标并覆盖 `max_drift_deg` |
| `pos_move_near` | **反例，预期失败** | +Z 5 cm：放松姿态仍受连杆长度/关节限位限制，position-only 残余约 0.031 m，仍被下发前拒绝 |
| `pos_axis_demo` | 正常 | `pos_move` 的 `orientation`：先保持 ready 方向下移 5 cm，再回到 ready 并把工具 z 抬高 15° |
| `box_move_demo` | 正常 | **抬高视觉场景**：`joint_move` 到 `box_pregrasp` 粗定位，再由 `box_move` 做最后约 10 cm 的视觉 5D 移动（位置残差 0.0015 m、工具轴误差 0.25°） |
| `box_move_low` | 正常（回退） | **低处视觉场景**：5D 预检拒绝后自动回退 position-only；实测残差约 1.25 cm（演示容差 2 cm） |
| `joint_console_ui` | 正常 | `joint_console` 的 Qt5 窗口版：6 个角度输入 + 执行时间 + Execute/Stop/Exit，实时当前角/限位 |
| `reset_then_cart` | **反例，预期失败** | 零位形机械臂完全伸直、处于工作空间边界，目标被判为不可达并在**下发前拒绝**（不产生任何运动） |
| `unreachable_near` | **反例，预期失败** | 从 `ready` 沿 +Z 抬升 5 cm，目标不可达 |
| `unreachable_far` | **反例，预期失败** | 从 `ready` 沿 +X 平移 2 m，远远超出工作空间 |
| `measure_only` | 正常 | **在 `measure.yaml` 里**（不在 `tasks.yaml`）：`idel → measure → idel`，参数辨识测量 |

### Joint Console Qt UI

`joint_console` 步骤写 `ui: true` 时不再读终端，而是打开 Qt5 窗口：6 个目标角度（度）、
执行时间、Execute/Stop/Exit/Use current，实时显示当前角度与各关节限位。推荐直接走
下面的「一条龙 launch」（默认就是 `joint_console_ui`）；已有仿真时也可以只起任务节点：

```bash
ros2 run tasks task_runner --ros-args -p task_sequence:="joint_console_ui" -p use_sim_time:=true
```

- 需要 `qtbase5-dev`（Ubuntu 24.04，本机已安装）与图形会话（`DISPLAY` 或
  `WAYLAND_DISPLAY`）；无显示时步骤失败并提示改用终端版 `joint_console_only`。
- 窗口输入通过 `ConsoleQueue` 注入，限位、单步、起点实测与 `stop` 语义和终端版
  **完全共用**；Execute 在运动中禁用，Stop 立即切 `idel` 就地保持，关闭窗口在
  运动中按 Stop 处理。快捷键：`Alt+E` 执行、`Alt+S` 停止、`Alt+X` 退出、
  `Alt+C` 取当前角。

### 一条龙 launch（推荐）

```bash
# 仿真 + 任务：默认打开 Qt joint_console
ros2 launch launch_pack arm_task_sim.launch.py

# 任意序列，任务结束后整个 launch 自动关闭
ros2 launch launch_pack arm_task_sim.launch.py task_sequence:=pos_move_demo

# 视觉一条龙：抬高场景 + 相机 TF/检测 + box_move_demo
ros2 launch launch_pack arm_task_vision_sim.launch.py

# 低处回退验证
ros2 launch launch_pack arm_task_vision_sim.launch.py \
  task_sequence:=box_move_low \
  mujoco_model:=$(ros2 pkg prefix arm_model)/share/arm_model/model/scene_vision.xml
```

`arm_task_sim.launch.py` 的 `exit_when_idle` 默认 `true`：任务/UI 退出后仿真一起关闭；
想常驻换序列传 `exit_when_idle:=false`，之后用
`ros2 param set /task_runner task_sequence "..."`。
视觉 launch 会把同一个 `mujoco_model` 同时传给仿真和 `view_sim` 的 `scene_path`，
避免相机 TF 与渲染场景不一致。

> 用过 Qt UI 的 `task_runner` 在退出时走 `std::_Exit` 硬退出：Qt/xcb 的 atexit
> 清理会卡住进程，导致 `ros2 launch` 一直等不到它结束。硬退出后 launch 能正常关停；
> 正常（无 UI）序列仍走 `rclcpp::shutdown()`。

`ros2 run` 仍可用于高级/实机场景，单独起任务链：

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
- 视觉：`python3-opencv`、`numpy`；实机相机驱动 `realsense2_camera`

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

> **第一次上真机请先读 [`docs/real_first_run.md`](docs/real_first_run.md)**：从构建、USB 权限、无硬件冒烟，
> 到关节标定（`dir` / `zero`）与小步运动验证的完整命令清单，未标定前只能停在 `idel`。

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
> `config/ros2_controller_real.yaml` 里**叠加覆盖**仿真默认值（100 Hz、`idel`、
> `real_mode: true`、`task_dim: 5`、重力前馈关闭）。
>
> 实机模式由 `ArmRealInterfaces` 通过 USB CDC（VID `0x0483` / PID `0x5741`）与 MCU 通信，
> 数据包格式是**固件 ABI 的字节级镜像**：`controller/inc/data_pack.h` 里的
> `MCUTargetPack`（78 B，下行）/ `MCUStatePack`（77 B，上行），帧头 `int32 0x01`。
> 77 字节帧在 USB FS 上会被拆成 64 + 13，接口内部自行拼帧。
> 超过 100 ms 收不到状态包会返回 ERROR，由 `controller_manager` 停用控制器。
>
> **上真机前必须标定关节**：固件角度零位与 URDF 零位没有共同基准，符号也可能相反。
> 换算参数在 `arm_controller/config/joint_calibration.yaml`（`q_urdf = dir·(q_raw − zero)`），
> 标定工具 `build/arm_controller/joint_calibrate`（不安装）。流程见
> [`docs/bugs.md` §8.1](docs/bugs.md)。
>
> 实机只有 5 个自由度（joint6 无电机、机械锁死），所以任务空间降为 5 维
> （位置 + 工具指向，不控绕工具轴滚转），`teach_pendant` 与 `measure` 在实机模式下被拒绝。

### 3. 视觉链路（仿真）

MuJoCo 场景 `scene_vision.xml` 里有一台固定深度相机和一个边长 0.35 m 的红色目标箱。
三个终端分别负责：仿真、相机 TF + 检测、数值验收。

```bash
# 1) 用视觉场景启动仿真（默认仍是 scene.xml，参数辨识共用，不能加箱子）
ros2 launch launch_pack arm_controller_test_sim.launch.py \
    mujoco_model:=$(ros2 pkg prefix arm_model)/share/arm_model/model/scene_vision.xml

# 2) 仿真相机 TF（解析 MJCF 里的相机位姿）+ 检测节点
ros2 launch view view_sim.launch.py            # display:=false 可关预览

# 3) 数值验收：退出码 0 = 通过
ros2 run view check_sim_vision
```

- 相机安装位姿只有 `scene_vision.xml` 一个来源，`view_sim.launch.py` 启动时直接解析
  `<camera pos/xyaxes>`，再固定补一段绕 X 180° 的 optical 帧（MuJoCo 相机是 X 右 / Y 上 / −Z 前）。
- 检测节点在仿真里**不开 `use_sim_time`**：`/clock` 是 500 Hz，rclpy 的 TimeSource 会吃掉
  约一个核（O13）；改用图像 `header.stamp` 作唯一时间基准（`stamp_from_image:=true`）。
- 图像订阅必须 `image_qos:=reliable`：相机插件是 `depth=1` 的 reliable 发布，best-effort
  订阅会丢掉约 2/3 的帧，深度过期后分面退化为单面（O14）。
- 实机检测仍用 `ros2 launch view view.launch.py`（RealSense），与仿真完全分离；
  该 launch 的默认参数未改动。

`box_move` 的 5D 演示需要把箱体和相机一起抬高：`scene_vision_raised.xml` 是
`scene_vision.xml` 的抬高变体（箱体/相机 z=0.6），`view_sim.launch.py` 增加
`scene_path` 参数。**仿真和相机 TF 必须指向同一个场景文件**，否则 TF 与渲染不一致：

```bash
# 抬高场景：5D box_move 演示（box_move_demo）
ros2 launch launch_pack arm_controller_test_sim.launch.py headless:=true \
    mujoco_model:=$(ros2 pkg prefix arm_model)/share/arm_model/model/scene_vision_raised.xml
ros2 launch view view_sim.launch.py display:=false \
    scene_path:=$(ros2 pkg prefix arm_model)/share/arm_model/model/scene_vision_raised.xml
ros2 run tasks task_runner --ros-args -p task_sequence:="box_move_demo" \
    -p exit_when_idle:=true -p use_sim_time:=true

# 低处场景：position-only 回退验证（box_move_low）
ros2 launch launch_pack arm_controller_test_sim.launch.py headless:=true \
    mujoco_model:=$(ros2 pkg prefix arm_model)/share/arm_model/model/scene_vision.xml
ros2 launch view view_sim.launch.py display:=false
ros2 run tasks task_runner --ros-args -p task_sequence:="box_move_low" \
    -p exit_when_idle:=true -p use_sim_time:=true
```

> `box_move` 会检查 `TargetFace.header.stamp` 与 TF 的新鲜度，仿真里 `task_runner`
> **必须带 `use_sim_time:=true`**；否则任务层用墙钟比较仿真时间戳，会一直超时。
> 低处箱体已接近该臂实际可达边界：5D 会超关节限位，回退 position-only 后残差约
> 1.25 cm，`box_move_low` 用 2 cm 演示容差；干净演示请用抬高场景。

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
| `ik_solve <urdf> <t0..t5> <s0..s5>` | 用给定目标与初值跑一次逆解；`--axis` 为位置 + 工具 z 的 5 维模式，`--position` 为只约束位置的 3 维模式；报告 `Converged` / `Unreachable` / `Failed`、残余与耗时 |
| `ik_benchmark` | 批量采样统计收敛率与迭代次数，用于扫描求解器参数 |
| `arm_real_joint_count_test` | 不连硬件校验实机接口的关节数处理（5/6/7 关节） |

`src/view` 另有一套 pytest（55 项，覆盖 HSV 分割、四边形拟合、深度分面、目标跟踪与节点发布），
直接跑：

```bash
cd src/view && python3 -m pytest -q
```

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
