# 多路点 C3 平滑轨迹 Spec

## Why

当前任务层的每条轨迹只有两个点（当前实测位姿 → 目标），A→B→C→D 必须写成三个步骤：控制器每段都减速到零、锁存保持，任务层再等一段固定时间才发下一段。用户需要一次传入多个路点，由控制器自动规划成**一条**位置、速度、加速度、jerk 均连续（C3）的轨迹，中途路点不停顿。

## What Changes

- 新增消息 `robot_msgs/msg/WaypointTrajCmd.msg`：`space`（joint / cartesian）、`total_duration`（≤0 表示自动）、`Vector[] waypoints`。
- 新增控制器侧规划库 `src/arm_controller/lib/calculate/waypoint_trajectory.{hpp,cpp}`：分段七次 Hermite、自动时间分配、全轨迹采样校验，可在实时路径外一次算完。
- `JointTrajState` 新增 `/arm_joint_waypoints` 订阅；`CartTrajState` 新增 `/arm_cart_waypoints` 订阅（逐点 IK → 关节路点 → 同一套关节空间规划）。复用现有 `joint_traj` / `cart_traj` 状态、保持与 `idel` 中断语义，**不新增 FSM 状态**。
- 控制器新增 3 个 `auto_declare` 参数：`multi_traj_max_vel` / `multi_traj_max_acc` / `multi_traj_max_jerk`（`vector<double>`，长度 1 或关节数），速度再与 URDF `velocity` 取 min。
- 任务层新增两个步骤类型 `joint_waypoints` / `cart_waypoints`，步骤内联 `waypoints`，`duration` 可选（总时长覆盖）。
- 新增离线规划器校验工具 `waypoint_planner_test` 与仿真验收工具 `waypoint_check`，并在 `tasks.yaml` 中新增 16 条验证序列（关节 / 笛卡尔 × 1..8 个用户路点）。
- **BREAKING**: 无。现有消息、参数、步骤类型与行为全部保持不变。

## Impact

- Affected specs: 控制器轨迹状态（`joint_traj` / `cart_traj`）、任务层步骤词汇表、`robot_msgs` 消息定义。
- Affected code:
  - `src/robot_msgs/msg/WaypointTrajCmd.msg`、`src/robot_msgs/CMakeLists.txt`
  - `src/arm_controller/lib/calculate/waypoint_trajectory.{hpp,cpp}`
  - `src/arm_controller/arm/arm_fsm.{hpp,cpp}`（两个状态新增订阅与规划分支）
  - `src/arm_controller/controller/src/controller.cpp`（参数声明与生效值日志）
  - `src/arm_controller/tools/waypoint_planner_test.cpp`
  - `src/tasks/executor/step_types.{hpp,cpp}`、`src/tasks/executor/task_executor.{hpp,cpp}`
  - `src/tasks/context/task_context.{hpp,cpp}`
  - `src/tasks/config/tasks.yaml`
  - `src/tasks/tools/waypoint_check.py`、`src/tasks/CMakeLists.txt`
  - `CLAUDE.md`（新增步骤类型、参数与验收入口说明）

### 本次不做（范围外）

- 不做碰撞检测 / 自碰撞约束（仅做关节限位校验）。
- 不做笛卡尔空间 C3 样条（笛卡尔路点只用于逐点 IK，段间形状由关节空间样条决定）。
- 不支持中间路点停顿；需要停顿请拆成两个步骤。
- 不新增 FSM 状态，不改 `kMaxArmTrajectoryPoints`（现有两消息的容量不变）。

## ADDED Requirements

### Requirement: 多路点输入与容量

系统 SHALL 允许每个步骤传入 1–8 个用户路点；控制器内部自动补一个起点后，规划器最多处理 9 个节点。

#### Scenario: 合法路点数
- **WHEN** 步骤给出 1 到 8 个路点
- **THEN** 步骤通过语法与语义校验并下发

#### Scenario: 路点数越界
- **WHEN** 步骤给出 0 个或超过 8 个路点
- **THEN** 任务层在加载配置时报出明确错误（含实际数量与上限），不执行任何运动

#### Scenario: 路点维度错误
- **WHEN** `joint_waypoints` 的某个路点不是关节数（当前 6）个数值，或 `cart_waypoints` 的某个路点不是 7 个数值
- **THEN** 任务层拒绝该步骤并指出第一个出错路点的下标

### Requirement: 自动补起点

系统 SHALL 在规划前自动把当前实测位姿作为第一个节点：关节模式取 `/joint_states` 的实测关节角，笛卡尔模式取当前末端位姿（与现有 `joint_move` / `pos_move` 的起点口径一致）。

#### Scenario: 用户首点远离当前位姿
- **WHEN** 用户给出的第一个路点与当前位姿不同
- **THEN** 轨迹从当前位姿出发并在起点的速度、加速度、jerk 均为零，t=0 无跳变

### Requirement: C3 平滑

系统 SHALL 用分段七次 Hermite 构造轨迹：每段在两端同时匹配位置、速度、加速度、jerk；相邻段共享同一组节点导数，因此所有内部节点处 C3 连续；起点与终点的速度、加速度、jerk 均为 0，中间路点速度非零、直接穿过。

#### Scenario: 内部节点连续性
- **WHEN** 轨迹在内部路点 B 处从 AB 段切换到 BC 段
- **THEN** B 点左右两侧的位置、速度、加速度、jerk 在数值精度内相等（解析校验阈值 1e-9）

#### Scenario: 单路点
- **WHEN** 用户只给 1 个路点
- **THEN** 轨迹退化为“当前位姿 → 目标”的一条七次曲线，与现有 `joint_move` 一样单次加/减速，但 jerk 连续

### Requirement: 自动时间分配与限位

系统 SHALL 根据关节速度、加速度、jerk 限位自动分配每段时长；速度限位取 `multi_traj_max_vel` 与 URDF `velocity` 的较小值，加速度与 jerk 限位取新参数；所有参数在构造期读取并打一行日志给出实际生效值。

#### Scenario: 默认自动时长
- **WHEN** 步骤未写 `duration`
- **THEN** 规划器迭代求解每段最短时长（导数峰值按 1/T、1/T²、1/T³ 缩放），使采样后的速度、加速度、jerk 均不超限，并据此执行

#### Scenario: duration 覆盖
- **WHEN** 步骤写了 `duration` 且不小于自动最短总时长
- **THEN** 所有段按同一比例缩放，总时长等于 `duration`，且仍满足限位

#### Scenario: duration 过短
- **WHEN** 步骤写的 `duration` 小于自动最短总时长
- **THEN** 整条轨迹在运动前被拒绝，日志给出自动最短时长与用户值

#### Scenario: 曲线位置过冲超限
- **WHEN** 规划曲线在路点之间的采样点超出关节限位（时间缩放无法修复位置过冲）
- **THEN** 整条轨迹在运动前被拒绝，日志指出关节与越界位置

### Requirement: 笛卡尔路点转换

系统 SHALL 对每个笛卡尔路点（`[x, y, z, qx, qy, qz, qw]`）依次求解 IK：第一个路点用当前实测关节角作种子，后续路点用前一个路点的解作种子；四元数在求解前归一化。

#### Scenario: 全部路点可达
- **WHEN** 所有路点 IK 收敛且关节角在限位内
- **THEN** 得到关节路点序列，转入关节空间 C3 规划

#### Scenario: 任一路点不可达
- **WHEN** 某个路点 IK 失败或解超限位
- **THEN** 整条轨迹在运动前被拒绝，日志指出第几个路点与残余误差

#### Scenario: 实机模式
- **WHEN** 控制器 `real_mode=true` 且收到 `cart_waypoints`
- **THEN** 与现有 `pos_move` / `box_move` 一致，在下发任何运动指令前拒绝

### Requirement: 失败原子性

系统 SHALL 在开始运动前完成全部校验（路点数与维度、IK、关节限位、曲线采样限位、时长可行性）；任一项失败都不产生任何运动，控制器回到 `idel`，任务步骤以明确错误失败。

#### Scenario: 预检失败
- **WHEN** 规划阶段任一项校验失败
- **THEN** 机械臂保持原位，控制器切回 `idel`，任务层输出失败原因与失败路点

### Requirement: 任务层步骤与 YAML

系统 SHALL 新增两个步骤类型：

- `joint_waypoints`：`exp_state = joint_traj`，`requires_idel = false`，`duration` 可选；
- `cart_waypoints`：`exp_state = cart_traj`，`requires_idel = false`，`duration` 可选。

两个类型都只接受 `waypoints`、`duration`、`tolerance` 字段。

```yaml
- type: joint_waypoints
  waypoints:
    - [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    - [0.4, 0.3, -0.2, 0.0, 0.2, 0.0]
    - [0.2, 0.1, 0.3, 0.1, 0.1, 0.2]
  duration: 6.0        # 可选，总时长覆盖

- type: cart_waypoints
  waypoints:
    - [0.45, 0.00, 0.35, 0.0, 1.0, 0.0, 0.0]
    - [0.45, 0.10, 0.30, 0.0, 1.0, 0.0, 0.0]
    - [0.50, 0.10, 0.25, 0.0, 1.0, 0.0, 0.0]
  tolerance: 0.01      # 可选，终点到位容差（米）
```

#### Scenario: 未知字段
- **WHEN** 步骤里出现 `waypoints` / `duration` / `tolerance` 之外的字段
- **THEN** 与现有步骤一致，配置加载直接报错

#### Scenario: 关节限位
- **WHEN** 关节路点或规划曲线超出 URDF 关节限位
- **THEN** 任务层或控制器在运动前拒绝（以控制器校验为准）

### Requirement: 仿真验收

系统 SHALL 提供：

1. 离线规划器校验工具：解析检查节点处导数连续性、起止导数是否为零、全轨迹限位与时长可行性；
2. 仿真验收工具：订阅 `/joint_states`，对每条序列检查终点/路点经过误差、关节限位与速度/加速度/jerk 的有限差分峰值；
3. `tasks.yaml` 中 16 条验证序列：关节与笛卡尔各覆盖 1、2、3、4、5、6、7、8 个用户路点。

#### Scenario: 全路点数验收
- **WHEN** 在 MuJoCo 仿真中依次执行 16 条序列
- **THEN** 每条序列退出码为 0，验收工具通过：关节路点误差 ≤ 0.01 rad，笛卡尔终点位置误差 ≤ 5 mm，全程无关节限位越界，速度/加速度/jerk 无超过配置限位 10% 的尖峰

## MODIFIED Requirements

### Requirement: joint_traj / cart_traj 状态输入

两个状态在原有“逐点采样轨迹”消息之外，SHALL 额外接受多路点消息：`joint_traj` 接受 `/arm_joint_waypoints`，`cart_traj` 接受 `/arm_cart_waypoints`。只在 `STOP` 阶段接受新消息；运动中仍只响应 `exp_state=idel` 的急停；轨迹结束后锁存终点并保持。原有 `JointTrajCmd` / `CartTrajCmd` 路径行为不变。

### Requirement: 步骤词汇表

`step_types.cpp` 的封闭步骤表 SHALL 增加 `joint_waypoints` 与 `cart_waypoints` 两项；字段白名单增加 `waypoints`；`Step` 结构增加路点列表字段。现有步骤类型的属性与行为不变。
