# RoboticArm 项目笔记

`src/arm_controller` 是机械臂控制核心包，构建共享库 `arm_controller`，
通过 pluginlib 导出控制器和硬件接口插件。

## 控制链路

- 仿真：`ArmController` -> `SimPidController` -> `mujoco_ros2_control/MujocoSystem`
- 实机：`ArmController` -> `ArmRealInterfaces` -> USB CDC -> MCU
- 控制配置在 `src/launch_pack/config/ros2_controller.yaml`，仿真默认 500 Hz；
  实机差异在 `config/ros2_controller_real.yaml` 叠加覆盖（100 Hz、`idel`、
  位置增益 20%、**重力前馈关闭**，`gravity_compensation_scale: 0.1` 已备好但未启用）。
- 仿真与实机共用同一份 `robotic_arm.urdf`，`arm_real.launch.py` 传
  `hardware:=real` 切换 `<ros2_control>` 块；`loopback:=true` 不打开 USB 设备，
  只用于无硬件冒烟。

## 插件

- `arm_controller/ArmController`：上层控制器。读取 URDF 关节名和默认
  `default_kp/default_kd`，每周期把状态送入 FSM，再输出
  `position/velocity/effort/kp/kd/ki` 命令。
- `arm_controller/SimPidController`：链式 PID 控制器。接收每关节 6 个
  reference interface，向仿真硬件输出 `effort`。
- `arm_controller/ArmRealInterfaces`：实机 `hardware_interface::SystemInterface`。
  关节名从 URDF 的 `<ros2_control>` 段读取，使用 libusb CDC 与 MCU 收发 packed struct
  数据包。MCU 通信帧固定为 6 个电机，**关节数与帧长不符时 `on_init` 直接拒绝启动**
  （越界读写会把垃圾数据当指令发给电机，见 `docs/bugs.md` O3）。
  导出 `position/velocity/effort` 状态与**恒 0 的 `acceleration`**（控制器无条件请求该接口）；
  `loopback` 参数跳过 USB 打开、把指令回读成状态；超过 `state_timeout_ms`（默认 100 ms）
  没收到状态包时 `read()` 返回 ERROR，让 `controller_manager` 停用控制器。

## 主要目录

- `controller/`：ROS 2 controller 与 hardware interface 插件实现。
- `arm/`：机械臂控制状态机和 URDF 模型封装。
- `lib/calculate/`：运动学、动力学、轨迹工具。
- `lib/cdc_trans/`：USB CDC 通信封装。
- `lib/fsm/`：轻量 FSM 基类与工厂。
- `lib/paramter_identify/`：激励轨迹生成（`ExcitationTrajectory` / `FourierTrajectory`）
  与测量记录（`ParamterIdentify`）。
- `lib/executer/`：DAG 组件执行器实验代码，当前未接入主库。

## 计算库

`ArmSolve -> IKSolver -> ModelBase + TaskMapping`。
`ModelFromURDF` 基于 Pinocchio 提供 FK、几何雅可比、RNEA 和关节限位；
`IKSolver` 用**信赖域 + Levenberg-Marquardt** 迭代求 IK，返回三态结果
`Converged` / `Unreachable` / `Failed`——`Unreachable` 表示迭代结束（停滞或次数用尽）
时残余仍超判据，`last_error_norm()` 给出「最近可达位姿离目标还有多远」；
`Trajectory` 使用五次 Hermite/Bezier 段插值位置、速度和加速度。

## 与安全相关的三处实现细节

- **保持命令必须锁存**。`latch_joint_position()` 只在**进入保持时调用一次**，之后每周期
  下发同一个固定目标；若每周期重读实测值，位置误差恒为 0，`kp` 再大也不产生恢复力，
  机械臂会在重力下坠落（`docs/bugs.md` F1）。
- **所有保持路径共用重力前馈**（`HoldGravityFeedforward`，`arm_fsm.hpp`）。用 RNEA 在
  `dq = ddq = 0` 下算出重力力矩写入 `command.torque`：没有它时 `reset` / `idel` 在
  零位姿下 joint4 的静差约 0.124 rad、远超 `reset_tolerance`，复位永远不达标（O6）；
  `cart_traj` / `joint_traj` / `admittance` 的保持路径则会在等待轨迹到达的约 0.9 s 里
  塌陷约 0.089 rad，笛卡尔运动的起点因此不是指令要求的位姿（O7）。开关是参数
  `gravity_compensation`，**必须先在 `ArmController::on_init` 里 `auto_declare`**，
  否则 YAML 里写了也读不到（C6）。buffer 只在状态构造阶段分配，`run()` 内不分配内存。
- **笛卡尔轨迹下发前先做可达性预检**。`cart_traj` 在 MOVING 的首个周期对轨迹**终点**
  做一次逆解，不收敛就整条拒绝、**一个运动指令都不下发**（O2）。只查终点是因为单次
  逆解最坏约 386 µs，而控制周期是 2 ms。

## 参数辨识（measure 状态）——已可用

`ParamterMeasureState`（`measure`）生成傅里叶激励轨迹、跟踪它，并把
`position/velocity/acceleration/effort` 与**仿真时间**记进 CSV。加速度不再是「对记录速度
二次求导」，而是直接读 MuJoCo 的 `qacc`（`mujoco_ros2_control` 导出 `acceleration`
状态接口，O8），因此辨识结果物理可行：

- 好激励下参数误差 **0.0011%**、力矩预测误差 **0.0003%**、`physical_validation` 通过；
  默认配置实测参数误差 **0.39%**（O8 修复前是 11.56%）。
- 激励轨迹带碰撞/地面/力矩可行性约束（O11、O12），生成约 50 s，measure launch 的
  `step_timeout` 默认已提到 240 s。
- **不要把辨识输出当下一轮辨识的先验**：可辨识的只有 36 个基参数，其余落在零空间、
  完全由先验填充，反复喂回去会让误差单调放大（30 轮 0.29% → 272.7%，C7）。

详见 `docs/bugs.md` 的 O8 / O10–O12 与 `src/parameter_identify/README.md`。

## 离线排查工具

`tools/` 下 4 个工具**不安装**，构建后在 `build/arm_controller/` 里：

- `fk_dump <urdf> <q1> <q2> <q3> <q4> <q5> <q6>`：打印 FK 的平移/四元数/旋转矩阵与任务空间 6 维向量
- `ik_solve <urdf> <t0..t5> <s0..s5>`：单次逆解，报告三态结果、残余、迭代次数与耗时
- `ik_benchmark`：批量采样统计收敛率与迭代次数，用于扫描求解器参数
- `arm_real_joint_count_test`：不连硬件校验实机接口的关节数处理（5/6/7 关节）

## 开发约定

- 优先保持 `TaskMapping` 承担任务空间语义，`ModelBase` 承担机器人模型数学。
- 改任务空间时同步检查 `position_map()` 与 `jacobian_map()`。
- 改控制器接口时同步检查 controller YAML、URDF `ros2_control` 接口和插件 XML。
- 修改后优先构建验证：`colcon build --packages-select arm_controller`。

## ros2_control 实时约束

- `ArmController::update()` 是高频实时控制路径，禁止读写 ROS 参数，禁止动态内存操作。
- FSM 的 `enter()/run()/check_switch()/exit()` 都可能由 `ArmController::update()` 间接调用，
  因此都属于实时控制路径；状态机内需要的内存只能在构造函数或控制器非实时初始化阶段申请。
- FSM 的 `run()` 必须只读已缓存数据并写控制命令，不能调用 `get_parameter()`、
  不能构造临时 `std::vector`，不能 `resize/reserve/assign/push_back`。
- FSM 的 `enter()` 只能重置标量状态、拷贝传感器/命令值到已预分配 buffer、切换标志位；
  不能读取 ROS 参数、创建对象、启动线程或执行可能分配内存/阻塞的计算。
- FSM 构造函数或控制器初始化阶段负责读取 `joints`、确定关节数、预分配状态缓存和参数缓存。
- `FSMArmControlFactory` 只保留 FSM 注册、共享运行态和 `node_` 等基础上下文；
  不缓存各 FSM 私有参数。新增或删除 FSM 时，不应为了私有参数修改 Factory 成员。
- 运行时可更新的数组参数，如 `default_kp/default_kd/reset_joint_pos`，长度必须不超过初始化时的
  `joints` 数量，避免状态切入时触发重新分配。
