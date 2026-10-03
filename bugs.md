# 机械臂项目问题清单（bugs）

> 整理时间：2026-10-03
> 范围：`src/arm_controller`（控制器/状态机/解算）、`src/tasks`（任务层）、`src/launch_pack`（启动与配置）
> 依据：三次 `task_cart_traj` 实跑日志（`task_traj.log`、`task_cart.log` ×2）、一次 `ros2 run` 复跑、`/joint_states` 与 `ros2 control list_controllers -v` 实测、以及用 pinocchio 复刻控制器求解器的数值实验。
> 结论摘要：**当前最致命的问题是 ①——控制器的 `reset` 状态永远判定不出"复位完成"，于是 FSM 永久卡住并静默忽略所有 `exp_state` 请求，所有任务"看起来下了指令、实际一条都没执行"。** 笛卡尔解算本身的问题（②）已被数值实验证实，但因 ① 阻塞尚未在真实链路上复现。

---

## 0. 问题之间的阻塞关系

```
① reset 永不完成 → FSM 卡死 → 所有任务指令被静默吞掉
                      │
                      └─→ ② 笛卡尔 IK 的姿态误差度量在 |rv|≈π 处失效（已被复刻实验证实，未在真实链路复现）
                            │
                            └─→ ③ IK 失败被当成致命错误 → 整条控制器链被停用 → 机械臂坠落
                                      │
                                      └─→ ④ 任务层把上述致命故障降级为 WARN + 退出码 0（故障被掩盖）
```

---

## 1. 已确认的问题

### A1. `ResetState` 永不完成 → FSM 永久卡死，静默吞掉所有 `exp_state` 请求【阻塞级】

**现象**

`task_cart.log` 两次 `cart_traj` 运行，机械臂**一毫米没动**，残差恰好等于下发的整个目标偏移量，且**全程没有任何控制器报错**：

| 运行 | 起点 → 目标 | 残差 | 控制器状态 |
|---|---|---|---|
| 19:33（带 6 s 延迟） | `[-0.023 -0.037 1.260]` → z 1.310 | **0.0500 m**（= 偏移量 0.05） | 无报错，进程正常退出 |
| 19:40（`ros2 run`，下移 20 cm） | `[-0.020 -0.037 1.260]` → z 1.060 | **0.2000 m**（= 偏移量 0.2） | 同上 |

`/joint_states` 显示 velocity ≈ 1e-10（完全静止），`ros2 control list_controllers -v` 显示 `arm_controller` / `sim_pid_controller` 均 `active`（没有停用）。

**根因（定量证实）**

`ArmController` 下发 `ki = 0`（纯 PD，无积分），每个关节都有重力静差 `|q| = |τ| / kp`：

| 关节 | kp | 实测 effort τ | τ/kp 预测 | 实测 position | 是否超容差 |
|---|---|---|---|---|---|
| joint1 | 200 | 6.7e-9 | 3.4e-11 | -3.6e-11 | 否 |
| joint2 | 200 | -1.2249 | 0.006125 | 0.006125 | 否 |
| joint3 | 200 | -1.1458 | 0.005729 | 0.005729 | 否 |
| **joint4** | **10** | **-1.0465** | **0.10465** | **0.104645** | **是（10 倍）** |
| joint5 | 10 | -0.00628 | 0.000628 | 0.000628 | 否 |
| joint6 | 1 | 1.4e-13 | ~0 | ~0 | 否 |

于是（`src/arm_controller/arm/arm_fsm.cpp:285-289`）：

```cpp
const float position_error = std::fabs(factory->state_[i].position - reset_joint_pos_[i]);
joint_reached              = joint_reached && position_error <= reset_tolerance_;   // joint4 永远不满足
reset_done_ = progress_ >= 1.0f && joint_reached;                                  // 永远 false
```

`ResetState::check_switch()`（`arm_fsm.cpp:260-265`）在 `reset_done_` 为 false 时**只返回自身**，于是：

* FSM 永远停在 `reset`；
* **外部任何 `exp_state` 请求（cart_traj / joint_traj / teach_pendant / measure）全部被静默忽略**，且没有任何日志；
* `/arm_cart_traj` 的消息其实**被接收了**（8 个状态的订阅在控制器构造时全部创建，与当前活动状态无关），但活动状态是 `reset`，`CartTrajState::run()` 永远不会被调用 → 机械臂保持复位目标位姿，残差 = 整个偏移量。

**同源的另一个表现**：任务层 `ResetTask` 用同一套 0.01 rad 判据，所以之前出现 `reset task did not reach the target position within the timeout`。

**注意**：`reset_tolerance` / `reset_duration` 是在 `ResetState` **构造函数**里读的（`arm_fsm.cpp:224-229`），运行时 `ros2 param set` **不生效**，必须改 YAML 并重启仿真。

**状态**：已定位，未修。

---

### A2. 笛卡尔姿态误差用"旋转向量相减"，在 `|rv| ≈ π` 处失效【已被数值实验证实，未在真实链路复现】

**机制**

控制器的任务空间姿态是旋转向量 `rv = angle * axis`（`src/arm_controller/arm/default6dof_task.cpp:20-27`，`angle ∈ [0, π]`），IK 的误差是 `target - current`（`lib/calculate/kinamic.cpp:95`）。但：

* 同一个旋转矩阵 `R(π, a) = R(π, −a) = R(−π, a)`，即当转角为 π 时**轴的正负在数学上不可分辨**；
* 于是 `rv` 这个 3 维表示在 `|rv| = π` 处有分支割线，同一姿态存在相差 2π 的两个 `rv`；
* "两个 rv 相减"**不是 SO(3) 上的距离**，物理上姿态差为 0 时误差也可能报出 ~2π。

**这台臂恰好长期停在 π 上**（pinocchio 实测）：

| 关节角 | link6 位置 | 旋转向量 rv | \|rv\| |
|---|---|---|---|
| `[0,0,0,0,0,0]`（零位形） | `[0.006, −0.037, 1.260]` | `[0, 0.003, −3.142]` | **3.142 = π** |
| `[0,−1.5,2.9,−2,0,0]`（重力塌陷姿态） | `[0.179, −0.037, 0.386]` | `[0.928, −0.002, 3.001]` | **3.141 = π** |
| `[0,−1.2,2,−1,0.5,−1.5]` | `[0.206, −0.002, 0.782]` | `[−0.22, 0.56, 1.648]` | 1.755 |
| `[0.5,1,−2.5,1.5,−0.5,2.5]` | `[0.038, −0.063, 0.609]` | `[0.422, 0.271, −0.139]` | 0.521 |

根因是 URDF 里 `joint5` / `joint6` 的固定偏置（`rpy="-1.5708 0 0"`、`rpy="0 -1.5708 3.14"`）让工具法兰相对基座天然带一个约 180° 的旋转。

**复刻实验**（用 pinocchio 复刻控制器同样的度量与 DLS 参数：`kTolerance=1e-6`、`kDamping=1e-4`、`kMaxJointStep=0.1`、200 次上限）：

| 目标位姿 | \|rv\| | 零位形种子（改前） | 实测关节角种子（改后） |
|---|---|---|---|
| FK(0)，z=1.260 | **3.142** | 收敛（0 次） | **不收敛**，残差 4.23 |
| 塌陷位形，z=0.386 | **3.141** | 不收敛，残差 2.73 | **不收敛**，残差 1.16 |
| 位形，z=0.782 | 1.755 | 不收敛，残差 3.65 | 收敛，**113 次** |
| 位形，z=0.609 | 0.521 | 不收敛，残差 0.49 | 收敛，**8 次** |

结论：`seed` 改成实测关节角**有效**（后两行），但 `|rv| ≈ π` 时**两种种子都解不出来**（前两行，含"只偏 0.02 rad"的情形）——这才是笛卡尔路径的根因。

**建议修法**：IK 内部把目标/当前的 rv 各自 `exp3` 成旋转矩阵，姿态误差用**相对旋转** `log3(R_target · R_currentᵀ)`（当前位姿切空间里的旋转向量，连续且与 `LOCAL_WORLD_ALIGNED` 雅可比局部一致），位置误差不变；只改 `IKSolver::solve` 的误差计算，外部任务空间表示不变。

**状态**：机制与数值影响已证实；**真实代码路径尚未直接观测**（被 A1 阻塞）。

---

### A3. IK 种子硬编码为全零位形【已修】

`lib/calculate/arm.cpp` 原先在 `inverse_kinamic()` 里 `joint_pos->setZero()`，每个控制周期都从零位形重新起步。当目标位姿远离零位形（实测 `FK(0)` 的 z=1.260，而重力塌陷姿态 z≈0.33，差 ~0.9 m）时，阻尼最小二乘容易在迭代上限内不收敛。

**已改为**：`*joint_pos` 既作输入又作输出，输入由调用方给；`CartTrajState::run()`（`arm_fsm.cpp:365-372`）与 `ServoState::run()`（`arm_fsm.cpp:596-602`）都先把 `factory->state_[i].position`（实测关节角）写进种子，`AdmittanceState` 本来就已这样做。复刻实验显示这能显著改善收敛（8 / 113 次 vs 200 次失败）。

**状态**：已改 + 已编译；效果受 A2 限制。

---

### A4. IK 失败被当成致命错误 → controller_manager 停用整条控制器链 → 机械臂坠落

**证据**（`task_traj.log`，16:15 那次）：

```
L136  task_runner: cartesian trajectory task: start [0.007 0.006 0.342], target [0.007 0.006 0.392]
L138  CM: Controller 'arm_controller' is part of a chain of 2 controllers that will be deactivated
L139  CM(ERROR): Deactivating controllers : [ arm_controller sim_pid_controller ] as their update resulted in an error!
L140-145  MujocoSystemInterface: Joint jointN: effort control disabled   ×6
L153  task_runner(WARN): ... residual position error of 0.2028 m (tolerance 0.0050 m)
```

`ArmController::update()` 在 `fsm_factory->run()` 返回 false 时返回 ERROR（`controller/src/controller.cpp:420-432`），CM 随即停用整条链；机械臂失去力矩后在重力下坠落，0.2 m 残差是**后果不是原因**。

`CartTrajState::run()`（`arm_fsm.cpp:358-377`）里能**不打日志**返回 false 的路径只有求解相关（抛异常才会打 `Cartesian trajectory solve failed`），因此怀疑是 `inverse_kinamic()` / `inverse_dynamic()` 返回 false（与 A2 一致）。轨迹发出到停用之间约 142 ms，与"单次 `update()` 跑满 200 次病态迭代阻塞 RT 循环"吻合（**未直接验证**）。

**状态**：现象确认（两次）；根因指向 A2，待真实链路复现。

---

### A5. 任务层把致命故障降级为 WARN + 退出码 0

* `task_traj.log` 中控制器已被停用、机械臂已坠落，任务只打一条 `residual ... 0.2028 m` 的 WARN，然后照常 `exp_state -> idel`、打印 `task chain finished` 并以 **0** 退出。
* 两次"完全没动"的运行（残差 = 整个偏移量）同样是 WARN + 退出 0。

对应代码：`src/tasks/task/task_space_segment_task.cpp:138-145`（残差只判 WARN）、`src/tasks/src/task_runner.cpp`（失败/超时的退出码逻辑）。目前任务层也观测不到"控制器已被停用"。

**状态**：已确认，待修。

---

### A6. 任务下发指令与控制器激活存在时序竞争【已缓解】

`task_traj.log`：

```
L124  1791015333.8165  task_runner: exp_state -> cart_traj      ← 写入
L134  1791015333.8262  CM: Successfully switched controllers!  ← arm_controller 激活完成（晚 10 ms）
```

后果：`exp_state` 在控制器激活前就被写走，**YAML 里的上电自动复位被顶掉**（首次 `task_cart.log` 里起点仍是塌陷姿态 z≈0.33，可佐证）。

**已缓解**：`ros2_controller.yaml` 把 `exp_state` 改为 `reset`（上电自动复位），并给 7 个 `task_*.launch.py` 加了 `task_delay`（`TimerAction`，默认 6.0 s ≈ 激活 2.5 s + 复位 3 s + 余量）。实测第二次 `task_cart.log` 的笛卡尔起点 z=1.260 精确等于 `FK(0)`，证明复位已执行、时序竞争已消除。

**状态**：已缓解（配置层面）。

---

### A7. `ResetState` 卡死是"静默失效"，没有任何兜底与日志

`ResetState` 没有超时、没有告警；`check_switch()` 硬返回自身，`exit()` 还会在离开时把 `exp_state_name` 写回 `idel`。这意味着**任何"复位未完成"都会让整个控制器变成哑巴**，而外部完全看不出来（A1 就是这么发生的）。

**建议**：超过 `reset_duration + N` 秒仍未到位时打 WARN 并强制退出 reset。

**状态**：待修（建议与 A1 一起处理）。

---

### A8. `robot_state_publisher` "Moved backwards in time" 刷屏

* 来源：`robot_state_publisher` 节点，`/opt/ros/jazzy/lib/librobot_state_publisher_node.so`（源码 `robot_state_publisher.cpp:318`）。
* 触发：收到的 `/joint_states` 时间戳比上次更早（时钟回退）。ROS 1 同处原文为 "probably because ROS clock was reset"。
* 本项目最常见的成因：**同时起了两个仿真** —— 两个 MuJoCo 各发一个 `/clock`，两个 `controller_manager` 各发一个 `/joint_states`，两套时间基准互相回退。
* 已做：在 `arm_controller_test_sim.launch.py` 加了一行提示（建议已有仿真时用 `start_sim:=false`）；需要静音时可用
  `ros2 service call /robot_state_publisher/set_logger_levels rcl_interfaces/srv/SetLoggerLevels "{levels: [{name: 'robot_state_publisher', level: 40}]}"`。
* 注意：关掉 RViz 没用，警告来自 RSP 本身，而 tasks 的笛卡尔任务依赖它发布的 TF。

**状态**：现象确认；根因（当时环境）未复现。

---

## 2. 潜在问题（分析推断，未直接观测）

### B1. 求解器参数过严 / 病态

`lib/calculate/kinamic.cpp:7-13`：`kTolerance = 1e-6`（位置 m 与姿态 rad 的范数）、`kDamping = 1e-4`（近奇异时方程病态）、`kMaxJointStep = 0.1`、`kMaxIterations = 200`。复刻实验中实测种子需要 113 次迭代才收敛，距离 200 的上限余量很小；容差 1e-6 也意味着 TF 与模型间的微小差异（1e-4 量级）也必须靠迭代压下去。

### B2. 静差问题会在换模型/换实机后再次出现

只要 `ki = 0`，任何"重力矩大 / kp 小"的关节都会有静差。A1 只是 joint4 暴露出来；改模型、换实机、换负载后可能换成别的关节。

### B3. 零位形处于工作空间边界，默认笛卡尔偏移不可达

`FK(0)` 时手臂竖直全伸直（0.153 + 0.52 + 0.436 + 0.078 + 0.074 ≈ 1.26 m 全竖直叠加），末端 z=1.260 已是该姿态下的最高点，因此 `task_cart_traj.launch.py` / `task_admittance.launch.py` 里默认的 `+z 5 cm` 偏移**不可达**。注意：A1 阻塞期间无法区分"不可达"与"没执行"，但用户用 −20 cm 也失败，说明那次失败不是不可达造成的；不过默认偏移本身仍不可达，早晚撞上。

### B4. 笛卡尔任务的起点可能取自"运动中/塌陷姿态"

仿真启动到 `arm_controller` 激活之间约 1 s 无人给力矩，`scene.xml` 有重力（`gravity="0 0 -9.81"`）、MJCF 无 keyframe 保持 → 臂被压塌；`IDELState` 锁的是塌陷姿态。启用上电自动复位后这一项已缓解（起点变为 `FK(0)`），但任务层"读起点"与控制器内部 `state_` 之间仍有 1~2 个控制周期的滞后。

### B5. 当前实现是 6 自由度专用，无法直接支持 5 轴

* 任务空间维度写死 6：`kinamic.cpp:26,27,30,31`、`arm.cpp:13-20`、`arm_fsm.hpp` 的 `task_dof_{6}`、`arm_fsm.cpp` 的 `point(6)` / `traj(6, …)` / `task_zero_.setZero(6)`、`default6dof_task.cpp:17`、`IKSolver::solve` 要求 `target.size() == 6`。
* 数学上 5 轴 + 6 维位姿目标过约束 → 一般无精确解，而收敛判据是 `‖e‖ < 1e-6` → 必然判失败。
* 但以下部分是自由度无关的：`ModelFromURDF`（pinocchio）、`Trajectory`、消息、`SimPidController`（joints 参数驱动）、`ArmSolve` 的伪逆（用 `CompleteOrthogonalDecomposition`，可处理非方阵最小二乘）、`TeachPendantState` 的重力补偿。
* 非解算但同样挡路：实机 CDC 收发包写死 6 关节（`controller/src/arm_real_interfaces.cpp:151,176`）；`ModelFromURDF(path, "link6")` 的末端 frame 名。

### B6. 控制器与任务层的失败语义不一致

控制器把"解不出"当致命错误（停整条链），任务层把"没动"当成功。两边都需要改（见第 4 节建议）。

### B7. 控制器不发布任何状态

`ArmController` 没有 publisher，任务层无法观测控制器内部阶段，因此 `reset` / `teach_pendant` / `measure` 只能靠固定时长猜完成；也无法得知"控制器已被停用"。

---

## 3. 等待被验证的问题

| # | 待验证问题 | 验证方法 | 判据 |
|---|---|---|---|
| C1 | 解锁 reset 后，笛卡尔 IK 在真实链路上能否跑通（A2 是否真的是障碍） | 放宽 `reset_tolerance` 或提高 joint4 `kp` → 重跑 `task_cart_traj` | 末端到达目标（残差 < 容差）、无 `Deactivating controllers` 日志 |
| C2 | 真实代码路径里是否真的出现 `\|rv\|≈π` 导致的 2π 误差 | 在控制器侧临时打印 `target` / `current` 的 rv 模，或用任务层打印 TF 四元数换算 | `\|target_rv − current_rv\|` 出现 ~6.28 量级 |
| C3 | 142 ms 的 `update()` 阻塞是否确实是 200 次 IK 迭代 | 在 `CartTrajState::run()` 统计 IK 迭代次数 / `update()` 耗时 | 迭代数接近 200、单周期耗时 ~百毫秒 |
| C4 | 提高 joint4 `kp`（10 → 200）后仿真是否稳定、会不会振荡 | 改 YAML 跑复位 + 笛卡尔，观察 `/joint_states` 是否有振荡 | 关节角单调收敛、无高频抖动 |
| C5 | 重力补偿前馈（`inverse_dynamic(q,0,0)`，`TeachPendantState` 已有现成写法）能否消除静差 | 在 `reset`/`idel` 的 `command.torque` 里加重力矩 | `/joint_states` 的 `position` 全部 < 1e-3 rad，此时 `reset_tolerance` 可保持 0.01 |
| C6 | 5 轴支持的工作量与可行性 | 用户确认是否需要；再决定是否做任务空间维度参数化 | — |
| C7 | `robot_state_publisher` 刷屏在当前环境是否还复现 | 确认是否重复起仿真；必要时按 A8 降噪 | 不再出现 WARN 刷屏 |

---

## 4. 建议的修复顺序

1. **立刻解锁**：把 `ros2_controller.yaml` 的 `reset_tolerance` 放宽到覆盖 joint4 静差（≥ 0.12，建议 0.15），或把 `default_kp` 的 joint4 从 10 提到 ≥110（与 joint2/3 一致取 200、`default_kd` 同步到 8）。注意任务侧 `ResetTask` 也有同名独立参数，要一起改；两者都在构造/启动时读取，运行时改参数无效。
2. **消除静差（根治）**：`reset`/`idel` 阶段加重力补偿前馈，让 `reset_tolerance=0.01` 也能成立。
3. **失效模式加固**：`ResetState` 加超时兜底 + WARN；IK 失败改为"保持当前位形 + 限流告警"，不再让 CM 停用整条控制器链。
4. **笛卡尔根因**：IK 姿态误差改用相对旋转 `log3(R_t·R_cᵀ)`；同时适度放宽 `kTolerance` / 增大 `kDamping`。
5. **任务层**：把"无进展 / 残差≈整个目标偏移量"判为失败并返回非 0；发轨迹前确认控制器已激活且已进入目标状态。
6. **默认目标**：`task_cart_traj` / `task_admittance` 的默认偏移改成可达方向（零位形下 +z 不可达）。
7. 可选：把笛卡尔类 launch 的默认任务链改成 `[reset, <任务>]`。

---

## 5. 当前代码状态（已做过的改动）

| 改动 | 位置 |
|---|---|
| IK 种子由全零改为当前实测关节角 | `lib/calculate/arm.cpp`、`lib/calculate/arm.hpp`、`arm/arm_fsm.cpp`（`CartTrajState` / `ServoState`） |
| 上电自动复位 | `launch_pack/config/ros2_controller.yaml`：`exp_state: reset` |
| 任务节点延迟启动（等复位完成） | 7 个 `launch_pack/launch/task_*.launch.py`：`task_delay`（默认 6.0 s）+ `TimerAction` |
| `startup_timeout` 参数（等控制器/关节反馈，默认 10 s，launch 给 60） | `tasks/src/task_runner.cpp` |
| `reset` 超时诊断（打印各关节残差、目标/实际数组） | `tasks/task/reset_task.cpp` |
| `reset` 加入 idel 门控自动补位 | `tasks/src/task_runner.cpp`：`kIdelGatedTasks` |
| 7 个任务的一键 launch（idel/reset/cart_traj/joint_traj/admittance/teach_pendant/measure） | `launch_pack/launch/task_*.launch.py` |
| 启动提示（避免重复起仿真） | `launch_pack/launch/arm_controller_test_sim.launch.py` |

**环境侧待清理的失效 launch**（引用了本工作区不存在的包）：

| 文件 | 缺失内容 |
|---|---|
| `arm_mujoco_sim.launch.py` | `arm_calc` |
| `arm_measure_sim.launch.py` | `parameter_measure`（现为 `parameter_identify`，且只剩 `identify_arm`） |
| `arm_real.launch.py` | `arm`、`arm_calc`、`arm_task`、`robot_driver`、`vision` |
| `arm_static_display.launch.py` | 包名 `arm` 应为 `arm_model` |

---

## 6. 关键复现步骤与数据

**复现 A1（FSM 卡在 reset）**

```bash
ros2 launch launch_pack task_cart_traj.launch.py
# 观察：机械臂停在 z≈1.260 不动；任务结束报 residual 0.0500 m
ros2 topic echo /joint_states --once
#   关键：joint4 ≈ 0.1046 rad（> reset_tolerance 0.01），其余关节 < 0.01
ros2 control list_controllers -v      # 两个控制器都 active（没有报错）
```

**复现 A2（数值层面，pinocchio 复刻控制器 DLS）**

```bash
python3 -c "
import numpy as np, pinocchio as pin
m = pin.buildModelFromUrdf('src/arm_model/model/robotic_arm.urdf')
d = m.createData(); f = m.getFrameId('link6')
for q in [np.zeros(6), [0,-1.5,2.9,-2,0,0]]:
    pin.framesForwardKinematics(m, d, np.array(q,dtype=float))
    print(np.round(q,2), 'pos=', np.round(d.oMf[f].translation,3), 'rv=', np.round(pin.log3(d.oMf[f].rotation),3))
"
# |rv| ≈ 3.14（π）→ 姿态误差度量失效区
```

**相关文件索引**

| 主题 | 文件 |
|---|---|
| 控制状态机 | `src/arm_controller/arm/arm_fsm.cpp` / `arm_fsm.hpp` / `arm_fsm_factory.hpp` |
| 逆解 / 逆动力学 / 雅可比 | `src/arm_controller/lib/calculate/kinamic.cpp` / `arm.cpp` / `trajectory.cpp` |
| 任务空间映射（姿态表示） | `src/arm_controller/arm/default6dof_task.cpp` |
| 控制器实时回调 | `src/arm_controller/controller/src/controller.cpp` |
| 控制器参数 | `src/launch_pack/config/ros2_controller.yaml` |
| 仿真场景（重力/初始位形） | `src/arm_model/model/scene.xml` / `robotic_arm.urdf` |
| 任务层框架 | `src/tasks/fsm/`、`src/tasks/context/`、`src/tasks/task/`、`src/tasks/src/task_runner.cpp` |
| 启动文件 | `src/launch_pack/launch/` |
