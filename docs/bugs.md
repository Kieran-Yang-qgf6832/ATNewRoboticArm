# 缺陷记录（任务层重构期间）

> 整理时间：2026-10-08
> 范围：`src/arm_controller`（控制器 / 状态机 / 求解器）、`src/tasks`（任务层）、`src/arm_model`（模型）
> 背景：本轮把任务层从「7 个 C++ 任务类」重构为「配置驱动执行器」的过程中，为验证链路而做的端到端测试暴露了一批缺陷。
>
> 说明：本文件承接已删除的 `bugs.md`。原 `bugs.md` 中的 A1/A4/A5/A7 已在本轮修复，A2 在本轮复现并修复，B3 确认为设计约束；其余条目未在本轮处理。

---

## 0. 总览

| 编号 | 缺陷 | 严重度 | 状态 | 提交 |
| --- | --- | --- | --- | --- |
| F1 | 保持命令失去刚度，轨迹状态在重力下坠落 | **严重** | 已修 | `393598d` |
| F2 | 旋转向量误差在 \|rv\|≈π 处失效（原 A2） | **严重** | 已修 | `729df71` |
| F3 | MJCF 与 URDF 关节限位不一致 | 高 | 已修 | `393598d` |
| F4 | IK 迭代震荡（固定阻尼 + 固定步长上限） | 高 | 已修 | `7519118` |
| F5 | 收敛判据 1e-6 过严，收敛了却报失败 | 高 | 已修 | `7519118` |
| F6 | IK 空转 200 次迭代拖垮实时控制回路 | 高 | 已修 | `7519118` |
| F7 | 故障阶段诊断在重构中丢失（重构自身引入的退步） | 中 | 已修 | `6543b23` |
| F8 | `AdmittanceState::exit()` 缺清理，重构会引爆 | 中 | 已修 | `393598d` |
| F9 | 状态切换竞态：步骤 0.5 ms 内假成功 | 中 | 已修 | `59bc567` |
| F10 | 退出码掩盖任务失败（原 A5） | 中 | 已修 | `59bc567` |
| F11 | IK 失败导致控制器链被停用、机械臂坠落（原 A4） | **严重** | 已修 | `ba63893` |
| F12 | `reset` 永不完成、FSM 卡死（原 A1） | **严重** | 已修（重构前） | — |
| **O1** | ~~TF 与控制器正运动学不一致~~ **已证伪**（二者完全一致，见 §6） | — | 已排查 | — |
| **O5** | `jacobian_map` 把几何雅可比直接当任务雅可比，姿态用旋转向量时在 \|rv\|=π 处线性化失效 | **高** | **已定位，未修** | — |
| O2 | `IKSolver` 不检测不可达目标，投影到最近可达位姿 | 高 | 未修 | — |
| **O3** | `ArmRealInterfaces` 硬编码关节名与关节数，越界读写 | **严重** | **已修** | `dbb7fe0` |
| O4 | launch 经 `/robot_description` 话题取 URDF，易静默加载错模型 | 中 | 未修 | — |
| **O6** | 复位静差超容差后回退 idel，重力静差被叠加成 12.4° 偏置 | **高** | **已修** | `c7abf73` |

---

## 1. 已修复

### F1 保持命令失去刚度，轨迹状态在重力下坠落【严重】

**现象**

`cart_move` / `joint_move` 执行完或刚切入时，机械臂失去保持能力，在重力下缓慢塌陷。实测进入 `cart_traj` 后 joint2 在 0.5 秒内从 0.012 rad 冲到限位 1.5715 rad，joint6 持续漂移（无驱动特征）。

**根因**

`hold_current_joint_position()` 把位置指令设成「**当前实测值**」，而且每个周期都重新读取：

```cpp
command.position = factory->state_[i].position;   // 每周期重读实测值
command.kp       = default_kp[i];
```

于是位置误差 `position - measured` 恒为 0，**kp 再大也不产生恢复力**，机械臂只剩 kd 阻尼。

对照 `IDELState` 才是正确做法：`enter()` 时锁存一次 `hold_position_`，之后 `run()` 一直下发该锁存值。

**影响范围**

`CartTrajState` / `JointTrajState` / `AdmittanceState` 三个状态、共 5 处调用点。此前 `cart_traj` 从未被真正执行到（原 A1 导致 FSM 卡在 reset），所以这个缺陷一直未被触发。

**修复**

拆成两个函数：

- `latch_joint_position(factory, joint_count, &hold_position)`：锁存当前实测关节角，**进入保持时调用一次**
- `write_hold_command(factory, joint_count, hold_position, default_kp, default_kd)`：按锁存值下发保持命令

调用时机：

| 状态 | 锁存时机 |
| --- | --- |
| `CartTrajState` | `enter()`；以及轨迹结束时重新锁存终点 |
| `JointTrajState` | `enter()`；以及轨迹结束时重新锁存终点 |
| `AdmittanceState` | `enter()` |

**验证**：进入 `cart_traj` 后连续两帧关节角完全一致，稳定保持。

---

### F2 旋转向量误差在 |rv| ≈ π 处失效【严重】（原 bugs.md A2）

**现象**

下发笛卡尔轨迹后 **2 毫秒**（收到轨迹的第一个控制周期）即报逆解失败：

```
target      = [... | 0.4732 -0.0003 -3.1053]     ← |rv| = 3.1412 ≈ π
measured    = [0.0005 0.0058 0.0108 0.2871 0.0019 0]   ← 关节角完全正常
```

关节角正常、位置误差≈0，逆解却必然失败。

**根因**

`IKSolver::solve` 用逐分量相减计算任务空间误差：

```cpp
workspace_.error.noalias() = target - workspace_.current;
```

姿态用旋转向量表示时，**`rv` 与 `-rv` 表示同一姿态**。任务层（TF → 四元数 → 旋转向量）与控制器（Eigen `AngleAxisd`）一旦落在 π 奇异点两侧，误差会被算成 ≈2π 而不是 ≈0，迭代随即发散。

原 `bugs.md` 已用数值实验证实该问题，但标注「未在真实链路复现」；本轮在真实链路上复现。

**修复**

按分层约定（任务空间语义归 `TaskMapping`，`ModelBase` 只管模型数学），在 `TaskMapping` 增加纯虚接口：

```cpp
virtual bool error_map(const Eigen::VectorXd& target,
                       const Eigen::VectorXd& current,
                       Eigen::VectorXd* error) = 0;
```

`Default6DofTaskSpaceMapping::error_map` 实现：位置直接相减；姿态用**相对旋转**

```
R_err = exp(target_rv) · exp(current_rv)ᵀ
error_rot = log(R_err)
```

其角度恒在 [0, π] 内、连续、无奇异。`IKSolver` 改为调用 `error_map`（迭代中与最终收敛判定各一处）。

**效果**：IK 失败点从 t=0.002 s 推迟到 t=0.086 s，旋转奇异消除。

---

### F3 MJCF 与 URDF 关节限位不一致【高】

**现象**

仿真中机械臂停在 joint4 = 2.2589 rad，超过 URDF 规定的上限 1.8 rad。

**根因**

| 关节 | URDF（控制器模型） | MJCF（MuJoCo） | 一致 |
| --- | --- | --- | --- |
| joint1 | ±1.8 | ±6.28319 | ✗ |
| joint3 | ±2.9 | ±3.14159 | ✗ |
| joint4 | −2.0 ~ 1.8 | ±3.14159 | ✗ |
| joint2 / joint5 | ±1.570796327 / ±0.610865238 | ±1.5708 / ±0.610865 | 精度不足 |

控制器用 URDF 建模，`IKSolver` 会把逆解种子按 URDF 限位截断、并把迭代步长限制在限位之内。MJCF 允许超出后，机械臂会停在控制器认为非法的位形上，此时**任何目标位姿都会变成数学上不可达**，逆解必然失败。

**修复**

以 URDF 为准对齐 MJCF 全部六个关节（含小数位完全一致），并在 `robotic_arm.xml` 顶部加注释说明必须保持一致及原因。

---

### F4 IK 迭代震荡（固定阻尼 + 固定步长上限）【高】

**现象**

残余误差仅 0.014（位置 0.001 m + 姿态 0.014 rad ≈ 0.8°）时，逆解迭代 200 次仍无法收敛。

**根因**

原迭代参数互相打架：

| 参数 | 原值 | 后果 |
| --- | --- | --- |
| `kDamping` | 1e-4 | 正则项 `(1e-4)² = 1e-8`，对 `JᵀJ` 几乎无影响 |
| `kMaxJointStep` | 0.1 | 接近奇异时算出的巨大步长被硬截断 |

接近奇异的方向 → 步长巨大 → 被截到 0.1 → 冲过头 → 反向再来 → **震荡不收敛**。

**修复**

改为信赖域 + Levenberg-Marquardt：

1. LM 步 `(JᵀJ + λI)⁻¹Jᵀe`，λ 随接受/拒绝自适应
2. 信赖域限制步长范数：超过半径按比例缩回，**保持方向、只缩短步长**
3. 只有实际误差下降才接受该步，否则收紧信赖域、加大阻尼重试

参数集中在 `IKSolver::Tuning`，可运行时修改以便离线调参。

---

### F5 收敛判据 1e-6 过严，收敛了却报失败【高】

**现象**

```
Cartesian IK failed at t=0.700 s: ik_err=1.018e-06 (iters=200)
```

IK **已经收敛到数值极限**，只因比 `1e-6` 大了一点点，跑满 200 次迭代后返回失败。

**根因**

判据是「米 + 弧度」混合的 6 维范数，双精度下不可能收敛到任意小；更关键的是**目标位姿往往与控制器自身正解存在微小不一致**（任务层经 TF 读取的末端位姿、实机上的标定误差），此时误差无论如何都降不到零。

**修复**

判据改为 `1e-3`（1 mm / 0.057°），并写明取值依据。

> **注意**：放宽判据只是让已知的微小不一致不再触发误报，**并没有消除不一致本身**——见 O1。

---

### F6 IK 空转 200 次迭代拖垮实时控制回路【高】

**现象**

`tracking_err` 一度达到 0.0986 —— 机械臂严重滞后于轨迹。

**根因**

IK 不收敛时会跑满 `kMaxIterations = 200`。这在 500 Hz（周期 2 ms）的实时回路里是灾难：每周期 200 次迭代 × 每次两次正解，控制周期被拖垮。

**修复**

增加停滞提前退出：连续 8 次步被拒绝即判定到达数值极限并返回。

**效果**：带扰动目标的平均迭代次数从 7.7 降到 3.7，收敛率不变。

---

### F7 故障阶段诊断在重构中丢失【中】（重构自身引入的退步）

**现象**

把 17 处分散的故障兜底收拢到 `ArmController::update()` 之后，控制器日志只剩一句笼统的：

```
state machine reported a fault; holding the current joint position and switching to idel
```

**丢失了「到底是采样、逆解还是逆动力学失败」这一关键定位信息**，导致排查 `cart_move` 失败时不得不先补回诊断。

**根因**

原实现在每个失败点调用 `fall_back_to_idel_after_solve_failure(..., stage)`，`stage` 参数同时承担了「日志定位」职责。收拢策略时把策略和诊断一起删掉了。

**修复**

- `FSM` 基类增加 `fault_stage_` 与 `set_fault_stage()` / `fault_stage()`；`FSMFactory::run()` 每次调用前清除
- 17 处失败点用 `set_fault_stage("...")` 记录阶段名（阶段名从 git 历史中提取，按 `return false;` 的序号精确映射回原位）
- 控制器在故障日志中一并打出状态名与阶段名
- 另把 `CartTrajState` 的求解失败拆成 IK / 逆动力学两级，并在 IK 失败时打印任务空间误差、目标位姿与「当前实测关节角的正解」

**教训**：把策略集中化时，**诊断信息必须一并迁移**，不能随策略代码一起删除。

---

### F8 `AdmittanceState::exit()` 缺清理，重构会引爆【中】

**现象（静态分析发现，重构前未触发）**

`AdmittanceState::check_switch()` 的切换前提是 `!trajectory_active_`：

```cpp
if (factory != nullptr && !trajectory_active_ && ...) { return factory->exp_state_name; }
```

而 `exit()` 原本什么都不做，`trajectory_active_` 只靠 `run()` 里的失败路径顺手清零。

**根因**

F7 的收拢会把 `run()` 失败路径里的 `trajectory_active_ = false` 一并删掉。一旦删除，`trajectory_active_` 会永久为 true，**导纳状态再也切不出去，故障恢复失效**。

**修复**

在 `exit()` 中补上清理：

```cpp
bool AdmittanceState::exit(const std::string& next_state) {
    (void)next_state;
    traj.stop();
    trajectory_active_ = false;
    return true;
}
```

**教训**：清理动作应放在 `exit()`（生命周期的正确位置），而不是散落在 `run()` 的失败路径里——否则重构时极易丢失。

---

### F9 状态切换竞态：步骤 0.5 ms 内假成功【中】

**现象**

`reset_only` 序列的 `reset` 步骤在 **0.5 毫秒**内宣告完成，实际什么都没做。

**根因**

控制器把**实际**状态发布到 `current_state` 参数是由 **500 ms wall timer** 驱动的。`set_exp_state("reset")` 之后立刻轮询该参数，读到的还是切换前的旧值 `idel`，于是 `wait_for_idel` 立刻返回假成功。

**修复**

任务执行器在 `set_exp_state` 之后先等待 `state_settle`（默认 0.6 s，**必须大于控制器的状态刷新周期**），再开始轮询。

**验证**：`reset` 步骤耗时从 0.5 ms 变为 5.2 s（`reset_duration` 3 s + `reset_settle` 2 s 兜底）。

> 该耦合（`state_settle` 必须大于控制器 `state_timer_` 周期）已写入代码注释与参数说明。

---

### F10 退出码掩盖任务失败【中】（原 bugs.md A5）

**现象**

任务链失败时进程仍以 0 退出，上层脚本会当成成功。

**修复**

`TaskRunner::run()` 记录最后一条序列的结果，退出时据此返回退出码。实测：失败序列 → 1，成功序列 → 0。

---

### F11 IK 失败导致控制器链被停用、机械臂坠落【严重】（原 bugs.md A4）

**现象**

任何求解失败都会让 `ArmController::update()` 返回 `ERROR`，`controller_manager` 随即**停用整条控制器链**，力矩消失，机械臂在重力下坠落。

**根因**

安全策略被放在了错误的层级：状态机报错 → 控制器返回 ERROR → 上层把整条链关掉。安全策略本应在控制器内部完成。

**修复**

- `FSMFactory::run()` 返回值由 `bool` 改为 `RunResult{Ok, Recoverable, Fatal}`，把「什么情况下允许停用控制器链」变成显式契约
- 可恢复故障返回 `OK` 而非 `ERROR`
- 新增 `ArmController::apply_safe_hold_and_idel()`：锁定当前位置、请求切回 `idel`、限流上报
- `exp_state_name` 上移到 `FSMFactory` 基类：状态报告失败时由工厂**直接请求切到期望状态**，不再依赖状态自身「清理 + 置 exp_state + 返回 true」来触发切换
- `apply_safe_hold_and_idel()` **同时写 `command_`**，防止切换周期下发故障前的陈旧指令

**验证**：注入求解失败后控制器保持 `active`、状态回到 `idel`、机械臂稳定保持（6 秒内漂移 ~1e-9）。

---

### F12 `reset` 永不完成、FSM 卡死【严重】（原 bugs.md A1，重构前已修）

**根因**

纯 PD 无积分，joint4 的重力静差 `τ/kp = 1.0465/10 = 0.1047 rad` 远超 `reset_tolerance = 0.01`，`reset_done_` 永远为 false，`check_switch()` 只返回自身，FSM 永久卡在 `reset` 并静默吞掉所有 `exp_state` 请求。

**修复**（本轮之前完成）

`ResetState::run()` 增加兜底：`reset_duration + reset_settle` 后仍未到位则打 WARN 并强制切回 `idel`。

---

## 2. 未修复（已搁置）

### O1 TF 与控制器正运动学不一致（~1e-3）【高】

**现象**

任务层经 TF 读到的末端位姿，与控制器 `forward_kinamic()` 对同一组关节角算出的位姿存在约 **1e-3** 的不一致。表现为逆解残余误差**总是恰好停在判据之上一点**：

| 判据 | IK 实际残余 |
| --- | --- |
| 1e-6 | 1.018e-6 |
| 1e-4 | 1.519e-4 |
| 1e-3 | 1.076e-3 |

这个规律说明：**目标位姿本身落在控制器可达集之外约 1e-3**，误差无论如何都降不到零。

**为什么重要**

- 目标轻微不一致是**常态**：实机上还有标定误差、连杆柔性、关节间隙叠加进来
- 当前只是靠放宽判据（F5）把它掩盖掉，并未消除
- 若不一致继续增大，会重新表现为「莫名其妙的求解失败」

**疑似方向**

在 \|rv\| ≈ π 附近，任务层的四元数→旋转向量转换（`task_context.cpp` 的 `quaternion_to_rotation_vector`）与 Eigen `AngleAxisd` 会给出不同但等价的旋转向量表示。日志中 `target` 与 `fk` 的旋转向量第一分量差了 0.15，经 `error_map` 后虽被压小，仍留下残余不一致。

**排查建议**

在仿真中对同一组关节角，分别打印 TF 给出的位姿与 `forward_kinamic()` 给出的位姿，逐项对比，确认差异来自：
1. 旋转向量转换算法；还是
2. 坐标系 / URDF 不一致（`world` 与 `base_link` 之间、xacro 处理前后、link6 帧定义）。

**当前对策**：判据放宽到 1e-3（1 mm / 0.057°），并**搁置**该问题。

---

### O2 `IKSolver` 不检测不可达目标【高】

**现象**

下发远超工作空间的目标（x = 5.0 m）时，阻尼最小二乘不会报错，而是**收敛到最近可达位姿** —— 机械臂朝目标方向大幅摆动约 90°，直到撞上关节限位才触发失败。

**影响**

任务层一个超出工作空间的坐标，换来的是**大幅意外运动**而不是立即拒绝。对「安全优先」是实打实的缺口。

**建议方向**

逆解返回后校验残差与关节限位（`ModelBase` 已有 `lower_joint_limit()` / `upper_joint_limit()`），不合格就拒绝执行；或在 `IKSolver::solve` 内对残差设上限。

---

### O3 `ArmRealInterfaces` 硬编码关节名与关节数，越界读写【严重】

**位置**：`src/arm_controller/controller/src/arm_real_interfaces.cpp`

**现象**（原记录只写到一半，实际有两个层次）

第一层是原记录写的问题：

```cpp
const auto joint_count = 6;                          // write() 与 on_pack_praser()
for (std::size_t i = 0; i < joint_count; ++i) { ... commands_[i] ... }
```

而 `commands_` / `received_states_` 是按 `joint_names_.size()` 分配的（同文件 `:38`）。
关节数不为 6 时：

- 关节数 **少于** 6：`on_pack_praser` 写 `received_states_[5]` —— **越界写**，破坏堆内存；
  `write` 读 `commands_[5]` —— 越界读，把垃圾数据当指令发给电机。
- 关节数 **多于** 6：第 7 个及以后的关节被静默丢弃，既不收也不发。

第二层是排查时才发现的更深原因：**`joint_names_` 本身就是硬编码的**

```cpp
std::vector<std::string> joint_names_{"joint1",...,"joint6"};
```

它从未从 `hardware_info` 读取，所以无论 URDF 怎么写，导出的接口名永远是
`joint1..joint6`。URDF 里把关节改名后，`export_state_interfaces()` 导出的接口名
和 URDF 对不上，而**报错信息完全指不到真正原因**。这也解释了为什么第一层
的硬编码 `6` 一直"碰巧"没炸——因为数组大小也恒为 6。

**修复**（`dbb7fe0`）

1. `joint_names_` 改由 `on_init` 从 `hardware_info.joints` 读取；
2. `on_init` 校验关节数必须等于 MCU 通信帧电机数，否则**拒绝启动**并打印明确原因；
3. 通信帧长度从 `data_pack.h` 的结构体定义反推（`sizeof(MCUTarget2Pack::motor)/sizeof(...)`），
   并加 `static_assert` 防止收发帧失配；
4. `write()` / `on_pack_praser()` 的循环边界收敛为 `min(容器大小, 帧电机数)`，作为纵深防御。

**验证**

新增离线工具 `tools/arm_real_joint_count_test.cpp`（不安装、不连硬件、不开 USB），
构造 5 / 6 / 7 关节的 URDF 走 `on_init` 与接口导出路径：

| 情况 | 断言 | 结果 |
| --- | --- | --- |
| 5 关节（少于帧） | `on_init` 拒绝启动 | PASS |
| 7 关节（多于帧） | `on_init` 拒绝启动 | PASS |
| 6 关节、关节名为 `arm_joint_1..6` | `on_init` 成功 | PASS |
| 同上 | 导出 18 个状态接口 | PASS |
| 同上 | 导出 36 个命令接口 | PASS |
| 同上 | 首个关节名取自 URDF（`arm_joint_1`） | PASS |

**负向对照**：把两个文件还原为修复前的版本重新构建，同样的工具在
「5 关节拒绝启动」「7 关节拒绝启动」「首个关节名取自 URDF」三项上**失败**
（实际导出 `joint1`），证明该工具确实能捕获此缺陷，而不是恒真断言。

---

### O4 launch 经 `/robot_description` 话题取 URDF，易静默加载错模型【中】

**现象**

`ros2_control_node` 是从 **`/robot_description` 话题**取 URDF 的，不是用自己的参数：

```
[controller_manager]: Subscribing to '/robot_description' topic for robot description.
[controller_manager]: Received robot description from topic.
```

同 ROS domain 内只要有**任何**其他 `robot_state_publisher`（实测遇到另一个项目的残留进程），controller_manager 就会静默加载那个机器人的模型，所有控制器加载失败，而报错信息完全指不到真正原因。

**建议方向**

在 launch 中显式等待/校验 `robot_description` 来源，或让 controller_manager 直接读 URDF 文件而非话题。

---

### O6 复位静差超容差后回退 `idel`，重力静差被叠加成 12.4° 偏置【高】

**发现经过**：修 O3 时跑仿真冒烟测试，每次控制器激活都稳定出现：

```
[WARN] [arm_controller]: reset did not reach the target within 5.00 s
       (reset_duration=3.00 s, reset_settle=2.00 s):
       max joint error 0.0953 rad on joint4 (tolerance 0.0100 rad);
       forcing the switch to idel, check reset_tolerance / default_kp (gravity sag)
       if the offset persists
```

**已确认不是 O3 引入的**：把两个源文件还原为修复前的版本重新构建，同样的 WARN
以同样数值出现（`0.0961 rad`，与 `0.0953` 的差别只是噪声）。`ArmRealInterfaces`
在仿真里根本不会被实例化（仿真 URDF 用的是 `mujoco_ros2_control/MujocoSystemInterface`），
所以与本次修改无关。

**这不是"复位没做准"，而是两个误差在叠加**

`reset` 因静差超容差放弃 → 强制切 `idel` → `IDELState::enter()` **锁存当前位置** →
`idel` 用同一组增益保持，重力把关节**再压低一段**。

实测（`ready` 以外的零位姿，仿真 `ROS_DOMAIN_ID=74`）：

| 量 | 值 |
| --- | --- |
| `reset` 放弃时的 joint4 误差 | 0.0953 rad |
| `idel` 稳态下 joint4 的力矩（`/joint_states.effort`） | 1.2438 N·m |
| `default_kp[3]` | 10.0 |
| 预期附加静差 `τ/kp` | 0.12438 rad |
| **预期最终误差** | **0.2197 rad** |
| **实测最终 joint4 位置** | **0.2171 rad（12.4°）** |

预测与实测差 1.2%，机制成立。最终关节速度约 `1.7e-13 rad/s`，是稳态而非还在下沉。
此时 `/arm_controller/state` 为 `idel`，机械臂**静默停在偏离零位 12.4° 的位置**。

**为什么这个偏置没有被任务层发现**

`task_runner` 的 `reset` 步骤只等 `idel`，而"复位成功"和"复位放弃后回退"都会到达
`idel`，两者不可区分。任务因此继续执行，带着 12.4° 的初始偏置。

**为什么只有零位姿暴露**

`default_kp = [200, 200, 200, 10, 10, 1]`。零位姿下 joint4 承担的力矩最大，
`kp = 10` 撑不住；而 `ready` 位姿（joint2=0.3, joint3=−0.3, joint5=0.3）下 joint4
力矩很小，所以 `joint_move` 到 `ready` 能到 0.0001 rad。**静差是位姿相关的**，
换个姿态就可能突然超标。

**影响**

- 任何以"复位完成 = 回到零位"为前提的任务都带着这个偏置，且**没有任何提示**
- `default_kp[5] = 1.0` 使 joint6 近乎自由，实机上风险更大
- 与 F1 同源：F1 是保持命令**没有**刚度，这里是有刚度但**不足**

**建议方向**（未定，需与用户确认）

1. 提高 joint4/5/6 的 `default_kp`——最直接，但可能引入振荡，需重新验证全部序列；
2. 加积分项 `ki` 或重力前馈补偿——能消静差而不必加大 `kp`，但要防积分饱和；
3. 让 `reset` 的放弃路径**报告失败**而不是静默回退，至少让任务层能区分；
4. **不要**用放宽 `reset_tolerance` 来掩盖——那只是把 12.4° 的偏置合法化。

**修复**（`c7abf73`，采用方向 2 的重力前馈）

`reset` / `idel` 用 RNEA 在 `dq = ddq = 0` 下算出重力力矩，写入 `command.torque`
作为力矩前馈。这与 `teach_pendant` 已有的机制完全一致（同一份模型、同一个前馈字段），
不必把 `kp` 加大到可能振荡的程度，也不引入积分饱和风险。

新增参数 `gravity_compensation`（默认 `true`）作为关闭开关：这是实机上唯一一处
**主动输出力矩**的保持路径，怀疑模型质量或力矩符号时应当能立即关掉。计算包在
`try/catch` 中并检查 `allFinite()`，失败时退回 `torque = 0` 并打限流 WARN——
即退化为修复前的行为（有静差但不会失控），不会把 FSM 拖进故障路径。

所有 buffer 在构造阶段分配，`run()` 中无内存分配，满足实时约束。

**验证（A/B 对照，MuJoCo 仿真）**

| 条件 | `reset` 告警 | joint4 稳态位置 | 序列结果 |
| --- | --- | --- | --- |
| `gravity_compensation: false` | **出现**，误差 0.0960 rad | 0.2171 rad（12.4°） | — |
| `gravity_compensation: true` | 无 | **−0.0050 rad** | 见下 |

`true` 时静差 −0.0050 rad，落在 `reset_tolerance = 0.01` 之内。关闭时复现的
0.0960 rad 与修复前实测的 0.0953 / 0.0961 rad 一致，说明开关确实控制住了这条路径。

端到端（`gravity_compensation: true`）：

| 序列 | 退出码 | 末端位置误差 | 控制器 WARN/ERROR |
| --- | --- | --- | --- |
| `ready_then_cart` | 0 | 0.0003 m | 无 |
| `pick_demo` | 0 | 0.0004 m | 无 |

**遗留**

- 静差 −0.0050 rad 而非 0，残差来自模型参数与实际动力学（含摩擦）的差异，
  由 `kp` 兜住。若模型参数经辨识后更新，这个残差还会变小。
- 任务层仍然无法区分「复位成功」与「复位放弃」（上面的方向 3 未做）。当前
  开启补偿后不再触发放弃路径，但**换个位姿或模型失配时仍可能触发**，
  那时任务层依旧会带着偏置继续执行。
- `default_kp[5] = 1.0` 使 joint6 近乎自由，实机上风险更大，未单独处理。

---

## 3. 设计约束（非缺陷，但必须遵守）

### C1 零位形是奇异位形

`reset` 的目标（全零）对应机械臂完全伸直的位形，**雅可比降秩**。从那里直接做笛卡尔运动，逆解不收敛（原 bugs.md B3）。

**做法**：笛卡尔任务务必先 `joint_move` 离开零位形。配置里 `reset_then_cart` 作为反例保留并注明，可用的参考序列是 `ready_then_cart`。

### C2 控制器状态发布有 500 ms 滞后

`current_state` 参数与 `~/state` 话题由 `ArmController` 的 500 ms wall timer 驱动。任务层任何「切状态后立刻判断」的逻辑都必须先等过该周期（见 F9 的 `state_settle`）。

### C3 轨迹状态 `enter()` 会把轨迹阶段重置为 STOP

`CartTrajState` / `JointTrajState` 的 `enter()` 都会执行 `state = TrajPhase::STOP; traj.stop();`。因此**从任何状态切入都保证轨迹阶段就绪**，`joint_move → cart_move` 可以直接串联，不需要中间插 `idel`。

### C4 保持命令必须基于锁存值

见 F1。任何「保持当前位置」的实现都必须在进入时锁存一次，绝不能每周期重读实测值。

### C5 `yaml_cpp_vendor` 的 CMake 定位不可靠


本机 `yaml_cpp_vendor` 的 ExternalProject 未构建，其 `extras.cmake` 会把 `yaml-cpp_DIR` 指向不存在的路径。必须显式 `find_package(yaml-cpp REQUIRED)` 并链接 `yaml-cpp::yaml-cpp`；且因 `ament_target_dependencies` 使用 plain 签名调用 `target_link_libraries`，不能混用 `PUBLIC`/`PRIVATE`。

### C6 控制器参数必须在 `on_init` 中 `auto_declare`，否则 YAML 里的键被静默忽略

`ArmController::on_init` 用 `auto_declare<...>()` 逐个声明参数（`controller.cpp:126` 起），
之后各处才用 `get_parameter()` 读取。**未声明的键即使写在 YAML 里也读不到**，
`get_parameter()` 返回 `false`，代码取的是 `auto_declare` 里的默认值，
而且**没有任何警告**。

修 O6 时踩到过：第一版只在 YAML 里加了 `gravity_compensation: false` 而没有声明它，
结果开关看起来完全无效，`joint4` 依然停在 −0.005 rad。加一行诊断才看清：

```
idel:   gravity_compensation=true (param found=no)
reset:  gravity_compensation=true (param found=no)
```

**影响**：任何控制器参数写错名字（或改名前忘了同步）都会静默退回默认值。
调 `reset_tolerance` / `default_kp` 这类安全相关参数时，这会导致「改了但没生效」
却毫无提示。

**对策**：新增参数时三处同步——`auto_declare`、YAML、读取处；关键参数建议像
`gravity_compensation` 一样在构造阶段打一行 INFO 把实际生效值打出来。

---

## 4. 修复与提交对照

| 提交 | 内容 |
| --- | --- |
| `ba63893` | 阶段 1：控制器故障策略收拢（F11） |
| `59bc567` | 阶段 2：任务层改为配置驱动（F9、F10） |
| `6543b23` | 阶段 3：`joint_move` / `cart_move` + 恢复故障阶段诊断（F7） |
| `393598d` | 修复保持命令失去刚度（F1）+ 对齐 MJCF 关节限位（F3、F8） |
| `729df71` | 修复旋转向量 \|rv\|≈π 奇异（F2） |
| `7519118` | 逆解器改为信赖域 + LM 并离线定参（F4、F5、F6） |
| `dbb7fe0` | 修复实机接口硬编码关节名与关节数（O3） |
| `c7abf73` | reset / idel 加入重力前馈（O6） |

---

## 5. 本轮的方法论教训

1. **把策略集中化时，诊断信息必须一并迁移**（F7）。否则定位问题时要先把诊断补回来。
2. **清理动作应放在生命周期钩子（`exit()`）里**，不要散落在 `run()` 的失败路径（F8）。
3. **调参必须有离线基准**。走「改代码 → 编译 → 起仿真 → 跑任务」一轮两三分钟，而离线基准几百用例只要毫秒级——信赖域参数就是靠它扫出来的。
4. **端到端测试不可替代**。F1（保持无刚度）与 F9（0.5 ms 假成功）都是静态审阅看不出来、必须真跑才会暴露的缺陷。
5. **「已经收敛却报失败」要先怀疑判据而非算法**（F5）。判据的物理含义（这里是米+弧度混合范数）常被忽略。
6. **实时回路里的迭代上限就是最坏执行时间**（F6）。不收敛时的空转必须提前退出。

---

## 6. O1 排查结论：TF 与正运动学**并不**不一致，真因是任务雅可比

### 6.1 原假设被证伪

原记录（§2 O1）推测「任务层经 TF 读到的末端位姿与控制器 Pinocchio 正运动学不一致约 1e-3」。
用新增的离线工具 `tools/fk_dump.cpp` 对**同一组关节角**逐项对比后，二者完全一致：

| 项 | TF（`robot_state_publisher`，xacro 处理后的 URDF） | Pinocchio（控制器用的原始 URDF） |
| --- | --- | --- |
| 平移 | `[-0.046, -0.037, 1.257]` | `[-0.046321928, -0.037280518, 1.257137189]` |
| 四元数 (xyzw) | `[-0.122, -0.000, 0.992, -0.000]` | `[-0.122420878, -0.000175566, 0.992478258, -0.000074760]` |

**结论：不存在坐标系 / URDF / 帧定义的不一致，也不存在四元数→旋转向量转换的差异。**

### 6.2 之前看到的差值其实是跟踪误差

日志里 `target` 与 `fk(measured)` 的差值（约 0.02）是**轨迹跟踪误差**——`target` 是当前时刻的轨迹点，`fk(measured)` 是机械臂的实际位姿，两者本就不该相等。此前把它误读成运动学不一致。

### 6.3 真正的现象：IK 平台永远贴着收敛判据

| 判据 | IK 实际残余 |
| --- | --- |
| 1e-6 | 1.018e-6 |
| 1e-4 | 1.519e-4 |
| 1e-3 | 1.051e-3 |
| 5e-3 | 5.083e-3 |

残余**始终恰好略高于判据**，无论判据取多少。这不是数值精度问题，而是迭代**根本无法继续下降**。

### 6.4 根因（O5）：任务雅可比在 \|rv\| = π 处失效

`Default6DofTaskSpaceMapping::jacobian_map` 的实现是：

```cpp
task_jacobian->noalias() = jacobian;   // 直接照搬几何雅可比
```

但任务空间的姿态坐标是**旋转向量 ρ**，其变化率与角速度 ω 之间差一个矩阵：

```
ω = J_r(ρ) · ρ̇
```

`J_r(ρ)` 在 **|ρ| = π 处奇异**（绕当前轴的转动退化，轴的正负号也不确定）。把几何雅可比直接当作任务雅可比，等于默认 `ρ̇ = ω`，这只在小角度下成立。于是在 |ρ| ≈ π 附近线性化失准，迭代只能缓慢逼近、永远够不到判据。

实测数据吻合：失败时 `target` 的 `|rv| = 3.1248`（≈ π），IK 在 41～63 次迭代后停滞。

### 6.5 修复方向（未实施）

在 `jacobian_map` 中把几何雅可比的角速度部分换算成旋转向量变化率：

```
J_task_rot = (dρ/dω) · J_geom_rot
```

其中 `dρ/dω` 是 `J_r(ρ)` 的逆。这样任务雅可比才与任务坐标（旋转向量）一致。

**更彻底的替代方案**：放弃旋转向量表示，任务空间姿态改用四元数或直接用旋转矩阵增量。这是原始 `bugs.md` B5 提到的方向（「当前实现是 6 自由度专用」）。

### 6.6 当前状态

- `cart_move` / `admittance_move` 仍被 O5 阻塞，`ready_then_cart` 无法通过
- 收敛判据暂定 `1e-3`（1 mm / 0.057°），并注明它是「实用阈值」而非可达的数值精度
- O2（不可达目标不报错）与 O5 是同一片区域的问题，建议一并处理

---

## 7. O5 与笛卡尔失败的真因（第二轮排查）

### 7.1 O5 修正已实现，但不是失败原因

按 §6.5 的方向实现了误差雅可比：`J_task_rot = +J_r(ρ_e)·J_geom_rot`
（右雅可比，`ρ_e` 为**误差**旋转向量）。

推导：`e_r = log(R_t R_cᵀ)` 的时间导数为 `de_r/dt = -J_r(ρ_e)·ω_c`；Gauss-Newton 需要
`J = -∂e/∂q`，故为 `+J_r(ρ_e)·J_ω`。小误差时 `J_r → I`，退化为原实现——这也解释了
为什么原实现在小误差下一直可用。

**实测：基准指标与修正前完全一致**（近种子 100% / 2.5 次迭代 / 最差 9.952e-04）。
原因是在涉及的误差量级（~0.09 rad）下 `J_r ≈ I`，修正数值上可忽略。修正本身是正确的
（线性化变精确），予以保留；但它**不是**笛卡尔失败的原因。

> 注意：中途曾误用 `J_l(ρ_c)⁻¹`（左雅可比、当前位姿的 ρ），基准从 100% 掉到 49.7%，
> 已回退。**误差用相对旋转、雅可比就必须用误差的右雅可比**，两者必须配套。

### 7.2 真因：目标位姿本身不可达

用新增的 `tools/ik_solve.cpp` 离线复现仿真失败用例，**逐位复现**（`ik_err = 1.0256e-03`）。

分离测试：

| 测试 | 目标 | 结果 |
| --- | --- | --- |
| A | 目标位置 + 初值姿态 | ✅ 2 次迭代收敛 |
| B | 初值位置 + 目标姿态 | ✅ 3 次迭代收敛 |
| C | 目标位置 + 目标姿态 | ❌ 74 次迭代，残余 1.03e-3 |

**位置单独可达、姿态单独可达，组合起来不可达。**

再用 **6 个差异很大的初值**求解同一目标，**全部收敛到同一个残余 1.025608e-03**——
这是全局最小值，证明目标落在可达集之外，而非 IK 陷入局部极小。

### 7.3 结论

`ready_then_cart` 要求从 `ready` 位姿沿 +Z 抬升 5 cm 且**姿态保持不变**。机械臂在该位姿
处已到工作空间边界，这个组合目标不可达。

**IK 报失败是正确行为**：收敛到最近点 → 残余超判据 → 保持并回退 idel。这正是 F11 建立的
安全语义。

**因此：**
- 不是 IK 缺陷，不是运动学不一致，不是雅可比问题
- 真正缺的是 **O2**：目标不可达时应当**明确报告「不可达」**，而不是表现为一次
  令人费解的「逆解失败」。任务层也应在下发前做可达性检查
- `ready_then_cart` 这个测试序列本身要求了不可达的运动，需改用可达的目标

### 7.4 新增排查工具（均不安装）

| 工具 | 用途 |
| --- | --- |
| `tools/fk_dump.cpp` | 打印指定关节角下的平移、四元数、旋转矩阵与任务空间 6 维向量，用于与 TF 逐项对比 |
| `tools/ik_solve.cpp` | 用给定目标与初值跑一次 IK，离线复现仿真失败用例 |
| `tools/ik_benchmark.cpp` | 批量采样统计收敛率与迭代次数，用于扫描求解器参数 |
