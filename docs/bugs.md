# 缺陷记录

> 范围：`src/arm_controller`（控制器 / 状态机 / 求解器）、`src/tasks`（任务层）、
> `src/arm_model`（模型）、`src/launch_pack`（launch / 控制器配置）
> 起点：把任务层从「7 个 C++ 任务类」重构为「配置驱动执行器」的过程中，为验证链路
> 而做的端到端测试暴露了一批缺陷；此后又做了两轮专项排查（O1/O5 的笛卡尔失败、
> O3/O6/O2/O4 的修复）。

**怎么读这份文件**

| 章节 | 内容 |
| --- | --- |
| §0 | **总览表——最新状态以它为准** |
| §1 | 已修复缺陷的详细记录（现象、根因、修法、验证） |
| §2 | O 系列条目正文（部分已修、部分已证伪，正文保留是因为实测数据仍有价值） |
| §3 | **设计约束 C1–C6，动手前必读** |
| §4 | 修复与提交对照 |
| §5 | 方法论教训 |
| §6 / §7 | O1 / O5 两轮专项排查的完整记录 |

**当前状态**：F1–F12 全部已修；O1 已证伪；O5 已修（数值影响可忽略）；
O2 / O3 / O4 / O6 / O7 已修；**唯一未修的是 O8**——参数辨识链路已跑通，但辨识结果非物理。
根因已定位：辨识用的加速度由 500 Hz 速度二次求导得到（误差 31.5%），是**辨识链路**的估计
误差，**不是**测量数据本身有问题——记录力矩与刚体动力学在机器精度上一致（见 §2 O8）。

> 说明：本文件承接已删除的旧 `bugs.md`。原文件中的 A1/A4/A5/A7 已修复，
> A2 已复现并修复，B3 确认为设计约束。

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
| **O5** | `jacobian_map` 把几何雅可比直接当任务雅可比，姿态用旋转向量时在 \|rv\|=π 处线性化失效 | **高** | **已修**（数学正确，但数值影响可忽略，见 §7.1） | `729df71` |
| **O2** | `IKSolver` 不检测不可达目标，投影到最近可达位姿 | 高 | **已修** | `214ad60` |
| **O3** | `ArmRealInterfaces` 硬编码关节名与关节数，越界读写 | **严重** | **已修** | `dbb7fe0` |
| **O4** | launch 经 `/robot_description` 话题取 URDF，易静默加载错模型 | 中 | **已修** | `ac474f0` |
| **O6** | 复位静差超容差后回退 idel，重力静差被叠加成 12.4° 偏置 | **高** | **已修** | `c7abf73` |
| **O7** | `write_hold_command` 无重力前馈，在轨迹状态等待期间手臂塌陷 5° | **高** | **已修** | `14810bf` |
| **O8** | 辨识用加速度由 500 Hz 速度二次求导得到（误差 31%），导致辨识结果非物理 | **高** | **根因已定位，未修** | — |
| **O9** | measure launch 的覆盖参数（`csv_file_path` / `trajectory_period` 等）全部静默失效 | 中 | **已修** | `00001b8` |
| **O10** | 激励轨迹不可复现、质量波动极大（CMA-ES 随机种子取自时钟、600 次评估对 36 维欠收敛） | **高** | **已修** | `06af06d` |

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

## 2. O 系列条目正文

> **本节标题原为「未修复（已搁置）」，现已不准确**：O2 / O3 / O4 / O6 / O7 已修，
> O1 已证伪，**O8 未修**。**最新状态以 §0 总览表为准。**
>
> 已修条目的正文刻意保留，因为其中的实测数据与推理过程仍是理解这些问题的依据。

### O1 TF 与控制器正运动学不一致（~1e-3）【高】—— **已证伪，见 §6**

> **结论先行：TF 与控制器正运动学并不存在不一致，二者完全一致。**
> 下面这段记录保留的是当时的**错误推测**，不要照着它去排查。
> 真正的现象、证伪过程与真因见 §6 与 §7：之前看到的 ~1e-3 差值其实是**轨迹跟踪
> 误差**，而笛卡尔失败的真正原因是**目标位姿本身不可达**（已按 O2 处理）。

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

**现象（修复前实测，原记录的现象已过时）**

原记录写「机械臂朝目标方向大幅摆动约 90°，直到撞上关节限位才触发失败」。这是
F11（IK 失败导致控制器链被停用）与信赖域逆解器之前的现象。修复前重新实测的结果是：

下发从 `ready` 沿 +X 平移 2 m 的目标，手臂会**沿轨迹一路走到工作空间边界**
（joint2 转 −0.404 rad、joint4 转 +0.428 rad，末端走了约 **0.40 m**），
然后因逆解失败中止。不再是 90° 摆动，但**大幅意外运动依然存在**。

**为什么诊断信息具有误导性**

失败日志打印的 `ik_err` 是「最近可达位姿到目标的距离」。临界不可达时这个数恰好
落在 `1.045e-3`，只比 `1e-3` 的判据高 **4.6%**，看上去像「差一点点没收敛」，
而不像「目标在可达集之外」。任务层则只报 `likely a solve failure`。

用 **6 个差异极大的初值**求解同一目标，**全部收敛到同一残余 `1.045997e-03`** ——
这是全局最小值，证明目标确实不可达，`ik_err` 不是数值噪声。

**影响**

任务层一个超出工作空间的坐标，换来的是「先大幅运动、再中止、且看不出原因」。

**修复**（`214ad60`，两部分）

1. **三态诊断**：`IKSolver::solve` 由 `bool` 改为
   `SolveResult{Converged, Unreachable, Failed}`。停滞与迭代次数用尽都归为
   `Unreachable`（目标没有达到），只有输入非法 / 雅可比不可用 / 步长非有限才归为
   `Failed`（可达性未知）。控制器据此把「目标不可达，最近可达位姿还差 X」与
   「逆解数值失败」分开报。

   > 迭代次数用尽必须算 `Unreachable`：实测 +X 2 m 的目标残余 1.2 m，但每一步都
   > 能略微下降，连续拒绝计数永远攒不满 8，只能靠 200 次上限退出。若把它归为
   > `Failed`，恰恰是 O2 最典型的用例反而报「可达性未知」。

2. **下发前可达性预检**：收到笛卡尔轨迹后先对**终点**做一次逆解，不收敛就
   **整条轨迹拒绝、一个运动指令都不下发**，并明确报告不可达及其残余。

   > 只查终点：单次逆解实测最坏约 **386 µs**（200 次迭代，`ik_solve` 重复 200 次测得），
   > 而控制周期是 2 ms——查 8 个途经点就可能超周期。预检在 `run()` 的 MOVING
   > 首周期完成，早于 `traj.update()` 与任何逆解/逆动力学，因此拒绝时运动尚未开始。

**验证（MuJoCo 仿真）**

离线分类（`tools/ik_solve.cpp`，种子均为 `ready`）：

| 目标 | 结果 | 残余 |
| --- | --- | --- |
| `ready` 自身 | Converged | 5.2e-10 |
| `ready` + X 5 cm | Converged | 7.6e-4 |
| `ready` + Z 5 cm | **Unreachable** | 3.86e-2 |
| `ready` + X 2 m | **Unreachable** | 1.20e+0 |

**拒绝确实没有产生运动**：`unreachable_near` 与 `unreachable_far` 两个反例终止时的
关节角几乎完全一致（`0.32083 / 0.32086`、`0.08919 / 0.08926`、`0.305404 / 0.305405`）。
修复前两者会各自朝自己的目标走到完全不同的位形。

回归：`reset_only` / `reset_then_joint` / `ready_then_cart` / `pick_demo` 退出码均为 0，
末端位置误差 0.0005 / 0.0004 m；控制器全程只有两次预期内的拒绝 ERROR 与配套
safety hold，无其他 WARN/ERROR。

新增 `unreachable_near` / `unreachable_far` 两个反例序列作为 O2 的回归用例。

**遗留**

- 只预检**终点**。中间途经点不可达时仍会先运动再中止（当前任务层只发起点+终点
  两个途经点，实际不受影响）。
- 预检的残余是**从当前位姿出发**得到的局部极小值，不是到可达集的全局距离。
  同一个目标换个初值可能给出不同的残余（实测 +Z 5 cm 从 `ready` 出发是 3.86e-2，
  从已经移动过的位姿出发是 1.046e-3）。两者都超判据，所以「拒绝」的结论不变。

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

**复现与补充实测**（`ac474f0` 之前）

先起一个发布 1 关节诱饵模型的 transient_local 发布者，再起本 launch：

```
[controller_manager]: Subscribing to '/robot_description' topic for robot description.
[controller_manager]: Received robot description from topic.
[controller_manager]: Received robot description from topic.          ← 收到两条
[ERROR] [controller_manager]: Exception caught while loading and initializing components:
                                  no 'ros2_control' tag found in the URDF   ← 诱饵导致的
```

两条额外结论：

1. **把 `robot_description` 作为参数传给 controller_manager 没有用**——实测它照样
   订阅话题。参数不能作为防护。
2. **初始化完成之后再收到外来描述会被忽略**，只打一条 WARN：

   ```
   [WARN] [controller_manager]: ResourceManager has already loaded a urdf and is initialized.
                                Ignoring attempt to reload a robot description.
   ```

   所以风险窗口只在**启动阶段**，但正是启动阶段决定了加载哪个模型。另外「最后一条
   描述获胜」意味着谁能抢到最后一条谁就赢——这是一场竞态。

**修复**（`ac474f0`）

把**发布端**（`robot_state_publisher`）与**订阅端**（`controller_manager`）一起重映射到
本工作区专用话题 `arm_robot_description`，从结构上杜绝串扰，而不是靠事后检测。
RViz 的 RobotModel 显示同步指向新话题。三个会拉起 controller_manager 的 launch
（`arm_controller_test_sim` / `arm_measure_sim` / `arm_mujoco_sim`）全部同步。

> 话题重映射不影响同名参数，`robot_state_publisher` 仍从 `robot_description`
> **参数**读模型，只是发布到新话题名。

**验证**（诱饵发布者全程在场）

| 检查项 | 修复前 | 修复后 |
| --- | --- | --- |
| CM 订阅的话题 | `/robot_description` | `/arm_robot_description` |
| CM 收到的描述条数 | 2（含诱饵） | **1**（只有本机的） |
| 诱饵导致的错误 | 1 条异常 | **0** |

功能回归：`reset_only` / `reset_then_joint` / `ready_then_cart` / `pick_demo`
退出码均为 0，`unreachable_far` 按预期退出码 1。

**遗留**

- 话题名不再是标准名，外部工具（如 `ros2 topic echo /robot_description`、其他
  依赖标准名的可视化工具）需要改用 `/arm_robot_description`。本工作区内除
  controller_manager 外没有订阅者，RViz 配置已同步。
- `arm_real.launch.py` / `arm_static_display.launch.py` 未同步——它们引用了不存在的包，
  当前不可用（见 CLAUDE.md）。

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

### O7 `write_hold_command` 没有重力前馈，在轨迹状态等待期间手臂塌陷 5°【高】

**发现经过**：验证 O2 时做 A/B 对照，发现两条都「被拒绝、没有运动」的序列，终止姿态
比只做关节运动的序列在 joint4 上多了 **0.089 rad（5.1°）**。追下去发现与 O2 无关。

**根因**：O6 只给 `reset` 和 `idel` 加了重力前馈，但**通用的保持路径
`write_hold_command()` 仍然写 `command.torque = 0`**（修复前位于 `arm_fsm.cpp:134`）。
`cart_traj` / `joint_traj` / `admittance` 等状态在等待轨迹到达、以及轨迹结束回到
`STOP` 期间，走的都是这条路径。

实测时间线（`ready_then_cart`）：

```
[task_runner] exp_state -> cart_traj      ← 控制器进入 cart_traj，开始用 torque=0 保持
   ...约 0.9 s...
[task_runner] cart_traj: start [...]      ← 轨迹才到达
```

这 0.9 s 里 joint4 按 τ/kp 自由塌陷：τ ≈ 0.89 N·m、`default_kp[3] = 10`
→ 静差 0.089 rad。证据是 `cart_traj` 打印的起始位姿 x = −0.186，而 `ready` 的
正解是 x = −0.1479——手臂在运动开始前就已经偏离 `ready` 了。

**影响**

- 任何进入 `cart_traj` / `joint_traj` / `admittance` 的动作，起点都不是指令要求的位姿，
  而是已经塌陷过的位姿，**且没有任何提示**。笛卡尔运动尤其危险：起点偏了，
  整个笛卡尔轨迹的绝对位置也跟着偏。
- 与 O6 同源（保持刚度不足），也与 F1 同源（F1 是保持**没有**刚度）。

**修复**（`14810bf`，即当初建议的方向）

把补偿下沉为 `HoldGravityFeedforward`（`arm_fsm.hpp`），`reset` / `idel` 与
`write_hold_command()` **共用同一份实现**。O6 当初只补了 `reset` / `idel` 两条路径，
这次把同一根因的第三条路径并进来，避免它们再次分叉：

- `write_hold_command()` 增加 `gravity` 参数，每周期 `refresh()` 后写
  `command.torque`；`CartTrajState` / `JointTrajState` / `AdmittanceState` 各持有一个
  实例，在构造阶段 `initialize()`（读参数 + 分配 buffer）；
- `reset` / `idel` 原来各自复制的 4 个成员与 `refresh_gravity_torque()` 一并删除，
  改走同一个类——**这是本次改动的重点：让「还有谁走这条路」在类型层面不再成立**；
- 实时约束不变：buffer 只在构造阶段分配，`refresh()` 只读写已分配内存，
  `run()` 中无动态分配；
- 复用 `gravity_compensation` 开关与 `compute_gravity_torque` 的
  `try/catch + allFinite()` 兜底，失败时退回 `torque = 0`（有静差但不会失控）。

> 实现细节：helper 是 `HoldGravityFeedforward` 的成员函数（声明在头文件、定义在
> `.cpp` 匿名命名空间之后），因此既能被各状态按值持有，又能调用 `.cpp` 内的
> `compute_gravity_torque`——绕开了 `FSMArmControlFactory` 只有前置声明的限制。

**验证（MuJoCo 仿真，`ROS_DOMAIN_ID=74`，A/B 对照）**

启动日志新增 5 行 `gravity_compensation=... (param found=yes)`（`idel` / `reset` /
`cart_traj` / `joint_traj` / `admittance`），可直接确认新路径已接入；修复前只有前两行。

| 条件 | `cart_traj` 起始位姿 x | `ready` 正解 | 复位告警 | 序列结果 |
| --- | --- | --- | --- | --- |
| `gravity_compensation: true` | **−0.148** | −0.1479 | 无 | `ready_then_cart` 退出码 0，末端误差 0.0004 m |
| `gravity_compensation: false` | **−0.185** | −0.1479 | joint4 静差 0.0992 rad | 复现修复前的塌陷与 O6 静差 |

两次测量都在 `exp_state -> cart_traj` 之后约 **0.90 s** 取起点位姿——正是上面那条
时间线里的窗口。关闭补偿时复现的 −0.185 与修复前记录的 −0.186 一致，说明这个差值
确实来自这条保持路径，而不是环境或时序变化。

回归（补偿开启）：`reset_only` / `reset_then_joint` / `pick_demo` 退出码均为 0，
`pick_demo` 的笛卡尔段起始位姿同样是 −0.148、末端误差 0.0007 m；`unreachable_far`
按预期退出码 1（下发前被拒绝）。控制器日志除该反例的两条预期 ERROR 外无 WARN/ERROR，
`reset did not reach the target` 一次都没出现。

**遗留**

- 补偿依赖模型参数，静差不会完全归零（`idel` 稳态仍有约 −0.0050 rad，见 O6）；
- `admittance` 的保持阶段现在也带前馈，但该状态本身仍没有序列覆盖（见 CLAUDE.md
  「已知问题」），本次只验证到「能进入并保持」这一层；
- 实机未验证。这是实机上第二处主动输出力矩的路径，怀疑模型质量或力矩符号时先关掉
  `gravity_compensation`（与 O6 同一个开关）。

**状态**：已修（`14810bf`）。O2 的验证曾排除它对结论的干扰（两条被拒序列的终止姿态一致）。

---

### O8 参数辨识结果非物理【高】——根因：加速度由 500 Hz 速度二次求导得到（误差 31.5%）

**发现经过**：跑通 measure → CSV → `identify_arm` 的完整链路时，辨识在物理一致性检查处
失败。先修掉两处工具链缺陷（见 §4 的 `f8b61c1` / `52cc021`），修完后才暴露出真正的问题：
辨识结果非物理，根因见下面的「排查过程」。

**现象**

`reconstruction.method: sdp` 报 `status=error`（SDP 求解失败）。把问题拆开实测：

| 测试 | 结果 |
| --- | --- |
| 只留逐杆伪惯量 PSD（去掉 `M·θ = φ_base`） | **可行** |
| 只留等式 `M·θ = φ_base`（去掉 PSD） | **可行** |
| 两者同时 | **不可行**（把等式放宽到 1.0 才可行） |

关键量：`‖φ_base‖ ≈ 1.98`，而 `‖φ_base − M·θ_prior‖ = 0.365`——辨识把基参数拉离 URDF
先验 **18.5%**，并且拉出了物理可行集。

**原始辨识值明显非物理**（`nullspace`，未投影）：

| 量 | 辨识值 | URDF 真值 |
| --- | --- | --- |
| joint3 `Izz` | **−0.153** | +0.00278 |
| joint3 `Ixx` | **−0.00372** | +0.00013 |

**投影路线能写出文件，但结果是退化的**（`nullspace` + `physical_projection: true`）：

| 量 | 投影后（绕质心） | 原始 URDF |
| --- | --- | --- |
| joint3 `ixx / iyy / izz` | `6.2e-10 / 6.4e-10 / −1.8e-10` | `1.3e-4 / 2.7e-3 / 2.8e-3` |

即 LMI 投影把连杆压成了**质点**（惯量塌 7 个数量级）。报告里的
`physical_validation.passed: true` 只表示"满足 PSD 约束"，**不代表模型可用**；投影后的基残差
是 `0.35`，等于放弃了 18% 的基参数拟合。

**已排除的原因**

| 假设 | 实测 |
| --- | --- |
| picos / cvxopt 缺失 | 已装（picos 2.6.2 / cvxopt 1.3.3），LP 与 SDP 都能正常求解 |
| CAD 约束过紧 | 去掉 `cad_constraints` 后照样不可行；放宽到 `mass_scale 0 ~ 1e6` 也一样 |
| URDF 与 MJCF 模型不一致 | 逐杆质量/质心/惯量**数值一致**（MJCF 的 `diaginertia` 与 URDF 只是主轴排列不同） |
| 仿真里的关节阻尼/摩擦 | MJCF 的 `<joint>` 没有 `damping` / `frictionloss` |

**结论（已定位）**：模型没问题、力矩也没问题——**唯一的问题是加速度算不准**。

**排查过程（2026-10-11）**

| 检查 | 方法 | 结果 |
| --- | --- | --- |
| 力矩语义 | 取准静态样本（\|dq\| 最小），比较记录力矩与 URDF 的 RNEA 重力力矩 | 比值 **1.003 / 1.006 / 1.001**——语义正确 |
| 接口实现 | 查 `mujoco_system_interface.cpp` | 导出的 effort = `data->qfrc_actuator[dof]`，即 MuJoCo 施加的关节力，语义正确 |
| 时序错位 | 力矩相对状态平移 ±6 个采样，看基拟合残差 | 残差平坦（0.0723 → 0.0790），**不是时序问题** |
| 管线正确性 | 用同一组 `(q, dq, dq̈)` 由模型**正演**出完美力矩，再跑同一条辨识 | 基参数残差 **5.5e-14**——管线能从完美数据精确复原真值 |
| 激励条件数 | `cond(W_base)` | **138**（奇异值 2260 → 16.3），激励充分，不是病态 |
| 模型一致性 | 逐杆比较 URDF 与 MJCF 的质量/质心/惯量 | 数值一致（<0.1%，仅主轴排列不同）；关节原点一致 |
| **加速度** | 由数据反解 `M(q)·a = τ − bias(q, dq)`，再与管线估计的加速度比较 | **相对误差 31.5%**（逐关节 30~45%）；改用均匀 dt 后 14.5% |

反解出的加速度满足 `‖M·a + bias − τ‖ = 1.4e-16`——**记录力矩与刚体动力学在机器精度上完全一致**，
说明模型、力矩语义、采样时刻都没有问题，唯一不准确的是"管线怎么得到加速度"。

管线目前的做法是"对 500 Hz 的**记录速度**做 Savitzky-Golay 平滑再求导"。换各种参数都救不回来：

| 加速度估计方法 | 相对误差 |
| --- | --- |
| SG(11,3) on vel + gradient（当前配置） | 0.315 |
| SG(5,3) ~ SG(201,5) on vel | 0.314 ~ 0.344 |
| SG(11,3) deriv=2 on pos | 0.204 |
| 二阶中心差分 | 2.17 |

即：**二次求导在 500 Hz 数据上恢复不出加速度**，最好的方法也只有约 20% 误差。而惯量项
`M(q)·q̈` 正是激励过程中力矩的主要成分，20~31% 的加速度误差直接污染了辨识，表现为基拟合
7.2% 的残差、基参数偏离 18.5%，以及最终的非物理。

**补充：时间戳抖动贡献了约一半误差**（2026-10-11 追加）

CSV 的时间列**不均匀**：`dt` 均值 0.002000 s（≈500 Hz），但 std 0.000375（**抖动 18.7%**），
范围 0.000804 ~ 0.002518 s。辨识按这些抖动的时间戳求导，误差被进一步放大。实测改用**均匀**
`dt = 均值` 后，加速度相对误差从 **31.5% 降到 14.5%**——即 31.5% 里大约一半来自时间戳抖动，
另一半才是二次求导本身的损失（`savgol_filter` 按等间隔设计，而 `np.gradient` 会按抖动
时间戳缩放，两者叠加更不稳）。

**顺带发现**：`identify.yaml` 的 `data.sample_time: 0.02`（50 Hz）与实际 500 Hz 的 CSV 差了
**10 倍**。当前链路里它没被用到（`build_regressor_basic` / `QRDecomposer` 都不读 `ts`），
但 vendored FIGAROH 的滤波工具会用它（`cutoff = ts * cut_off_frequency_butterworth / 2`），
一旦启用那条路径就会静默出错，应按实际采样周期修正。

**修复方向**

1. **把加速度记录进 CSV（推荐）**。CSV 格式与 `csv_loader` 已经支持 `acc_*` 列和
   `acceleration_source: recorded`，缺的只是控制器侧把它写出来：
   - 首选 MuJoCo 的真实 `qacc`——但当前系统接口只导出 position/velocity/effort
     （`mujoco_system_interface.cpp` 里没有关节加速度），需要给 `mujoco_ros2_control`
     加状态接口，改动落在第三方仓库；
   - 次选**激励轨迹的解析加速度**：`FourierTrajectory::acceleration(t)` 已经存在，但
     `ExcitationTrajectory` 只暴露了 `get_target_position`。加一个取速度/加速度的接口，
     在 `ParamterMeasureState::run()` 里随采样写入 `acc_*` 即可，改动全在本仓库内。
     注意解析加速度是**指令值**，其精度取决于跟踪质量。
2. 降低激励带宽或提高采样率——治标，且会牺牲激励信息量。

**状态**：已定位根因（加速度估计），**未修**。修法是让 measure 记录加速度（见上）；
在此之前 `identify.yaml` 保持"失败就报错"，不默认切到投影路线。

---

### O9 measure launch 的覆盖参数全部静默失效【中】

**现象**：下面这条命令**看起来一切正常**——launch 起得来、测量跑得完、退出码 0——
但 CSV 仍然落在 `ros2_controller.yaml` 的默认路径，测量时长仍然是默认的 10 s：

```bash
ros2 launch launch_pack arm_measure_sim.launch.py \
    csv_file_path:=/tmp/foo.csv trajectory_period:=120.0
```

**根因**：launch 把这些覆盖项拼成一个字典传给 `Node(parameters=[...])`。launch_ros 会把
字典序列化成 **node key 为 `/**`** 的 params 文件：

```yaml
/**:
  ros__parameters:
    measure_csv_file_path: /tmp/foo.csv
    measure_trajectory_period: 120.0
```

而 `controller_manager` 只把**本节点参数里 `arm_controller.*` 前缀**的项转给控制器
（`ros2_controller.yaml` 用的正是 `arm_controller:` + `ros__parameters:` 这种显式 node key）。
`/**` 下的裸参数只会留在 `controller_manager` 自己身上——`ros2 param dump /controller_manager`
**能看到**它们，控制器却读不到，于是 `auto_declare` 取默认值，**没有任何警告**。

**实测证据（2026-10-12）**

| 传入 | 生成的 params 文件 | 控制器实际生效值 |
| --- | --- | --- |
| `csv_file_path:=identify_loop2/meas_smoke.csv` | `/**` 下有该键 | 仍是 `measured_for_identification.csv`（写在进程 CWD） |
| `trajectory_period:=12.0` | `/**` 下有 `12.0` | 仍是 10.0（CSV 4998 行 ≈ 10 s） |
| 改为 node key = `arm_controller` 的补充 params 文件 | — | **6000 行 = 12 s，CSV 落在指定路径** ✔ |

**修法**：launch 改用 `OpaqueFunction`，在启动时生成一份 node key 为 `arm_controller` 的补充
params 文件，放在 `ParameterFile(ros2_controller.yaml)` **之后**（同名参数后者优先）。
见 `src/launch_pack/launch/arm_measure_sim.launch.py`。

**为什么值得单独记一条**：这是 C6 / 教训 12 的又一实例，而且比 C6 更隐蔽——C6 是"参数没声明"，
这次**参数声明了、YAML 里也写了、params 文件内容也完全正确**，错的只是 params 文件的 node key。
只检查"我传进去了"永远发现不了，必须查"目标节点读到了什么"（`ros2 param get`）。

**状态**：已修（`00001b8`）。

---

### O10 激励轨迹不可复现、质量波动极大【高】

**现象**：同一套被控对象、同一套控制器参数，连续跑多次测量，辨识残差在
**2.66 ~ 18.01 N·m** 之间波动（相关系数 0.46 ~ 0.94），相差近 7 倍。
更糟的是其中一部分测量把机械臂推到了模型限位之外：

```
[arm_controller] joint3 is outside its model limit (-3.3055 rad)
```

而 URDF 里 joint3 的限位是 ±2.9 rad，激励轨迹的软限位（`traj_pos_soft_limit_rate = 0.7`）
更只允许 ±2.03 rad——**指令**始终合法，是**实际关节角**被甩出了 1.27 rad，
即 PID 在 20 N·m 出力上限处饱和、完全跟不上指令。

**根因（两条，都在 `excitation_trajectory.cpp`）**

1. **不可复现**：`CMAParameters(x0, sigma, lambda, /*seed=*/0, gp)` 里 seed 传 0，
   而 libcmaes 的语义是"seed 为 0 就用当前时间生成"（`parameters.h:80`）。
   于是**每次启动的激励轨迹都不同**，且无法重放。
2. **欠收敛**：`k_max_fevals = 600`、`k_lambda = 24`，而优化变量是
   `nq × 2 × harmonics = 6 × 2 × 3 = 36` 维。600 次评估 ≈ 17 × 维度，
   对 CMA-ES 来说远不够（常用预算是 10²~10⁴ × 维度）。实测最优系数向量的范数在
   2.40 ~ 3.39 之间跳动，返回的基本是"随机可行解"而不是收敛解。

**后果**

- 测量质量主要由这一次随机抽样决定，**与"用哪个模型生成激励轨迹"无关**——
  任何"换一版模型再测一次、看误差是否下降"的迭代实验都会被这个随机性淹没。
- 单次测量的数据质量不可比：残差 2.66 与 18.01 的两个数据集谈不上"同一实验"。
- 偶尔提前结束：`record_index` 只到 41556 ~ 45934（约 83 ~ 92 s），而测量实际跑了 120 s。
  controller_manager 是按 ROS 时钟触发的，时钟消息偶尔丢拍就会少跑控制周期，记录条数
  随之减少；这些数据本身仍可用（时间列在，只是采样更不均匀），但会让"行数够不够"这类
  检查误报。

**修法**

1. 把 seed 做成控制器参数 `measure_trajectory_seed`（0 = 随机，非 0 = 固定），
   固定后同一 URDF 必然生成同一条轨迹——这是做任何迭代对比实验的前提。
2. 提高 `k_max_fevals`（至少 100 × 维度）或改成确定性初始化 + 局部优化，
   并把最终 score 打进日志，让"这条轨迹有多好"可见。
3. 软限位之外再对**实际关节角**留保护：激励轨迹的可行性判据应留出跟踪误差余量，
   否则"合法指令"照样会把关节甩出硬限位。

**补充（2026-10-12 实测）：返回的轨迹**每一次**都违反软限位**

控制器把最优傅里叶系数 `best_x` 打在日志里，按 `FourierTrajectory::position()` 的定义
（`q(t) = Σ_k [A_k sin(kωt) + B_k cos(kωt)]`，`q0 = 0`）可以离线重建**指令轨迹**。
把 21 次测量的 `best_x` 全部重建后，与 `traj_constraint_violation()` 用的同一套软限位
（`traj_pos_soft_limit_rate = 0.7`）对比：

| 关节 | 硬限位 | 软限位（70%） |
| --- | --- | --- |
| joint1 | ±1.80 | ±1.26 |
| joint2 | ±1.571 | ±1.10 |
| joint3 | ±2.90 | ±2.03 |
| joint4 | −2.00 ~ 1.80 | −1.43 ~ 1.23 |
| joint5 | ±0.611 | **±0.428** |
| joint6 | ±3.142 | ±2.199 |

21 次里**没有一次**的软限位违例为 0，违例量从 0.119 rad 到 **6.675 rad**：

| 轮次 | 违例[rad] | 关节3 指令行程 | 关节6 指令行程 |
| --- | --- | --- | --- |
| 03 | 0.119 | 3.15 | 3.13 |
| 08 | 0.688 | 4.42 | 3.87 |
| 07 | 6.675 | 6.30 | 7.07 |
| 09 | 4.188 | 4.95 | 6.60 |
| 10 | 1.411 | 3.67 | 3.35 |
| 11 | 2.853 | 5.24 | 5.06 |

**为什么约束检查没拦住**：`traj_constraint_violation()` 本身没问题，问题在优化器**找不到
可行点**。CMA-ES 从 `x0 = 0`、`k_initial_sigma = 0.7`、系数上界 ±2 起步，而 joint5 的软限位
只有 **±0.428 rad**——初始种群里的样本几乎全部违例。违例样本的目标值被抬到
`1000 + 1000 × violation`，可行样本只有 `−score ≈ −1 ~ −2`，所以 CMA-ES 会拼命往可行域挤，
但 600 次评估（36 维的 17 倍）根本挤不进去，最后 `get_best_seen_candidate()` 返回的是
**"最不违反"的那个样本**，而不是可行样本。

**后果（实测链条）**：指令远超限位 → 关节被甩出硬限位（实测 joint6 实际行程 15.37 rad、
速度峰值 **1064 rad/s**；joint1/2/3/4 也都越限）→ PID 在 20 N·m 上限饱和、完全跟不上 →
记录的数据不可用（残差 0.22 ~ 18.0 N·m，相关系数 0.46 ~ 0.999）→ 辨识重建偶尔发散到
**负质量**（实测 −1026 ~ −6.4×10⁸ kg），`_validate_physical_parameters()` 抛
`ValueError: mass must be positive and finite`，`identify_arm` 直接退出 1，那一轮连 URDF
都写不出来。30 轮迭代里有 2 轮（10、11）就是这样连续 3 次失败的。

**临时绕法**：`identify_loop2/config_proj.yaml`（`physical_projection: true`）能让这类数据
照样写出 URDF——LMI 投影会把参数拉回物理可行集（代价是惯量被投影得很小）。
（这条绕法是给 **O8** 的负质量兜底的，O10 修好后不再需要它来救"轨迹不可行"。）

### 修法（2026-10-13 已实施，commit `06af06d`）

1. **可复现**：新增控制器参数 `measure_trajectory_seed`（`auto_declare<int64_t>`，
   默认 0 = 随机，保持旧行为）。`ExcitationTrajectory::set_seed()` 把它透传给
   `CMAParameters(..., seed, gp)`；实际使用的种子连同可行性诊断一起打进控制器日志，
   照抄回来即可复现某一次测量。launch 也加了 `trajectory_seed:=`。
2. **收敛**：`k_max_fevals` 600 → 1500（约 42 倍维度），`k_initial_sigma` 0.7 → 0.25
   （让初始种群落在可行域内，目标函数不再被惩罚项淹没）。另外把单次评估的采样步长
   做成自适应 `max(0.01, period/2000)`，避免长周期把评估次数放大 12 倍。
3. **可行性兜底（关键）**：新增 `find_feasible_scale()`。可行域是以 0 为内点的**凸集**
   （`q(t)` 对系数线性、约束是逐关节箱式限位），所以按比例整体缩小一定能回到可行域；
   二分出最大的可行缩放系数，**保证输出轨迹一定可行**。若连 λ=0 都不可行
   （末端箱式约束把原点排除），`generate_failed()` 置位、`run()` 直接失败，不再静默
   把不可行轨迹发下去。最终违例、缩放系数、得分都进日志。

**验证（2026-10-13）**

| 检查 | 修前 | 修后 |
| --- | --- | --- |
| 指令轨迹软限位违例 | 21/21 次非零（0.119 ~ 6.675 rad） | 6/6 次 **0**（`raw_violation=0`，连缩放都不需要） |
| 实际关节角越限 | 每次测量都刷 `joint* is outside its model limit` | **0 条** |
| 关节速度峰值 | 最高 1064 rad/s | ≤ 3.2 rad/s |
| 同一 seed 复现 | 不可能（seed 取自时钟） | 两次运行 `best_x` 的 36 个系数**逐位相同** |
| 120 s 测量辨识残差 | 27 次：**0.14 ~ 18.0** N·m（均值 5.28，std 5.03） | 6 个种子：**0.77 ~ 2.04** N·m（均值 1.27，std 0.45） |

逐种子明细（`identify_loop2/verify/seeds.txt`）：

| seed | 1 | 2 | 3 | 4 | 5 | 6 |
| --- | --- | --- | --- | --- | --- | --- |
| rmse [N·m] | 1.498 | 1.395 | 2.041 | 0.837 | 1.057 | 0.765 |
| corr | 0.991 | 0.992 | 0.980 | 0.996 | 0.995 | 0.997 |
| 软限位违例 | 0 | 0 | 0 | 0 | 0 | 0 |
| 越限报错条数 | 0 | 0 | 0 | 0 | 0 | 0 |

注意：O10 修好只是让**数据可用**，辨识结果仍然非物理（`reconstruction_status: error`）——
那是 O8（加速度由速度二次求导得到）的账，两者独立。

**状态**：已修（`06af06d`）。

---

## 3. 设计约束（非缺陷，但必须遵守）

### C1 零位形处于工作空间边界，不要从那里直接做笛卡尔运动

`reset` 的目标（全零）对应机械臂完全伸直的位形，**雅可比降秩**，位于工作空间边界。
从那里直接做笛卡尔运动，目标不可达。

**做法**：笛卡尔任务务必先 `joint_move` 离开零位形。配置里 `reset_then_cart` 作为反例
保留并注明，可用的参考序列是 `ready_then_cart`。

**实测补充（2026-10-10，更正原先「逆解不收敛」的说法）**

原先这里写「从那里直接做笛卡尔运动，**逆解不收敛**」。实测的失败方式不是这样：

```
[ERROR] Cartesian trajectory REJECTED before any motion: the target is UNREACHABLE —
        the nearest reachable pose is still 0.001109 away from it (IK tolerance is 1.0e-03)
```

即：目标被判定为**不可达**，由 O2 的**下发前预检**拒绝，**一个运动指令都不下发**，
任务层退出码 1。这比「逆解跑到一半失败」安全得多。

离线核实（`tools/ik_solve.cpp`，目标 = 零位姿 + X 5 cm）：

| 目标 | 结果 | 残余 |
| --- | --- | --- |
| 零位姿本身 | Converged | 3.2e-4 |
| 零位姿 + X 5 cm，**零位姿姿态** | **Unreachable** | 1.305636e-03 |
| 零位姿 + X 5 cm，**ready 姿态** | **Unreachable** | 6.96e-03 |

前两行之外还试了 6 个差异极大的初值求解同一个目标，**全部收敛到同一残余
`1.305636e-03`** → 全局不可达，只差约 **1.3 mm**。换成 `ready` 的姿态后残余**更大**，
说明这是**位置**不可达，与姿态约束、与 `|rv| = π` 参数化都无关。

> **注意这个反例很弱**：余量只有 1.3 mm，是**判据边界**案例而不是明显不可行。
> 真正的教训是「零位形在工作空间边界」，不是「零位形一做笛卡尔就炸」。

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
| `214ad60` | 不可达目标下发前拒绝 + 三态诊断（O2） |
| `ac474f0` | robot_description 话题隔离到本工作区专用名（O4） |
| `14810bf` | 通用保持路径加入重力前馈，消除笛卡尔起点塌陷（O7） |
| `f8b61c1` | `identify_arm` 适配 vendored FIGAROH 0.6.0 的签名，不再 TypeError（O8 的前置缺陷） |
| `52cc021` | 修复包内绝对导入被误判为"求解器缺失"、PSD 校验容差过严（O8 的前置缺陷） |
| `00001b8` | measure launch 的覆盖参数改为 arm_controller 专属 params 文件，不再静默失效（O9） |
| `06af06d` | 激励轨迹：种子可固定、提高 CMA-ES 评估次数、加可行缩放兜底（O10） |

---

## 5. 本轮的方法论教训

1. **把策略集中化时，诊断信息必须一并迁移**（F7）。否则定位问题时要先把诊断补回来。
2. **清理动作应放在生命周期钩子（`exit()`）里**，不要散落在 `run()` 的失败路径（F8）。
3. **调参必须有离线基准**。走「改代码 → 编译 → 起仿真 → 跑任务」一轮两三分钟，而离线基准几百用例只要毫秒级——信赖域参数就是靠它扫出来的。
4. **端到端测试不可替代**。F1（保持无刚度）与 F9（0.5 ms 假成功）都是静态审阅看不出来、必须真跑才会暴露的缺陷。
5. **「已经收敛却报失败」要先怀疑判据而非算法**（F5）。判据的物理含义（这里是米+弧度混合范数）常被忽略。
6. **实时回路里的迭代上限就是最坏执行时间**（F6）。不收敛时的空转必须提前退出。

### 后续几轮补充的教训

7. **错误日志里的数字要有物理含义，否则比没有更糟**（O2）。`ik_err = 1.045e-3` 看起来
   像「差一点点没收敛」，实际含义是「目标落在可达集之外 1.046 mm」——判据恰好也是
   1e-3，两个数字撞在一起，把人引向了完全错误的方向。**报数字时要同时报它是什么。**
8. **一个数只能证明局部结论，要多初值才能证明全局结论**（O2）。单次逆解的残余是
   *从某个初值出发* 的局部极小值；用 6 个差异极大的初值全部收敛到同一残余，才能断言
   目标不可达。同理，`+Z 5 cm` 从 `ready` 出发残余 3.86e-2，从已经移动过的位姿出发
   只有 1.046e-3——**同一个目标，不同初值给出不同的"距离"**。
9. **修复要覆盖同一根因的所有路径**（O6 → O7）。给 `reset`/`idel` 加完重力前馈就以为
   完事了，但通用保持路径 `write_hold_command()` 是**另一条**同类路径，它仍然塌陷。
   找根因时要问「还有谁走这条路」。（O7 已修：现在两条路径共用
   `HoldGravityFeedforward`，见 §2。）
10. **A/B 对照能同时暴露被测项之外的问题**（O6 → O7）。验证 O2 时顺手对比两条序列的
    终止姿态，才发现 joint4 差了 0.089 rad——那个差值跟 O2 毫无关系，却是真缺陷。
11. **对照实验必须能失败**（O3）。新增的校验工具如果永远返回 PASS，它就毫无价值。
    还原成修复前的代码跑一遍、确认它会 FAIL，才算证明工具有效。
12. **配置项的"没生效"往往比"配错了"更隐蔽**（C6）。YAML 里写了、代码里读了、但参数
    没声明，于是静默取默认值——**没有任何提示**。关键参数应在启动时把实际生效值打出来。
13. **结构性隔离优于事后检测**（O4）。与其在启动后检测「是不是加载了错模型」，
    不如让错误模型根本进不来（话题重映射）。

### O8 补充的教训

14. **先做"数据 ↔ 模型"的自洽性检查，再去猜是哪一环错了**（O8）。给定 `(q, dq, τ)`，
    由 `M(q)·a = τ − bias(q, dq)` 反解出加速度；若它满足 `‖M·a + bias − τ‖ ≈ 1e-16`，
    就说明**模型、力矩语义、采样时刻全都是对的**，误差只可能出在"怎么得到加速度"这一步。
    这一步花十分钟，省掉了一轮逐个假设去试的排查——此前把"力矩语义"列为主嫌疑，实测才
    发现它是清白的（教训 7 的另一种形态：**先量，再猜**）。
15. **用被测对象自己的输入去造"完美数据"，只能验证下游，验证不了那个输入**（O8）。
    用管线算出的 `dq̈` 正演出完美力矩再跑辨识，残差 5.5e-14——这证明了回归器、QR 分解、
    基参数机制都对，却**看不见** `dq̈` 本身 31% 的误差，因为正演和辨识用的是同一个错的
    `dq̈`。要验证一个输入，必须找**不依赖它**的参照（这里是反解得到的加速度）。

### O9 补充的教训

16. **"我传进去了"不是判据，"目标节点读到了什么"才是**（O9）。这次传给 launch 的 params 文件
    内容**逐字正确**（`/**` 下 `measure_trajectory_period: 12.0` 写得明明白白），
    但控制器读到的仍是 YAML 默认值——错的只是 params 文件的 **node key**。
    验证方式必须落到目标节点上：`ros2 param get <node> <param>`，或者干脆用**结果**验证
    （这里是 CSV 行数：12 s 应该 6000 行，实测 4998 行 ≈ 10 s）。
    只检查中间产物（"文件里写了"）等于没检查。

### O10 补充的教训

17. **随机优化器必须先固定种子，才谈得上"对比实验"**（O10）。激励轨迹生成用了 CMA-ES，
    而 seed 传 0（libcmaes 语义：取当前时间），600 次评估又只够 36 维问题的 17 倍——
    于是"激励轨迹"实际是一次随机抽样。在这种前提下做"每轮换一版模型、看误差是否下降"
    的迭代实验，测到的全是抽样噪声：残差 2.66 ~ 18.01 N·m 的波动与模型改了什么毫无关系。
    **做迭代/对比实验之前，先确认被测环节是可复现的**；否则再多轮次也只是在测随机数。
18. **"指令合法"不等于"实际合法"**（O10）。激励轨迹的可行性判据只检查**指令**关节角
    （软限位 70%），但 PID 在出力上限饱和后实际关节角会甩出硬限位 1.27 rad，
    控制器只能打一行 ERROR。约束检查必须落在**被约束的物理量**上（这里是实际关节角），
    或者至少为跟踪误差留出余量。

---

## 6. O1 排查（第一轮）：TF 与正运动学**并不**不一致

> **本节结论已被 §7 取代。** 本轮正确证伪了 O1，但把真因归给了任务雅可比（O5）——
> **那个归因是错的**，§7 用 6 个初值的收敛实验证明真因是**目标位姿本身不可达**。
> 本节保留，因为 6.1（TF 与正运动学完全一致）与 6.2（差值是跟踪误差）这两条结论
> 是**正确且有价值**的，也是 §7 的起点。

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

### 6.3 观察到的现象：IK 平台永远贴着收敛判据

> 这张表本身是**可靠**的实测数据，但当时的解读（「雅可比线性化失准导致降不下去」）
> 是错的。正确解读见 §7.2：残余恰好略高于判据，是因为**目标就落在可达集之外**，
> 可达集边界到目标的距离正好是这个量级。

| 判据 | IK 实际残余 |
| --- | --- |
| 1e-6 | 1.018e-6 |
| 1e-4 | 1.519e-4 |
| 1e-3 | 1.051e-3 |
| 5e-3 | 5.083e-3 |

残余**始终恰好略高于判据**，无论判据取多少。这不是数值精度问题，而是迭代**根本无法继续下降**。

### 6.4 当时的归因（O5）：任务雅可比在 \|rv\| = π 处失效

> **这一节的归因是错的**，但它描述的问题（O5）是**真实存在**的，只是**数值影响可忽略**，
> 不是笛卡尔失败的原因。详见 §7.1。

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

### 6.5 修复方向

在 `jacobian_map` 中把几何雅可比的角速度部分换算成旋转向量变化率：

```
J_task_rot = J_r(ρ_e) · J_geom_rot
```

**已实施**（`729df71`，`23daaa8` 微调）：用**误差**旋转向量的**右雅可比** `J_r(ρ_e)`。
中途曾误用左雅可比 `J_l(ρ_c)⁻¹`，离线基准收敛率从 100% 掉到 49.7%，已回退。

修正后基准指标**与修正前完全一致**——该误差量级下 `J_r ≈ I`，数值上可忽略（§7.1）。

**更彻底的替代方案**：放弃旋转向量表示，任务空间姿态改用四元数或直接用旋转矩阵增量。这是原始 `bugs.md` B5 提到的方向（「当前实现是 6 自由度专用」）。

### 6.6 本轮结束时的状态（已被 §7 取代）

- ~~`cart_move` / `admittance_move` 仍被 O5 阻塞，`ready_then_cart` 无法通过~~
  → **错误**。O5 修正已实现但**不是**阻塞原因；真因是目标不可达，见 §7。
- 收敛判据 `1e-3`（1 mm / 0.057°）保留至今，并注明它是「实用阈值」而非可达的数值精度。
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
