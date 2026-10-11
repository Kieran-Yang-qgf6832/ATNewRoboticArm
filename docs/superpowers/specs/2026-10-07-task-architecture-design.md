# 机械臂系统架构简化设计

> 日期：2026-10-07
> 范围：`src/arm_controller`（控制器/状态机）、`src/tasks`（任务层）
> 状态：**已实施完毕**——同目录 plan 的六个阶段全部完成，旧任务层已在阶段 6 删除
> （`f112272`）。**本文档是历史设计记录**，描述当时的设计意图；当前实现与最新状态请以
> [`docs/bugs.md`](../../bugs.md) 与 [`CLAUDE.md`](../../CLAUDE.md) 为准。

---

## 1. 背景

本项目是 ROS 2 Jazzy + `ros2_control` 的 6 自由度机械臂控制系统，支持 MuJoCo 仿真与实机（USB CDC → MCU）两种运行模式。

| 项 | 现状 |
| --- | --- |
| 用途 | 参加竞赛 |
| 运行要求 | 实机长期运行，安全性与故障恢复优先，宁可复杂也不能失控 |
| 验证程度 | **仅在仿真中验证过，实机从未运行** |
| 后续演进 | 不新增控制算法；会在任务层频繁添加/修改目标点和任务类型 |
| 末端执行器 | 当前为吸盘；通过开关量支持多种末端执行器（夹爪 / 气泵） |

## 2. 目标与非目标

### 目标

| 编号 | 目标 |
| --- | --- |
| G1 | 可恢复故障（逆解失败、超限、通信异常）不再导致控制器链被停用，机械臂不失控 |
| G2 | 任务层添加任务类型、修改目标点不需要写 C++、不需要重新编译 |
| G3 | 消除双层状态机之间的重复逻辑 |

### 非目标

- 不修改控制器 8 个状态的算法语义
- 不合并两层状态机（实时约束决定分层必要）
- 不修改 URDF / MJCF 模型
- 本轮不做实机通信看门狗（缺乏验证条件）

## 3. 约束

| 编号 | 约束 |
| --- | --- |
| C1 | `ArmController::update()` 是 500 Hz 实时路径：禁止 ROS 参数读写、动态内存分配、阻塞操作 |
| C2 | 任务层运行在普通 ROS 节点线程：允许阻塞式服务调用与内存分配 |
| C3 | `reset` / `teach_pendant` / `measure` 只能从 `idel` 进入、只能切回 `idel`（控制器侧约束） |
| C4 | 实机从未验证，所有实机相关改动延后 |

## 4. 现状问题

### P1 故障策略分散（安全，最高优先级）

当前每个状态各自处理求解失败，共 17 处 `fall_back_to_idel_after_solve_failure()` 调用。每个调用点都必须写对参数、记得修改 `exp_state`、记得返回 `true` 而非 `false`。

**结构性风险：新增状态时漏写一处，就是一次坠落。**

同时 `ArmController::update()` 在 FSM 返回失败时返回 `ERROR`，会让 `controller_manager` 停用整条控制器链 → 力矩消失 → 机械臂在重力下坠落。

### P2 任务层代码化（扩展性）

任务层由 7 个任务类 + `TaskFSM` + `TaskFSMFactory` + `create_task()` + `CMakeLists` 条目组成，约 2756 行。添加一个任务需要改三处并重新编译，与"频繁添加任务类型"的诉求直接冲突。

### P3 双层重复

| 重复项 | 说明 |
| --- | --- |
| `reset` 收敛判定写两遍 | 同一公式、同一组参数名（`reset_joint_pos` / `reset_duration` / `reset_tolerance`）声明在 `/arm_controller` 与 `/task_runner` 两个节点上，且两套独立的放弃阶梯 |
| 两套 FSM 引擎 | `FSMFactory` 与 `TaskFSMFactory` 骨架约 90% 相同；`FSM` 与 `TaskFSM` 基类同形 |
| `kIdelGatedTasks` 硬编码 | `task_runner.cpp` 中 `{"reset", "teach_pendant", "measure"}` 镜像了控制器侧的前置条件；控制器新增受限状态时任务层会静默失效 |

## 5. 设计

### 5.1 控制器故障策略收拢

把故障裁决从各状态收拢到 `ArmController::update()` 一处。

```
update(time, period):
  1. 读 state_interfaces_ → fsm_factory->state_
  2. 采集故障标志：
       - 通信异常（由硬件接口置位；本轮预留接口，暂不启用）
       - 关节位置 / 速度越限
       - 上一周期 FSM 返回的失败标志
  3. if (存在故障):
       写保持命令：position = 当前实测值, velocity / torque / ki = 0,
                   kp / kd = 默认值
       fsm_factory->exp_state_name = "idel"
       限流上报（2 s 一条）
       return OK                    ← 关键
  4. ret = fsm_factory->run(time)
  5. if (ret == Fatal): return ERROR
     if (ret == Recoverable): 执行第 3 步，return OK
  6. 写 6 个命令接口
     return OK
```

**要点**

| 要点 | 说明 |
| --- | --- |
| 可恢复故障一律返回 `OK` | 安全策略在控制器内部完成，不把控制器链交给上层关掉。这是 P1 的根因修复 |
| 状态只报告失败 | FSM 内的 17 处兜底删除，`run()` 失败即失败，不自己写命令、不自己改状态 |
| 命令连续性 | 故障当拍写保持命令；下一拍 `FSMFactory` 执行 `exit`/`enter` 切到 `idel`，该拍不更新 `command_`，保持值延续到 `idel` 接手，不存在空窗 |

**接口变更**：`FSM::run()` 保持 `bool`（状态级只需报告成功/失败），仅将 `FSMFactory::run()` 的返回值从 `bool` 改为枚举，把"什么情况下允许停用控制器链"从隐含约定变成显式契约。

```cpp
enum class RunResult { Ok, Recoverable, Fatal };
```

映射规则：状态返回 `false` → `Recoverable`；仅在工厂内部结构异常时返回 `Fatal`（例如 `fsm_map` 中找不到当前状态名、注册的指针为空），此时才允许让 `update()` 返回 `ERROR`。因此 `arm_fsm.hpp` / `arm_fsm.cpp` 中状态类的函数签名**不需要改动**。

### 5.2 任务层配置驱动重写

**采用直接重写**，不保留新旧并存开关。

#### 结构

```
tasks/
  context/task_context.{hpp,cpp}     # 保留：ROS 接口封装（参数服务 / 话题 / TF），~500 行
  executor/task_executor.{hpp,cpp}   # 新增：读配置、走步骤、校验、失败回退，~300 行
  executor/step_types.{hpp,cpp}      # 新增：步骤类型注册表（含 requires_idel 等属性）
  config/tasks.yaml                  # 新增：任务定义
  src/task_runner.cpp                # 瘦身为入口，~80 行
```

`TaskContext` 已经是干净的"如何与控制器通信"抽象，不做改动。

#### 执行流程

```
run_sequence(name):
  1. 读 YAML → 解析 steps
  2. 校验（在机械臂运动之前）：
       - 步骤类型是否存在
       - 目标点是否存在、关节数是否匹配
       - joint_move 目标是否在 URDF 限位内
       - 时长 / 容差是否为正
     任一不合法 → 直接报错，机械臂一步不动
  3. for each step:
       requires_idel 则先等待 current_state == idel
       enter：设 exp_state / 发布轨迹 / 发布开关量
       loop 直到完成或超时：
         spin_some；检查控制器故障；检查步骤完成条件
       verify
       失败 → 回 idel + 上报 + 等待新指令（进程不退出）
```

**校验前置**是相对现状的实质安全提升：现状错误要执行到任务链中间才暴露，新方案在机械臂运动前拒绝。

#### `reset` 步骤的简化

`reset` 的完成条件仅为 **`current_state == idel`**。控制器侧的 `reset_settle` 兜底已保证它必然回到 `idel`。因此任务层**不需要任何关节误差计算**，`reset_joint_pos` / `reset_tolerance` 在 `/task_runner` 上的重复声明一并删除（消除 P3 第一项）。

#### 热加载

复用现有触发方式：`ros2 param set /task_runner task_sequence "pick_demo"` 时重新读取 YAML 文件。

> 参数语义变更：原 `task_sequence` 是任务类型列表（`std::vector<std::string>`，如 `"[reset, cart_traj]"`），重写后改为**单个序列名**（`std::string`），指向 `tasks.yaml` 中 `sequences` 下的一个键。

```
编辑 tasks.yaml → ros2 param set /task_runner task_sequence "pick_demo"
```

无需重启节点，无需 `colcon build`。

### 5.3 末端执行器开关量

末端执行器是**纯开关量**（吸盘气泵 / 夹爪电磁阀），没有位置、速度、力矩，**不建模为 ros2_control 关节**，也不修改 URDF / MJCF。

```
任务层发布 /tool_cmd (std_msgs/Bool)
        │
        ├→ 实机：ArmRealInterfaces 收下，写入 MCU 包的一个 bit（延后，见 D2）
        └→ 仿真：无对应模型，仅发布不产生动作
```

**消息类型**：本轮使用 `std_msgs/Bool`，不新增自定义消息。将来若需要多路输出（例如夹爪需要两个电磁阀），再升级为 `robot_msgs/ToolCmd.msg`。

「更换末端执行器」的成本 = 修改配置中的一段语义映射，代码零改动：

```yaml
tool:
  name: suction          # suction | gripper | ...
  on:  suck              # "开" 的语义名
  off: release           # "关" 的语义名
```

`tool` 步骤的 `action` 同时接受配置定义的语义名（`suck` / `release`）与规范的 `on` / `off`。

**与 `wait` 的关系**：吸盘吸住后必须等待真空建立才能抬升，因此 `tool` 步骤之后通常跟一个 `wait` 步骤。

## 6. 配置 schema

```yaml
# tasks/config/tasks.yaml

# 当前挂载的末端执行器：只影响语义名与日志
tool:
  name: suction
  on:  suck
  off: release

# 命名目标点：修改一处，所有引用它的任务一起变化
points:
  home:      [0.0, 0.0,  0.0, 0.0, 0.0, 0.0]
  ready:     [0.0, 0.3, -0.3, 0.0, 0.3, 0.0]
  above_box: [0.2, 0.1,  0.4, 0.0, 0.5, 0.0]

# 任务序列：步骤列表
sequences:
  pick_demo:
    steps:
      - {type: reset}
      - {type: joint_move, to: ready, duration: 3.0}
      - {type: tool, action: suck}
      - {type: wait, duration: 1.5}
      - {type: cart_move, offset: [0.0, 0.0, 0.05], duration: 2.0}
      - {type: idel}
```

YAML 解析使用 `yaml_cpp_vendor`（`rclcpp` 已依赖，`CMakeLists.txt` 加一行即可）。

**配置文件路径**：由 `task_runner` 的 `tasks_config` 参数指定，默认指向 `tasks` 包安装后的 `share/tasks/config/tasks.yaml`。便于在不重新安装的情况下用 `-p tasks_config:=/path/to/my.yaml` 指定调试用配置。

## 7. 步骤词汇表

封闭集合，共 9 类。

| type | 下发 | 完成条件 | requires_idel |
| --- | --- | --- | --- |
| `wait` | 不切换控制器状态，纯等待 | duration 到 | 否 |
| `idel` | `exp_state = idel` | `current_state == idel` + hold 秒 | 否 |
| `reset` | `exp_state = reset` | `current_state == idel` | 是 |
| `joint_move` | `exp_state = joint_traj` + `JointTrajCmd` | 时长 + 关节误差校验 | 否 |
| `cart_move` | `exp_state = cart_traj` + `CartTrajCmd` | 时长 + 位姿误差校验 | 否 |
| `admittance_move` | `exp_state = admittance` + `AdmittanceCmd` | 时长 | 否 |
| `tool` | `/tool_cmd` | 发布即完成 | 否 |
| `teach` | `exp_state = teach_pendant` | duration | 是 |
| `measure` | `exp_state = measure` | duration | 是 |

**`requires_idel` 是步骤类型的属性，不是 runner 中的硬编码表** —— 这消除了 P3 第三项。

### `wait` 与 `idel` 的区别

| 步骤 | 行为 | 适用场景 |
| --- | --- | --- |
| `wait` | 不切换控制器状态，只等待 `duration` 秒 | 上一步刚完成运动，控制器自身保持不动 |
| `idel` | 主动切换 `exp_state = idel`，锁定当前位置 | 需要明确回到安全态 |

**前提说明**：`wait` 的"保持不动"保证来自控制器处于保持态（`idel` / `joint_traj` 的 `STOP` 阶段 / `cart_traj` 的 `STOP` 阶段，均会调用 `hold_current_joint_position()`）。若上一步是 `servo` 或 `admittance`，控制器不会保持，`wait` 期间机械臂仍会运动。需要硬保证时使用 `idel`。此约束写入文档，不额外增加参数。

## 8. 改动清单

### 删除

```
task/idel_task.{hpp,cpp}           task/reset_task.{hpp,cpp}
task/cart_traj_task.{hpp,cpp}      task/joint_traj_task.{hpp,cpp}
task/admittance_task.{hpp,cpp}     task/teach_pendant_task.{hpp,cpp}
task/measure_task.{hpp,cpp}        task/task_space_segment_task.{hpp,cpp}
fsm/task_fsm.hpp                   fsm/task_fsm_factory.hpp
task_runner.cpp 中的 create_task / link_chain / kIdelGatedTasks / needs_idel_gate
```

### 保留

```
tasks/context/task_context.{hpp,cpp}
```

### 新增

```
tasks/executor/task_executor.{hpp,cpp}
tasks/executor/step_types.{hpp,cpp}
tasks/config/tasks.yaml
```

### 修改

```
src/arm_controller/lib/fsm/fsm_factory.hpp    # run() 返回值改 RunResult
src/arm_controller/controller/src/controller.cpp  # update() 统一故障裁决
src/arm_controller/arm/arm_fsm.cpp            # 删除 17 处兜底调用
src/arm_controller/controller/inc/controller.hpp
src/tasks/src/task_runner.cpp                 # 瘦身为入口
src/tasks/CMakeLists.txt
```

### 规模变化

任务层约 2756 行 → 约 900 行（其中 `TaskContext` 约 500 行保留）。

## 9. 安全属性

| 编号 | 属性 | 保障方式 |
| --- | --- | --- |
| S1 | 可恢复故障不停用控制器链 | `update()` 统一裁决，返回 `OK` |
| S2 | 任何状态忘记处理失败都不会失控 | 策略只有一处，结构上无法绕过 |
| S3 | 非法配置在机械臂运动前被拒绝 | 执行器前置校验 |
| S4 | 任务失败后机械臂保持不动 | 回退 `idel` + 保持命令，进程不退出 |
| S5 | 每个步骤与整条任务链都有超时 | 步骤级 + 链级超时 |
| S6 | 复位未到位不会永久卡住状态机 | 控制器 `reset_settle` 兜底 + 任务层只观察 `current_state` |

## 10. 延后项

以下项目因缺少实机验证条件而延后，**实施本设计时不做**：

| 编号 | 项目 | 说明 |
| --- | --- | --- |
| D1 | 通信层看门狗 | 主机侧超时后输出零力矩 / 纯阻尼；MCU 侧需固件配合，是覆盖物理断链的唯一手段 |
| D2 | MCU 协议增加开关量字段 | `/tool_cmd` 落到实机需要扩展 `MCUTarget2Pack` 或新增包类型 |
| D3 | `ArmRealInterfaces` 越界缺陷 | `arm_real_interfaces.cpp` 中 `joint_count = 6` 硬编码，而 `commands_` / `received_states_` 按 `joint_names_.size()` 分配；关节数不为 6 时越界读写。**上实机前必须修复** |
| D4 | 仿真故障注入 | 让 `mujoco_ros2_control` 硬件接口支持人为丢包，用于验证 D1 |

## 11. 验收标准

| 编号 | 标准 | 验证方式 |
| --- | --- | --- |
| A1 | 注入一次求解失败，控制器链保持 `active` | `ros2 control list_controllers -v` 观察，机械臂保持原位 |
| A2 | 新任务通过修改 YAML 添加，无需重新编译 | 编辑 `tasks.yaml` 后 `ros2 param set` 触发，任务可执行 |
| A3 | 修改 `points` 中一个目标点，所有引用它的序列同步生效 | 编辑 YAML 后重跑序列 |
| A4 | 非法配置（未知步骤类型 / 关节数不匹配 / 目标超限位）在运动前被拒绝 | 构造非法 YAML，确认机械臂不动且报错明确 |
| A5 | `[reset, joint_traj]` 与 `[reset, cart_traj]` 行为与旧任务层一致 | 对比两版日志与末端轨迹 |
| A6 | 任务失败后控制器回到 `idel`，机械臂保持，进程不退出 | 构造失败步骤，确认可继续下发新序列 |
| A7 | `reset_joint_pos` / `reset_tolerance` 仅存在于 `/arm_controller` | `ros2 param list /task_runner` 确认无重复声明 |

## 12. 实施顺序

1. **控制器故障策略收拢**（5.1）—— 独立于任务层，可先验证 A1
2. **任务执行器骨架 + 配置加载 + 前置校验**（5.2）—— 验证 A4
3. **迁移 9 类步骤**（7）—— 先做 `reset` / `joint_move` / `cart_move` / `idel` / `wait`
4. **对齐已验证序列**（A5）—— 跑通 `[reset, joint_traj]` 与 `[reset, cart_traj]`，确认行为与旧任务层一致
5. **补齐其余步骤**（`admittance_move` / `tool` / `teach` / `measure`）
6. **删除旧任务层代码**（8）

> 第 4 步是对"直接重写"的风险控制：旧任务层在仿真中可正常工作，先确认新执行器复现相同行为，再删除旧代码。
