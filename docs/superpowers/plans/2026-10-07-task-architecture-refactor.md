# 任务架构重构实施计划

> 日期：2026-10-07
> 设计文档：[2026-10-07-task-architecture-design.md](../specs/2026-10-07-task-architecture-design.md)
> 备份提交：`2c6b187 准备进行任务层重构，执行前的备份。`
> 状态：**六个阶段全部完成**（阶段 6「删除旧任务层代码」见 `f112272`）。
> **本文档是历史实施记录**，保留用于回溯当时的决策与验证步骤。

## 0. 总体约定

- 每个阶段结束后必须 `colcon build --packages-select arm_controller tasks --symlink-install` 通过
- 每个阶段的验证项通过后才进入下一阶段
- 阶段 1 独立于任务层，可单独验证与回退
- 阶段 4 是「直接重写」的风险控制点：确认新执行器与旧任务层行为一致后才执行阶段 6

```bash
# 每次构建
cd /home/kangy/MyProjects/ATNewRoboticArm
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arm_controller tasks --symlink-install
```

---

## 阶段 1：控制器故障策略收拢

**目标**：G1 —— 可恢复故障不再停用控制器链（对应验收 A1）

### 1.1 `src/arm_controller/lib/fsm/fsm_factory.hpp`

新增枚举，并把 `run()` 的返回值从 `bool` 改为 `RunResult`：

```cpp
enum class RunResult { Ok, Recoverable, Fatal };
```

映射规则（逐条对应现有 `return false`）：

| 现有失败点 | 新返回值 | 理由 |
| --- | --- | --- |
| 首次 `enter()` 返回 false | `Recoverable` | 状态拒绝进入，保持即可 |
| `current_state_name_` 在 `fsm_map` 中找不到 | `Fatal` | 结构损坏，代码缺陷 |
| 找到但指针为 null | `Fatal` | 同上 |
| `current_fsm.exit()` 返回 false | `Recoverable` | 状态拒绝离开 |
| `next_fsm.enter()` 返回 false | `Recoverable` | 状态拒绝进入 |
| `current_fsm.run()` 返回 false | `Recoverable` | 求解失败等，本阶段主要场景 |
| `next_state` 在 `fsm_map` 中找不到 | `Fatal` | 目标状态名不存在，配置/代码缺陷 |

**注意**：`FSM::run()` 保持 `bool` 不变，`fsm.hpp` 与 `arm_fsm.hpp` 的函数签名均不改动。

### 1.2 `src/arm_controller/controller/inc/controller.hpp`

新增私有方法声明：

```cpp
/// 故障兜底：锁定当前实测位置并请求切回 idel
void apply_safe_hold_and_idel();
```

### 1.3 `src/arm_controller/controller/src/controller.cpp`

实现 `apply_safe_hold_and_idel()`：

```
for i in joints:
    command_interfaces_[i*6+0].set_value( state_interfaces_[i*3+0].get_value() )  // position = 实测
    command_interfaces_[i*6+1].set_value(0.0)                                    // velocity
    command_interfaces_[i*6+2].set_value(0.0)                                    // effort
    command_interfaces_[i*6+3].set_value( default_kp[i] )                        // kp
    command_interfaces_[i*6+4].set_value( default_kd[i] )                        // kd
    command_interfaces_[i*6+5].set_value(0.0)                                    // ki
fsm_factory->exp_state_name = "idel";
RCLCPP_ERROR_THROTTLE(logger, *clock, 2000, "fault detected; holding position and switching to idel");
```

改写 `update()`：

```
读 state_interfaces_ → fsm_factory->state_          // 保持不变

const auto result = fsm_factory->run(time);

if (result == RunResult::Fatal) {
    apply_safe_hold_and_idel();      // 停用前也先保持一拍
    return ERROR;                    // 仅结构损坏才允许停用链条
}
if (result == RunResult::Recoverable) {
    apply_safe_hold_and_idel();
    return OK;                       // ← G1 的关键
}

// 正常下发 6 个命令接口（保持不变）
return OK;
```

**要点**：`RCLCPP_ERROR_THROTTLE` 是实时安全的（只做限流判断，不分配）。日志是本路径上唯一的非纯计算操作，与现有 `RCLCPP_WARN_THROTTLE` 用法一致。

**本阶段暂不做关节越限检查**。设计文档 5.1 列了该项，但它有误报风险（限位偏紧时会误触发保持），需要单独在仿真中验证阈值。留到阶段 5 之后单独处理。

### 1.4 `src/arm_controller/arm/arm_fsm.cpp`

**（a）删除辅助函数** `fall_back_to_idel_after_solve_failure()`（第 64 行起）。

**（b）删除 17 处调用点**，改为 `return false`：

| 状态 | 行号 |
| --- | --- |
| `CartTrajState` | 419, 437, 446 |
| `JointTrajState` | 536, 546, 556 |
| `ServoState` | 672, 687, 692 |
| `AdmittanceState` | 821, 842, 849, 860, 867, 883, 896, 903 |

原本形如：

```cpp
state = TrajPhase::STOP;
traj.stop();
fall_back_to_idel_after_solve_failure(factory, joint_count_, default_kp_, default_kd_, "...");
return true;
```

改为：

```cpp
return false;
```

清理动作（`traj.stop()` 等）不再在 `run()` 里做 —— 由 `exit()` 负责。

**（c）修复 `AdmittanceState::exit()`（第 791 行）—— 必须做，否则重构会引爆 bug**

现状：

```cpp
bool AdmittanceState::exit(const std::string& next_state) {
    (void)next_state;
    return true;                    // ← 没有清理
}
```

而 `AdmittanceState::check_switch()` 的切换前提是 `!trajectory_active_`：

```cpp
if (factory != nullptr && !trajectory_active_ && ...) {
    return factory->exp_state_name;
}
```

当前之所以能切换，是因为 8 处兜底里顺手把 `trajectory_active_` 置了 false。**一旦删除这些兜底，`trajectory_active_` 会永久为 true，导纳状态再也切不出去，故障恢复失效。**

修复：

```cpp
bool AdmittanceState::exit(const std::string& next_state) {
    (void)next_state;
    traj.stop();
    trajectory_active_ = false;
    return true;
}
```

**（d）核对其余状态的 `exit()` 是否已完成清理**：

| 状态 | `exit()` 现状 | 结论 |
| --- | --- | --- |
| `CartTrajState` | `traj.stop(); state = STOP;` | 已完备 |
| `JointTrajState` | `traj.stop(); state = STOP;` | 已完备 |
| `ServoState` | 空 | 无需清理（无轨迹标志位，`check_switch()` 无状态门控） |
| `AdmittanceState` | 空 | **需按 (c) 修复** |

### 验证 1

```bash
colcon build --packages-select arm_controller --symlink-install
ros2 launch launch_pack arm_controller_test_sim.launch.py
# 另开终端
ros2 control list_controllers -v
```

| 编号 | 验证项 | 通过标准 |
| --- | --- | --- |
| V1.1 | 构建通过 | exit code 0 |
| V1.2 | 正常任务仍可执行 | `ros2 run tasks task_runner --ros-args -p use_sim_time:=true -p task_sequence:="[reset, cart_traj]"` 正常完成 |
| V1.3 | 注入一次求解失败，控制器链保持 active | `ros2 control list_controllers -v` 中 `arm_controller` 仍为 `active`，机械臂保持原位不坠落 |
| V1.4 | 故障后状态回到 idel | `ros2 param get /arm_controller current_state` 返回 `idel` |

V1.3 的注入方式：临时在 `CartTrajState::run()` 里强制 `return false`（或给一个不存在的 IK 目标），观察后还原。

---

## 阶段 2：任务执行器骨架

**目标**：G2 的基础设施 + 前置校验（对应验收 A4）

### 2.1 构建配置

**`src/tasks/package.xml`**：添加

```xml
<depend>yaml_cpp_vendor</depend>
```

**`src/tasks/CMakeLists.txt`**：

```cmake
find_package(yaml_cpp_vendor REQUIRED)

add_library(tasks_core STATIC
  context/task_context.cpp
  executor/step_types.cpp
  executor/task_executor.cpp
)

target_include_directories(tasks_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/context>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/executor>
)

# 安装任务配置
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
```

（阶段 2 期间 `task/*.cpp` 暂时保留在库中，阶段 6 再删）

**`src/tasks/config/tasks.yaml`**：按设计文档第 6 节创建。

### 2.2 `src/tasks/executor/step_types.hpp` / `.cpp`

```cpp
/// 单个步骤的解析结果
struct Step {
    std::string type;
    double duration{0.0};
    double tolerance{0.0};
    std::vector<double> target;      // joint_move: 绝对关节角
    std::vector<double> offset;      // cart_move: 相对末端位移
    std::string action;              // tool: 语义名
    std::string to;                  // joint_move: 命名目标点
    std::size_t source_line{0};      // 报错时定位 YAML 行
};

/// 步骤类型的静态属性
struct StepSpec {
    bool requires_idel{false};
    bool needs_duration{false};
};

/// 类型名 → 属性；未知类型返回 nullptr
const StepSpec* step_spec(const std::string& type);

/// 已知的步骤类型名列表，用于错误提示
std::vector<std::string> known_step_types();

/// 从 YAML 节点解析一个步骤；失败时写 error
bool parse_step(const YAML::Node& node, Step* step, std::string* error);
```

`step_spec()` 内部是一张静态表，把 `requires_idel` 作为**类型属性**而非 runner 中的硬编码列表（消除设计文档 P3 第三项）：

| type | requires_idel | needs_duration |
| --- | --- | --- |
| `wait` | false | true |
| `idel` | false | false |
| `reset` | **true** | false |
| `joint_move` | false | true |
| `cart_move` | false | true |
| `admittance_move` | false | true |
| `tool` | false | false |
| `teach` | **true** | true |
| `measure` | **true** | true |

### 2.3 `src/tasks/executor/task_executor.hpp` / `.cpp`

```cpp
class TaskExecutor {
public:
    TaskExecutor(const rclcpp::Node::SharedPtr& node, TaskContext* ctx);

    /// 读取并解析 YAML；失败时写 error
    bool load_config(const std::string& path, std::string* error);

    /// 可用的序列名，用于错误提示
    std::vector<std::string> sequence_names() const;

    /// 执行一条序列；返回是否成功走完
    bool run_sequence(const std::string& name);

private:
    /// 执行前校验：类型存在、目标点存在、关节数匹配、时长为正、目标在限位内
    bool validate_step(const Step& step, std::string* error) const;

    /// 执行单个步骤
    bool run_step(const Step& step, std::string* error);

    /// 等待 current_state 变为 idel
    bool wait_for_idel(double timeout, std::string* error);

    /// 失败收尾：切回 idel 并保持
    void fall_back_to_idel();

    struct Sequence { std::string name; std::vector<Step> steps; };

    rclcpp::Node::SharedPtr node_;
    TaskContext* ctx_;
    std::map<std::string, std::vector<double>> points_;
    std::map<std::string, Sequence> sequences_;
    std::string tool_on_, tool_off_;
    double step_timeout_{30.0};
};
```

**`run_sequence()` 流程**（对应设计文档 5.2）：

```
1. 查 sequences_，不存在则报错并列出可用序列名
2. 对每个 step 调 validate_step()；任一失败 → 报错返回，机械臂一步不动
3. for each step:
     if (spec.requires_idel) wait_for_idel(step_timeout_)
     run_step(step)
     失败 → fall_back_to_idel() + 报错 + return false
4. return true
```

**校验内容**（`validate_step`）：

| 检查 | 适用类型 |
| --- | --- |
| 类型已知 | 全部 |
| `duration > 0` | needs_duration 的类型 |
| `to` 存在于 `points_` | `joint_move` |
| `target.size() == 关节数` | `joint_move` |
| 目标在 URDF 关节限位内 | `joint_move` |
| `offset.size() == 3` | `cart_move` |
| `action` 能解析成 on/off | `tool` |

### 2.4 `src/tasks/src/task_runner.cpp` 重写

保留现有对外行为：

- 参数：`tasks_config`（新增）、`task_sequence`（改为 `std::string`）、`controller_node`、`joints`、`startup_timeout`、`exit_when_idle`、`task_status_topic`
- 话题 `task_status`（transient_local），内容约定不变
- 常驻等待新 `task_sequence`

### 验证 2

| 编号 | 验证项 | 通过标准 |
| --- | --- | --- |
| V2.1 | 构建通过 | exit code 0 |
| V2.2 | 配置文件正确安装 | `ros2 pkg prefix tasks` 下存在 `share/tasks/config/tasks.yaml` |
| V2.3 | 合法配置可加载 | 启动后日志列出可用序列名 |
| V2.4 | 未知步骤类型被拒绝且机械臂不动 | 构造 `type: nonexist`，报错明确、无运动 |
| V2.5 | 关节数不匹配被拒绝 | 构造 5 维 `target`，报错明确 |
| V2.6 | 超限位目标被拒绝 | 构造 `joint1: 100.0`，报错明确 |

---

## 阶段 3：迁移核心步骤

**目标**：实现 `reset` / `idel` / `wait` / `joint_move` / `cart_move`

| 步骤 | 实现要点 | 参考旧实现 |
| --- | --- | --- |
| `wait` | 不切状态，只计时 | — |
| `idel` | `set_exp_state("idel")`，等 `current_state == idel` | `idel_task.cpp` |
| `reset` | `set_exp_state("reset")`，等 `current_state == idel`。**不做任何关节误差计算** | `reset_task.cpp`（简化掉误差计算） |
| `joint_move` | 等 settle → 读 `joint_positions()` 作起点 → 发 `JointTrajCmd` → 等时长 → 校验误差 | `joint_traj_task.cpp` |
| `cart_move` | 读 TF 末端位姿 → 加 `offset` → 发 `CartTrajCmd` → 等时长 → 校验误差 | `task_space_segment_task.cpp` |

**关键简化**：`reset` 只观察 `current_state`，因此 `/task_runner` 上不再声明 `reset_joint_pos` / `reset_duration` / `reset_tolerance`（对应验收 A7）。

---

## 阶段 4：对齐已验证序列

**目标**：确认新执行器行为与旧任务层一致（对应验收 A5）

**这一步不写新代码，只做对照验证。**

```bash
# 新路径
ros2 run tasks task_runner --ros-args -p use_sim_time:=true -p task_sequence:="reset_then_joint"
ros2 run tasks task_runner --ros-args -p use_sim_time:=true -p task_sequence:="reset_then_cart"
```

对应 YAML：

```yaml
sequences:
  reset_then_joint:
    steps:
      - {type: reset}
      - {type: joint_move, to: ready, duration: 3.0}
      - {type: idel}
  reset_then_cart:
    steps:
      - {type: reset}
      - {type: cart_move, offset: [0.0, 0.0, 0.05], duration: 2.0}
      - {type: idel}
```

| 编号 | 对比项 | 通过标准 |
| --- | --- | --- |
| V4.1 | 末端轨迹 | 与旧版 `[reset, joint_traj]` / `[reset, cart_traj]` 目视一致 |
| V4.2 | 日志时序 | 各阶段耗时量级一致 |
| V4.3 | 收敛残差 | 末态关节误差量级一致 |
| V4.4 | 失败回退 | 任务失败后控制器回 idel、机械臂保持、进程不退出（对应验收 A6） |

**此阶段通过后才可执行阶段 6 的删除操作。**

---

## 阶段 5：补齐其余步骤

| 步骤 | 实现要点 |
| --- | --- |
| `admittance_move` | `set_exp_state("admittance")` + 发 `AdmittanceCmd`，等时长 |
| `tool` | 发布 `std_msgs/Bool` 到 `/tool_cmd`；`action` 解析 `tool_on_` / `tool_off_` / `on` / `off` |
| `teach` | `set_exp_state("teach_pendant")`，等 duration，切回 idel |
| `measure` | `set_exp_state("measure")`，等 duration，切回 idel |

**同时处理阶段 1 遗留项**：关节越限检查（设计文档 5.1）。在仿真中确认阈值不误报后再合入 `update()`。

---

## 阶段 6：删除旧任务层代码

**前置条件：阶段 4 验证通过。**

### 删除文件

```
src/tasks/task/idel_task.{hpp,cpp}
src/tasks/task/reset_task.{hpp,cpp}
src/tasks/task/cart_traj_task.{hpp,cpp}
src/tasks/task/joint_traj_task.{hpp,cpp}
src/tasks/task/admittance_task.{hpp,cpp}
src/tasks/task/teach_pendant_task.{hpp,cpp}
src/tasks/task/measure_task.{hpp,cpp}
src/tasks/task/task_space_segment_task.{hpp,cpp}
src/tasks/fsm/task_fsm.hpp
src/tasks/fsm/task_fsm_factory.hpp
```

### 修改

- `src/tasks/CMakeLists.txt`：从 `tasks_core` 移除上述 `.cpp`，移除 `task/` 与 `fsm/` 的 include 目录
- 删除空的 `task/` 与 `fsm/` 目录

### 验证 6

| 编号 | 验证项 | 通过标准 |
| --- | --- | --- |
| V6.1 | 构建通过，无残留引用 | exit code 0，`grep -r "task_fsm" src/tasks` 无结果 |
| V6.2 | 阶段 4 的序列仍可执行 | 与 V4.1 结果一致 |
| V6.3 | 无重复参数声明 | `ros2 param list /task_runner` 中无 `reset_joint_pos` / `reset_tolerance` |
| V6.4 | 规模达标 | `find src/tasks -name "*.cpp" -o -name "*.hpp" \| xargs wc -l` 总量约 900 行 |

---

## 附：阶段依赖关系

```
阶段1（控制器故障策略）─┐
                       ├─→ 阶段4（对齐验证）─→ 阶段6（删除旧代码）
阶段2（执行器骨架）────┤
   └─→ 阶段3（核心步骤）┘
                └─→ 阶段5（补齐步骤）
```

阶段 1 与阶段 2/3 相互独立，可并行推进。
