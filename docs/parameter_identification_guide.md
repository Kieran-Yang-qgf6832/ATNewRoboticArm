# 参数辨识操作指南

适用范围：`feat/identify-without-qdd` 分支及之后。整套链路**不依赖关节加速度**
（实机 MCU 不上报 `q̈`，joint6 无电机），只吃 `q / q̇ / τ`。
数学原理与踩坑记录见 `docs/bugs.md` §9；这里只写「怎么操作」。

## 0. 流程总览

```
仿真/实机 measure 采样（激励轨迹，落 CSV，只有 q/q̇/τ）
        │
        ▼
identify_arm（分部积分滤波回归量 + QR 基参数 + 物理重建/投影）
        │
        ▼
辨识 URDF + identify 报告
        │
        ▼
verify_torque.py（用另一条没训练过的 CSV 留出验证）
```

- 训练集与验证集必须**不同 seed**（`measure_trajectory_seed` 换一个数即可）。
- 仿真验收判据：`base_parameter_relative_error_norm ≤ 1%`（范数度量，与 O8 的 0.39% 同口径）。
- 实机验收判据：留出集力矩 RMS ≤ 5% 额定（50 Hz 采样下参数误差指标不可靠，实测 1.44%，
  是样本数减半的统计损失，不是公式错误）。

## 1. 调参须知与典型配置

激励轨迹不是手写关节角，而是 CMA-ES 在傅里叶系数空间里优化出来的，形状被六类约束夹出来：
关节软限位、关节速度、关节力矩、末端箱式、连杆碰撞间距、地面净空。所以「调幅度」等于调这些约束。

参数都在 `arm_controller` 的两份 YAML 里（都在**控制器构造期**读取，改完必须重启 launch）：

- 仿真：`src/launch_pack/config/ros2_controller.yaml`
- 实机：`src/launch_pack/config/ros2_controller_real.yaml`（叠加在仿真那份之上，只写差异）

逐参数的影响、默认值与设置方式见第 2 节（仿真）与第 3 节（实机）——两节各自完整列出，
重复的参数也分别写，方便按环境照抄。

三条硬规则：

1. **参数在控制器构造期只读一次**：改 YAML 后必须重启 launch，运行中 `ros2 param set` 无效；
   写错参数名也不报错（C6），只会静默用默认值。具体设法见 2.2 / 3.2 末尾的「怎么设置」，
   实际生效值看启动日志 `激励约束：关节软限位 ... 速度上限 ... 谐波 ... 力矩上限 ...` 一行。
2. **改 period / max_fevals 要同步 `step_timeout`**。总耗时 ≈ 生成 + `move_to_start`(3 s)
   + `period × repeat_cnt`，而超时是从任务层请求 idel（`duration: 90`）时起算的。
   `measure.yaml` 的 `duration` 保持 90 即可，它不是预算。
   另外：走 `arm_measure_sim.launch.py` 时，`period` / `repeat_cnt` / `move_to_start_duration` /
   `seed` / `csv_file_path` 这 5 项**由 launch 覆盖**，改 YAML 无效，必须用 launch 参数（见 2.1）。
3. **想让动作变慢优先调 period**。默认 `velocity=10` rad/s、3 谐波、T=10 s、幅度 1 rad 时峰值
   速度约 1.9 rad/s，速度约束根本不生效；压 `measure_traj_vel_limit_rate` 通常只是让 CMA-ES
   换一个可行解，不如把周期拉长。

典型配置：

```yaml
# 更柔和、幅度收窄（实机首测 / 换新机械臂）
measure_trajectory_period: 15.0
measure_traj_pos_soft_limit_rate: 0.5
measure_traj_torque_limit_rate: 0.6
measure_traj_collision_safe_distance: 0.03
measure_trajectory_seed: 42

# 激励更强的仿真数据（想压辨识残差）
measure_trajectory_period: 8.0
measure_traj_pos_soft_limit_rate: 0.9
measure_traj_harmonics: 4
measure_traj_torque_limit_rate: 1.0
```

省时技巧：试参数时先把 `measure_traj_max_fevals` 调到 200（本机生成约 7 s），只跑一遍看轨迹形状
与日志里的 `激励约束` / `最终违例`，确认没撞限位再调回 1500 正式取数（200 次评估的轨迹不要拿去辨识）。
无论怎么调，验证集都必须**换一个 seed** 重录一份（见第 5 节）。

仍未暴露成参数的项（要改 `excitation_trajectory.cpp` 的 `constexpr` 并重新 colcon build）：
`k_lambda=24`、`k_initial_sigma=0.25`、傅里叶系数硬边界 `k_coeff_bound=2.0`（2 rad，通常软限位
会先卡住）。注意成员 `svd_pos_lower/upper`、`svd_vel_upper/acc_upper`、`svd_Y_stack_num` 只用于
随机采样评估回归矩阵的激励度，**不约束轨迹**，改它们不会让轨迹变大或变小。

## 2. 仿真测量（一条命令）

```bash
cd <项目根目录>
colcon build --packages-select arm_controller arm_model launch_pack tasks parameter_identify --symlink-install
source install/setup.bash

# 完整测量：起仿真 + 跑 measure_only 序列 + 落 CSV，结束后 launch 自动退出
# step_timeout 默认 300 s（本机实测生成 53 s + 移动 3 s + 激励 10 s ≈ 66 s，留约 4.5 倍余量）
ros2 launch launch_pack arm_measure_sim.launch.py

# 常用覆盖（period × repeat 变长时按比例加大 step_timeout）
ros2 launch launch_pack arm_measure_sim.launch.py \
    trajectory_seed:=42 trajectory_period:=10.0 trajectory_repeat_cnt:=1 \
    csv_file_path:=$PWD/meas_seed42.csv
```

### 2.1 launch 层参数（`arm_measure_sim.launch.py`）

这些是**启动命令行**上的名字，launch 会把它们写进一份补充 params 文件（`arm_controller.*` 前缀），
并且这份文件排在 `ParameterFile(ros2_controller.yaml)` **之后**加载——所以**同名参数以 launch 为准**。

> ⚠️ 这就是「改了 yaml 的 `measure_trajectory_period` 却不生效」的原因：本 launch 固定覆盖
> **5 个** measure 参数（`period` / `repeat_cnt` / `move_to_start_duration` / `seed` / `csv`）。
> 走这个 launch 时，这几项必须用 launch 参数（`trajectory_period:=20.0`），改 yaml 无效；
> 其余 measure 参数 launch 不覆盖，仍然只认 YAML。想验证 yaml 里的值，请用
> `arm_mujoco_sim.launch.py`（或 `run_measure:=false` 起仿真）后手动跑 `task_runner`。

| launch 参数 | 默认 | 落点与影响 |
| --- | --- | --- |
| `trajectory_seed` | 0 | 落到 `measure_trajectory_seed`。0 = 每次随机不可复现，非 0 固定可复现 |
| `trajectory_period` | 10.0 | 落到 `measure_trajectory_period`。单条轨迹周期（秒）；**改了它会同时改变生成耗时**（见 2.2 表） |
| `trajectory_repeat_cnt` | 1 | 落到 `measure_trajectory_repeat_cnt`。总测量时长 = `move_to_start + period × cnt + 生成时间` |
| `move_to_start_duration` | 3.0 | 落到 `measure_move_to_start_duration`。起点移动时长（秒） |
| `csv_file_path` | `<项目根>/measured_for_identification.csv` | 落到 `measure_csv_file_path`。输出 CSV |
| `controller_urdf_path` | arm_model URDF | 落到控制器 `urdf_path`。只改**控制器内部模型**（生成激励轨迹用）；被控对象始终是 arm_model 真值 |
| `run_measure` | true | 不落到控制器参数：`false` 只起仿真，测量另开终端手动跑（反复调参时用） |
| `step_timeout` | 300.0 | 不落到控制器参数：任务层 `task_runner` 的单步骤超时（从请求 idel 起算）。本机实测 T=10 总耗时 ≈66 s、T≥20 ≈130 s、T=120 ≈229 s，300 s 约有 2.3 倍余量；换更慢机器或调大 `max_fevals` 时再上调 |

### 2.2 仿真生效的完整 measure 参数（`ros2_controller.yaml`）

下表就是仿真实际读取的全部 measure 参数。**改这些必须改 yaml 并重启 launch**
（`ros2 param set` 无效，见下方「怎么设置」）。

| 参数 | 默认值 | 影响 |
| --- | --- | --- |
| `measure_trajectory_period` | 10.0 | 傅里叶基频为 `k/T`（k = 1..谐波数）。T 越大动作越慢越柔；**生成耗时**：T=10 每周期 1000 点 → 本机 ≈53 s，T ≥ 20 时 `eval_dt = max(0.01, T/2000)` 让点数封顶 2000 → ≈106 s，T 再大也不再增长。但**执行段按 T 线性变长**，改大要同步加 `step_timeout`，否则报 `measure did not reach 'idel'` |
| `measure_trajectory_repeat_cnt` | 1 | 周期重复次数，按倍数增加样本量与执行时长（10 s × cnt）。改大要同步加 `step_timeout` |
| `measure_move_to_start_duration` | 3.0 | 生成后走到轨迹起点的时间。太短会让起点跳变（关节从当前位置被硬拽），太长只是白等 |
| `measure_trajectory_seed` | 0 | CMA-ES 随机种子。**0 = 每次按时间随机，轨迹不可复现**；非 0 可复现，日志 `激励轨迹就绪 ... seed=` 打实际值。验证集必须换一个非 0 值 |
| `measure_csv_file_path` | `measured_for_identification.csv` | CSV 落盘路径，相对路径按**控制器进程 CWD** 解析。launch 默认覆盖成项目根绝对路径；路径过长或上次落盘未结束会导致保存失败（日志 `CSV 落盘请求失败`） |
| `measure_record_sample_rate` | 500.0 | **只用于估算记录缓冲区大小，不控制落盘频率**（仿真 500 Hz 控制 → 每周期 10 s 记 5000 行）。按低于实际控制频率填会写满缓冲区丢样本 |
| `measure_frozen_joints` | 空（不写该键） | 冻结关节不参与优化、位置锁在当前位形。仿真留空 = 6 轴全激励。**不要写 `[]`**：rclcpp 无法推断空数组元素类型，会存成 NOT_SET 并在声明时抛异常 |
| `measure_traj_pos_soft_limit_rate` | 0.7 | 关节位置软限位：允许区间 = 行程中点 ± rate × 半行程。joint5（URDF 行程 ±0.611 rad）在 0.7 下只有 **±0.428 rad**。调大（0.9）幅度更大更激进，调小（0.5）更保守。合法范围 (0, 1]，**>1 会被夹到 1.0**——软限位是唯一的位置约束，超过 1 就会让轨迹越过 URDF 行程 |
| `measure_traj_vel_limit_rate` | 1.0 | 关节速度上限 = rate × URDF `velocity`（现在全是 10 rad/s）。按 3 谐波 / 10 s 周期估算峰值速度仅 ≈1.9 rad/s，**默认这条约束基本不生效**，压它通常只是让 CMA-ES 换个可行解；想让动作变慢应调大 period。`<= 0` 关闭速度约束（不建议，历史上关掉后关节被甩到高速、q̈ 出现尖峰） |
| `measure_traj_harmonics` | 3 | 傅里叶谐波数（频率成分 1/T…k/T），内部夹到 1–8。调大（4–5）激励带宽更宽但动作更剧烈、生成更慢；调到 1 只剩单频，**辨识信息明显不足** |
| `measure_traj_max_fevals` | 1500 | CMA-ES 评估次数，**决定生成耗时**，近似线性（本机 1500 ≈ 53 s @T=10 / 106 s @T≥20；200 ≈ 7 s / 14 s）。低于约 20 倍优化维度（6 轴 36 维 → 720 次）时远未收敛，返回的基本是「随机可行解」，**这样的轨迹不要拿去辨识** |
| `measure_traj_torque_limit_rate` | 1.0 | RNEA 力矩 ≤ rate × 各关节额定 effort（11 / 20 / 6 / 6 / 1.8 / 50 N·m）。0 = 关闭。调小会同时压低幅值与加速度；**关掉后仿真里关节会被甩到高速、记录数据报废** |
| `measure_traj_collision_safe_distance` | 0.02 | 非相邻连杆网格间最小间距（米），0 = 关闭。调大会缩小可行域、幅度下降；调到可行域为空时生成失败（最终违例 ≠ 0） |
| `measure_traj_ground_clearance` | 0.01 | 每个连杆网格最低点高于 z = 该值（米），base_link 除外。调大同样压缩可行域；0 = 关闭时 link4/5/6 可能钻进地面，MuJoCo 接触冲量会污染辨识数据 |
| `measure_end_effector_x/y/z_lower/upper_limit` | ±inf（关闭） | 末端位置箱式约束，**收紧是最直接的「压幅度」手段**。注意 z 下限卡的是「工具最低点」= 末端原点 z − `tool_offset` |
| `measure_end_effector_tool_offset` | 0.07 | 工具最低点相对末端 frame 原点的偏移（沿 −z，米），只在 z 下限有限时才有意义 |

**怎么设置（仿真）**：

1. **启动时生效（唯一可靠的改法）**
   - 改 yaml 后重启：编辑 `ros2_controller.yaml` → 重跑 `ros2 launch launch_pack arm_measure_sim.launch.py`；
   - launch 覆盖（2.1 里那 5 项 measure 参数；**这几项改 yaml 会被 launch 顶掉**）：
     `ros2 launch launch_pack arm_measure_sim.launch.py trajectory_seed:=42 trajectory_period:=15.0`；
   - 手动跑控制器时用 `--ros-args -p`，但参数必须带 `arm_controller.` 前缀落在 controller_manager 上，
     例如 `ros2 run controller_manager ros2_control_node --ros-args -p arm_controller.measure_trajectory_seed:=42`
     （直接给控制器节点传裸参数会被静默忽略）。
2. **运行中修改：`measure_*` 无效**。它们在 `ParamterMeasureState` 构造期只读一次
   （`arm_fsm.cpp:1284-1321`），启动后 `ros2 param set` 改了既不会报错也不会生效。唯一能确认
   实际生效值的地方是启动日志的 `激励约束：关节软限位 ... 速度上限 ... 谐波 ...` 一行。

手动跑测量序列（仿真已起、控制器已激活时）：

```bash
ros2 run tasks task_runner --ros-args \
    -p tasks_config:=$(ros2 pkg prefix tasks)/share/tasks/config/measure.yaml \
    -p task_sequence:=measure_only -p use_sim_time:=true -p exit_when_idle:=true \
    -p step_timeout:=300.0
```

> ⚠️ **测量耗时与超时**（2026-10-31 本机实测：ThinkBook 16 G6，6 关节全开 + 碰撞约束，seed=0）：
> 进入 measure 后控制器先用 CMA-ES 优化激励轨迹（默认 1500 次评估），期间机械臂保持不动；
> 生成过程只在开始与就绪时各打一条日志，需要细看进度可把控制器日志级别开到 debug。
>
> **生成耗时 ≈ 每周期采样点数 × 评估次数**，而采样点数由 `eval_dt = max(0.01, T/2000)` 决定：
>
> - `T = 10 s`：每周期 1000 点 → 生成 **53 s**，加 `move_to_start` 3 s + 执行 10 s ≈ **66 s**，落盘 4952 样本；
> - `T ≥ 20 s`：点数封顶 2000 → 生成 ≈ **106 s**，且 T 再变大也不再增长；
> - 所以 `T = 120 s` 的总耗时 ≈ 106 + 3 + 120 ≈ **229 s**（不是旧文档里的 1263 s）。
>
> `measure.yaml` 的 `duration` **不是**这个超时：它只是任务层「开始计时/等待 idel 的起点」，
> 90 s 后任务层会请求 idel，控制器要到 `HoldingEnd` 才真正接受，期间测量照跑、数据不丢。
> 真正会掐断测量的是 `step_timeout`——从发出 idel 请求时开始计，必须满足
> `生成 + move_to_start + period × repeat_cnt ≲ duration + step_timeout`。
> launch 默认 **300 s**（对本机 T=10 / 20 / 120 分别留出约 4.5 / 2.3 / 1.3 倍余量），
> 直接 `ros2 launch launch_pack arm_measure_sim.launch.py` 即可跑通。
>
> ```bash
> ros2 launch launch_pack arm_measure_sim.launch.py                        # 默认 300 s
> ros2 launch launch_pack arm_measure_sim.launch.py trajectory_period:=20.0 # 改周期用 launch 参数
> ros2 launch launch_pack arm_measure_sim.launch.py step_timeout:=900.0     # 慢机器 / 加大 max_fevals
> ```
>
> 三段提示按顺序出现：`激励轨迹就绪`（含 seed/违例/得分）与 `激励约束` →
> `开始执行激励轨迹`（机械臂此刻才开始动）→ `激励轨迹执行完成，已记录 N 个样本；正在落盘 CSV`
> → `sequence 'measure_only' finished`。CSV 落盘在 `HoldingEnd` 之前完成。
> 生成耗时随 CPU、谐波数、`max_fevals` 与约束配置变化；换机器后按实测留 50% 以上余量。

输出 CSV 表头（仿真带 `acc_*`，实机没有——两条列名不要混用）：

```text
仿真: time,pos_0..pos_5,vel_0..vel_5,acc_0..acc_5,torque_0..torque_5
实机: time,pos_0..pos_5,vel_0..vel_5,torque_0..torque_5
```

## 3. 实机测量

```bash
# 1) 起实机链路（确认 USB/标定完成后）
ros2 launch launch_pack arm_real.launch.py

# 2) 另开终端，跑 measure 序列（生成阶段机械臂不动，本机约 53 s，别在此时断电/急停）
ros2 run tasks task_runner --ros-args \
    -p tasks_config:=$(ros2 pkg prefix tasks)/share/tasks/config/measure.yaml \
    -p task_sequence:=measure_only -p exit_when_idle:=true -p step_timeout:=300.0
```

实机实际生效的参数 = `ros2_controller_real.yaml`（**叠加**在 `ros2_controller.yaml` 之上）
+ 未被覆盖的仿真默认值。所有 `measure_*` 都在控制器**构造期**读取，改完必须重启
`arm_real.launch.py`；`arm_real.launch.py` **不提供**任何 measure 覆盖项（`arm_measure_sim.launch.py`
才提供 `trajectory_seed` / `csv_file_path` 等），所以**实机的 seed 与 CSV 路径只能改 YAML**。

### 3.1 实机特有的非 measure 参数

文件：`src/launch_pack/config/ros2_controller_real.yaml`（measure 参数见 3.2）

| 参数 | 实机值 | 说明 |
| --- | --- | --- |
| `controller_manager.update_rate` | 100 | `write()` 是同步 USB bulk 发送（超时 2 ms），200 Hz 会被抖动拖到周期超时 |
| `command_interface_prefix` | `""` | 直连硬件接口，不带 `<ros2_control name>` 前缀 |
| `real_mode` | true | 拒绝 `teach_pendant`（MCU 不接收阻抗增益）；measure 不受影响 |
| `task_dim` | 5 | 位置 3 + 工具指向 2，绕工具轴滚转不参与求解（只影响笛卡尔运动） |
| `exp_state` | idel | 上电只锁住当前姿态，**不自动复位**（臂可能停在任意位置） |
| `default_kp` | `[40, 40, 40, 2, 2, 0.2]` | 固件不接收，仅用于满足 ros2_control 命令接口 |
| `default_kd` | `[1.6, 1.6, 1.6, 0.08, 0.04, 0.01]` | 同上 |
| `gravity_compensation` | false | 实机力矩会被固件直接叠加到速度环且无内环限幅，先关；符号核实后从 0.1 起加 |
| `gravity_compensation_scale` | 0.1 | 与上一项配套 |

### 3.2 实机生效的完整 measure 参数

实机 = `ros2_controller_real.yaml` 显式项 + 继承 `ros2_controller.yaml` 的默认值。下表是合并后的
**实际生效值**，与 2.2 的仿真表逐项对应（重复项照写，便于直接照抄）。

| 参数 | 实机生效值 | 影响 |
| --- | --- | --- |
| `measure_trajectory_period` | 10.0 | 傅里叶基频 `k/T`。实机首测建议拉到 **15.0**（更柔和、MCU 更容易跟）；生成耗时 T=10 约 53 s、T ≥ 20 约 106 s（点数封顶 2000，不再随 T 增长），但执行段按 T 线性变长，改大要同步加 `step_timeout`，否则报 `measure did not reach 'idel'` |
| `measure_trajectory_repeat_cnt` | 1 | 周期重复次数。实机 100 Hz 落盘、10 s 一个周期约 1000 行；执行时长 = 10 s × cnt，改大要同步加 `step_timeout` |
| `measure_move_to_start_duration` | 3.0 | 生成后走到轨迹起点的时间。实机固件内环 1 ms、外环 100 Hz，3 s 足够；调小会让起点跳变 |
| `measure_trajectory_seed` | 0 | **实机必须改成非 0**：0 = 每次按时间随机、不可复现，且 `arm_real.launch.py` 不透传该参数，只能改 yaml。验证集换另一个非 0 值 |
| `measure_csv_file_path` | `measured_for_identification.csv` | 相对路径按**启动进程 CWD** 解析，**实机建议改成绝对路径**（仿真 launch 会自动覆盖成项目根，实机没有这层覆盖）。写错目录或上次落盘未结束会报 `CSV 落盘请求失败` |
| `measure_record_sample_rate` | 500.0 | **只用于估算记录缓冲区大小，不控制落盘频率**。实机必须按 ≥100 Hz 填（实际每周期约 1000 行），按 50 填会写满缓冲区丢样本 |
| `measure_frozen_joints` | `[joint6]` | joint6 无电机、机械锁死：傅里叶系数恒为 0、位置锁在当前位形，**优化维度只剩 joint1–joint5**。写错关节名只会打一条 WARN 然后忽略（`measure_frozen_joints 里的 "xxx" 不在 joints 列表里`） |
| `measure_traj_pos_soft_limit_rate` | 0.7 | 关节位置软限位 = 行程中点 ± rate × 半行程。joint5（±0.611 rad）在 0.7 下只有 **±0.428 rad**；实机首测建议降到 **0.5**（±0.306 rad）。范围 (0, 1]，>1 会夹到 1.0，否则轨迹会越过 URDF 行程 |
| `measure_traj_vel_limit_rate` | 1.0 | rate × URDF `velocity`（10 rad/s）。**实机真正的速度上限在 MCU 固件内环，这里只约束生成**：压到 0.5 会让轨迹更慢更平缓、更容易被判可跟踪，但默认约束本来就不生效。`<= 0` 关闭（不建议） |
| `measure_traj_harmonics` | 3 | 谐波数 1–8。实机 100 Hz 控制、50 Hz 报文，**不建议超过 3**——更高频成分跟不出来，实测力矩会与指令背离、辨识数据失真 |
| `measure_traj_max_fevals` | 1500 | CMA-ES 评估次数，生成耗时近似线性（本机 1500 ≈ 53 s @T=10 / 106 s @T≥20，200 ≈ 7 s / 14 s）。低于约 20 倍优化维度时返回「随机可行解」，**实机上不要拿这种轨迹去跑**（幅度/速度不可控） |
| `measure_traj_torque_limit_rate` | 1.0 | RNEA 力矩 ≤ rate × 各关节额定 effort（11 / 20 / 6 / 6 / 1.8 N·m；joint6 冻结不参与）。实机首测建议 **0.6**。**实机超限会直接跟踪失败**，仿真里则会甩到高速、q̈ 出现尖峰 |
| `measure_traj_collision_safe_distance` | 0.02 | 非相邻连杆网格最小间距（米），0 = 关闭。实机加大到 0.03–0.04 更安全，但可行域变小、幅度下降；调到可行域为空则生成失败 |
| `measure_traj_ground_clearance` | 0.01 | 每个连杆网格最低点高于 z = 该值（米），base_link 除外。实机构成硬安全边界：调大压缩幅度，0 = 关闭时 link4/5/6 可能撞地 |
| `measure_end_effector_x/y/z_lower/upper_limit` | ±inf（关闭） | 末端位置箱式约束，收紧是最直接的「压幅度」手段；实机可以按工作台/障碍物范围收紧。z 下限卡的是工具最低点（末端 z − `tool_offset`） |
| `measure_end_effector_tool_offset` | 0.07 | 工具最低点相对末端 frame 原点的偏移（沿 −z，米），含 0.0569 m 的 STL 实测值 + 余量；只在 z 下限有限时才有意义 |

**怎么设置（实机）**：

1. **启动时生效（唯一可靠的改法）**
   - 改 yaml 后重启：编辑 `ros2_controller_real.yaml`（或改 `ros2_controller.yaml` 里的公共默认）
     → 重跑 `ros2 launch launch_pack arm_real.launch.py`；
   - `arm_real.launch.py` **没有** measure 覆盖项，所以不能像仿真那样 `trajectory_seed:=`；
   - 也可以在启动命令上盖 controller_manager 的参数，但必须带 `arm_controller.` 前缀，
     且 `$(find ...)` 这类名字要写全，例如
     `ros2 run controller_manager ros2_control_node --ros-args -p arm_controller.measure_csv_file_path:=/abs/path/meas.csv`
     （裸参数会被 controller_manager 吞掉，落到它自己身上而不到控制器）。
2. **运行中修改：`measure_*` 无效**。参数在 `ParamterMeasureState` 构造期只读一次
   （`arm_fsm.cpp:1284-1321`），运行中 `ros2 param set` 既不报错也不生效。实机确认生效值同样
   只能看启动日志 `激励约束：关节软限位 ... 速度上限 ... 谐波 ...` 一行（以及 `冻结 N 个关节`）。

三点提醒：

- **CSV 没有 `acc_*` 列**（实机 CSV 只有 `time, pos_*, vel_*, torque_*`），这是正常的，辨识不需要加速度；
- **相邻两行是同一帧报文的重复**（MCU 50 Hz 报文、控制器 100 Hz 落盘），不影响最小二乘；
- 实机与仿真的**规划层参数完全相同**（同一份代码、同一个 URDF），差别只在 joint6 冻结、
  控制频率与驱动模型——所以仿真能验证「参数怎么改变轨迹」，验证不了「实机跟不跟得上」。

⚠️ 实机测量前的安全检查（docs/real_first_run.md）：标定完成、`exp_state: idel`、
急停可及。measure 会先花 `measure_move_to_start_duration`（默认 3 s）走到轨迹起点再激励。

## 4. 离线辨识

```bash
cd <项目根目录>
ros2 run parameter_identify identify_arm \
    --csv measured_for_identification.csv \
    --urdf src/arm_model/model/robotic_arm.urdf \
    --output /tmp/robotic_arm_identified.urdf \
    --config src/parameter_identify/config/identify.yaml \
    --report /tmp/robotic_arm_identify_report.yaml
```

关键配置（`src/parameter_identify/config/identify.yaml`）：

| 键 | 默认 | 说明 |
| --- | --- | --- |
| `filter.enabled` | true | **实机唯一可行路径**；false 需要 `acc_*`，实机会被明确拒绝 |
| `filter.cutoff_hz` | 2.0 | 低通截止频率；低于结构模态又不滤掉激励能量，50–500 Hz 采样实测都能用 |
| `filter.drop_initial_seconds` | 0.5 | 丢开头瞬态（零初值滤波器必需），**不可调小** |
| `filter.derivative_scheme` | `central` | `L(Ḃ)` 离散格式；`identity` 只留作对照（会留 ~1% 模型残差） |
| `filter.qr_tolerance` | 1.0e-6 | 滤波路径秩判据；改成 1e-8 会让弱列进基、参数爆炸（Q6） |
| `data.acceleration_source` | `recorded` | 实机 CSV 无 `acc_*` 时返回 `None`（报告记 `absent`），**不再报错** |

报告里看这些字段：

| 字段 | 判据/含义 |
| --- | --- |
| `base_parameter_relative_error_norm` | **验收用这个**（仿真 ≤1%）；`..._max` 逐项分母含 0 参数，只看定位 |
| `rmse` | 滤波后力矩拟合残差 |
| `physical_validation.passed` | 必须为 true |
| `reconstruction_status: error` | **正常的**：原始 SDP 判不可行（真值贴在可行集边界），物理可行性由 `physical_projection` 保证（bugs.md O8） |
| `filter.recorded_acceleration_ignored` | 出现即说明 CSV 里的 `acc_*` 被忽略（滤波路径本来就不吃它） |

## 5. 留出验证（必做）

用**另一条 seed** 的 CSV 验证辨识 URDF，别信报告里的 `rmse`。**第一份 `--csv` 视为训练集，其余视为留出集**：

```bash
python3 src/parameter_identify/tools/verify_torque.py \
    --urdf /tmp/robotic_arm_identified.urdf \
    --reference src/arm_model/model/robotic_arm.urdf \
    --csv measured_for_identification.csv \
    --csv meas_seed42.csv
```

工具按 CSV 列数自动选路径：

- **带 `acc_*`（仿真，1+4×dof = 25 列）**：pinocchio 用记录的 q̈ 做 RNEA 正演，输出相对误差；
- **不带 `acc_*`（实机，1+3×dof = 19 列）**：复用辨识链路的滤波回归量
  （`build_acceleration_free_regressor`），把该 URDF 的惯性参数当作固定 φ，算
  `τ_f − W_f φ` 的残差 RMS，再按各关节**额定力矩**归一。这一步不需要加速度，实机 CSV 可直接用。

实机判据（工具会打印 PASS/FAIL）：

| 判据 | 阈值 | 含义 |
| --- | --- | --- |
| 留出集残差 | ≤ 5% 额定力矩 | 绝对精度够用 |
| 留出 / 训练 | ≤ 1.5 | 没有过拟合到那一条轨迹 |
| 被测 / 参照 | < 1.0（1.0–1.05 只提示持平） | 辨识确实比先验更贴合数据 |

输出示例（真值 URDF 自比，用来确认链路本身无误）：

```
[被测] robotic_arm.urdf  vs  sim_joint6_frozen_500hz.csv  (4750 样本 @2.00 ms，无 acc_* → 滤波回归量)
    残差 RMS  0.00226 N·m   占额定力矩 最大  0.025%   （判据 ≤ 5%）
    逐关节 [% 额定]:   0.021   0.023   0.025   0.022   0.022   0.000
    残差 vs |dq| 相关 +0.129
```

两个注意点：

- **比值只在两边同量级时有意义**。若参照模型残差已接近 0（例如拿仿真真值数据去验证实机辨识出的
  URDF），比值会被放大到几十上百倍——这时看绝对百分比，别盯比值。
- **只给一份 CSV 时**工具会提示「只能说明拟合能力，无法判断泛化」；留出集必须换 seed 重录。

## 6. 回归测试

```bash
source /opt/ros/jazzy/setup.bash      # pinocchio 在 ROS jazzy 的 site-packages，不 source 会 import 失败
cd src/parameter_identify
python3 -m pytest test -q
```

fixture：`test/fixtures/sim_joint6_frozen_500hz.csv`（500 Hz、10 s、joint6 冻结、无 `acc_*`）。
断言：`base_parameter_relative_error_norm ≤ 0.01` 且 `rmse < 0.01`（实测 0.75% / 0.0021）。

## 7. 注意事项速查

1. `measure_csv_file_path` 由控制器在**构造阶段**读取——`ros2 param set` 不生效，必须用
   launch 传参或改 YAML 后**重启控制器**（C6）。
2. `measure.yaml` 的 `duration` 不是总超时，只是任务层开始等待 idel 的时刻；
   真正的上界是 `step_timeout`（从发出 idel 请求开始计）。控制器实际耗时 =
   `生成（本机 T=10 约 53 s、T≥20 约 106 s）+ move_to_start + period × repeat_cnt`，
   所以 `step_timeout` 至少要覆盖「生成 + 移动 + 执行」并留余量。
3. 不要叠加跑两遍 measure 把数据接在一起辨识——`time` 列不单调会在
   `estimate_sample_time` 被判掉。
4. `identify.yaml` 里 `strict_base_constraints / base_residual_weight / shape_prior /
   central_moment_prior / tensor_trust_region` 这组开关 vendored FIGAROH 0.6.0 不接受，
   启动时会打 WARN 并写进报告的 `inactive_reconstruction_options`，属已知行为。
5. 仿真里 `acc_*` 只在 `state_acceleration_available_` 为真时写；实机恒不写。
6. 不要把上一轮辨识输出当下一轮先验喂回去——误差会复利累积（C7）。
7. 实机 50 Hz 的有效样本减半，参数范数误差会到 ~1.4%，这**不影响**力矩 RMS 判据；
   追求参数精度就加长测量（提高 `trajectory_repeat_cnt`），不要调 cutoff。
