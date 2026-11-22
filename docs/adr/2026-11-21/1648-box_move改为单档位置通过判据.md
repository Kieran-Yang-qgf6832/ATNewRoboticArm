# ADR-20261121-1648：box_move 改为单档位置通过判据

- 状态：已采纳
- 日期：2026-11-21 16:48
- 相关：`src/robot_msgs/msg/CartTrajCmd.msg`、`src/arm_controller/arm/arm_fsm.cpp`、`src/tasks/executor/task_executor.cpp`

## 背景

`box_move` 把视觉目标面转成 5 维（位置 + 工具 z 轴）目标。原设计在 5D 预检拒绝
（位置 > 1 mm 或工具轴 > 5°）且**未产生运动**时，自动回退到 position-only：
位置仍到同一个 link6 点，姿态完全不约束。

这有两个问题：

- 回退后工具尖端位置失去定义（尖端 = link6 原点 + 旋转后的 tool 轴偏移，
  最大可差 2 × `tool_offset` ≈ 78 mm），「到了」不等于「贴到面」；
- 目标只是略超可达边界时，直接失败过于武断，操作者其实希望「能多近就多近」。

曾考虑多级降级阶梯（位置 3 cm → 轴 5° → 位置 5 cm → 放弃），但一旦把位置容差
定义为**通过判据**（收敛仍 1 mm，追不上时停在最近可达位姿），3 cm 与 5 cm 两级
停的位置完全相同，中间只差「3~5 cm 算不算成功」——阶梯是空转，只剩一档有意义。

## 决策

1. **单档通过容差**：`box_move` 步骤的 `tolerance`（默认 5 cm）同时作为
   `CartTrajCmd.position_pass_tolerance` 和任务层验收，一个值三处使用
   （预检、执行、验收）。
2. **通过语义**：IKSolver 的位置收敛判据仍是 `pos_move_tolerance`（1 mm）；
   跑完未收敛时把停滞点（最近可达位姿）作为解，位置误差 ≤ 通过容差即算成功
   并执行；超过则整条轨迹在下发前被拒绝，臂原地保持。
3. **按轨迹下发**：通过容差放在消息字段里，控制器无隐式档位；0 = 与收敛判据相同，
   旧发布者行为不变。6 维混合范数模式不使用该放宽。
4. **预检采样点同权**：终点与 2 个中间采样点都用通过容差。只用 1 mm 检查中间点的话，
   一条 10 cm 的贴面路径差 4 cm 时会在 2/3 处被拒，臂一步不动。
5. **删除 position-only 回退**：工具轴 > 5° 直接失败。宁可明确失败，也不丢姿态。
6. **失败提前收尾**：任务层在 `duration` 内轮询 `current_state`，控制器离开
   `cart_traj` 就立刻收尾（约 0.5 s，受 500 ms 状态刷新限制），不再白等满时长。

## 备选方案

- **多级阶梯（3 cm → 轴 5° → 5 cm）**：空转，见背景。
- **全局把 `pos_move_tolerance` 改成 5 cm**：`pos_move` 差 4.9 cm 也会判成功，
  演示与回归测试失去失败信号。位置通过容差因此按轨迹下发。
- **控制器内部跑三级阶梯**：单次预检 3 档 × 3 个点 ≈ 3.5 ms，超过 2 ms 控制周期；
  任务层也看不出最终用了哪一档。
- **保留 position-only 回退**：轴不可达时仍能保住位置，但工具尖端落点无定义，
  与「贴面」的目标冲突。

## 后果

- `box_move` 的 `tolerance` 语义从「到位精度」变为「允许差多远仍算成功」。
- 低处箱体（`box_move_low`）从「回退后通过」变为**预期失败**，作为「不可达即放弃」
  的回归用例保留。它和 `box_move_demo` / `real_compat_box` 在当前模型下都不可达，
  根因是 O18（模型换代后目标点未重测），与本变更无关。
- 控制器预检靠放宽判据通过时打 WARN：
  `target accepted with the relaxed position pass tolerance: nearest reachable pose is ...`。
- `CartTrajCmd` 新增字段，需重建 `robot_msgs` 与两个节点；不填该字段的旧发布者
  行为完全不变。

## 相关链接

- `src/robot_msgs/msg/CartTrajCmd.msg`
- `src/arm_controller/lib/calculate/kinamic.hpp`（`set_solve_options` 第 5 个参数）
- [02 仿真运行 § 5 维轨迹的两档角容差](../../execute/02-仿真运行.md#5-维轨迹的两档角容差)
- [05 任务层与序列](../../execute/05-任务层与序列.md)
- [变更记录](../../changelog/2026-11-21/1648-box_move单档通过判据与取消回退.md)
