# 变更：box_move 改为单档位置通过判据，取消 position-only 回退

- 日期：2026-11-21 16:48
- 类型：行为变化 / 接口变更
- 相关：[ADR](../../adr/2026-11-21/1648-box_move改为单档位置通过判据.md)、[02 仿真运行](../../execute/02-仿真运行.md)、[05 任务层与序列](../../execute/05-任务层与序列.md)、[07 离线工具与排错](../../execute/07-离线工具与排错.md)、[O18 模型更新后目标点未重测](../../bugs/2026-11-14/1730-模型更新后目标点未重测.md)

## 背景

原 `box_move` 在 5D 预检拒绝且未产生运动时回退到 position-only：位置到同一个
link6 点，姿态完全不约束。回退后工具尖端落点失去定义（最大偏差约 79 mm），
「到了」并不等于「贴到面」；而目标只是略超可达边界时，直接失败又过于武断。

尝试过多级降级阶梯（位置 3 cm → 轴 5° → 位置 5 cm → 放弃），结论是：
一旦位置容差变成**通过判据**（收敛仍 1 mm、追不上时停在最近可达位姿），
3 cm 与 5 cm 两档停的位置完全相同，阶梯是空转。最终只保留一档 5 cm。

## 变更

- `CartTrajCmd` 新增 `float32 position_pass_tolerance`（米，0 = 与收敛判据相同，
  旧发布者行为不变）。
- `IKSolver::set_solve_options()` 增加第 5 个参数（位置通过容差），
  `ArmSolve` 一并转发；新增 `last_used_relaxed_position()` /
  `last_ik_used_relaxed_position()`。6 维混合范数模式不使用该放宽。
- 控制器 `CartTrajState` 按消息设置通过容差（非法值退回 0，>0.5 m 被 clamp 并 WARN）；
  预检的终点与 2 个中间采样点用同一个值；靠放宽通过时打
  `target accepted with the relaxed position pass tolerance: nearest reachable pose is ...`。
- 控制器预检拒绝日志同时打印位置的收敛/通过容差与工具轴的收敛/通过容差。
- 任务层 `run_position_move()` 增加 `position_pass_tolerance` 参数并透传到消息；
  等待期间轮询 `current_state`，控制器离开 `cart_traj` 立即收尾（约 0.5 s），
  不再白等满 `duration`。
- `box_move` 默认 `tolerance` 从 5 mm 改为 **5 cm**（`kDefaultBoxTolerance`），
  同一个值同时作为消息里的通过容差与任务层验收；删除 position-only 回退分支。
- `ik_solve` 增加 `--accept-position <m>`，用于严格/放宽的离线 A/B。
- `box_move_low` 改为**预期失败**用例（不再有回退）。

## 影响与迁移

- `box_move` 的 `tolerance` 语义从「到位精度」变为「允许差多远仍算成功」。
  想回到更严的行为，给步骤显式写 `tolerance`（例如 `0.005`），消息里的通过容差
  会随之收紧。
- 工具轴 >5° 或位置 >5 cm 时：控制器在下发任何运动前拒绝、原地保持并回 idel；
  任务层以退出码 1 结束，不重试、不回退。
- 消息字段位于消息末尾，`robot_msgs` 与两个节点需一起重建；不填该字段的
  旧发布者拿到的是严格行为。
- 控制器参数 `pos_move_tolerance`（1 mm）仍是**收敛**判据，本次未改。

## 验证

- 构建：`robot_msgs` / `arm_controller` / `tasks` 三个包编译通过（仅有既有的
  `set_value` nodiscard 警告）。
- 离线 A/B（`ik_solve --axis`，目标 = ready 沿工具轴外推 0.35 m，初值 ready）：
  严格档 `Unreachable`（最近可达差 5.77 mm / 2.98°）；`--accept-position 0.05`
  `Converged` + `relaxed_position = yes`；外推 0.45 m（5.22 cm）两档都拒绝；
  外推 0.65 m 时位置 4.96 cm 但轴 11.6°，仍拒绝（位置与工具轴各自判据）。
- 仿真 `box_move_demo`（抬高视觉场景）：控制器日志确认收到并按 5 cm 判据预检
  （`position converge 1.0e-03 m / pass 5.0e-02 m`）；目标不可达时预检拒绝、
  臂原地保持，任务层在发布后约 0.2 s 报错退出（轮询提前收尾生效）。
- **未通过**：`box_move_demo` 无法完成运动。根因是 [O18](../../bugs/2026-11-14/1730-模型更新后目标点未重测.md)：
  `fb76cb4` 换代后 `box_pregrasp` 等目标点未重测，视觉目标在当前模型下
  5D 不可达（本次实测 48.2 mm / 7.8°）。与本次改动无关，演示序列待 O18 修复后重跑。
