# 变更：新增 pos_move 位置约束运动

- 日期：2026-11-08 09:56
- 类型：功能
- 相关：[pos_move 实现与验证](../../bugs/2026-11-08/0956-pos_move实现与验证.md)

## 背景

原有 `cart_move` 只能相对当前位姿平移且严格保持姿态，无法直接给基座系绝对位置，
也无法只约束位置、放开姿态。

## 变更

- 新增 `pos_move` 步骤：基座系绝对位置，默认 `MODE_POSITION_ONLY`；
- 可选 `orientation`，只约束 link6 的 z 轴，形成 5 维“位置 + 工具轴”模式；
- `TaskMapping` 显式投影掉不约束的旋转分量；
- `IKSolver` 增加位置/工具轴分离判据与零空间就近偏好；
- 新增 `pos_move_tolerance` / `pos_move_axis_tolerance_deg` /
  `pos_move_nullspace_weight` / `pos_move_precheck_samples` 参数。

## 影响与迁移

- `CartTrajCmd` 仍是 6 维消息，新增 `mode` 字段；旧发布者行为不变；
- 位置约束轨迹的放行由 `allow_constrained_traj` 控制（实机默认 false）；
- `pos_move_demo` 实测位置残差 0.0009 m，姿态漂移只告警。

## 相关链接

- [O2 不可达目标](../../bugs/2026-10-10/1015-不可达目标未检测.md)
- [任务层与序列](../../execute/05-任务层与序列.md)
