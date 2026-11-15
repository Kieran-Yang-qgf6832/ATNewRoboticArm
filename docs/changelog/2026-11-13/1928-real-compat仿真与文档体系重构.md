# 变更：real-compat 仿真与文档体系重构

- 日期：2026-11-13 19:28
- 类型：功能 / 文档
- 相关：[real-compat ADR](../../adr/2026-11-13/1928-real-compat仿真方案.md)、
  [位置约束开关 ADR](../../adr/2026-11-13/1928-位置约束轨迹开关拆分.md)

## 背景

实机只有 5 自由度，但仿真始终是 6 自由度；位置约束轨迹又被 `real_mode` 一刀切拒绝，
导致 real-compat 仿真无法验证要验证的任务。同时 `docs/bugs.md` 已增长到 2000+ 行。

## 变更

### 功能

- 拆分 `real_mode` / `allow_constrained_traj`；
- 新增 `robotic_arm_real_compat.xml`（`motor6 gear=0`）、
  `scene_real_compat.xml`、`scene_vision_raised_real_compat.xml`；
- 场景用 `<equality><joint joint1="joint6" polycoef="0 1 0 0 0"/></equality>` 锁死 joint6；
- 三个 launch 增加 `controller_overlay`；
- 新增 `box_pregrasp_5dof` / `box_pregrasp_5dof_high` 与三条 real-compat 验收序列；
- `joint_calibrate` 增加 `print_zero`，打印可粘贴的标定片段。

### 文档

- 建立 `docs/bugs/`、`docs/adr/`、`docs/execute/`、`docs/changelog/`；
- `docs/bugs.md` 与 `docs/real_first_run.md` 保留为兼容入口；
- README 瘦身为目标/特性/快速开始/文档地图；CLAUDE 压缩为规则与导航。

## 影响与迁移

- 实机默认 `allow_constrained_traj: false`，验证通过后再显式打开；
- 旧的 `docs/bugs.md` 锚点不再维护，从新索引进入；
- `docs/superpowers` 内容并入 ADR 后删除，`AGENT.md` 并入 CLAUDE 后删除。

## 验证

- `real_compat_joint` / `real_compat_pos` / `real_compat_box` 退出码 0；
- `box_move` 位置残差 0.0021 m、工具轴 0.34°；
- `/joint_states` 中 joint6 ≈ 1e-11；
- 默认 `pos_move_demo` 回归通过。

## 相关链接

- [缺陷索引](../../bugs/README.md)
- [ADR 索引](../../adr/README.md)
- [执行说明](../../execute/README.md)
