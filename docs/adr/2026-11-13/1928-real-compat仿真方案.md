# ADR-20261113-1928：real-compat 仿真方案

- 状态：已采纳
- 日期：2026-11-13 19:28
- 相关：[位置约束轨迹开关拆分](1928-位置约束轨迹开关拆分.md)、[实机首次运行](../../execute/04-实机首次运行与标定.md)

## 背景

实机只有 5 个自由度（joint6 无电机、机械锁死），但普通仿真始终是 6 自由度。
很多轨迹在仿真里“能到”，上电后才发现 joint6 被锁死、工具轴约束不可满足，或者
粗定位点根本到不了。例如 `box_pregrasp` 的 joint6 是 0.7908 rad，实机不可能达到。
需要一个**不依赖真机**的 5 自由度仿真，把这类问题提前暴露。

## 决策

在仿真里增加 real-compat 模式，由三部分组成：

1. **模型与场景**：
   - `robotic_arm_real_compat.xml`：与主模型只差 `motor6 gear=0`；
   - `scene_real_compat.xml` / `scene_vision_raised_real_compat.xml`：include 上述模型，
     并在顶层用 `<equality><joint joint1="joint6" polycoef="0 1 0 0 0" .../></equality>`
     把 joint6 约束到 0。
2. **控制器 overlay**：`ros2_controller_real_compat.yaml`
   （`real_mode=true`、`task_dim=5`、`allow_constrained_traj=true`）。
3. **验收序列**：`real_compat_joint` / `real_compat_pos` / `real_compat_box`；
   反例 `real_compat_joint6_lock` 预期失败。

## 关键实现细节

- **`polycoef` 全 0 等于没有约束**，必须是 `[0, 1, 0, 0, 0]`；只加 equality 不够，
  motor6 的 PID 力矩仍会把 joint6 推动约 0.04 rad，因此同时把 `motor6 gear` 置 0。
- joint6 锁死后夹爪朝向改变，会挡住相机看到的目标面；`box_pregrasp_5dof_high`
  改为从箱体上方接近，避免遮挡。
- `box_pregrasp_5dof` 只把原 `box_pregrasp` 的 joint6 置 0：joint6 绕工具轴旋转，
  不改变 5 维位姿。

## 备选方案

- **控制器加 `real_compat` 镜像 joint6 命令/状态**：场景无关，但会掩盖“锁死导致轨迹
  不可达”的问题，且要改实时路径。
- **新建包装 MujocoSystem 的硬件接口**：可同时模拟 100 Hz/USB/状态超时，最保真，
  但工作量最大；本轮不做。
- **只把任务点的 joint6 改成 0**：最省事，但 `task_dim=6` 的 IK 仍会使用 joint6，
  验证不到 5 维可达集。

## 后果

- 2026-11-13 实测：三条验收退出码 0；`box_move` 位置残差 0.0021 m、工具轴 0.34°；
  `/joint_states` 中 joint6 ≈ 1e-11；
- 这仍然是运动学层验证，不覆盖 100 Hz/USB 时序、断链行为与实机标定；
- 实机启用 `allow_constrained_traj` 前，必须先跑完这三条。

## 相关链接

- [位置约束轨迹开关拆分](1928-位置约束轨迹开关拆分.md)
- [real-compat 验收序列](../../execute/05-任务层与序列.md#real-compat-验收序列)
- `src/arm_model/model/robotic_arm_real_compat.xml`
- `src/launch_pack/config/ros2_controller_real_compat.yaml`
- `src/tasks/config/tasks.yaml`
