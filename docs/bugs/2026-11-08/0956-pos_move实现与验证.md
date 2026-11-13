---
id: INV9
title: pos_move实现与验证
type: investigation
date: 2026-11-08 09:56
---

# pos_move实现与验证

> 类型：investigation ｜ 日期：2026-11-08 09:56

> 2026-11-07，`jazzy/main`。记录新增第 11 类步骤的落地与实测结论，
> 尤其是**原计划的 A/B 正向用例被仿真证伪**这一点，别再照旧设计文档理解。

### 9.1 落地要点

- 任务层：`pos_move` 用 `positions` 段的基座系 `[x, y, z]` 绝对目标（或直接给
  `position`），姿态不参与收敛；结束时量测姿态漂移，超过 `max_drift_deg`
  **只告警、不判失败**。
- 消息：`CartTrajCmd` 增加 `mode`（`MODE_FULL_POSE=0` / `MODE_POSITION_ONLY=1`），
  `Vector` 仍 6 维。默认 0 保证旧发布者行为不变；`AdmittanceCmd` 未改。
- 控制器：`TaskMapping::set_pose_mode()` 在 `error_map()` / `jacobian_map()` 里显式
  投影掉全部 3 个旋转分量（不是软权重）；`IKSolver` 用独立的纯位置容差
  `pos_move_tolerance` 与零空间就近偏好 `pos_move_nullspace_weight`；预检在终点之外
  再查 `pos_move_precheck_samples` 个等分中间点（受 2 ms 周期约束，内部上限 3）。
- 三个参数按 C6 在 `on_init` 里 `auto_declare`、构造期读取并打 `position-only:` INFO。
- `allow_constrained_traj=false` 时 position-only 轨迹在**下发前拒绝**（实机默认；
  2026-11-13 前由 `real_mode` 控制），走 `update()` 的统一失效安全路径回 `idel`。

### 9.2 实测结论

| 项 | 结果 |
| --- | --- |
| `colcon build --packages-select robot_msgs tasks arm_controller` | 通过 |
| 配置加载校验 | 3 points / 3 positions / 11 sequences，全部步骤前置校验通过 |
| `pos_move_demo`（ready → 向下 5 cm） | 通过：位置残差 0.0009 m，姿态漂移 14.42° 触发 WARN |
| `pos_move_direct`（直接坐标，阈值 15°） | 通过：位置残差 0.0009 m，未告警 |
| `pos_move_near`（+Z 5 cm） | **预期失败**：position-only 残余 0.031 m，下发前拒绝 |
| 6 维 `reset_only` / `reset_then_joint` / `ready_then_cart` / `pick_demo` | 通过 |
| 6 维反例 `reset_then_cart` / `unreachable_near` / `unreachable_far` | 均退出码 1，且都是「下发前拒绝、完全不运动」 |

**A/B 用例证伪**：原计划拿 `unreachable_near`（+Z 5 cm，6 维残余 1.045997e-3）
当「6 维不可达 → position-only 可达」的正向对照。实测 position-only 同样不可达，
残余 0.031 m（最近可达位置约在起点上方 1.9 cm）。结论是**放松姿态能放宽
「位置+姿态组合」约束，但不能突破连杆长度与关节限位决定的高度上限**。
目前还没有找到能直接证明可达集扩大的目标，需要后续仿真扫描；`pos_move_near`
保留为反例，不要再把它写成正向验收。

**零空间偏好整定**：参考位形必须跨整条轨迹固定为起点关节角，否则每周期随实测角
重置就退化成「偏好当前位形」。实测权重 0.005 可用（漂移 14.77° → 14.42°），
权重 0.05 会让轨迹后段（t≈1.58 s）IK 停滞在 1.16e-3 > 1e-3 判据并把序列判失败。
抑制效果有限，**不能靠它保证工具朝向**；姿态不约束是语义，告警才是可观测性保障。

**本轮未执行**：`joint_console_only`（交互式，无法无人值守）、`measure_only`
（本轮未重复；该链路已在 2026-10-10 / 10-12 独立验证，见 CLAUDE.md 的测量记录）。
